#!/usr/bin/env python3
"""
Test script for the ODrive custom binary command/telemetry protocol (EP2).

Firmware side lives in Firmware/communication/custom_protocol.cpp.

USB layout:
  interface 3 (vendor class 0x00, subclass 0x02)
  EP 0x02 OUT  host -> device
  EP 0x82 IN   device -> host

Framing:
  Host -> device:
      [u8 cmd][u8 req_id][u16 arg_len LE][args...]
  Device -> host:
      [u8 status][u8 req_id][u32 total_len LE][payload...]

Commands:
  0x00  echo      -> args returned unchanged
  0x01  telemetry -> args = [u16 LE count], returns min(count, 256) bytes of
                     pattern 0,1,2,...
  else            -> status 1 (unknown command), empty payload

Requirements:
  pip install pyusb libusb-package
  Windows: WinUSB driver must be bound to interface 3 (see usbd_cdc.c MS-OS
           descriptor). If the interface is not enumerable with pyusb, use
           Zadig/libwdi to install WinUSB on interface 3.
"""

import struct
import sys
import time

import usb.core
import usb.util
import libusb_package

VID = 0x1209
PID = 0x0D32

INTERFACE = 3
EP_OUT = 0x02
EP_IN = 0x82

TIMEOUT = 2000  # ms


def fail(msg):
    print("FAIL: %s" % msg)
    sys.exit(1)


def find_device():
    backend = libusb_package.get_libusb1_backend()
    dev = usb.core.find(idVendor=VID, idProduct=PID, backend=backend)
    if dev is None:
        fail("device %04x:%04x not found" % (VID, PID))
    return dev


def configure(dev):
    if sys.platform.startswith("linux"):
        if dev.is_kernel_driver_active(INTERFACE):
            try:
                dev.detach_kernel_driver(INTERFACE)
            except usb.core.USBError as e:
                fail("could not detach kernel driver: %s" % e)

    try:
        dev.set_configuration()
    except usb.core.USBError:
        pass  # already configured

    usb.util.claim_interface(dev, INTERFACE)

    intf = dev.get_active_configuration()[(INTERFACE, 0)]
    ep_out = usb.util.find_descriptor(intf, bEndpointAddress=EP_OUT)
    ep_in = usb.util.find_descriptor(intf, bEndpointAddress=EP_IN)
    if ep_out is None or ep_in is None:
        fail("endpoints %#04x/%#04x not found on interface %d" % (EP_OUT, EP_IN, INTERFACE))
    return ep_out, ep_in


def read_response(ep_in):
    data = bytearray()
    while len(data) < 6:
        data.extend(bytes(ep_in.read(64, timeout=TIMEOUT)))

    status, rid, length = struct.unpack_from("<BBI", data)

    while len(data) < 6 + length:
        data.extend(bytes(ep_in.read(64, timeout=TIMEOUT)))

    return status, rid, bytes(data[6:6 + length])


def request(ep_out, ep_in, c, arg_len, args=b"", req_id=1):
    frame = bytes([c, req_id]) + struct.pack("<H", arg_len) + args
    ep_out.write(frame, timeout=TIMEOUT)
    return read_response(ep_in)


def echo(ep_out, ep_in, args, req_id=1):
    return request(ep_out, ep_in, 0x00, len(args), args, req_id)


def telemetry(ep_out, ep_in, count, req_id=1):
    return request(ep_out, ep_in, 0x01, 2, struct.pack("<H", count), req_id)


def check(name, condition, detail=""):
    if condition:
        print("PASS  %s%s" % (name, ("  (%s)" % detail) if detail else ""))
    else:
        print("FAIL  %s%s" % (name, ("  (%s)" % detail) if detail else ""))
    return condition


def run_tests(ep_out, ep_in):
    ok = True

    # 1. echo, small payload
    payload = b"\x01\x02\x03\x04\x05"
    status, rid, data = echo(ep_out, ep_in, payload, req_id=1)
    ok &= check("echo small", status == 0 and rid == 1 and data == payload,
                "status=%d rid=%d data=%r" % (status, rid, data))

    # 2. echo, empty payload
    status, rid, data = echo(ep_out, ep_in, b"", req_id=2)
    ok &= check("echo empty", status == 0 and rid == 2 and data == b"",
                "status=%d rid=%d data=%r" % (status, rid, data))

    # 3. echo, large payload (506 bytes = firmware response cap, multi-chunk TX)
    payload = bytes((i * 7 + 3) & 0xFF for i in range(506))
    t0 = time.monotonic()
    status, rid, data = echo(ep_out, ep_in, payload, req_id=3)
    dt = time.monotonic() - t0
    ok &= check("echo large 506B", status == 0 and rid == 3 and data == payload,
                "status=%d rid=%d len=%d" % (status, rid, len(data)))
    print("      throughput: %.1f KB/s" % ((506 + 6) / dt / 1024.0))

    # 4. telemetry, small
    status, rid, data = telemetry(ep_out, ep_in, 10, req_id=4)
    expect = bytes(range(10))
    ok &= check("telemetry 10B", status == 0 and rid == 4 and data == expect,
                "status=%d rid=%d data=%r" % (status, rid, data))

    # 5. telemetry, over the 256-byte firmware cap -> should return 256 bytes
    status, rid, data = telemetry(ep_out, ep_in, 300, req_id=5)
    expect = bytes(range(256))
    ok &= check("telemetry 300B->256B", status == 0 and rid == 5 and data == expect,
                "status=%d rid=%d len=%d" % (status, rid, len(data)))

    # 6. unknown command -> status 1, empty payload
    status, rid, data = request(ep_out, ep_in, 0x7F, 0, b"", req_id=6)
    ok &= check("unknown command", status == 1 and rid == 6 and data == b"",
                "status=%d rid=%d data=%r" % (status, rid, data))

    return ok


def main():
    dev = find_device()
    print("device: %s" % dev)
    ep_out, ep_in = configure(dev)
    print("interface %d: OUT=%#04x IN=%#04x" % (INTERFACE, EP_OUT, EP_IN))
    print("-" * 60)

    try:
        ok = run_tests(ep_out, ep_in)
    finally:
        usb.util.release_interface(dev, INTERFACE)

    print("-" * 60)
    if ok:
        print("ALL TESTS PASSED")
        return 0
    else:
        print("SOME TESTS FAILED")
        return 1


if __name__ == "__main__":
    sys.exit(main())
