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
  0x02  read telemetry -> [u32 overflow][N x 24-byte records]
                     record = [u8 type][u8 pad x3][u32 timestamp][f32 ofs_phB][f32 ofs_phC][f32 raw_phB][f32 raw_phC]
                     ofs_phX = 0.5 * (ofs1_phX + ofs2_phX), the average offset around raw
                     type: 0=data, 1=START, 2=END, 3=ENC (enc_count in raw_phC)
  0x04  trigger calibration -> starts encoder offset calibration on axis 0
  else            -> status 1 (unknown command), empty payload

Requirements:
  pip install pyusb libusb-package
  Windows: WinUSB driver must be bound to interface 3 (see usbd_cdc.c MS-OS
           descriptor). If the interface is not enumerable with pyusb, use
           Zadig/libwdi to install WinUSB on interface 3.
"""

import argparse
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


def open_custom_interface():
    """Open the custom interface (3) on the ODrive.

    On Windows, WinUSB is loaded per-interface, so odrivetool (interface 2) and
    the custom protocol (interface 3) are separate libusb devices that share the
    same VID/PID. Enumerate all of them and use the one where claiming interface 3
    and speaking the custom protocol actually works.
    """
    backend = libusb_package.get_libusb1_backend()
    candidates = list(usb.core.find(find_all=True, idVendor=VID, idProduct=PID, backend=backend))
    print("found %d candidate device(s) for %04x:%04x" % (len(candidates), VID, PID))
    if not candidates:
        fail("device %04x:%04x not found" % (VID, PID))

    last_err = None
    for dev in candidates:
        try:
            if sys.platform.startswith("linux"):
                if dev.is_kernel_driver_active(INTERFACE):
                    dev.detach_kernel_driver(INTERFACE)

            try:
                dev.set_configuration()
            except usb.core.USBError:
                pass  # already configured

            usb.util.claim_interface(dev, INTERFACE)

            intf = dev.get_active_configuration()[(INTERFACE, 0)]
            ep_out = usb.util.find_descriptor(intf, bEndpointAddress=EP_OUT)
            ep_in = usb.util.find_descriptor(intf, bEndpointAddress=EP_IN)
            if ep_out is None or ep_in is None:
                raise usb.core.USBError("endpoints not found")

            # sanity-check that this device actually speaks the custom protocol
            status, _, _ = request(ep_out, ep_in, 0x00, 0, b"")
            if status != 0:
                raise usb.core.USBError("bad echo response")

            return dev, ep_out, ep_in
        except (usb.core.USBError, IndexError, KeyError) as e:
            last_err = e
            try:
                print("  candidate bus=%d addr=%d failed: %s" % (dev.bus, dev.address, e))
            except Exception:
                print("  candidate failed: %s" % e)
            continue

    if last_err is not None:
        fail("could not open interface %d: %s" % (INTERFACE, last_err))
    fail("interface %d not found on device %04x:%04x" % (INTERFACE, VID, PID))


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


RECORD = struct.Struct("<B3xIffff")  # type u8 + pad3 + timestamp u32 + 4 floats

START, DATA, END, ENC = 1, 0, 2, 3


def read_telemetry(ep_out, ep_in, req_id=1):
    status, rid, data = request(ep_out, ep_in, 0x02, 0, b"", req_id)
    if status != 0:
        return 0, [], rid
    overflow = struct.unpack("<I", data[0:4])[0] if len(data) >= 4 else 0
    records = [RECORD.unpack(data[i:i + RECORD.size]) for i in range(4, len(data), RECORD.size)]
    return overflow, records, rid


def trigger_calibration(ep_out, ep_in, req_id=1):
    status, _, _ = request(ep_out, ep_in, 0x04, 0, b"", req_id)
    return status


def _save(records, out_path):
    with open(out_path, "w") as f:
        f.write("timestamp,ofs_phB,ofs_phC,raw_phB,raw_phC\n")
        for ts, a, b, c, d in records:
            f.write("%u,%.6f,%.6f,%.6f,%.6f\n" % (ts, a, b, c, d))


def _save_enc(enc_records, out_path):
    path = out_path
    if path.lower().endswith(".csv"):
        path = path[:-4] + "_enc.csv"
    else:
        path = path + "_enc.csv"
    with open(path, "w") as f:
        f.write("timestamp,enc_count\n")
        for ts, cnt in enc_records:
            f.write("%u,%d\n" % (ts, cnt))


def drain_telemetry(ep_out, ep_in, out_path=None):
    """Drain until the firmware's END marker (Calibrator::run() auto-arm flow).

    The firmware pushes START at initMotor() and END when calibration finishes;
    we collect the DATA (current) and ENC (encoder) records in between and stop
    on END.
    """
    records = []
    enc_records = []
    print("draining telemetry... waiting for START, auto-stop on END")
    rid = 1
    started = False
    done = False
    overflow = 0
    try:
        while not done:
            ovf, recs, _ = read_telemetry(ep_out, ep_in, req_id=rid)
            rid = (rid + 1) & 0xFF
            overflow = ovf  # monotonic within a run
            for t, ts, a, b, c, d in recs:
                if t == START:
                    records = []
                    enc_records = []
                    started = True
                elif t == END:
                    done = True
                    break
                elif started:
                    if t == DATA:
                        records.append((ts, a, b, c, d))
                    elif t == ENC:
                        enc_records.append((ts, int(round(d))))
    except KeyboardInterrupt:
        pass

    if out_path:
        _save(records, out_path)
        _save_enc(enc_records, out_path)

    return records, enc_records, overflow


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


def main(argv=None):
    parser = argparse.ArgumentParser(description="ODrive custom EP2 protocol test / recorder")
    parser.add_argument("--drain", action="store_true",
                        help="drain telemetry until END marker (auto-arm flow)")
    parser.add_argument("--calibrate", action="store_true",
                        help="trigger encoder offset calibration (axis 0), then drain until END")
    parser.add_argument("--out", type=str, metavar="FILE",
                        help="save recorded telemetry to CSV (with --drain/--calibrate)")
    args = parser.parse_args(argv)

    dev, ep_out, ep_in = open_custom_interface()
    print("device: %s" % dev)
    print("interface %d: OUT=%#04x IN=%#04x" % (INTERFACE, EP_OUT, EP_IN))
    print("-" * 60)

    try:
        if args.calibrate:
            return run_calibrate(ep_out, ep_in, args.out)
        if args.drain:
            return run_drain(ep_out, ep_in, args.out)
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


def uint32_diff(a, b):
    return (a - b) & 0xFFFFFFFF


def report_result(records, enc_records, overflow, out_path):
    print("-" * 60)
    print("current records: %d" % len(records))
    print("encoder records: %d" % len(enc_records))
    print("overflow (lost records): %d" % overflow)

    # DATA timestamp wraps at 2^32, so compare with modular (wrap-safe) deltas
    if len(records) > 1:
        gaps = [uint32_diff(records[i + 1][0], records[i][0]) for i in range(len(records) - 1)]
        expected = gaps[0]
        uniform = all(g == expected for g in gaps)
        nonzero = all(g > 0 for g in gaps)
        print("timestamp delta (ticks/sample): %d" % expected)
        print("deltas uniform & >0: %s" % (uniform and nonzero))
    else:
        uniform = True
        nonzero = True

    if out_path:
        print("saved to %s (encoder -> %s_enc.csv)" % (out_path, out_path[:-4] if out_path.lower().endswith(".csv") else out_path))

    ok = ((len(records) > 0) or (len(enc_records) > 0)) and (overflow == 0) and uniform and nonzero
    if ok:
        print("OK: no data loss detected")
    else:
        print("WARNING: data loss detected (overflow=%d, uniform=%s, nonzero=%s)" % (overflow, uniform, nonzero))
    return 0 if ok else 1


def run_drain(ep_out, ep_in, out_path):
    records, enc_records, overflow = drain_telemetry(ep_out, ep_in, out_path)
    return report_result(records, enc_records, overflow, out_path)


def run_calibrate(ep_out, ep_in, out_path):
    print("triggering encoder offset calibration (axis 0)...")
    status = trigger_calibration(ep_out, ep_in, req_id=90)
    if status != 0:
        print("FAILED to trigger calibration (status=%d)" % status)
        return 1
    return run_drain(ep_out, ep_in, out_path)


if __name__ == "__main__":
    sys.exit(main())
