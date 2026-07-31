"""Watch the K230 -> MCU stream on a PC, and say whether the MCU would be happy.

Wiring: a USB-TTL adapter's RX to the K230's TX (J8 pin 44), GND to GND. Leave
the MCU connected - this only listens, so it can run while the real link works.
Do not connect the adapter's TX unless you are using --ping.

  python tools/serial/sniff_k230.py --port COM5
  python tools/serial/sniff_k230.py --port COM5 --seconds 10 --quiet
  python tools/serial/sniff_k230.py --port COM5 --ping     # also test downlink
  python tools/serial/sniff_k230.py --selftest             # no hardware needed

--ping needs the adapter's TX on the K230's RX (pin 45) instead of the MCU, so
the MCU must be unplugged from that line first: two transmitters on one wire
means neither is readable.

Needs pyserial:  pip install pyserial
"""

import argparse
import sys
import time

from link_monitor import BALANCE_DATA_TIMEOUT_MS, LinkMonitor, P


def now_ms():
    return int(time.monotonic() * 1000.0)


def open_port(port, baud):
    try:
        import serial
    except ImportError:
        raise SystemExit("pyserial is not installed. Run: pip install pyserial")
    try:
        return serial.Serial(port=port, baudrate=baud, bytesize=8,
                             parity="N", stopbits=1, timeout=0.05)
    except Exception as exc:
        raise SystemExit("could not open %s: %s\n%s"
                         % (port, exc, list_ports_hint()))


def list_ports_hint():
    try:
        from serial.tools import list_ports
    except ImportError:
        return ""
    found = list(list_ports.comports())
    if not found:
        return "No serial ports found. Is the USB-TTL adapter plugged in?"
    return "Available ports:\n" + "\n".join(
        "  %s - %s" % (p.device, p.description) for p in found)


def run_live(args):
    port = open_port(args.port, args.baud)
    monitor = LinkMonitor(timeout_ms=args.timeout)
    started = now_ms()
    deadline = None if args.seconds <= 0 else started + int(args.seconds * 1000)
    last_report = started
    next_ping = started + 500 if args.ping else None
    ping_arg = 0x5A
    pings_sent = 0

    print("listening on %s at %d baud, Ctrl-C to stop"
          % (args.port, args.baud))
    if args.ping:
        print("downlink test on: sending CMD_PING every second")
    print("")

    try:
        while True:
            t = now_ms()
            chunk = port.read(256)
            if chunk:
                for event in monitor.feed(chunk, t):
                    print("  ! %s" % event)

            stall = monitor.check_stall(t)
            if stall:
                print("  ! %s" % stall)

            if next_ping is not None and t >= next_ping:
                port.write(P.encode_cmd(P.CMD_PING, ping_arg))
                pings_sent += 1
                next_ping = t + 1000

            if not args.quiet and t - last_report >= args.interval:
                if monitor.last_ball is not None:
                    print("  %s" % monitor.ball_line())
                last_report = t

            if deadline is not None and t >= deadline:
                break
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        port.close()

    print("\n--- summary ---")
    print(monitor.summary())
    if args.ping:
        print("pings sent=%d pongs received=%d" % (pings_sent, monitor.pongs))
        if pings_sent and monitor.pongs == 0:
            print("FAIL no PONG - downlink dead. Check the adapter TX is on "
                  "K230 pin 45 and the MCU is off that wire.")
        elif monitor.pongs:
            print("OK downlink works, the K230 is answering commands")
    return 0 if monitor.balls > 0 else 1


def run_selftest():
    """Drive LinkMonitor with synthetic frames. Proves the tool, not the link."""
    print("selftest: no hardware, feeding synthetic frames\n")
    monitor = LinkMonitor()
    t = 0
    seq = 0
    flags = P.FLAG_VALID | P.FLAG_DETECTED

    for pos in range(-40, 41, 10):
        frame = P.encode_ball(seq, flags, pos, pos * 2, 27)
        for event in monitor.feed(frame, t):
            print("  ! %s" % event)
        print("  %s" % monitor.ball_line())
        seq = (seq + 1) & 0xFF
        t += 37

    # Expectations are derived from what gets injected, not hardcoded, so the
    # two cannot drift apart when this block is edited.
    good_frames = monitor.balls
    want_gaps = 3

    # `seq` already holds the next number to send, so nudging it forward by N
    # leaves exactly N frames unsent.
    print("\n  injecting %d lost frame(s), a corrupt frame and a stall"
          % want_gaps)
    seq = (seq + want_gaps) & 0xFF
    for event in monitor.feed(P.encode_ball(seq, flags, 0, 0, 27), t):
        print("  ! %s" % event)
    good_frames += 1

    bad = bytearray(P.encode_ball((seq + 1) & 0xFF, flags, 0, 0, 27))
    bad[11] ^= 0xFF
    monitor.feed(bytes(bad), t)

    t += BALANCE_DATA_TIMEOUT_MS + 50
    stall = monitor.check_stall(t)
    if stall:
        print("  ! %s" % stall)

    print("\n--- summary ---")
    print(monitor.summary())

    expected = (
        ("balls", monitor.balls, good_frames),
        ("crc_errors", monitor.crc_errors, 1),
        ("gaps", monitor.gaps, want_gaps),
        ("stalls", monitor.stalls, 1),
        ("usable", monitor.usable_frames, good_frames),
    )
    ok = True
    for name, got, want in expected:
        if got != want:
            ok = False
            print("  FAIL %s = %d, want %d" % (name, got, want))
    print("\nselftest %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=0,
                    help="stop after N seconds (default: until Ctrl-C)")
    ap.add_argument("--interval", type=int, default=500,
                    help="ms between status lines (default 500)")
    ap.add_argument("--timeout", type=int, default=BALANCE_DATA_TIMEOUT_MS,
                    help="stall threshold in ms, mirrors the MCU's")
    ap.add_argument("--quiet", action="store_true",
                    help="only print events and the summary")
    ap.add_argument("--ping", action="store_true",
                    help="also send CMD_PING to test the downlink")
    ap.add_argument("--selftest", action="store_true",
                    help="check the tool itself against synthetic frames")
    ap.add_argument("--list", action="store_true", help="list serial ports")
    args = ap.parse_args()

    if args.list:
        print(list_ports_hint())
        return 0
    if args.selftest:
        return run_selftest()
    if not args.port:
        ap.error("--port is required (or use --selftest / --list)")
    return run_live(args)


if __name__ == "__main__":
    sys.exit(main())
