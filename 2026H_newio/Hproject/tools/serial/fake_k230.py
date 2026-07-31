"""Pretend to be the K230, so the MCU can be tested without the vision stack.

Streams synthetic MSG_BALL frames at a chosen rate and answers MSG_CMD with
MSG_PONG. Use it to prove the MCU's receive path, OLED display and balance PID
work before trusting the camera - or to reproduce link faults on demand.

Wiring: USB-TTL TX to the MCU's RX (PB7), GND to GND. The K230 must be off that
wire, or two transmitters fight over it. To watch the MCU's replies, also connect
the adapter's RX to the MCU's TX (PB6).

  python tools/serial/fake_k230.py --port COM5                  # ball sweeping
  python tools/serial/fake_k230.py --port COM5 --mode static --pos 40
  python tools/serial/fake_k230.py --port COM5 --mode lost      # no ruler lock
  python tools/serial/fake_k230.py --port COM5 --mode drop --drop-rate 0.2
  python tools/serial/fake_k230.py --port COM5 --mode stall     # go quiet
  python tools/serial/fake_k230.py --dry-run                    # no hardware

Modes
  sweep   ball oscillates across the tube, velocity consistent with position
  static  ball parked at --pos
  lost    frames keep coming with FLAG_VALID clear: link alive, ball unseen
  drop    sweep, but --drop-rate of frames are not sent (SEQ still advances,
          so the MCU should count seq_gaps)
  noise   sweep, with junk bytes injected between frames
  stall   sweep for --stall-after seconds, then go silent
"""

import argparse
import math
import random
import sys
import time

from link_monitor import LinkMonitor, P


def now_ms():
    return int(time.monotonic() * 1000.0)


def open_port(port, baud):
    try:
        import serial
    except ImportError:
        raise SystemExit("pyserial is not installed. Run: pip install pyserial")
    try:
        return serial.Serial(port=port, baudrate=baud, bytesize=8,
                             parity="N", stopbits=1, timeout=0)
    except Exception as exc:
        raise SystemExit("could not open %s: %s" % (port, exc))


class BallSource:
    """Generates the position/velocity/flags a real K230 would report."""

    def __init__(self, mode, pos_mm, amplitude_mm, period_s, fps):
        self.mode = mode
        self.pos_mm = pos_mm
        self.amplitude_mm = amplitude_mm
        self.period_s = period_s
        self.fps = fps

    def sample(self, t_s):
        if self.mode == "static":
            return self.pos_mm, 0, P.FLAG_VALID | P.FLAG_DETECTED

        if self.mode == "lost":
            # Link alive, ruler not locked. The MCU must stay fresh but idle.
            return 0, 0, 0

        omega = 2.0 * math.pi / self.period_s
        pos = self.amplitude_mm * math.sin(omega * t_s)
        # Analytic derivative, so position and velocity agree the way the
        # K230's own filter would make them agree.
        vel = self.amplitude_mm * omega * math.cos(omega * t_s)
        return int(round(pos)), int(round(vel)), P.FLAG_VALID | P.FLAG_DETECTED


def run(args, port=None):
    source = BallSource(args.mode, args.pos, args.amplitude,
                        args.period, args.fps)
    monitor = LinkMonitor()          # tracks what we get back, if anything
    reader = P.CommandReader()
    rng = random.Random(args.seed)

    interval_ms = 1000.0 / float(args.fps)
    started = now_ms()
    deadline = None if args.seconds <= 0 else started + int(args.seconds * 1000)
    stall_at = (None if args.stall_after <= 0
                else started + int(args.stall_after * 1000))

    seq = 0
    sent = 0
    skipped = 0
    pings = 0
    next_frame = started
    last_report = started
    stalled = False

    if port is not None:
        print("sending on %s at %d baud, mode=%s, %d fps, Ctrl-C to stop"
              % (args.port, args.baud, args.mode, args.fps))
    print("")

    try:
        while True:
            t = now_ms()

            if stall_at is not None and t >= stall_at and not stalled:
                stalled = True
                print("  ! going silent now - the MCU should stand the PID down "
                      "within %d ms" % monitor.timeout_ms)

            if t >= next_frame:
                pos, vel, flags = source.sample((t - started) / 1000.0)
                frame = P.encode_ball(seq, flags, pos, vel, args.fps)
                seq = (seq + 1) & 0xFF

                drop = (args.mode == "drop" and rng.random() < args.drop_rate)
                if stalled:
                    drop = True

                if not drop:
                    payload = frame
                    if args.mode == "noise":
                        junk = bytes(rng.randrange(256)
                                     for _ in range(rng.randrange(1, 4)))
                        payload = junk + frame
                    if port is not None:
                        port.write(payload)
                    sent += 1
                else:
                    skipped += 1

                next_frame += interval_ms

            if port is not None:
                chunk = port.read(256)
                if chunk:
                    # Anything the MCU sends us: commands to answer, and any
                    # frames it echoes (it should not, but worth seeing).
                    for cmd, arg in reader.feed(chunk):
                        pings += 1
                        if cmd == P.CMD_PING:
                            port.write(P.encode_pong(arg))
                            print("  ! got CMD_PING arg=%02X, replied PONG" % arg)
                        else:
                            print("  ! got CMD %02X arg=%02X" % (cmd, arg))

            if t - last_report >= args.interval:
                pos, vel, flags = source.sample((t - started) / 1000.0)
                print("  sent=%d skipped=%d pos=%+6d mm vel=%+6d mm/s %s"
                      % (sent, skipped, pos, vel, P.flags_str(flags)))
                last_report = t

            if deadline is not None and t >= deadline:
                break

            time.sleep(0.002)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        if port is not None:
            port.close()

    print("\n--- summary ---")
    print("frames sent=%d skipped=%d commands received=%d" % (sent, skipped, pings))
    if args.mode == "drop" and skipped:
        print("the MCU's seq_gaps should be about %d" % skipped)
    if pings == 0 and port is not None:
        print("no commands arrived - normal unless the firmware calls "
              "Camera_Ping(), which it currently does not")
    return 0


def run_dry():
    """Generate every mode without a port, checking the frames are well formed."""
    print("dry run: no hardware, checking generated frames\n")
    ok = True
    for mode in ("sweep", "static", "lost", "drop", "noise", "stall"):
        source = BallSource(mode, 40, 75, 4.0, 27)
        monitor = LinkMonitor()
        t_ms = 0
        seq = 0
        for i in range(30):
            pos, vel, flags = source.sample(t_ms / 1000.0)
            frame = P.encode_ball(seq, flags, pos, vel, 27)
            monitor.feed(frame, t_ms)
            seq = (seq + 1) & 0xFF
            t_ms += 37
        good = monitor.balls == 30 and monitor.crc_errors == 0
        want_usable = 0 if mode == "lost" else 30
        good = good and monitor.usable_frames == want_usable
        print("  %-7s balls=%d usable=%d crc_err=%d  %s"
              % (mode, monitor.balls, monitor.usable_frames,
                 monitor.crc_errors, "ok" if good else "FAIL"))
        ok = ok and good

    # A sweep must stay inside int16 and never saturate at sane amplitudes.
    source = BallSource("sweep", 0, 75, 4.0, 27)
    sat = False
    for i in range(200):
        pos, vel, flags = source.sample(i * 0.037)
        frame = P.encode_ball(0, flags, pos, vel, 27)
        if P.decode_ball(frame)["flags"] & P.FLAG_SATURATED:
            sat = True
    print("  sweep never saturates: %s" % ("ok" if not sat else "FAIL"))
    ok = ok and not sat

    print("\ndry run %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", default="sweep",
                    choices=("sweep", "static", "lost", "drop", "noise", "stall"))
    ap.add_argument("--fps", type=int, default=27,
                    help="frames per second (default 27, the camera's rate)")
    ap.add_argument("--pos", type=int, default=0,
                    help="position in mm for --mode static")
    ap.add_argument("--amplitude", type=int, default=75,
                    help="sweep amplitude in mm (default 75, a ~15cm tube)")
    ap.add_argument("--period", type=float, default=4.0,
                    help="sweep period in seconds")
    ap.add_argument("--drop-rate", type=float, default=0.2,
                    help="fraction of frames to drop for --mode drop")
    ap.add_argument("--stall-after", type=float, default=5.0,
                    help="seconds before going silent for --mode stall")
    ap.add_argument("--seconds", type=float, default=0,
                    help="stop after N seconds (default: until Ctrl-C)")
    ap.add_argument("--interval", type=int, default=500,
                    help="ms between status lines")
    ap.add_argument("--seed", type=int, default=1,
                    help="RNG seed, so drop/noise runs are reproducible")
    ap.add_argument("--dry-run", action="store_true",
                    help="check the generator without a serial port")
    args = ap.parse_args()

    if args.dry_run:
        return run_dry()
    if not args.port:
        ap.error("--port is required (or use --dry-run)")
    if args.mode != "stall":
        args.stall_after = 0
    return run(args, open_port(args.port, args.baud))


if __name__ == "__main__":
    sys.exit(main())
