"""End-to-end link check: the real uart_send_ball() against the real C parser.

test_protocol.py checks protocol.py against camera_uart.c. This checks the layer
above it: the actual uart_send_ball() and uart_poll_commands() lifted out of
k230D/main.py, so a correct protocol.py wired up wrongly still fails here.

main.py cannot be imported on a PC (it pulls in CanMV's PipeLine, YOLO11 and
Sensor at module scope and runs the capture loop on import), so the two UART
functions are extracted from its source and exec'd against a fake UART. That
keeps the code under test byte-identical to what runs on the device instead of a
copy that can drift.

Run:  python tools/test/test_k230_link.py
"""

import ast
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HPROJECT = os.path.abspath(os.path.join(HERE, "..", ".."))
K230D = os.path.abspath(os.path.join(HPROJECT, "..", "k230D"))

sys.path.insert(0, K230D)
import protocol as P  # noqa: E402

BALANCE_DATA_TIMEOUT_MS = 150

SOURCES = [
    os.path.join(HERE, "test_protocol.c"),
    os.path.join(HERE, "stubs", "stubs.c"),
    os.path.join(HPROJECT, "drivers", "camera_uart.c"),
    os.path.join(HPROJECT, "control", "balance_controller.c"),
]

FAILURES = []
CHECKS = [0]


def check(cond, what):
    CHECKS[0] += 1
    if not cond:
        FAILURES.append(what)
        print("  FAIL " + what)


# ------------------------------------------------------- load main.py's UART fns

def load_main_uart_functions():
    """exec just the UART functions from main.py, against a stub namespace."""
    path = os.path.join(K230D, "main.py")
    with open(path, "r", encoding="utf-8") as fh:
        source = fh.read()
    tree = ast.parse(source)

    wanted = ("uart_send_ball", "uart_poll_commands")
    picked = [n for n in tree.body
              if isinstance(n, ast.FunctionDef) and n.name in wanted]
    missing = set(wanted) - {n.name for n in picked}
    if missing:
        raise SystemExit("main.py is missing: " + ", ".join(sorted(missing)))

    ns = {
        "P": P,
        "print": lambda *a, **k: None,      # keep test output clean
        "uart_stream_enabled": True,
        "pos_zero_offset_cm": 0.0,
        "last_valid_raw_x_cm": None,
    }
    exec(compile(ast.Module(body=picked, type_ignores=[]), path, "exec"), ns)
    return ns


class FakeUart:
    """Records what main.py writes; replays what we want it to read."""

    def __init__(self):
        self.written = bytearray()
        self.to_read = bytearray()
        self.fail_next_write = False

    def write(self, data):
        if self.fail_next_write:
            self.fail_next_write = False
            raise OSError("simulated UART failure")
        self.written.extend(data)
        return len(data)

    def read(self, n):
        if not self.to_read:
            return None                      # CanMV returns None, not b""
        chunk = bytes(self.to_read[:n])
        del self.to_read[:n]
        return chunk

    def take(self):
        out = bytes(self.written)
        self.written = bytearray()
        return out


class Harness:
    """One live instance of the C binary, driven line by line."""

    def __init__(self, exe):
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, text=True, bufsize=1)

    def cmd(self, line):
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()
        out = self.p.stdout.readline().strip()
        if not out:
            raise SystemExit("harness died on: " + line)
        return out

    def close(self):
        self.p.stdin.close()
        self.p.wait(timeout=10)

    @staticmethod
    def fields(line):
        out = {}
        for tok in line.split()[1:]:
            if "=" in tok:
                k, v = tok.split("=", 1)
                out[k] = v
        return out

    def feed(self, now_ms, data):
        self.cmd("feed %d %s" % (now_ms, " ".join("%02X" % b for b in data)))

    def data(self):
        f = self.fields(self.cmd("data"))
        return {k: int(f[k], 16) if k == "flags" else int(f[k]) for k in f}

    def stats(self):
        return {k: int(v) for k, v in self.fields(self.cmd("stats")).items()}

    def fresh(self, now_ms, timeout=BALANCE_DATA_TIMEOUT_MS):
        f = self.fields(self.cmd("fresh %d %d" % (now_ms, timeout)))
        return int(f["fresh"]), int(f["usable"])

    def tick(self, now_ms):
        f = self.fields(self.cmd("tick %d" % now_ms))
        return {k: int(v) for k, v in f.items()}

    def reset(self):
        self.cmd("reset")


def build():
    exe = os.path.join(HERE, "test_k230_link.exe")
    cmd = ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-O1",
           "-DBALANCE_USE_CAMERA_VELOCITY=1",
           "-I" + os.path.join(HPROJECT, "drivers"),
           "-I" + os.path.join(HPROJECT, "control"),
           "-I" + os.path.join(HERE, "stubs")] + SOURCES + ["-o", exe]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit("build failed:\n" + r.stdout + r.stderr)
    return exe


# ------------------------------------------------------------------ test cases

def test_frame_shape(ns, uart, h):
    """One send must produce exactly one 12-byte frame the parser accepts."""
    send = ns["uart_send_ball"]
    uart.take()
    send(uart, 0, True, False, True, 0.0, 0.0, 27)
    frame = uart.take()
    check(len(frame) == P.BALL_FRAME_LEN,
          "frame is %d bytes, want %d" % (len(frame), P.BALL_FRAME_LEN))
    check(frame[:4] == bytes([P.SYNC0, P.SYNC1, P.MSG_BALL, P.BALL_PAYLOAD_LEN]),
          "header %s" % " ".join("%02X" % b for b in frame[:4]))

    before = h.stats()["frames"]
    h.feed(1000, frame)
    check(h.stats()["frames"] == before + 1, "parser accepts a real frame")
    check(h.stats()["crc_err"] == 0, "no CRC error on a real frame")


def test_units_and_sign(ns, uart, h):
    """cm on the K230 must land as mm on the MCU, sign intact."""
    send = ns["uart_send_ball"]
    now = 2000
    # Realistic tube positions: the ruler spans ~10 cm between the red lines.
    cases = [(0.0, 0.0), (1.2, 5.0), (-1.2, -5.0), (5.0, 30.0), (-5.0, -30.0),
             (7.5, 120.0), (-7.5, -120.0), (0.05, 0.0), (-0.05, 0.0),
             (12.34, -56.78), (-12.34, 56.78)]
    for x_cm, v_cm_s in cases:
        uart.take()
        send(uart, 1, True, False, True, x_cm, v_cm_s, 27)
        h.feed(now, uart.take())
        d = h.data()
        check(d["pos"] == int(round(x_cm * 10.0)),
              "pos %.2fcm -> %dmm (got %d)"
              % (x_cm, int(round(x_cm * 10.0)), d["pos"]))
        check(d["vel"] == int(round(v_cm_s * 10.0)),
              "vel %.2fcm/s -> %dmm/s (got %d)"
              % (v_cm_s, int(round(v_cm_s * 10.0)), d["vel"]))
        now += 40


def test_flags(ns, uart, h):
    """FLAGS must carry the detection state the MCU gates on."""
    send = ns["uart_send_ball"]
    now = 3000
    cases = [
        # valid, predicted, detected -> expected flag bits
        (True, False, True, P.FLAG_VALID | P.FLAG_DETECTED),
        (True, True, True, P.FLAG_VALID | P.FLAG_DETECTED | P.FLAG_PREDICTED),
        (True, True, False, P.FLAG_VALID | P.FLAG_PREDICTED),
        (True, False, False, P.FLAG_VALID),
        (False, False, True, P.FLAG_DETECTED),
        (False, False, False, 0),
    ]
    for valid, predicted, detected, want in cases:
        uart.take()
        send(uart, 2, valid, predicted, detected, 3.0, 10.0, 27)
        h.feed(now, uart.take())
        d = h.data()
        check(d["flags"] == want,
              "flags v=%d p=%d d=%d -> %02X (got %02X)"
              % (valid, predicted, detected, want, d["flags"]))
        # The gate the balance PID actually uses.
        want_usable = 1 if (want & P.FLAG_VALID) else 0
        check(h.fresh(now)[1] == want_usable,
              "usable v=%d -> %d" % (valid, want_usable))
        now += 40


def test_no_lock_sends_zero(ns, uart, h):
    """Without a ruler lock the position must not be passed off as metric."""
    send = ns["uart_send_ball"]
    uart.take()
    send(uart, 3, False, False, True, 987.6, 543.2, 27)
    h.feed(4000, uart.take())
    d = h.data()
    check(d["pos"] == 0, "unlocked pos zeroed (got %d)" % d["pos"])
    check(d["vel"] == 0, "unlocked vel zeroed (got %d)" % d["vel"])
    check(d["flags"] & P.FLAG_VALID == 0, "unlocked clears FLAG_VALID")


def test_always_sends(ns, uart, h):
    """The contract: a frame every iteration, ball or no ball.

    This is the regression that mattered most - the old code returned early when
    the ball was not visible, so the MCU could not tell that from a dead link.
    """
    send = ns["uart_send_ball"]
    for valid in (True, False):
        for detected in (True, False):
            uart.take()
            send(uart, 4, valid, False, detected, 2.0, 0.0, 27)
            check(len(uart.take()) == P.BALL_FRAME_LEN,
                  "sends when valid=%d detected=%d" % (valid, detected))

    # Ball invisible for a long stretch: the link must stay fresh throughout,
    # while never becoming usable.
    h.reset()
    now = 5000
    for i in range(40):
        uart.take()
        send(uart, i & 0xFF, False, False, False, 0.0, 0.0, 27)
        h.feed(now, uart.take())
        now += 37                              # ~27 fps
    check(h.fresh(now - 37) == (1, 0),
          "40 ball-less frames: link fresh, position not usable")
    check(h.stats()["frames"] == 40, "all 40 frames accepted")
    check(h.stats()["gaps"] == 0, "sequential SEQ produces no gaps")


def test_seq_and_fps(ns, uart, h):
    """SEQ must advance so the MCU's drop counter works; FPS must survive."""
    send = ns["uart_send_ball"]
    h.reset()
    now = 6000
    seq = 0
    for _ in range(10):
        uart.take()
        send(uart, seq, True, False, True, 1.0, 0.0, 27)
        h.feed(now, uart.take())
        check(h.data()["seq"] == seq, "seq %d round-trips" % seq)
        seq = (seq + 1) & 0xFF
        now += 40
    check(h.stats()["gaps"] == 0, "no spurious gaps")

    # A skipped frame must show up as a gap, not pass silently.
    uart.take()
    send(uart, (seq + 3) & 0xFF, True, False, True, 1.0, 0.0, 27)
    h.feed(now, uart.take())
    check(h.stats()["gaps"] == 3, "skipped frames counted as gaps")

    # clock.fps() is a float on the device; it must not break the encoder.
    for fps in (0, 1, 26.7, 27.4, 255, 300, -5):
        uart.take()
        send(uart, 0, True, False, True, 1.0, 0.0, fps)
        frame = uart.take()
        check(len(frame) == P.BALL_FRAME_LEN, "fps=%r keeps frame length" % fps)
        h.feed(now, frame)
        want = max(0, min(255, int(fps)))
        check(h.data()["fps"] == want, "fps %r -> %d" % (fps, want))


def test_saturation(ns, uart, h):
    """A wild position must clamp and say so, not wrap to the far end."""
    send = ns["uart_send_ball"]
    for x_cm, want in ((5000.0, 32767), (-5000.0, -32768)):
        uart.take()
        send(uart, 0, True, False, True, x_cm, 0.0, 27)
        h.feed(7000, uart.take())
        d = h.data()
        check(d["pos"] == want, "clamp %.0fcm -> %d (got %d)" % (x_cm, want, d["pos"]))
        check(d["flags"] & P.FLAG_SATURATED, "saturation flagged at %.0fcm" % x_cm)


def test_write_failure_is_survivable(ns, uart, h):
    """A UART write error must not kill the vision loop."""
    send = ns["uart_send_ball"]
    uart.take()
    uart.fail_next_write = True
    try:
        send(uart, 0, True, False, True, 1.0, 0.0, 27)
        raised = False
    except Exception:
        raised = True
    check(not raised, "write failure is swallowed")
    check(uart.take() == b"", "nothing written on failure")

    send(uart, 1, True, False, True, 1.0, 0.0, 27)
    check(len(uart.take()) == P.BALL_FRAME_LEN, "recovers after a failed write")


def test_downlink_ping(ns, uart, h):
    """CMD_PING must be answered with a well-formed PONG."""
    poll = ns["uart_poll_commands"]
    reader = P.CommandReader()

    uart.take()
    uart.to_read.extend(P.encode_cmd(P.CMD_PING, 0x5A))
    n = poll(uart, reader)
    check(n == 1, "one command handled (got %d)" % n)
    reply = uart.take()
    check(reply == P.encode_pong(0x5A), "pong echoes ARG")

    before = h.stats()["pongs"]
    h.feed(8000, reply)
    check(h.stats()["pongs"] == before + 1, "MCU parser accepts the pong")

    # Nothing to read must be harmless and silent.
    check(poll(uart, reader) == 0, "empty read handled")
    check(uart.take() == b"", "no reply when no command")

    # A command split across two reads must still be seen.
    frame = P.encode_cmd(P.CMD_PING, 0x11)
    uart.to_read.extend(frame[:3])
    check(poll(uart, reader) == 0, "partial command not yet complete")
    uart.to_read.extend(frame[3:])
    check(poll(uart, reader) == 1, "split command reassembled")
    check(uart.take() == P.encode_pong(0x11), "pong after split command")

    # Corrupt CRC must be rejected, and the reader must recover afterwards.
    bad = bytearray(P.encode_cmd(P.CMD_PING, 0x22))
    bad[6] ^= 0xFF
    uart.to_read.extend(bytes(bad))
    check(poll(uart, reader) == 0, "bad CRC rejected")
    check(uart.take() == b"", "no reply to a corrupt command")
    uart.to_read.extend(P.encode_cmd(P.CMD_PING, 0x33))
    check(poll(uart, reader) == 1, "recovers after bad CRC")
    check(uart.take() == P.encode_pong(0x33), "pong after recovery")


def test_downlink_set_send(ns, uart, h):
    """CMD_SET_SEND must actually gate the stream."""
    poll = ns["uart_poll_commands"]
    reader = P.CommandReader()

    uart.to_read.extend(P.encode_cmd(P.CMD_SET_SEND, 0))
    poll(uart, reader)
    check(ns["uart_stream_enabled"] is False, "SET_SEND 0 stops the stream")

    uart.to_read.extend(P.encode_cmd(P.CMD_SET_SEND, 1))
    poll(uart, reader)
    check(ns["uart_stream_enabled"] is True, "SET_SEND 1 resumes the stream")


def test_downlink_set_zero(ns, uart, h):
    """CMD_SET_ZERO must re-zero against the real position, and only then."""
    poll = ns["uart_poll_commands"]
    send = ns["uart_send_ball"]
    reader = P.CommandReader()

    # No lock yet -> must be ignored rather than zeroing against nothing.
    ns["last_valid_raw_x_cm"] = None
    ns["pos_zero_offset_cm"] = 0.0
    uart.to_read.extend(P.encode_cmd(P.CMD_SET_ZERO, 0))
    poll(uart, reader)
    check(ns["pos_zero_offset_cm"] == 0.0, "SET_ZERO ignored without a lock")

    # With a lock, the offset must become the current raw position, so the
    # position main.py reports for that spot becomes zero.
    ns["last_valid_raw_x_cm"] = 3.75
    uart.to_read.extend(P.encode_cmd(P.CMD_SET_ZERO, 0))
    poll(uart, reader)
    check(ns["pos_zero_offset_cm"] == 3.75, "SET_ZERO takes the raw position")

    uart.take()
    send(uart, 0, True, False, True, 3.75 - ns["pos_zero_offset_cm"], 0.0, 27)
    h.feed(9000, uart.take())
    check(h.data()["pos"] == 0, "re-zeroed point reports 0 mm")

    # 2 cm past the new zero must read as +20 mm.
    uart.take()
    send(uart, 1, True, False, True, 5.75 - ns["pos_zero_offset_cm"], 0.0, 27)
    h.feed(9040, uart.take())
    check(h.data()["pos"] == 20, "2cm past new zero reads +20mm")


def test_stream_drives_pid(ns, uart, h):
    """The whole point: a real K230 stream must move the stepper."""
    send = ns["uart_send_ball"]
    h.reset()
    h.cmd("disable")
    h.cmd("enable")
    h.cmd("target 0")

    now = 10000
    seq = 0
    # Ball sitting 4 cm right of centre and still.
    for _ in range(3):
        uart.take()
        send(uart, seq, True, False, True, 4.0, 0.0, 27)
        h.feed(now, uart.take())
        t = h.tick(now)
        seq = (seq + 1) & 0xFF
        now += 37
    check(t["track"] == 1, "PID tracks a real stream")
    check(t["err"] == -40, "error is -40mm for a ball 4cm right (got %d)" % t["err"])
    check(t["out"] < 0, "ball right of target drives negative (got %d)" % t["out"])
    check(t["step"] == t["out"], "stepper follows the PID output")

    # Mirror image, on the other side.
    h.cmd("disable")
    h.cmd("enable")
    h.cmd("target 0")
    for _ in range(3):
        uart.take()
        send(uart, seq, True, False, True, -4.0, 0.0, 27)
        h.feed(now, uart.take())
        t = h.tick(now)
        seq = (seq + 1) & 0xFF
        now += 37
    check(t["err"] == 40, "error is +40mm for a ball 4cm left (got %d)" % t["err"])
    check(t["out"] > 0, "ball left of target drives positive (got %d)" % t["out"])

    # Ball goes out of sight: frames keep coming, PID must stand down and the
    # stepper must stop - without the link ever being declared dead.
    for _ in range(5):
        uart.take()
        send(uart, seq, False, False, False, 0.0, 0.0, 27)
        h.feed(now, uart.take())
        t = h.tick(now)
        seq = (seq + 1) & 0xFF
        now += 37
    check(h.fresh(now - 37)[0] == 1, "link still fresh with the ball lost")
    check(t["track"] == 0, "PID stands down with no ruler lock")
    check(t["step"] == 0, "stepper stopped with the ball lost")

    # Ball comes back.
    for _ in range(3):
        uart.take()
        send(uart, seq, True, False, True, 4.0, 0.0, 27)
        h.feed(now, uart.take())
        t = h.tick(now)
        seq = (seq + 1) & 0xFF
        now += 37
    check(t["track"] == 1, "PID reacquires after the ball returns")


def test_stall_is_detected(ns, uart, h):
    """If the K230 stops sending, the MCU must notice."""
    send = ns["uart_send_ball"]
    h.reset()
    now = 20000
    uart.take()
    send(uart, 0, True, False, True, 2.0, 0.0, 27)
    h.feed(now, uart.take())
    check(h.fresh(now) == (1, 1), "fresh right after a frame")
    check(h.fresh(now + BALANCE_DATA_TIMEOUT_MS - 1)[0] == 1,
          "still fresh just inside the timeout")
    check(h.fresh(now + BALANCE_DATA_TIMEOUT_MS)[0] == 0,
          "stall detected at the timeout")

    # At 27 fps the send interval must leave plenty of margin on that timeout.
    interval_ms = 1000.0 / 27.0
    check(interval_ms * 2 < BALANCE_DATA_TIMEOUT_MS,
          "send interval %.1fms has margin on the %dms timeout"
          % (interval_ms, BALANCE_DATA_TIMEOUT_MS))


def test_old_format_would_fail(ns, uart, h):
    """Guard against a regression to the legacy 5-byte frame."""
    h.reset()
    now = 30000
    for x_cm in (0.0, 1.2, -1.2, 5.0, -5.0, 12.34):
        x_mm = int(x_cm * 10.0) & 0xFFFF
        f = bytearray([0xAA, 0x55, x_mm & 0xFF, (x_mm >> 8) & 0xFF])
        f.append(f[0] ^ f[1] ^ f[2] ^ f[3])
        h.feed(now, bytes(f))
        now += 37
    check(h.stats()["frames"] == 0, "legacy 5-byte frames are not accepted")
    check(h.fresh(now)[1] == 0, "legacy frames never become usable")


def main():
    exe = build()
    ns = load_main_uart_functions()
    uart = FakeUart()
    h = Harness(exe)
    try:
        test_frame_shape(ns, uart, h)
        test_units_and_sign(ns, uart, h)
        test_flags(ns, uart, h)
        test_no_lock_sends_zero(ns, uart, h)
        test_always_sends(ns, uart, h)
        test_seq_and_fps(ns, uart, h)
        test_saturation(ns, uart, h)
        test_write_failure_is_survivable(ns, uart, h)
        test_downlink_ping(ns, uart, h)
        test_downlink_set_send(ns, uart, h)
        test_downlink_set_zero(ns, uart, h)
        test_stream_drives_pid(ns, uart, h)
        test_stall_is_detected(ns, uart, h)
        test_old_format_would_fail(ns, uart, h)
    finally:
        h.close()

    print("\n%d checks, %d failures" % (CHECKS[0], len(FAILURES)))
    if FAILURES:
        for f in FAILURES:
            print("  - " + f)
        sys.exit(1)
    print("PASS")


if __name__ == "__main__":
    main()
