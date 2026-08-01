"""Byte-level cross-test: K230 encoder (Python) against MCU parser (C).

Builds tools/test/test_protocol.c together with the real drivers/camera_uart.c
and control/balance_controller.c, then drives that binary from here. Every frame
fed in is produced by k230D/protocol.py, so a disagreement in layout, byte
order, CRC or units shows up as a failing case rather than as a silent link.

Run:  python tools/test/test_protocol.py
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HPROJECT = os.path.abspath(os.path.join(HERE, "..", ".."))
# k230D sits next to Hproject, not two levels up.
K230D = os.path.abspath(os.path.join(HPROJECT, "..", "k230D"))

sys.path.insert(0, K230D)
import protocol as P  # noqa: E402

# Mirrors of control_config.h. Checked against the C build via the "config"
# command, so a change on one side cannot drift silently.
BALANCE_PERIOD_MS = 20
BALANCE_DATA_TIMEOUT_MS = 150
BALANCE_LEVEL_AB_COUNT = 300
BALANCE_TILT_AB_COUNTS_PER_DEGREE = 4096.0 / 360.0
BALANCE_FORWARD_POSITION_KP = 0.61
BALANCE_FORWARD_POSITION_KI = 0.80
BALANCE_FORWARD_VELOCITY_KD = 1.00
BALANCE_FORWARD_FIXED_TILT_DISTANCE_MM = 33.0
BALANCE_FORWARD_FIXED_TILT_ANGLE_DEG = 5.2734375
BALANCE_REVERSE_POSITION_KP = 0.50
BALANCE_REVERSE_POSITION_KI = 0.70
BALANCE_REVERSE_VELOCITY_KD = 0.60
BALANCE_REVERSE_FIXED_TILT_DISTANCE_MM = 20.0
BALANCE_REVERSE_FIXED_TILT_ANGLE_DEG = -4.39453125
BALANCE_POSITIVE_AB_OFFSET_SCALE = 2.0
BALANCE_ANGLE_KP = 8.0
BALANCE_TILT_LIMIT_AB = 250.0
BALANCE_POSITION_INTEGRAL_LIMIT = 500.0
BALANCE_OUTPUT_LIMIT = 1500

SOURCES = [
    os.path.join(HERE, "test_protocol.c"),
    os.path.join(HERE, "stubs", "stubs.c"),
    os.path.join(HPROJECT, "drivers", "camera_uart.c"),
    os.path.join(HPROJECT, "control", "balance_controller.c"),
]


def build(use_camera_velocity):
    exe = os.path.join(HERE, "test_protocol_v%d.exe" % use_camera_velocity)
    cmd = [
        "gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-O1",
        "-DBALANCE_USE_CAMERA_VELOCITY=%d" % use_camera_velocity,
        "-I" + os.path.join(HPROJECT, "drivers"),
        "-I" + os.path.join(HPROJECT, "control"),
        "-I" + os.path.join(HERE, "stubs"),
    ] + SOURCES + ["-o", exe]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit("build failed:\n" + r.stdout + r.stderr)
    return exe


def hexs(data):
    return " ".join("%02X" % b for b in data)


class Harness:
    """One live instance of the C binary, driven line by line."""

    def __init__(self, exe):
        self.p = subprocess.Popen([exe], stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE, text=True,
                                  bufsize=1)

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

    def crc(self, data):
        return int(self.cmd("crc " + hexs(data)).split()[1], 16)

    def feed(self, now_ms, data):
        self.cmd("feed %d %s" % (now_ms, hexs(data)))

    def data(self):
        f = self.fields(self.cmd("data"))
        return {k: int(f[k], 16) if k == "flags" else int(f[k]) for k in f}

    def stats(self):
        return {k: int(v) for k, v in self.fields(self.cmd("stats")).items()}

    def fresh(self, now_ms, timeout=BALANCE_DATA_TIMEOUT_MS):
        f = self.fields(self.cmd("fresh %d %d" % (now_ms, timeout)))
        return int(f["fresh"]), int(f["usable"])

    def send(self, cmd, arg):
        parts = self.cmd("send %d %d" % (cmd, arg)).split()
        return bytes(int(x, 16) for x in parts[2:])

    def ping(self, arg):
        parts = self.cmd("ping %d" % arg).split()
        return bytes(int(x, 16) for x in parts[2:])

    def tick(self, now_ms):
        f = self.fields(self.cmd("tick %d" % now_ms))
        return {k: int(v) for k, v in f.items()}

    def ab(self, count):
        self.cmd("ab %d" % count)

    def config(self):
        return self.fields(self.cmd("config"))

    def reset(self):
        self.cmd("reset")


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class PidModel:
    """Independent reimplementation of Balance_Tick's arithmetic."""

    def __init__(self, use_velocity, reverse=False):
        self.use_velocity = use_velocity
        self.reverse = reverse
        if reverse:
            self.position_kp = BALANCE_REVERSE_POSITION_KP
            self.position_ki = BALANCE_REVERSE_POSITION_KI
            self.velocity_kd = BALANCE_REVERSE_VELOCITY_KD
            self.fixed_tilt_distance_mm = \
                BALANCE_REVERSE_FIXED_TILT_DISTANCE_MM
            self.fixed_tilt_ab_offset = \
                (BALANCE_REVERSE_FIXED_TILT_ANGLE_DEG *
                 BALANCE_TILT_AB_COUNTS_PER_DEGREE)
            self.leg_start_mm = -50
        else:
            self.position_kp = BALANCE_FORWARD_POSITION_KP
            self.position_ki = BALANCE_FORWARD_POSITION_KI
            self.velocity_kd = BALANCE_FORWARD_VELOCITY_KD
            self.fixed_tilt_distance_mm = \
                BALANCE_FORWARD_FIXED_TILT_DISTANCE_MM
            self.fixed_tilt_ab_offset = \
                (BALANCE_FORWARD_FIXED_TILT_ANGLE_DEG *
                 BALANCE_TILT_AB_COUNTS_PER_DEGREE)
            self.leg_start_mm = 0
        self.integral = 0.0
        self.prev_error = None
        self.prev_derivative = 0.0
        self.prev_seq = None
        self.prev_frame_ms = None

    def lost(self):
        self.integral = 0.0
        self.prev_error = None
        self.prev_derivative = 0.0
        self.prev_seq = None
        self.prev_frame_ms = None

    def tick(self, target, pos, vel, seq, frame_ms, ab):
        error = target - pos
        fixed_tilt = False
        if self.fixed_tilt_distance_mm > 0.0:
            if not self.reverse and target == -50:
                fixed_tilt = (self.leg_start_mm - pos <
                              self.fixed_tilt_distance_mm)
            elif self.reverse and target == 50:
                fixed_tilt = (pos - self.leg_start_mm <
                              self.fixed_tilt_distance_mm)
        if fixed_tilt:
            self.integral = 0.0
        else:
            self.integral = clamp(
                self.integral + error * (BALANCE_PERIOD_MS / 1000.0),
                -BALANCE_POSITION_INTEGRAL_LIMIT,
                BALANCE_POSITION_INTEGRAL_LIMIT)
        if self.use_velocity:
            ball_velocity = float(vel)
        else:
            if self.prev_error is None:
                ball_velocity = 0.0
                self.prev_error, self.prev_seq = float(error), seq
                self.prev_frame_ms = frame_ms
            elif seq != self.prev_seq:
                dt = (frame_ms - self.prev_frame_ms) & 0xFFFFFFFF
                derivative = 0.0 if dt == 0 else (
                    (error - self.prev_error) * (1000.0 / float(dt)))
                self.prev_error, self.prev_seq = float(error), seq
                self.prev_frame_ms = frame_ms
                ball_velocity = -derivative
            else:
                ball_velocity = -self.prev_derivative
            self.prev_derivative = -ball_velocity

        if fixed_tilt:
            outer = self.fixed_tilt_ab_offset
        else:
            outer = -(self.position_kp * error
                      + self.position_ki * self.integral
                      - self.velocity_kd * ball_velocity)
            if outer > 0.0:
                outer *= BALANCE_POSITIVE_AB_OFFSET_SCALE
        outer = clamp(outer, -BALANCE_TILT_LIMIT_AB,
                      BALANCE_TILT_LIMIT_AB)
        target_ab = BALANCE_LEVEL_AB_COUNT + int(outer)
        out = BALANCE_ANGLE_KP * (target_ab - ab)
        out = clamp(out, -float(BALANCE_OUTPUT_LIMIT),
                    float(BALANCE_OUTPUT_LIMIT))
        return error, int(ball_velocity), target_ab, int(out)


FAILURES = []
CHECKS = [0]


def check(cond, what):
    CHECKS[0] += 1
    if not cond:
        FAILURES.append(what)
        print("  FAIL " + what)


# ---------------------------------------------------------------- test cases

def test_config(h):
    c = h.config()
    check(int(c["period"]) == BALANCE_PERIOD_MS, "period mirror")
    check(int(c["timeout"]) == BALANCE_DATA_TIMEOUT_MS, "timeout mirror")
    check(int(c["level"]) == BALANCE_LEVEL_AB_COUNT, "level AB mirror")
    check(abs(float(c["ab_per_deg"]) -
              BALANCE_TILT_AB_COUNTS_PER_DEGREE) < 0.000001,
          "tilt angle conversion mirror")
    check(float(c["f_kp"]) == BALANCE_FORWARD_POSITION_KP,
          "forward position kp mirror")
    check(float(c["f_ki"]) == BALANCE_FORWARD_POSITION_KI,
          "forward position ki mirror")
    check(float(c["f_v_kd"]) == BALANCE_FORWARD_VELOCITY_KD,
          "forward velocity kd mirror")
    check(float(c["f_fixed_dist"]) ==
          BALANCE_FORWARD_FIXED_TILT_DISTANCE_MM,
          "forward fixed-tilt distance mirror")
    check(float(c["f_fixed_angle"]) ==
          BALANCE_FORWARD_FIXED_TILT_ANGLE_DEG,
          "forward fixed-tilt angle mirror")
    check(float(c["r_kp"]) == BALANCE_REVERSE_POSITION_KP,
          "reverse position kp mirror")
    check(float(c["r_ki"]) == BALANCE_REVERSE_POSITION_KI,
          "reverse position ki mirror")
    check(float(c["r_v_kd"]) == BALANCE_REVERSE_VELOCITY_KD,
          "reverse velocity kd mirror")
    check(float(c["r_fixed_dist"]) ==
          BALANCE_REVERSE_FIXED_TILT_DISTANCE_MM,
          "reverse fixed-tilt distance mirror")
    check(float(c["r_fixed_angle"]) ==
          BALANCE_REVERSE_FIXED_TILT_ANGLE_DEG,
          "reverse fixed-tilt angle mirror")
    check(float(c["pos_ab_scale"]) == BALANCE_POSITIVE_AB_OFFSET_SCALE,
          "positive AB scale mirror")
    check(float(c["a_kp"]) == BALANCE_ANGLE_KP, "angle kp mirror")
    check(float(c["tilt"]) == BALANCE_TILT_LIMIT_AB, "tilt limit mirror")
    check(float(c["ilim"]) == BALANCE_POSITION_INTEGRAL_LIMIT, "ilim mirror")
    check(int(c["olim"]) == BALANCE_OUTPUT_LIMIT, "olim mirror")


def test_crc(h):
    """Python crc8 and Camera_Crc8 must agree, including on the edges."""
    vectors = [
        b"", b"\x00", b"\xFF", b"\x01\x02\x03", bytes(range(32)),
        b"\xAA\x55\x01\x07", bytes([0xFF] * 16), b"123456789",
    ]
    for v in vectors:
        check(h.crc(v) == P.crc8(v), "crc8 %s" % hexs(v))
    # Known answer, so a matching pair of wrong implementations still fails.
    check(P.crc8(b"123456789") == 0xF4, "crc8 check value 0xF4")


def test_ball_roundtrip(h):
    cases = [
        (0, 0, 0, 27), (1, P.FLAG_DETECTED | P.FLAG_VALID, 120, 350, 27),
        (2, P.FLAG_VALID, -120, -350, 30), (3, P.FLAG_PREDICTED, 1, -1, 1),
        (4, P.FLAG_DETECTED, 32767, 32767, 255),
        (5, P.FLAG_VALID, -32768, -32768, 0),
        (6, 0x0F, -1, 1, 60), (255, P.FLAG_VALID, 100, -100, 25),
    ]
    now = 1000
    for case in cases:
        seq, flags, pos, vel, fps = case if len(case) == 5 else \
            (case[0], case[1], case[2], case[3], case[3])
        frame = P.encode_ball(seq, flags, pos, vel, fps)
        check(len(frame) == P.BALL_FRAME_LEN, "ball frame length")
        h.feed(now, frame)
        d = h.data()
        check(d["pos"] == pos, "pos %d -> %d" % (pos, d["pos"]))
        check(d["vel"] == vel, "vel %d -> %d" % (vel, d["vel"]))
        check(d["seq"] == seq, "seq %d -> %d" % (seq, d["seq"]))
        check(d["fps"] == fps, "fps %d -> %d" % (fps, d["fps"]))
        check(d["ts"] == now, "timestamp stamped")
        check(d["valid"] == 1, "valid set")
        # FLAG_SATURATED is added by the encoder, so compare against the frame.
        check(d["flags"] == frame[5], "flags %02X" % frame[5])
        now += 40


def test_saturation(h):
    """Out-of-range values clamp and announce themselves."""
    for pos, want in ((40000, 32767), (-40000, -32768)):
        frame = P.encode_ball(1, P.FLAG_VALID, pos, 0, 27)
        check(frame[5] & P.FLAG_SATURATED, "encoder flags saturation")
        h.feed(2000, frame)
        check(h.data()["pos"] == want, "clamp %d -> %d" % (pos, want))
    frame = P.encode_ball(1, P.FLAG_VALID, 0, 99999, 27)
    check(frame[5] & P.FLAG_SATURATED, "velocity saturation flagged")


def test_framing_robustness(h):
    """Garbage, split frames and sync stutter must not lose a good frame."""
    before = h.stats()["frames"]
    frame = P.encode_ball(9, P.FLAG_VALID | P.FLAG_DETECTED, 42, -7, 27)

    h.feed(3000, b"\x00\x11\x22garbage")          # noise, no sync
    h.feed(3000, frame)
    check(h.stats()["frames"] == before + 1, "frame after garbage")
    check(h.data()["pos"] == 42, "pos after garbage")

    for i in range(len(frame)):                    # split at every offset
        f = P.encode_ball(10, P.FLAG_VALID, 100 + i, 0, 27)
        n = h.stats()["frames"]
        h.feed(3100 + i, f[:i])
        h.feed(3100 + i, f[i:])
        check(h.stats()["frames"] == n + 1, "split at %d" % i)
        check(h.data()["pos"] == 100 + i, "split payload at %d" % i)

    n = h.stats()["frames"]                        # AA AA 55 ... stutter
    h.feed(3300, b"\xAA" + frame)
    check(h.stats()["frames"] == n + 1, "leading 0xAA stutter survives")

    n = h.stats()["frames"]                        # sync inside noise
    h.feed(3400, b"\xAA\x01\xAA\x55" + frame)
    check(h.stats()["frames"] == n + 1, "false sync then real frame")


def test_crc_rejection(h):
    """Every single-byte corruption in the payload must be caught."""
    base = P.encode_ball(20, P.FLAG_VALID, 55, 66, 27)
    h.feed(4000, base)
    good_pos = h.data()["pos"]
    check(good_pos == 55, "baseline accepted")

    for i in range(2, P.BALL_FRAME_LEN):
        bad = bytearray(base)
        bad[i] ^= 0x01
        errs = h.stats()["crc_err"]
        frames = h.stats()["frames"]
        h.feed(4000, bytes(bad))
        s = h.stats()
        # Flipping a bit in TYPE or LEN breaks framing instead of the CRC, and
        # is rejected during resync; either way the frame must not be accepted.
        if i in (2, 3):
            check(s["frames"] == frames, "byte %d rejected by framing" % i)
        else:
            check(s["crc_err"] == errs + 1, "byte %d caught by CRC" % i)
            check(s["frames"] == frames, "byte %d not accepted" % i)

    # The old XOR checksum missed this exactly: same bit flipped in both
    # position bytes cancelled out. CRC8 must catch it.
    bad = bytearray(base)
    bad[6] ^= 0x01
    bad[7] ^= 0x01
    errs = h.stats()["crc_err"]
    h.feed(4000, bytes(bad))
    check(h.stats()["crc_err"] == errs + 1, "paired bit flip caught")


def test_seq_gaps(h):
    # From a clean parser, so the first frame has no predecessor to be a gap
    # against. Every later baseline is re-read rather than assumed: a setup feed
    # that jumps SEQ is itself a gap and would otherwise be counted twice.
    h.reset()
    h.feed(5000, P.encode_ball(100, P.FLAG_VALID, 0, 0, 27))
    check(h.stats()["gaps"] == 0, "no gap on first frame after reset")
    h.feed(5040, P.encode_ball(101, P.FLAG_VALID, 0, 0, 27))
    check(h.stats()["gaps"] == 0, "no gap on consecutive")
    h.feed(5080, P.encode_ball(105, P.FLAG_VALID, 0, 0, 27))
    check(h.stats()["gaps"] == 3, "three dropped counted")

    h.feed(5120, P.encode_ball(255, P.FLAG_VALID, 0, 0, 27))
    gaps = h.stats()["gaps"]                       # 105 -> 255 is itself a gap
    h.feed(5160, P.encode_ball(0, P.FLAG_VALID, 0, 0, 27))
    check(h.stats()["gaps"] == gaps, "wrap 255->0 is not a gap")
    h.feed(5200, P.encode_ball(2, P.FLAG_VALID, 0, 0, 27))
    check(h.stats()["gaps"] == gaps + 1, "gap across wrap counted")


def test_unknown_and_pong(h):
    frames = h.stats()["frames"]
    h.feed(6000, bytes([P.SYNC0, P.SYNC1, 0x7E, 0x04, 0, 0, 0, 0]))
    check(h.stats()["frames"] == frames, "unknown type not accepted")
    h.feed(6000, P.encode_ball(1, P.FLAG_VALID, 11, 0, 27))
    check(h.stats()["frames"] == frames + 1, "recovers after unknown type")

    bad_len = bytearray(P.encode_ball(2, P.FLAG_VALID, 0, 0, 27))
    bad_len[3] = 0x05
    frames = h.stats()["frames"]
    h.feed(6100, bytes(bad_len))
    check(h.stats()["frames"] == frames, "wrong LEN rejected")

    pongs = h.stats()["pongs"]
    frames = h.stats()["frames"]
    for arg in (0, 1, 0x5A, 0xFF):
        h.feed(6200, P.encode_pong(arg))
    check(h.stats()["pongs"] == pongs + 4, "pongs counted")
    check(h.stats()["frames"] == frames, "pong is not a ball frame")


def test_uplink_decoder(h):
    """The PC-side uplink decoder must mirror the C parser, byte for byte.

    The sniffer decides what it accepts with UplinkReader; the MCU decides with
    camera_uart.c. If those two disagree, the sniffer lies about what the MCU is
    seeing - which is worse than having no sniffer.
    """
    # decode_ball must undo encode_ball, which test_ball_roundtrip already
    # checked against the C parser, so agreement chains through.
    cases = [
        (0, 0, 0, 0, 27), (1, P.FLAG_DETECTED | P.FLAG_VALID, 120, 350, 27),
        (2, P.FLAG_VALID, -120, -350, 30), (3, P.FLAG_PREDICTED, 1, -1, 1),
        (4, P.FLAG_DETECTED, 32767, 32767, 255),
        (5, P.FLAG_VALID, -32768, -32768, 0),
        (255, 0x0F, -1, 1, 60),
    ]
    for seq, flags, pos, vel, fps in cases:
        frame = P.encode_ball(seq, flags, pos, vel, fps)
        got = P.decode_ball(frame)
        check(got is not None, "decode_ball accepts a valid frame")
        check(got["pos_mm"] == pos, "decode pos %d" % pos)
        check(got["vel_mm_s"] == vel, "decode vel %d" % vel)
        check(got["seq"] == seq, "decode seq %d" % seq)
        check(got["fps"] == fps, "decode fps %d" % fps)
        check(got["flags"] == frame[5], "decode flags %02X" % frame[5])

    check(P.decode_pong(P.encode_pong(0x5A)) == 0x5A, "decode_pong round-trips")

    bad = bytearray(P.encode_ball(1, P.FLAG_VALID, 10, 0, 27))
    bad[11] ^= 0xFF
    check(P.decode_ball(bytes(bad)) is None, "decode_ball rejects bad CRC")
    check(P.decode_ball(b"\xAA\x55") is None, "decode_ball rejects a short frame")

    # Framing agreement on identical streams, garbage included.
    good = P.encode_ball(7, P.FLAG_VALID | P.FLAG_DETECTED, 42, -7, 27)
    streams = [
        good,
        b"\x00\x11\x22garbage" + good,
        b"\xAA" + good,                                  # sync stutter
        b"\xAA\x01\xAA\x55" + good,                      # false sync
        bytes([P.SYNC0, P.SYNC1, 0x7E, 0x04, 0, 0]) + good,   # unknown type
        P.encode_pong(0x11) + good + P.encode_pong(0x22),
        good + good + good,
        bytes(range(256)) + good,                        # every byte value
    ]
    for i, stream in enumerate(streams):
        h.reset()
        reader = P.UplinkReader()
        h.feed(1000, stream)
        items = reader.feed(stream)
        c = h.stats()
        balls = len([x for x in items if x[0] == "ball"])
        pongs = len([x for x in items if x[0] == "pong"])
        check(balls == c["frames"],
              "stream %d: balls %d vs C %d" % (i, balls, c["frames"]))
        check(pongs == c["pongs"],
              "stream %d: pongs %d vs C %d" % (i, pongs, c["pongs"]))
        check(reader.crc_errors == c["crc_err"],
              "stream %d: crc_err %d vs C %d"
              % (i, reader.crc_errors, c["crc_err"]))
        check(reader.resyncs == c["resync"],
              "stream %d: resyncs %d vs C %d"
              % (i, reader.resyncs, c["resync"]))

    # Byte-at-a-time delivery must not change the outcome.
    h.reset()
    reader = P.UplinkReader()
    stream = b"\x00\xAA\xAA\x55" + good + P.encode_pong(3)
    got = []
    for byte in stream:
        got.extend(reader.feed(bytes([byte])))
    h.feed(2000, stream)
    c = h.stats()
    check(len([x for x in got if x[0] == "ball"]) == c["frames"],
          "byte-at-a-time balls match C")
    check(len([x for x in got if x[0] == "pong"]) == c["pongs"],
          "byte-at-a-time pongs match C")

    check(P.flags_str(P.FLAG_DETECTED | P.FLAG_VALID) == "D-V-", "flags_str DPVS")
    check(P.flags_str(0) == "----", "flags_str empty")
    check(P.flags_str(0x0F) == "DPVS", "flags_str all")


def test_monitor_agrees_with_mcu(h):
    """LinkMonitor's health numbers must match what the MCU actually counts.

    The sniffer's verdict is only worth reading if its gap and frame counts are
    the MCU's. UplinkReader has no notion of SEQ, so this is the only place the
    gap arithmetic gets checked against camera_uart.c.
    """
    sys.path.insert(0, os.path.join(HPROJECT, "tools", "serial"))
    from link_monitor import LinkMonitor  # noqa: E402

    good = P.encode_ball(7, P.FLAG_VALID | P.FLAG_DETECTED, 42, -7, 27)
    bad_crc = bytearray(P.encode_ball(8, P.FLAG_VALID, 0, 0, 27))
    bad_crc[11] ^= 0xFF

    def ball(seq, flags=P.FLAG_VALID):
        return P.encode_ball(seq, flags, 0, 0, 27)

    streams = [
        b"".join(ball(s) for s in range(5)),               # clean run
        b"".join(ball(s) for s in (0, 1, 5, 6)),           # three lost
        b"".join(ball(s) for s in (250, 253, 255, 0, 1, 4)),   # gaps over wrap
        b"".join(ball(s) for s in (10, 10, 10)),           # repeats, not gaps
        b"".join(ball(s) for s in (5, 4, 3)),              # SEQ going backwards
        b"garbage" + ball(0) + bytes(bad_crc) + ball(2),   # loss looks like a gap
        P.encode_pong(0x11) + ball(0) + P.encode_pong(0x22) + ball(9),
        bytes(range(256)) + good,
    ]
    for i, stream in enumerate(streams):
        h.reset()
        mon = LinkMonitor()
        h.feed(1000, stream)
        mon.feed(stream, 1000)
        c = h.stats()
        check(mon.balls == c["frames"],
              "monitor %d: balls %d vs C %d" % (i, mon.balls, c["frames"]))
        check(mon.pongs == c["pongs"],
              "monitor %d: pongs %d vs C %d" % (i, mon.pongs, c["pongs"]))
        check(mon.gaps == c["gaps"],
              "monitor %d: gaps %d vs C %d" % (i, mon.gaps, c["gaps"]))
        check(mon.crc_errors == c["crc_err"],
              "monitor %d: crc_err %d vs C %d" % (i, mon.crc_errors,
                                                  c["crc_err"]))

    # usable_frames must follow FLAG_VALID the way Camera_IsBallUsable does.
    h.reset()
    mon = LinkMonitor()
    stream = ball(0) + ball(1, 0) + ball(2, P.FLAG_DETECTED) \
        + ball(3, P.FLAG_VALID | P.FLAG_PREDICTED)
    mon.feed(stream, 1000)
    check(mon.balls == 4 and mon.usable_frames == 2, "usable follows FLAG_VALID")
    for seq, want in ((0, 1), (1, 0), (2, 0), (3, 1)):
        h.reset()
        h.feed(1000, ball(seq) if want else ball(seq, 0))
        check(h.fresh(1000)[1] == want, "C usable agrees on seq=%d" % seq)


def test_downlink(h):
    """C encoder output must decode with the K230's decoder, and match it."""
    for cmd, arg in ((P.CMD_PING, 0), (P.CMD_SET_ZERO, 0),
                     (P.CMD_SET_SEND, 1), (P.CMD_SET_SEND, 0),
                     (0xFF, 0xFF), (0x42, 0x7F)):
        got = h.send(cmd, arg)
        want = P.encode_cmd(cmd, arg)
        check(got == want,
              "cmd %02X/%02X: %s vs %s" % (cmd, arg, hexs(got), hexs(want)))
        check(P.decode_cmd(got) == (cmd, arg), "decode_cmd %02X" % cmd)

    got = h.ping(0x5A)
    check(got == P.encode_cmd(P.CMD_PING, 0x5A), "ping frame")
    check(P.decode_cmd(got) == (P.CMD_PING, 0x5A), "ping decodes")

    bad = bytearray(P.encode_cmd(P.CMD_SET_ZERO, 0))
    bad[6] ^= 0xFF
    check(P.decode_cmd(bytes(bad)) is None, "decoder rejects bad CRC")


def test_freshness(h):
    """Fresh is about the link; usable is about the position."""
    h.feed(10000, P.encode_ball(1, P.FLAG_VALID | P.FLAG_DETECTED, 30, 0, 27))
    check(h.fresh(10000) == (1, 1), "fresh and usable at t=0")
    check(h.fresh(10000 + BALANCE_DATA_TIMEOUT_MS - 1) == (1, 1),
          "still fresh just inside timeout")
    check(h.fresh(10000 + BALANCE_DATA_TIMEOUT_MS) == (0, 0),
          "not fresh at the timeout")

    # Ball out of view: frames still arrive, so the link is fresh but the
    # position is not usable. This is the case the old code could not tell from
    # a dead link.
    h.feed(11000, P.encode_ball(2, 0, 30, 0, 27))
    check(h.fresh(11000) == (1, 0), "fresh but not usable with no ruler lock")

    # Extrapolated but ruler locked is still usable.
    h.feed(12000, P.encode_ball(3, P.FLAG_VALID | P.FLAG_PREDICTED, 31, 5, 27))
    check(h.fresh(12000) == (1, 1), "predicted position is usable")


def test_balance_gate(h):
    h.cmd("disable")
    h.cmd("target 0")
    t = h.tick(20000)
    check(t["track"] == 0 and t["out"] == 0, "disabled does not drive")
    check(t["step"] == 0, "disabled leaves stepper at zero")

    h.cmd("enable")
    check(h.tick(20000)["en"] == 1, "enable powers the stepper")

    # Enabled but the camera has nothing usable -> no drive, no stale error use.
    h.feed(20000, P.encode_ball(1, 0, 100, 0, 27))   # no FLAG_VALID
    t = h.tick(20000)
    check(t["track"] == 0, "no ruler lock is not tracking")
    check(t["out"] == 0 and t["step"] == 0, "no drive without lock")

    # Stale link -> not tracking.
    h.feed(21000, P.encode_ball(2, P.FLAG_VALID, 100, 0, 27))
    t = h.tick(21000 + BALANCE_DATA_TIMEOUT_MS + 1)
    check(t["track"] == 0, "stale link is not tracking")
    check(t["step"] == 0, "stale link stops the stepper")


def test_balance_pid(h, use_velocity):
    """Both cascade loops must match an independent arithmetic model."""
    h.cmd("disable")
    h.cmd("enable")
    h.cmd("target 0")
    h.ab(BALANCE_LEVEL_AB_COUNT)
    model = PidModel(use_velocity)

    seq, now = 0, 30000
    flags = P.FLAG_VALID | P.FLAG_DETECTED
    traj = [(0, 0), (10, 50), (25, 120), (40, 150), (30, -80),
            (5, -200), (-20, -150), (-45, -40), (-30, 60), (0, 10)]
    for pos, vel in traj:
        seq = (seq + 1) & 0xFF
        now += 40
        h.feed(now, P.encode_ball(seq, flags, pos, vel, 25))
        t = h.tick(now)
        want_err, want_vel, want_tab, want_out = model.tick(
            0, pos, vel, seq, now, BALANCE_LEVEL_AB_COUNT)
        check(t["track"] == 1, "tracking at pos=%d" % pos)
        check(t["err"] == want_err,
              "error pos=%d: %d vs %d" % (pos, t["err"], want_err))
        check(abs(t["vel"] - want_vel) <= 1,
              "velocity pos=%d: %d vs %d" % (pos, t["vel"], want_vel))
        check(abs(t["tab"] - want_tab) <= 1,
              "target AB pos=%d: %d vs %d" % (pos, t["tab"], want_tab))
        # float on the C side vs double here: allow one unit of rounding.
        check(abs(t["out"] - want_out) <= 1,
              "output pos=%d vel=%d: %d vs %d" % (pos, vel, t["out"], want_out))
        check(t["step"] == t["out"], "stepper follows output")

    # Q3 forward starts with a literal fixed AB offset. At exactly the
    # configured distance it hands control to PID without retained integral.
    h.cmd("disable")
    h.cmd("enable")
    h.cmd("q3dir 0")
    h.cmd("target -50")
    h.ab(BALANCE_LEVEL_AB_COUNT)
    model = PidModel(use_velocity)
    forward_inside_pos = -int(BALANCE_FORWARD_FIXED_TILT_DISTANCE_MM) + 1
    forward_boundary_pos = -int(BALANCE_FORWARD_FIXED_TILT_DISTANCE_MM)
    seq, now = (seq + 1) & 0xFF, now + 40
    h.feed(now, P.encode_ball(seq, flags, forward_inside_pos, 0, 25))
    t = h.tick(now)
    want_err, want_vel, want_tab, want_out = model.tick(
        -50, forward_inside_pos, 0, seq, now, BALANCE_LEVEL_AB_COUNT)
    check(t["tab"] == BALANCE_LEVEL_AB_COUNT +
          int(BALANCE_FORWARD_FIXED_TILT_ANGLE_DEG *
              BALANCE_TILT_AB_COUNTS_PER_DEGREE),
          "forward fixed-tilt segment")
    check(t["tab"] == want_tab and t["out"] == want_out,
          "forward fixed-tilt model")

    seq, now = (seq + 1) & 0xFF, now + 40
    h.feed(now, P.encode_ball(seq, flags, forward_boundary_pos, 0, 25))
    t = h.tick(now)
    want_err, want_vel, want_tab, want_out = model.tick(
        -50, forward_boundary_pos, 0, seq, now, BALANCE_LEVEL_AB_COUNT)
    check(t["tab"] == want_tab and t["out"] == want_out,
          "forward fixed-tilt boundary enters PID")
    check(t["tab"] != BALANCE_LEVEL_AB_COUNT +
          int(BALANCE_FORWARD_FIXED_TILT_ANGLE_DEG *
              BALANCE_TILT_AB_COUNTS_PER_DEGREE),
          "forward fixed-tilt segment ends at configured distance")

    # Capture the actual position at the direction change, then remain one
    # millimetre inside the configured reverse fixed-angle segment.
    h.cmd("disable")
    h.cmd("enable")
    h.cmd("q3dir 0")
    h.cmd("target -50")
    h.ab(BALANCE_LEVEL_AB_COUNT)
    seq, now = (seq + 1) & 0xFF, now + 40
    h.feed(now, P.encode_ball(seq, flags, -43, 0, 25))
    h.tick(now)
    h.cmd("q3dir 1")
    h.cmd("target 50")
    reverse_model = PidModel(use_velocity, reverse=True)
    reverse_model.leg_start_mm = -43
    reverse_inside_pos = (-43 +
                          int(BALANCE_REVERSE_FIXED_TILT_DISTANCE_MM) - 1)
    seq, now = (seq + 1) & 0xFF, now + 40
    h.feed(now, P.encode_ball(seq, flags, reverse_inside_pos, 0, 25))
    t = h.tick(now)
    want_err, want_vel, want_tab, want_out = reverse_model.tick(
        50, reverse_inside_pos, 0, seq, now, BALANCE_LEVEL_AB_COUNT)
    check(t["tab"] == BALANCE_LEVEL_AB_COUNT +
          int(BALANCE_REVERSE_FIXED_TILT_ANGLE_DEG *
              BALANCE_TILT_AB_COUNTS_PER_DEGREE),
          "reverse fixed tilt uses actual leg start")
    check(t["tab"] == want_tab and t["out"] == want_out,
          "reverse fixed-tilt model")

    # After the fixed-angle segment, forward uses one PID group all the way
    # to the -50 mm target.
    h.cmd("disable")
    h.cmd("enable")
    h.cmd("q3dir 0")
    h.cmd("target -50")
    h.ab(BALANCE_LEVEL_AB_COUNT)
    model = PidModel(use_velocity)
    first_pid_pos = -int(BALANCE_FORWARD_FIXED_TILT_DISTANCE_MM) - 1
    for pos, vel in [(first_pid_pos, 0), (first_pid_pos - 4, -100)]:
        seq, now = (seq + 1) & 0xFF, now + 40
        h.feed(now, P.encode_ball(seq, flags, pos, vel, 25))
        t = h.tick(now)
        want_err, want_vel, want_tab, want_out = model.tick(
            -50, pos, vel, seq, now, BALANCE_LEVEL_AB_COUNT)
    check(abs(t["tab"] - want_tab) <= 1,
          "forward segment uses single PID")
    check(abs(t["out"] - want_out) <= 1,
          "forward single-PID output")

    # The reverse leg must load the separate PID group for +50 mm.
    h.cmd("disable")
    h.cmd("enable")
    h.cmd("q3dir 1")
    h.cmd("target 50")
    h.ab(BALANCE_LEVEL_AB_COUNT)
    model = PidModel(use_velocity, reverse=True)
    for pos, vel in [(21, 0), (25, 100)]:
        seq, now = (seq + 1) & 0xFF, now + 40
        h.feed(now, P.encode_ball(seq, flags, pos, vel, 25))
        t = h.tick(now)
        want_err, want_vel, want_tab, want_out = model.tick(
            50, pos, vel, seq, now, BALANCE_LEVEL_AB_COUNT)
    check(abs(t["tab"] - want_tab) <= 1,
          "reverse target loads reverse PID group")
    check(abs(t["out"] - want_out) <= 1,
          "reverse PID output")

    # A ball on/moving toward +axis needs +STEP to raise that end and brake it.
    h.cmd("disable")
    h.cmd("enable")
    h.cmd("q3dir 0")
    h.cmd("target 0")
    h.ab(BALANCE_LEVEL_AB_COUNT)
    seq, now = (seq + 1) & 0xFF, now + 40
    h.feed(now, P.encode_ball(seq, flags, 50, 0, 25))
    t = h.tick(now)
    check(t["err"] == -50 and t["out"] > 0, "positive position drives positive")

    h.cmd("disable")
    h.cmd("enable")
    h.cmd("target 0")
    h.ab(BALANCE_LEVEL_AB_COUNT)
    seq, now = (seq + 1) & 0xFF, now + 40
    h.feed(now, P.encode_ball(seq, flags, -50, 0, 25))
    t = h.tick(now)
    check(t["err"] == 50 and t["out"] < 0, "negative position drives negative")


def test_beam_inner_loop(h):
    """The AB inner loop must reach and retain its requested beam angle."""
    h.cmd("disable")
    h.cmd("enable")
    h.cmd("target 0")
    flags = P.FLAG_VALID | P.FLAG_DETECTED

    h.ab(BALANCE_LEVEL_AB_COUNT + 20)
    h.feed(39000, P.encode_ball(1, flags, 0, 0, 25))
    t = h.tick(39000)
    check(t["tab"] == BALANCE_LEVEL_AB_COUNT, "centered ball requests level AB")
    check(t["out"] == -160, "AB above target drives negative")

    h.ab(BALANCE_LEVEL_AB_COUNT - 20)
    h.feed(39040, P.encode_ball(2, flags, 0, 0, 25))
    t = h.tick(39040)
    check(t["out"] == 160, "AB below target drives positive")

    # If vision is lost while tilted, the outer loop requests horizontal and
    # the AB loop remains in charge instead of holding the stale tilt.
    h.ab(BALANCE_LEVEL_AB_COUNT + 20)
    h.feed(39080, P.encode_ball(3, 0, 0, 0, 25))
    t = h.tick(39080)
    check(t["track"] == 0 and t["tab"] == BALANCE_LEVEL_AB_COUNT,
          "vision loss requests horizontal")
    check(t["out"] == -160, "vision loss still runs the AB inner loop")
    h.ab(BALANCE_LEVEL_AB_COUNT)


def test_output_clamp(h):
    h.cmd("disable")
    h.cmd("enable")
    h.cmd("target 0")
    h.ab(BALANCE_LEVEL_AB_COUNT)
    seq, now = 200, 40000
    flags = P.FLAG_VALID | P.FLAG_DETECTED
    for _ in range(30):
        seq = (seq + 1) & 0xFF
        now += 40
        h.feed(now, P.encode_ball(seq, flags, -3000, 0, 25))
        t = h.tick(now)
    check(t["out"] == -BALANCE_OUTPUT_LIMIT, "output clamps at the limit")
    check(t["step"] == -BALANCE_OUTPUT_LIMIT, "clamped value reaches stepper")


def test_outer_state_cleared_on_loss(h):
    """A dropout must clear the outer-loop history before reacquisition."""
    h.cmd("disable")
    h.cmd("enable")
    h.cmd("target 0")
    h.ab(BALANCE_LEVEL_AB_COUNT)
    seq, now = 0, 50000
    flags = P.FLAG_VALID | P.FLAG_DETECTED
    seq = (seq + 1) & 0xFF
    now += 40
    h.feed(now, P.encode_ball(seq, flags, -20, -200, 25))
    h.tick(now)
    seq = (seq + 1) & 0xFF
    now += 40
    h.feed(now, P.encode_ball(seq, flags, -100, -200, 25))
    charged = h.tick(now)["out"]

    now += BALANCE_DATA_TIMEOUT_MS + 10            # lose the link
    check(h.tick(now)["track"] == 0, "dropout detected")

    seq = (seq + 1) & 0xFF                          # reacquire at the same spot
    now += 40
    h.feed(now, P.encode_ball(seq, flags, -100, 0, 25))
    t = h.tick(now)
    first_error = 100
    first_integral = first_error * (BALANCE_PERIOD_MS / 1000.0)
    first_outer = -(BALANCE_FORWARD_POSITION_KP * first_error
                    + BALANCE_FORWARD_POSITION_KI * first_integral)
    if first_outer > 0.0:
        first_outer *= BALANCE_POSITIVE_AB_OFFSET_SCALE
    first_outer = clamp(first_outer, -BALANCE_TILT_LIMIT_AB,
                        BALANCE_TILT_LIMIT_AB)
    first_target_ab = BALANCE_LEVEL_AB_COUNT + int(first_outer)
    first = int(BALANCE_ANGLE_KP *
                (first_target_ab - BALANCE_LEVEL_AB_COUNT))
    check(t["track"] == 1, "reacquired")
    check(abs(t["out"] - first) <= 1,
          "outer state cleared: %d vs %d (was %d)"
          % (t["out"], first, charged))
    check(abs(t["out"]) < abs(charged), "post-dropout output below charged output")


def run(use_velocity):
    print("=== BALANCE_USE_CAMERA_VELOCITY=%d ===" % use_velocity)
    exe = build(use_velocity)
    h = Harness(exe)
    try:
        test_config(h)
        test_crc(h)
        test_ball_roundtrip(h)
        test_saturation(h)
        test_framing_robustness(h)
        test_crc_rejection(h)
        test_seq_gaps(h)
        test_unknown_and_pong(h)
        test_uplink_decoder(h)
        test_monitor_agrees_with_mcu(h)
        test_downlink(h)
        test_freshness(h)
        test_balance_gate(h)
        test_balance_pid(h, use_velocity)
        test_beam_inner_loop(h)
        test_output_clamp(h)
        test_outer_state_cleared_on_loss(h)
        print("  stats: " + str(h.stats()))
    finally:
        h.close()


if __name__ == "__main__":
    for uv in (1, 0):
        run(uv)
    print("\n%d checks, %d failures" % (CHECKS[0], len(FAILURES)))
    if FAILURES:
        for f in FAILURES:
            print("  - " + f)
        sys.exit(1)
    print("PASS")
