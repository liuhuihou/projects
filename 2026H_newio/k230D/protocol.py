"""K230D <-> MSPM0 vision link: the wire format, defined once.

This is the authoritative definition. Hproject/drivers/camera_uart.h mirrors it
in C, and Hproject/tools/test/test_protocol.py feeds frames built here through
the real C parser so the two cannot drift apart silently.

Frames
  MSG_BALL  K230 -> MCU, 12 bytes, one per camera iteration
    AA 55 01 07 SEQ FLAGS pos_l pos_h vel_l vel_h FPS CRC8
    pos is int16 little-endian millimetres from the origin, vel is int16 mm/s,
    CRC8 covers bytes 2..10 (type, length, payload - not the sync pair).

  MSG_PONG  K230 -> MCU, 6 bytes, reply to CMD_PING
    AA 55 02 01 ARG CRC8      ARG echoed from the command, CRC8 over bytes 2..4

  MSG_CMD   MCU -> K230, 7 bytes
    AA 55 81 02 CMD ARG CRC8  CRC8 over bytes 2..5

A MSG_BALL frame goes out on every iteration, ball or no ball. That is what
makes "the link is dead" and "the ball is out of sight" distinguishable on the
MCU: frames keep arriving with FLAG_DETECTED clear in the second case. Skipping
the send when there is nothing to report breaks the MCU's freshness timeout.

CRC8 is poly 0x07, init 0x00, no reflection, no final xor - the standard
CRC-8/ATM, whose check value over b"123456789" is 0xF4. It replaced an XOR
checksum that was nearly useless here: 0xAA ^ 0x55 is constant, so the XOR only
protected pos_l ^ pos_h, and the same bit flipping in both position bytes
cancelled out undetected.

Written for MicroPython, so no typing, dataclasses, or struct.pack tricks.
"""

SYNC0 = 0xAA
SYNC1 = 0x55

MSG_BALL = 0x01
MSG_PONG = 0x02
MSG_CMD = 0x81

BALL_PAYLOAD_LEN = 7
BALL_FRAME_LEN = 12
PONG_PAYLOAD_LEN = 1
PONG_FRAME_LEN = 6
CMD_PAYLOAD_LEN = 2
CMD_FRAME_LEN = 7

# FLAGS bits in a MSG_BALL frame.
FLAG_DETECTED = 0x01   # real detection this frame, not extrapolated
FLAG_PREDICTED = 0x02  # motion model contributed to pos
FLAG_VALID = 0x04      # ruler locked, so pos is genuinely in millimetres
FLAG_SATURATED = 0x08  # pos or vel hit the int16 clamp

# Commands the MCU can send.
CMD_PING = 0x01
CMD_SET_ZERO = 0x02  # ARG ignored: re-zero at the current position
CMD_SET_SEND = 0x03  # ARG 0 stop the stream, 1 resume

INT16_MIN = -32768
INT16_MAX = 32767


def crc8(data):
    """CRC-8/ATM over `data`. Mirrors Camera_Crc8() in camera_uart.c."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _clamp_int16(value):
    """Clamp to int16, reporting whether the clamp actually bit."""
    value = int(value)
    if value < INT16_MIN:
        return INT16_MIN, True
    if value > INT16_MAX:
        return INT16_MAX, True
    return value, False


def encode_ball(seq, flags, pos_mm, vel_mm_s, fps):
    """Build one MSG_BALL frame.

    pos_mm and vel_mm_s are clamped into int16 rather than allowed to wrap: a
    wrap would put the ball at the far end of the tube and drive the balance PID
    hard the wrong way. FLAG_SATURATED is added when that happens so the MCU can
    tell a clamped reading from a real one at the limit.
    """
    pos, pos_sat = _clamp_int16(pos_mm)
    vel, vel_sat = _clamp_int16(vel_mm_s)

    flags = int(flags) & 0xFF
    if pos_sat or vel_sat:
        flags |= FLAG_SATURATED

    payload = bytearray()
    payload.append(MSG_BALL)
    payload.append(BALL_PAYLOAD_LEN)
    payload.append(int(seq) & 0xFF)
    payload.append(flags)
    payload.append(pos & 0xFF)
    payload.append((pos >> 8) & 0xFF)
    payload.append(vel & 0xFF)
    payload.append((vel >> 8) & 0xFF)
    payload.append(_clamp_fps(fps))

    frame = bytearray()
    frame.append(SYNC0)
    frame.append(SYNC1)
    frame.extend(payload)
    frame.append(crc8(payload))
    return bytes(frame)


def _clamp_fps(fps):
    """FPS is one unsigned byte; a float frame rate rounds toward zero."""
    try:
        value = int(fps)
    except (TypeError, ValueError):
        return 0
    if value < 0:
        return 0
    if value > 255:
        return 255
    return value


def encode_pong(arg):
    """Build the MSG_PONG reply to CMD_PING, echoing `arg`."""
    payload = bytearray()
    payload.append(MSG_PONG)
    payload.append(PONG_PAYLOAD_LEN)
    payload.append(int(arg) & 0xFF)

    frame = bytearray()
    frame.append(SYNC0)
    frame.append(SYNC1)
    frame.extend(payload)
    frame.append(crc8(payload))
    return bytes(frame)


def encode_cmd(cmd, arg):
    """Build a MSG_CMD frame. Only the MCU sends these; used by the tests."""
    payload = bytearray()
    payload.append(MSG_CMD)
    payload.append(CMD_PAYLOAD_LEN)
    payload.append(int(cmd) & 0xFF)
    payload.append(int(arg) & 0xFF)

    frame = bytearray()
    frame.append(SYNC0)
    frame.append(SYNC1)
    frame.extend(payload)
    frame.append(crc8(payload))
    return bytes(frame)


def decode_cmd(frame):
    """Decode one MSG_CMD frame, returning (cmd, arg) or None if malformed.

    Strict: the frame must be exactly CMD_FRAME_LEN and pass CRC. Finding a
    command inside a longer buffer is CommandReader's job, not this function's.
    """
    if frame is None or len(frame) != CMD_FRAME_LEN:
        return None
    if frame[0] != SYNC0 or frame[1] != SYNC1:
        return None
    if frame[2] != MSG_CMD or frame[3] != CMD_PAYLOAD_LEN:
        return None
    if crc8(frame[2:6]) != frame[6]:
        return None
    return frame[4], frame[5]


def decode_ball(frame):
    """Decode one MSG_BALL frame into a dict, or None if malformed.

    Only the PC-side tools need this - the K230 sends ball frames and the MCU
    parses them in C. It lives here anyway so the uplink layout is defined in
    exactly one place instead of being re-implemented in the sniffer.
    """
    if frame is None or len(frame) != BALL_FRAME_LEN:
        return None
    if frame[0] != SYNC0 or frame[1] != SYNC1:
        return None
    if frame[2] != MSG_BALL or frame[3] != BALL_PAYLOAD_LEN:
        return None
    if crc8(frame[2:11]) != frame[11]:
        return None

    pos = frame[6] | (frame[7] << 8)
    vel = frame[8] | (frame[9] << 8)
    if pos >= 0x8000:
        pos -= 0x10000
    if vel >= 0x8000:
        vel -= 0x10000

    return {
        "seq": frame[4],
        "flags": frame[5],
        "pos_mm": pos,
        "vel_mm_s": vel,
        "fps": frame[10],
    }


def decode_pong(frame):
    """Decode one MSG_PONG frame, returning the echoed ARG or None."""
    if frame is None or len(frame) != PONG_FRAME_LEN:
        return None
    if frame[0] != SYNC0 or frame[1] != SYNC1:
        return None
    if frame[2] != MSG_PONG or frame[3] != PONG_PAYLOAD_LEN:
        return None
    if crc8(frame[2:5]) != frame[5]:
        return None
    return frame[4]


def flags_str(flags):
    """Render FLAGS as a fixed-width DPVS string for log alignment."""
    return "".join((
        "D" if flags & FLAG_DETECTED else "-",
        "P" if flags & FLAG_PREDICTED else "-",
        "V" if flags & FLAG_VALID else "-",
        "S" if flags & FLAG_SATURATED else "-",
    ))


class UplinkReader:
    """Byte-at-a-time parser for the K230 -> MCU direction, for PC tools.

    Deliberately mirrors the C parser in camera_uart.c, including its resync
    behaviour, so what the sniffer accepts is what the MCU accepts. Counters are
    free-running and match the names in CameraStats.
    """

    def __init__(self):
        self._buf = bytearray()
        self._expect = 0
        self.rx_bytes = 0
        self.crc_errors = 0
        self.resyncs = 0

    def reset(self):
        self._buf = bytearray()
        self._expect = 0

    def feed(self, data):
        """Push bytes in, get a list of ("ball", dict) / ("pong", arg) out."""
        out = []
        if not data:
            return out
        for byte in data:
            item = self._feed_byte(byte)
            if item is not None:
                out.append(item)
        return out

    def _restart_on(self, byte):
        self.resyncs += 1
        self._expect = 0
        if byte == SYNC0:
            self._buf = bytearray([SYNC0])
        else:
            self._buf = bytearray()

    def _feed_byte(self, byte):
        self.rx_bytes += 1
        n = len(self._buf)

        if n == 0:
            if byte == SYNC0:
                self._buf.append(byte)
            else:
                self.resyncs += 1
            return None

        if n == 1:
            if byte == SYNC1:
                self._buf.append(byte)
            elif byte == SYNC0:
                self.resyncs += 1
            else:
                self._buf = bytearray()
                self.resyncs += 1
            return None

        if n == 2:
            if byte == MSG_BALL:
                self._expect = BALL_FRAME_LEN
            elif byte == MSG_PONG:
                self._expect = PONG_FRAME_LEN
            else:
                self._restart_on(byte)
                return None
            self._buf.append(byte)
            return None

        if n == 3:
            want = (BALL_PAYLOAD_LEN if self._buf[2] == MSG_BALL
                    else PONG_PAYLOAD_LEN)
            if byte != want:
                self._restart_on(byte)
                return None
            self._buf.append(byte)
            return None

        self._buf.append(byte)
        if len(self._buf) < self._expect:
            return None

        frame = bytes(self._buf)
        is_ball = frame[2] == MSG_BALL
        self._buf = bytearray()
        self._expect = 0

        if is_ball:
            decoded = decode_ball(frame)
            if decoded is None:
                self.crc_errors += 1
                return None
            return ("ball", decoded)

        arg = decode_pong(frame)
        if arg is None:
            self.crc_errors += 1
            return None
        return ("pong", arg)


class CommandReader:
    """Byte-at-a-time MSG_CMD parser for the K230 end of the link.

    Mirrors the MCU's receive state machine, including its resync behaviour: a
    byte that breaks the frame under construction is reconsidered as a possible
    new SYNC0 rather than discarded, so noise ending in 0xAA does not swallow
    the first byte of the command that follows it.
    """

    def __init__(self):
        self._buf = bytearray()
        self.crc_errors = 0
        self.resyncs = 0

    def reset(self):
        self._buf = bytearray()

    def feed(self, data):
        """Push received bytes in, get a list of (cmd, arg) pairs out."""
        out = []
        if not data:
            return out
        for byte in data:
            result = self._feed_byte(byte)
            if result is not None:
                out.append(result)
        return out

    def _restart_on(self, byte):
        self.resyncs += 1
        if byte == SYNC0:
            self._buf = bytearray([SYNC0])
        else:
            self._buf = bytearray()

    def _feed_byte(self, byte):
        n = len(self._buf)

        if n == 0:
            if byte == SYNC0:
                self._buf.append(byte)
            else:
                self.resyncs += 1
            return None

        if n == 1:
            if byte == SYNC1:
                self._buf.append(byte)
            elif byte == SYNC0:
                # Stutter: stay armed instead of dropping back to hunting.
                self.resyncs += 1
            else:
                self._buf = bytearray()
                self.resyncs += 1
            return None

        if n == 2:
            if byte != MSG_CMD:
                self._restart_on(byte)
                return None
            self._buf.append(byte)
            return None

        if n == 3:
            if byte != CMD_PAYLOAD_LEN:
                self._restart_on(byte)
                return None
            self._buf.append(byte)
            return None

        self._buf.append(byte)
        if len(self._buf) < CMD_FRAME_LEN:
            return None

        frame = bytes(self._buf)
        self._buf = bytearray()
        decoded = decode_cmd(frame)
        if decoded is None:
            self.crc_errors += 1
        return decoded
