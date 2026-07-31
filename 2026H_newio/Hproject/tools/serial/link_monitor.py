"""Link health tracking for the K230 <-> MSPM0 serial tools.

Pure logic: bytes in, report lines out. No serial dependency, so the same code
that runs against real hardware in sniff_k230.py is exercised offline by
`sniff_k230.py --selftest` and cross-checked against the MCU's C parser by
tools/test/test_protocol.py (test_monitor_agrees_with_mcu).

Framing and decoding come from k230D/protocol.py, so this agrees with the MCU's
C parser by construction rather than by a third re-implementation.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HPROJECT = os.path.abspath(os.path.join(HERE, "..", ".."))
K230D = os.path.abspath(os.path.join(HPROJECT, "..", "k230D"))
if K230D not in sys.path:
    sys.path.insert(0, K230D)

import protocol as P  # noqa: E402

# Matches BALANCE_DATA_TIMEOUT_MS in Hproject/control/control_config.h. A gap
# longer than this is what makes the MCU's balance PID stand down, so it is the
# threshold worth reporting on.
BALANCE_DATA_TIMEOUT_MS = 150


class LinkMonitor:
    """Decodes an uplink byte stream and tracks whether the MCU would be happy.

    Timestamps are passed in rather than read from the clock so tests can drive
    the stall detection deterministically.
    """

    def __init__(self, timeout_ms=BALANCE_DATA_TIMEOUT_MS):
        self.timeout_ms = timeout_ms
        self.reader = P.UplinkReader()

        self.balls = 0
        self.pongs = 0
        self.gaps = 0                 # frames the K230 never sent, from SEQ
        self.stalls = 0               # times the stream went quiet too long
        self.worst_interval_ms = 0
        self.usable_frames = 0        # frames the balance PID could act on

        self._prev_seq = None
        self._last_frame_ms = None
        self._stalled = False
        self.last_ball = None

    # ---------------------------------------------------------------- counters

    @property
    def crc_errors(self):
        return self.reader.crc_errors

    @property
    def resyncs(self):
        return self.reader.resyncs

    @property
    def rx_bytes(self):
        return self.reader.rx_bytes

    def usable_ratio(self):
        if self.balls == 0:
            return 0.0
        return float(self.usable_frames) / float(self.balls)

    def delivery_ratio(self):
        """Fraction of frames the K230 sent that arrived intact."""
        expected = self.balls + self.gaps
        if expected == 0:
            return 0.0
        return float(self.balls) / float(expected)

    # ------------------------------------------------------------------ feeding

    def feed(self, data, now_ms):
        """Push received bytes in. Returns a list of human-readable events."""
        events = []
        for kind, payload in self.reader.feed(data):
            if kind == "pong":
                self.pongs += 1
                events.append("PONG arg=%02X" % payload)
                continue

            self.balls += 1
            self.last_ball = payload

            if payload["flags"] & P.FLAG_VALID:
                self.usable_frames += 1

            if payload["flags"] & P.FLAG_SATURATED:
                events.append("SATURATED pos=%d vel=%d"
                              % (payload["pos_mm"], payload["vel_mm_s"]))

            # SEQ discontinuity means frames were sent and lost, or the K230
            # restarted. Unsigned 8-bit difference wraps correctly.
            if self._prev_seq is not None:
                step = (payload["seq"] - self._prev_seq) & 0xFF
                if step > 1:
                    self.gaps += step - 1
                    events.append("GAP %d frame(s) lost before seq=%d"
                                  % (step - 1, payload["seq"]))
            self._prev_seq = payload["seq"]

            if self._last_frame_ms is not None:
                interval = now_ms - self._last_frame_ms
                if interval > self.worst_interval_ms:
                    self.worst_interval_ms = interval
                if interval >= self.timeout_ms:
                    self.stalls += 1
                    events.append(
                        "STALL %d ms between frames, over the %d ms timeout"
                        % (interval, self.timeout_ms))
            self._last_frame_ms = now_ms
            self._stalled = False

        return events

    def check_stall(self, now_ms):
        """Call between reads: report a stream that has gone quiet. Once only."""
        if self._last_frame_ms is None or self._stalled:
            return None
        idle = now_ms - self._last_frame_ms
        if idle < self.timeout_ms:
            return None
        self._stalled = True
        self.stalls += 1
        return "STALL no frame for %d ms, MCU has stood the PID down" % idle

    # ------------------------------------------------------------------ output

    def ball_line(self):
        """One line describing the most recent ball frame."""
        b = self.last_ball
        if b is None:
            return "no ball frame yet"
        return ("seq=%3d %s pos=%+6d mm vel=%+6d mm/s fps=%3d"
                % (b["seq"], P.flags_str(b["flags"]),
                   b["pos_mm"], b["vel_mm_s"], b["fps"]))

    def summary(self):
        """Multi-line verdict. This is what tells you if the link is healthy."""
        lines = []
        lines.append("bytes=%d balls=%d pongs=%d"
                     % (self.rx_bytes, self.balls, self.pongs))
        lines.append("crc_errors=%d resyncs=%d seq_gaps=%d stalls=%d"
                     % (self.crc_errors, self.resyncs, self.gaps, self.stalls))
        if self.balls:
            lines.append("delivered=%.2f%% usable=%.2f%% worst_interval=%d ms"
                         % (self.delivery_ratio() * 100.0,
                            self.usable_ratio() * 100.0,
                            self.worst_interval_ms))

        verdict = []
        if self.balls == 0:
            verdict.append("FAIL no ball frames decoded")
            if self.rx_bytes:
                verdict.append("     bytes arrived but never framed - wrong "
                               "baud, or the K230 is still on the old format")
            else:
                verdict.append("     nothing arrived at all - check wiring, "
                               "pin mux and that main.py is running")
        else:
            if self.crc_errors:
                verdict.append("WARN %d CRC error(s) - noise or a baud mismatch"
                               % self.crc_errors)
            if self.gaps:
                verdict.append("WARN %d frame(s) lost in transit" % self.gaps)
            if self.stalls:
                verdict.append("WARN %d stall(s) over the %d ms timeout"
                               % (self.stalls, self.timeout_ms))
            if self.usable_frames == 0:
                verdict.append("WARN no frame had FLAG_VALID - link is fine but "
                               "the ruler never locked, so the PID stays idle")
            if not verdict:
                verdict.append("OK link healthy, MCU would track normally")
        lines.extend(verdict)
        return "\n".join(lines)
