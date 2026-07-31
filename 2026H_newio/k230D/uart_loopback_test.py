"""On-device UART self-test for the K230D. Run this before blaming the MCU.

Proves the parts of the link that live entirely on the K230: the FPIOA pin mux,
the baud rate, the UART object, and protocol.py's encoder. Those are exactly the
parts a PC-side sniffer cannot check, and the pin mux is the one that silently
sends bytes nowhere if it is missing.

Wiring: jumper J8 pin 44 (TX) directly to pin 45 (RX). Nothing else connected -
unplug the MCU first, or its transmitter fights the loopback.

Run it from the CanMV IDE with protocol.py already copied to the device:

    exec(open("/sdcard/uart_loopback_test.py").read())

Expected result: "LOOPBACK PASS". Anything else means the fault is on this board,
not on the wire or the MCU.
"""

from machine import FPIOA, UART
import utime

import protocol as P

UART_ID = 2
BAUDRATE = 115200
TX_PIN = 44
RX_PIN = 45

# Generous: a 12-byte frame at 115200 is about 1 ms, so this is ~200x margin.
READ_TIMEOUT_MS = 200


def drain(uart, ms=50):
    """Discard anything already sitting in the RX buffer."""
    deadline = utime.ticks_add(utime.ticks_ms(), ms)
    while utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
        try:
            if not uart.read(64):
                break
        except Exception:
            break


def read_until(uart, reader, want, timeout_ms=READ_TIMEOUT_MS):
    """Collect decoded items until `want` of them arrive or time runs out."""
    got = []
    deadline = utime.ticks_add(utime.ticks_ms(), timeout_ms)
    while len(got) < want and utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
        try:
            data = uart.read(64)
        except Exception:
            data = None
        if data:
            got.extend(reader.feed(data))
        else:
            utime.sleep_ms(2)
    return got


def main():
    failures = []
    checks = [0]

    def check(cond, what):
        checks[0] += 1
        if not cond:
            failures.append(what)
            print("  FAIL " + what)

    print("K230D UART loopback test")
    print("jumper J8 pin %d (TX) to pin %d (RX), MCU unplugged" % (TX_PIN, RX_PIN))
    print("")

    fpioa = FPIOA()
    fpioa.set_function(TX_PIN, FPIOA.UART2_TXD)
    fpioa.set_function(RX_PIN, FPIOA.UART2_RXD)
    print("pin mux set: %d=UART2_TXD %d=UART2_RXD" % (TX_PIN, RX_PIN))

    uart = UART(
        UART_ID,
        baudrate=BAUDRATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
    )
    print("uart %d open at %d baud" % (UART_ID, BAUDRATE))
    drain(uart)

    # 1. Raw bytes make the round trip at all. If this fails, the jumper is not
    #    on, or the mux did not take - no point testing frames.
    probe = b"\x5A\xA5\x00\xFF\x0F\xF0"
    uart.write(probe)
    echoed = bytearray()
    deadline = utime.ticks_add(utime.ticks_ms(), READ_TIMEOUT_MS)
    while len(echoed) < len(probe) and utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
        data = uart.read(64)
        if data:
            echoed.extend(data)
        else:
            utime.sleep_ms(2)
    check(bytes(echoed) == probe,
          "raw loopback: sent %d bytes, got %d back" % (len(probe), len(echoed)))
    if bytes(echoed) != probe:
        print("")
        print("  Raw bytes did not come back. Check, in this order:")
        print("   1. the jumper between pin %d and pin %d" % (TX_PIN, RX_PIN))
        print("   2. that no other code holds UART%d open" % UART_ID)
        print("   3. that the MCU is really off the RX line")
        print("")
        print("LOOPBACK FAIL (%d/%d checks)" % (checks[0] - len(failures), checks[0]))
        return False

    # 2. A real MSG_BALL frame survives, and decodes to what went in.
    drain(uart)
    reader = P.UplinkReader()
    cases = [
        (0, P.FLAG_VALID | P.FLAG_DETECTED, 0, 0, 27),
        (1, P.FLAG_VALID | P.FLAG_DETECTED, 123, -456, 27),
        (2, P.FLAG_VALID, -123, 456, 30),
        (3, 0, 0, 0, 1),
        (255, 0x0F, 32767, -32768, 255),
    ]
    for seq, flags, pos, vel, fps in cases:
        uart.write(P.encode_ball(seq, flags, pos, vel, fps))
        items = read_until(uart, reader, 1)
        if len(items) != 1 or items[0][0] != "ball":
            check(False, "ball frame seq=%d did not come back" % seq)
            continue
        ball = items[0][1]
        check(ball["seq"] == seq, "seq %d -> %d" % (seq, ball["seq"]))
        check(ball["pos_mm"] == pos, "pos %d -> %d" % (pos, ball["pos_mm"]))
        check(ball["vel_mm_s"] == vel, "vel %d -> %d" % (vel, ball["vel_mm_s"]))
        check(ball["fps"] == fps, "fps %d -> %d" % (fps, ball["fps"]))
    check(reader.crc_errors == 0,
          "no CRC errors in loopback (got %d)" % reader.crc_errors)

    # 3. The downlink parser sees a command we send ourselves. This is the path
    #    that handles CMD_PING/SET_ZERO from the MCU.
    drain(uart)
    cmd_reader = P.CommandReader()
    uart.write(P.encode_cmd(P.CMD_PING, 0x5A))
    got = []
    deadline = utime.ticks_add(utime.ticks_ms(), READ_TIMEOUT_MS)
    while not got and utime.ticks_diff(deadline, utime.ticks_ms()) > 0:
        data = uart.read(64)
        if data:
            got.extend(cmd_reader.feed(data))
        else:
            utime.sleep_ms(2)
    check(got == [(P.CMD_PING, 0x5A)], "CMD_PING round-trips: %s" % str(got))

    # 4. Sustained streaming at the camera's rate, which is what actually runs.
    #    A byte lost here but not above means a buffer problem, not a wiring one.
    drain(uart)
    reader = P.UplinkReader()
    sent = 0
    received = []
    seq = 0
    for _ in range(60):
        uart.write(P.encode_ball(seq, P.FLAG_VALID | P.FLAG_DETECTED,
                                 seq * 2 - 60, 0, 27))
        sent += 1
        seq = (seq + 1) & 0xFF
        utime.sleep_ms(37)                     # ~27 fps
        data = uart.read(256)
        if data:
            received.extend(reader.feed(data))
    received.extend(read_until(uart, reader, sent - len(received), 200))
    balls = len([x for x in received if x[0] == "ball"])
    check(balls == sent, "streamed %d frames, decoded %d" % (sent, balls))
    check(reader.crc_errors == 0,
          "no CRC errors while streaming (got %d)" % reader.crc_errors)

    try:
        uart.deinit()
    except Exception:
        pass

    print("")
    passed = checks[0] - len(failures)
    if failures:
        print("LOOPBACK FAIL (%d/%d checks)" % (passed, checks[0]))
        for f in failures:
            print("  - " + f)
        return False

    print("LOOPBACK PASS (%d checks)" % checks[0])
    print("")
    print("The K230 side is sound: pin mux, baud, UART and encoder all work.")
    print("Remove the jumper and wire pin %d to the MCU's PB7." % TX_PIN)
    return True


main()
