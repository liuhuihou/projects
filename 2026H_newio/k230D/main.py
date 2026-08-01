from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from media.sensor import *
import os
import gc
import time
import utime

# The wire format lives in protocol.py, next to this file. Copy both to the
# device together: a missing protocol.py must fail loudly here rather than let
# the link come up speaking a format the MCU does not understand.
import protocol as P

try:
    from machine import UART, FPIOA
except Exception:
    UART = None
    FPIOA = None


kmodel_path = "/sdcard/yolo11n_det_320.kmodel"
labels = {0: "steel"}
model_input_size = [320, 320]

UART_ENABLE = True
UART_ID = 2
UART_BAUDRATE = 115200

# Send on every camera iteration. The MCU distinguishes "link dead" from "ball
# out of sight" by frame arrival alone, so its 150 ms freshness timeout needs a
# frame far more often than that. At ~27 fps this is one frame per ~37 ms;
# raising the divisor thins the stream out and eventually trips that timeout.
UART_SEND_EVERY_N_FRAMES = 1

# J8 port pin mux on the ALIENTEK K230D BOX. Without these the pins stay on
# their power-on function and nothing reaches the connector, however healthy the
# UART object looks. MCU TX (PB6) -> K230D RX (45), MCU RX (PB7) <- K230D TX (44).
UART_TX_PIN = 44
UART_RX_PIN = 45

display_mode = "st7701"
display_size = [640, 480]
rgb888p_size = [640, 360]
ref_size = [640, 360]

# The tube is horizontal and only appears around the vertical middle of the
# image. Search the full width, but only this y band in ref_size coordinates.
FORCE_HORIZONTAL_TUBE = True
TUBE_SEARCH_Y0_RATIO = 0.30
TUBE_SEARCH_Y1_RATIO = 0.70

# Fixed on-screen ruler. Default is based on the stable tube rectangle reported
# during the last run: tube_rect=(0, 96, 640, 168).
USE_FIXED_REFERENCE = True
FIXED_TUBE_RECT = (0, 96, 640, 168)
FIXED_BLUE_X = 320
FIXED_RED_LEFT_X = 135
FIXED_RED_RIGHT_X = 503

# Tube body: gray-yellow / milk-tea color.
TUBE_THRESHOLDS = [
    (20, 100, -20, 35, -5, 75),
    (28, 100, -15, 35, 0, 70),
    (70, 100, -12, 12, -12, 12),
]

# Reference lines. The script accepts red lines and blue center line.
RED_LINE_THRESHOLD = (5, 100, 20, 127, -30, 95)
BLUE_LINE_THRESHOLD = (0, 85, -10, 90, -128, -10)

# Two red reference lines are treated as 10 cm apart. If a blue center line is
# detected, it is treated as the zero point. Otherwise the red midpoint is zero.
RED_TO_RED_CM = 10.0

TUBE_MIN_PIXELS = 1000
TUBE_MIN_AREA = 1000
TUBE_MIN_ASPECT = 1.4

REF_LINE_MIN_PIXELS = 80
REF_LINE_MIN_AREA = 80
REF_LINE_MIN_ASPECT = 1.2
REF_LINE_VERTICAL_MIN_ASPECT = 1.6

TUBE_FALLBACK_PAD_X = 35
TUBE_FALLBACK_PAD_Y = 12

CONFIDENCE_THRESHOLD = 0.08
NMS_THRESHOLD = 0.45
MAX_BOXES_NUM = 50
LOCK_SCORE_THRESHOLD = 0.18
TRACK_SCORE_THRESHOLD = 0.08
MIN_BALL_SIZE = 18
MAX_BALL_SIZE = 150
MIN_BALL_AREA = MIN_BALL_SIZE * MIN_BALL_SIZE
MAX_BALL_AREA = MAX_BALL_SIZE * MAX_BALL_SIZE
MIN_BALL_RATIO = 0.55
MAX_BALL_RATIO = 1.80
TRACK_GATE_RADIUS = 120
MISSED_GATE_GROW = 45
MAX_MISSED_FRAMES = 6
MAX_PREDICT_STEP = 170
BALL_PREDICT_LEAD_MS = 70
PREDICT_HOLD_FRAMES = 4
FUSION_LOW_SPEED_PX_S = 80.0
FUSION_HIGH_SPEED_PX_S = 320.0
PRED_WEIGHT_AT_LOW_SPEED = 0.20
PRED_WEIGHT_AT_HIGH_SPEED = 0.85

REF_UPDATE_EVERY_N_FRAMES = 3
PRINT_EVERY_N_FRAMES = 5
POS_ALPHA = 0.55
VEL_ALPHA = 0.45


def absf(v):
    if v < 0:
        return -v
    return v


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def uart_init():
    if not UART_ENABLE or UART is None:
        return None
    try:
        if FPIOA is not None:
            fpioa = FPIOA()
            fpioa.set_function(UART_TX_PIN, FPIOA.UART2_TXD)
            fpioa.set_function(UART_RX_PIN, FPIOA.UART2_RXD)
        uart = UART(
            UART_ID,
            baudrate=UART_BAUDRATE,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )
        print("uart ok id=%d baud=%d tx=%d rx=%d"
              % (UART_ID, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN))
        return uart
    except Exception as e:
        print("uart init failed:", repr(e))
        return None


def uart_send_ball(uart, seq, valid, predicted, detected, x_cm, v_cm_s, fps):
    """Send one MSG_BALL frame. Called every iteration, ball or no ball.

    Sending unconditionally is the contract the MCU relies on: frames arriving
    with FLAG_VALID clear mean "link alive, position not usable", which is what
    lets it tell a lost ball from a dead link. Returning early when there is
    nothing to report would stall its freshness timeout instead.
    """
    if uart is None:
        return

    flags = 0
    if detected:
        flags |= P.FLAG_DETECTED
    if predicted:
        flags |= P.FLAG_PREDICTED

    if valid:
        flags |= P.FLAG_VALID
        pos_mm = int(round(x_cm * 10.0))
        vel_mm_s = int(round(v_cm_s * 10.0))
    else:
        # No ruler lock: the position is not in millimetres at all, so send zero
        # rather than a number the MCU would read as metric.
        pos_mm = 0
        vel_mm_s = 0

    try:
        uart.write(P.encode_ball(seq, flags, pos_mm, vel_mm_s, fps))
    except Exception:
        pass


def uart_poll_commands(uart, reader):
    """Drain the RX buffer and act on any MSG_CMD frames from the MCU.

    Returns the number of commands handled. PING is answered with PONG so the
    MCU can prove the link works in both directions.
    """
    global uart_stream_enabled, pos_zero_offset_cm

    if uart is None or reader is None:
        return 0

    try:
        data = uart.read(64)
    except Exception:
        return 0
    if not data:
        return 0

    handled = 0
    for cmd, arg in reader.feed(data):
        handled += 1
        if cmd == P.CMD_PING:
            try:
                uart.write(P.encode_pong(arg))
            except Exception:
                pass
        elif cmd == P.CMD_SET_ZERO:
            # Shift the reported origin so the ball reads zero where it stands.
            # Held as an offset rather than written into geometry["origin"],
            # because the reference geometry is recomputed every few frames and
            # would overwrite it. A constant offset leaves the velocity filter
            # untouched, since d(constant)/dt is zero.
            if last_valid_raw_x_cm is not None:
                pos_zero_offset_cm = last_valid_raw_x_cm
                print("cmd set_zero at %.2fcm" % pos_zero_offset_cm)
            else:
                print("cmd set_zero ignored: no ruler lock")
        elif cmd == P.CMD_SET_SEND:
            uart_stream_enabled = arg != 0
            print("cmd set_send %d" % arg)
        else:
            print("cmd unknown %02X arg=%02X" % (cmd, arg))
    return handled


def ref_to_disp_x(x):
    return int(x * display_size[0] // ref_size[0])


def ref_to_disp_y(y):
    return int(y * display_size[1] // ref_size[1])


def disp_to_ref_x(x):
    return float(x) * float(ref_size[0]) / float(display_size[0])


def disp_to_ref_y(y):
    return float(y) * float(ref_size[1]) / float(display_size[1])


def tube_search_roi():
    y0 = int(ref_size[1] * TUBE_SEARCH_Y0_RATIO)
    y1 = int(ref_size[1] * TUBE_SEARCH_Y1_RATIO)
    y0 = clamp(y0, 0, ref_size[1] - 1)
    y1 = clamp(y1, y0 + 1, ref_size[1])
    return (0, y0, ref_size[0], y1 - y0)


def draw_text(osd, x, y, text, color=(255, 255, 255), size=22):
    try:
        osd.draw_string_advanced(x, y, size, text, color=color)
    except Exception:
        osd.draw_string(x, y, text, color=color, scale=2)


def draw_cross_disp(osd, x, y, color, size=10, thickness=2):
    x = int(x)
    y = int(y)
    osd.draw_line(x - size, y, x + size, y, color=color, thickness=thickness)
    osd.draw_line(x, y - size, x, y + size, color=color, thickness=thickness)


def draw_line_ref(osd, a, b, color, thickness=2):
    osd.draw_line(
        ref_to_disp_x(a[0]),
        ref_to_disp_y(a[1]),
        ref_to_disp_x(b[0]),
        ref_to_disp_y(b[1]),
        color=color,
        thickness=thickness,
    )


def draw_rect_ref(osd, rect, color, thickness=2):
    x, y, w, h = rect
    osd.draw_rectangle(
        ref_to_disp_x(x),
        ref_to_disp_y(y),
        int(w * display_size[0] // ref_size[0]),
        int(h * display_size[1] // ref_size[1]),
        color=color,
        thickness=thickness,
    )


def rect_center(rect):
    x, y, w, h = rect
    return (x + w / 2.0, y + h / 2.0)


def normalize(vx, vy):
    length = (vx * vx + vy * vy) ** 0.5
    if length <= 0:
        return None
    return (vx / length, vy / length)


def dot(point, origin, axis):
    return (point[0] - origin[0]) * axis[0] + (point[1] - origin[1]) * axis[1]


def line_rect_segment(point, direction, rect):
    px, py = point
    dx, dy = direction
    x, y, w, h = rect
    x0 = float(x)
    y0 = float(y)
    x1 = float(x + w)
    y1 = float(y + h)
    pts = []

    if absf(dx) > 0.0001:
        t = (x0 - px) / dx
        yy = py + t * dy
        if yy >= y0 - 0.5 and yy <= y1 + 0.5:
            pts.append((x0, yy))
        t = (x1 - px) / dx
        yy = py + t * dy
        if yy >= y0 - 0.5 and yy <= y1 + 0.5:
            pts.append((x1, yy))

    if absf(dy) > 0.0001:
        t = (y0 - py) / dy
        xx = px + t * dx
        if xx >= x0 - 0.5 and xx <= x1 + 0.5:
            pts.append((xx, y0))
        t = (y1 - py) / dy
        xx = px + t * dx
        if xx >= x0 - 0.5 and xx <= x1 + 0.5:
            pts.append((xx, y1))

    if len(pts) < 2:
        return None

    best_i = 0
    best_j = 1
    best_d2 = -1.0
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            d2 = (pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2
            if d2 > best_d2:
                best_d2 = d2
                best_i = i
                best_j = j
    return pts[best_i], pts[best_j]


def detect_tube_rect(img):
    best = None
    best_score = -1
    roi = tube_search_roi()
    for threshold in TUBE_THRESHOLDS:
        try:
            blobs = img.find_blobs(
                [threshold],
                roi=roi,
                pixels_threshold=TUBE_MIN_PIXELS,
                area_threshold=TUBE_MIN_AREA,
                merge=True,
            )
        except Exception:
            blobs = []

        for blob in blobs:
            w = blob.w()
            h = blob.h()
            if w <= 0 or h <= 0:
                continue
            aspect = float(max(w, h)) / float(min(w, h))
            if aspect < TUBE_MIN_ASPECT:
                continue
            score = blob.pixels()
            if score > best_score:
                best = blob
                best_score = score

    if best is None:
        return None, 0, 0
    return best.rect(), best.pixels(), best.area()


def ref_line_from_blob(blob, color_id):
    w = blob.w()
    h = blob.h()
    if w <= 0 or h <= 0:
        return None
    area = blob.area()
    if area < REF_LINE_MIN_AREA:
        return None
    aspect = float(max(w, h)) / float(min(w, h))
    if aspect < REF_LINE_MIN_ASPECT:
        return None
    if h < w:
        return None
    vertical_aspect = float(h) / float(w)
    if vertical_aspect < REF_LINE_VERTICAL_MIN_ASPECT:
        return None

    cx = blob.cx()
    cy = blob.cy()
    direction = (0.0, 1.0)
    p1 = (cx, blob.y())
    p2 = (cx, blob.y() + h)

    return {
        "center": (cx, cy),
        "direction": direction,
        "line": (p1, p2),
        "rect": blob.rect(),
        "pixels": blob.pixels(),
        "area": area,
        "color_id": color_id,
    }


def find_reference_lines(img, tube_rect):
    out = []
    roi = tube_rect
    if roi is None:
        roi = tube_search_roi()
    thresholds = [
        ("red", RED_LINE_THRESHOLD),
        ("blue", BLUE_LINE_THRESHOLD),
    ]
    for color_id, threshold in thresholds:
        try:
            blobs = img.find_blobs(
                [threshold],
                roi=roi,
                pixels_threshold=REF_LINE_MIN_PIXELS,
                area_threshold=REF_LINE_MIN_AREA,
                merge=True,
            )
        except Exception:
            blobs = []
        for blob in blobs:
            item = ref_line_from_blob(blob, color_id)
            if item is not None:
                out.append(item)
    return out


def make_fixed_line(x, color_id, rect):
    y = rect[1]
    h = rect[3]
    return {
        "center": (x, y + h / 2.0),
        "direction": (0.0, 1.0),
        "line": ((x, y), (x, y + h)),
        "rect": (x - 2, y, 4, h),
        "pixels": h * 4,
        "area": h * 4,
        "color_id": color_id,
    }


def make_fixed_reference():
    rect = FIXED_TUBE_RECT
    lines = [
        make_fixed_line(FIXED_RED_LEFT_X, "red", rect),
        make_fixed_line(FIXED_RED_RIGHT_X, "red", rect),
        make_fixed_line(FIXED_BLUE_X, "blue", rect),
    ]
    return rect, lines, choose_reference_geometry(lines, rect)


def horizontal_tube_rect(rect):
    x, y, w, h = rect
    y0 = clamp(y - TUBE_FALLBACK_PAD_Y, 0, ref_size[1] - 1)
    y1 = clamp(y + h + TUBE_FALLBACK_PAD_Y, y0 + 1, ref_size[1])
    return (0, int(y0), ref_size[0], int(y1 - y0))


def tube_rect_from_ref_lines(ref_lines):
    if len(ref_lines) < 2:
        return None

    min_y = ref_size[1]
    max_y = 0
    red_count = 0
    for line in ref_lines:
        if line["color_id"] == "red":
            red_count += 1
        x, y, w, h = line["rect"]
        if y < min_y:
            min_y = y
        if y + h > max_y:
            max_y = y + h

    if red_count < 2:
        return None

    min_y = clamp(min_y - TUBE_FALLBACK_PAD_Y, 0, ref_size[1] - 1)
    max_y = clamp(max_y + TUBE_FALLBACK_PAD_Y, min_y + 1, ref_size[1])
    return (0, int(min_y), ref_size[0], int(max_y - min_y))


def tube_axes_from_rect(rect):
    x, y, w, h = rect
    center = rect_center(rect)
    if FORCE_HORIZONTAL_TUBE or w >= h:
        tube_axis = (1.0, 0.0)
        ref_dir = (0.0, 1.0)
        tube_start = (x, center[1])
        tube_end = (x + w, center[1])
    else:
        tube_axis = (0.0, 1.0)
        ref_dir = (1.0, 0.0)
        tube_start = (center[0], y)
        tube_end = (center[0], y + h)
    return center, tube_axis, ref_dir, tube_start, tube_end


def choose_reference_geometry(ref_lines, tube_rect):
    tube_center, tube_axis, default_ref_dir, tube_start, tube_end = tube_axes_from_rect(tube_rect)
    ref_dir = default_ref_dir
    best_line = None
    best_pixels = -1
    for line in ref_lines:
        if line["pixels"] > best_pixels:
            best_line = line
            best_pixels = line["pixels"]
    if best_line is not None:
        ref_dir = best_line["direction"]

    red_lines = []
    blue_lines = []
    for line in ref_lines:
        if line["color_id"] == "red":
            red_lines.append(line)
        elif line["color_id"] == "blue":
            blue_lines.append(line)

    origin = tube_center
    scale_cm_per_px = None
    red_span_px = None

    if len(red_lines) >= 2:
        best_a = red_lines[0]
        best_b = red_lines[1]
        best_span = -1.0
        for i in range(len(red_lines)):
            for j in range(i + 1, len(red_lines)):
                si = dot(red_lines[i]["center"], tube_center, tube_axis)
                sj = dot(red_lines[j]["center"], tube_center, tube_axis)
                span = absf(sj - si)
                if span > best_span:
                    best_span = span
                    best_a = red_lines[i]
                    best_b = red_lines[j]
        if best_span > 1:
            scale_cm_per_px = RED_TO_RED_CM / best_span
            red_span_px = best_span
            origin = (
                (best_a["center"][0] + best_b["center"][0]) / 2.0,
                (best_a["center"][1] + best_b["center"][1]) / 2.0,
            )

    if len(blue_lines) >= 1:
        best_blue = blue_lines[0]
        best_dist = 1000000.0
        for line in blue_lines:
            dist = absf(dot(line["center"], origin, tube_axis))
            if dist < best_dist:
                best_dist = dist
                best_blue = line
        origin = best_blue["center"]

    return {
        "origin": origin,
        "tube_axis": tube_axis,
        "ref_dir": ref_dir,
        "tube_line": (tube_start, tube_end),
        "scale_cm_per_px": scale_cm_per_px,
        "red_span_px": red_span_px,
        "red_count": len(red_lines),
        "blue_count": len(blue_lines),
    }


def extract_ball_candidates(res):
    # CanMV YOLO detect result: res[0] boxes are [x, y, w, h] in display coords,
    # res[1] classes, res[2] scores. This is the same format draw_result() uses.
    candidates = []
    raw_count = 0
    kept_count = 0
    max_area = 0
    best_score = -1.0
    try:
        boxes = res[0]
        classes = res[1]
        scores = res[2]
        raw_count = len(boxes)
        for i in range(len(boxes)):
            box = boxes[i]
            if len(box) < 4:
                continue
            x = float(box[0])
            y = float(box[1])
            w = float(box[2])
            h = float(box[3])
            score = float(scores[i])
            class_id = int(classes[i])
            area = w * h
            if area > max_area:
                max_area = int(area)
            if score > best_score:
                best_score = score
            if w <= 0 or h <= 0:
                continue
            if area < MIN_BALL_AREA:
                continue
            if area > MAX_BALL_AREA:
                continue
            ratio = w / h
            if ratio < MIN_BALL_RATIO or ratio > MAX_BALL_RATIO:
                continue
            kept_count += 1
            candidates.append({
                "x": x,
                "y": y,
                "w": w,
                "h": h,
                "score": score,
                "class_id": class_id,
                "area": area,
                "cx": x + w / 2.0,
                "cy": y + h / 2.0,
            })
    except Exception:
        pass
    return candidates, raw_count, kept_count, max_area, best_score


def pick_tracked_ball(candidates, locked, last_cx, last_cy, vx, vy, dt_s, missed_frames):
    if len(candidates) == 0:
        return None, None

    pred_x = last_cx
    pred_y = last_cy
    gate = None
    if locked and last_cx is not None and last_cy is not None:
        step_x = clamp(vx * dt_s, -MAX_PREDICT_STEP, MAX_PREDICT_STEP)
        step_y = clamp(vy * dt_s, -MAX_PREDICT_STEP, MAX_PREDICT_STEP)
        pred_x = clamp(last_cx + step_x, 0, display_size[0] - 1)
        pred_y = clamp(last_cy + step_y, 0, display_size[1] - 1)
        gate = TRACK_GATE_RADIUS + missed_frames * MISSED_GATE_GROW

    best = None
    best_rank = -1000000.0
    for ball in candidates:
        score = ball["score"]
        if locked:
            if score < TRACK_SCORE_THRESHOLD:
                continue
        else:
            if score < LOCK_SCORE_THRESHOLD:
                continue

        rank = score
        if gate is not None:
            dx = ball["cx"] - pred_x
            dy = ball["cy"] - pred_y
            dist2 = dx * dx + dy * dy
            if dist2 > gate * gate:
                continue
            rank = score - 0.0009 * (absf(dx) + absf(dy))

        if rank > best_rank:
            best = ball
            best_rank = rank

    return best, gate


def draw_ball_box_disp(osd, ball, color=(255, 0, 255), thickness=3):
    osd.draw_rectangle(
        int(ball["x"]),
        int(ball["y"]),
        int(ball["w"]),
        int(ball["h"]),
        color=color,
        thickness=thickness,
    )


def predicted_ball_from_last(now_ms, extra_lead_ms=0):
    if last_ball_cx is None or last_ball_cy is None or last_ball_t_ms is None:
        return None
    if last_ball_w is None or last_ball_h is None:
        return None
    dt_ms = utime.ticks_diff(now_ms, last_ball_t_ms) + extra_lead_ms
    if dt_ms < 0:
        dt_ms = 0
    dt_s = dt_ms / 1000.0
    step_x = clamp(ball_vx * dt_s, -MAX_PREDICT_STEP, MAX_PREDICT_STEP)
    step_y = clamp(ball_vy * dt_s, -MAX_PREDICT_STEP, MAX_PREDICT_STEP)
    cx = clamp(last_ball_cx + step_x, 0, display_size[0] - 1)
    cy = clamp(last_ball_cy + step_y, 0, display_size[1] - 1)
    return {
        "x": cx - last_ball_w / 2.0,
        "y": cy - last_ball_h / 2.0,
        "w": last_ball_w,
        "h": last_ball_h,
        "score": 0.0,
        "class_id": 0,
        "area": last_ball_w * last_ball_h,
        "cx": cx,
        "cy": cy,
        "predicted": True,
    }


def predict_weight_from_speed(vx, vy):
    speed = (vx * vx + vy * vy) ** 0.5
    if speed <= FUSION_LOW_SPEED_PX_S:
        return PRED_WEIGHT_AT_LOW_SPEED
    if speed >= FUSION_HIGH_SPEED_PX_S:
        return PRED_WEIGHT_AT_HIGH_SPEED
    t = (speed - FUSION_LOW_SPEED_PX_S) / (FUSION_HIGH_SPEED_PX_S - FUSION_LOW_SPEED_PX_S)
    return PRED_WEIGHT_AT_LOW_SPEED + t * (PRED_WEIGHT_AT_HIGH_SPEED - PRED_WEIGHT_AT_LOW_SPEED)


def fuse_ball_measurement_prediction(measured, predicted, vx, vy):
    if measured is None:
        return predicted
    if predicted is None:
        return measured

    wp = predict_weight_from_speed(vx, vy)
    wm = 1.0 - wp
    cx = measured["cx"] * wm + predicted["cx"] * wp
    cy = measured["cy"] * wm + predicted["cy"] * wp
    w = measured["w"]
    h = measured["h"]

    return {
        "x": cx - w / 2.0,
        "y": cy - h / 2.0,
        "w": w,
        "h": h,
        "score": measured["score"],
        "class_id": measured["class_id"],
        "area": measured["area"],
        "cx": cx,
        "cy": cy,
        "predicted": True,
        "pred_weight": wp,
    }


def draw_extended_ref_line(osd, line, rect, color, thickness=2):
    seg = None
    if rect is not None:
        seg = line_rect_segment(line["center"], line["direction"], rect)
    if seg is not None:
        draw_line_ref(osd, seg[0], seg[1], color, thickness=thickness)
    else:
        draw_line_ref(osd, line["line"][0], line["line"][1], color, thickness=thickness)


pl = None
yolo = None
sensor = None
uart = None
uart_seq = 0
uart_cmd_reader = None
uart_stream_enabled = True

# Raw filtered position, before pos_zero_offset_cm is applied. Kept so a
# CMD_SET_ZERO can re-zero against what the camera actually sees.
last_valid_raw_x_cm = None
pos_zero_offset_cm = 0.0

tube_rect = None
tube_pixels = 0
tube_area = 0
ref_lines = []
geometry = None

last_t_ms = None
last_pos = None
pos_f = None
vel_f = 0.0

last_ball_t_ms = None
last_ball_cx = None
last_ball_cy = None
last_ball_w = None
last_ball_h = None
ball_vx = 0.0
ball_vy = 0.0
ball_locked = False
missed_ball_frames = 0

frame_count = 0

try:
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_framesize(width=ref_size[0], height=ref_size[1], chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_1)

    pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
    pl.create(sensor=sensor)
    display_size = pl.get_display_size()

    yolo = YOLO11(
        task_type="detect",
        mode="video",
        kmodel_path=kmodel_path,
        labels=labels,
        rgb888p_size=rgb888p_size,
        model_input_size=model_input_size,
        display_size=display_size,
        conf_thresh=CONFIDENCE_THRESHOLD,
        nms_thresh=NMS_THRESHOLD,
        max_boxes_num=MAX_BOXES_NUM,
        debug_mode=0,
    )
    yolo.config_preprocess()
    uart = uart_init()
    uart_cmd_reader = P.CommandReader()
    clock = time.clock()

    while True:
        os.exitpoint()
        clock.tick()
        frame_count += 1

        # Service the downlink first: a CMD_SET_ZERO or CMD_SET_SEND should take
        # effect on this iteration's frame, not the next one.
        uart_poll_commands(uart, uart_cmd_reader)

        now_ms = utime.ticks_ms()
        dt_s = 0.0
        if last_ball_t_ms is not None:
            dt_ms_for_pick = utime.ticks_diff(now_ms, last_ball_t_ms)
            if dt_ms_for_pick > 0:
                dt_s = dt_ms_for_pick / 1000.0

        ai_img = pl.get_frame()
        res = yolo.run(ai_img)
        candidates, raw_n, kept_n, max_area, best_score = extract_ball_candidates(res)
        ball, ball_gate = pick_tracked_ball(
            candidates,
            ball_locked,
            last_ball_cx,
            last_ball_cy,
            ball_vx,
            ball_vy,
            dt_s,
            missed_ball_frames,
        )

        coord_ball = None
        if ball is not None:
            if last_ball_cx is not None and last_ball_cy is not None and dt_s > 0:
                ball_vx = (ball["cx"] - last_ball_cx) / dt_s
                ball_vy = (ball["cy"] - last_ball_cy) / dt_s
            last_ball_cx = ball["cx"]
            last_ball_cy = ball["cy"]
            last_ball_w = ball["w"]
            last_ball_h = ball["h"]
            last_ball_t_ms = now_ms
            ball_locked = True
            missed_ball_frames = 0
            pred_ball = predicted_ball_from_last(now_ms, BALL_PREDICT_LEAD_MS)
            coord_ball = fuse_ball_measurement_prediction(ball, pred_ball, ball_vx, ball_vy)
        else:
            if ball_locked:
                missed_ball_frames += 1
                if missed_ball_frames > MAX_MISSED_FRAMES:
                    ball_locked = False
                    last_ball_cx = None
                    last_ball_cy = None
                    last_ball_w = None
                    last_ball_h = None
                    last_ball_t_ms = None
                    ball_vx = 0.0
                    ball_vy = 0.0
                elif missed_ball_frames <= PREDICT_HOLD_FRAMES:
                    coord_ball = predicted_ball_from_last(now_ms, 0)
            else:
                last_ball_t_ms = now_ms

        if frame_count <= 3 or frame_count % REF_UPDATE_EVERY_N_FRAMES == 0:
            try:
                if USE_FIXED_REFERENCE:
                    tube_rect, ref_lines, geometry = make_fixed_reference()
                    tube_pixels = 0
                    tube_area = tube_rect[2] * tube_rect[3]
                else:
                    ref_img = sensor.snapshot(chn=CAM_CHN_ID_1)
                    new_rect, tube_pixels, tube_area = detect_tube_rect(ref_img)
                    if new_rect is not None:
                        tube_rect = horizontal_tube_rect(new_rect)
                        ref_lines = find_reference_lines(ref_img, tube_rect)
                        geometry = choose_reference_geometry(ref_lines, tube_rect)
                    else:
                        ref_lines = find_reference_lines(ref_img, None)
                        fallback_rect = tube_rect_from_ref_lines(ref_lines)
                        if fallback_rect is not None:
                            tube_rect = fallback_rect
                            tube_pixels = 0
                            tube_area = fallback_rect[2] * fallback_rect[3]
                            geometry = choose_reference_geometry(ref_lines, tube_rect)
            except Exception as ref_e:
                if frame_count % PRINT_EVERY_N_FRAMES == 0:
                    print("tube/ref update failed:", repr(ref_e))

        try:
            pl.osd_img.clear()
        except Exception:
            pass

        if ball is not None:
            draw_ball_box_disp(pl.osd_img, ball, (255, 0, 255), thickness=3)
        if coord_ball is not None and coord_ball is not ball:
            draw_ball_box_disp(pl.osd_img, coord_ball, (255, 255, 0), thickness=2)

        if tube_rect is not None:
            draw_rect_ref(pl.osd_img, tube_rect, (0, 255, 0), thickness=2)
        if geometry is not None:
            draw_line_ref(pl.osd_img, geometry["tube_line"][0], geometry["tube_line"][1], (255, 255, 0), thickness=2)
            ox, oy = geometry["origin"]
            seg = line_rect_segment(geometry["origin"], geometry["ref_dir"], tube_rect)
            if seg is not None:
                draw_line_ref(pl.osd_img, seg[0], seg[1], (0, 0, 255), thickness=2)
            draw_cross_disp(pl.osd_img, ref_to_disp_x(ox), ref_to_disp_y(oy), (0, 0, 255), size=9, thickness=2)

        for line in ref_lines:
            color = (255, 0, 0) if line["color_id"] == "red" else (0, 0, 255)
            draw_extended_ref_line(pl.osd_img, line, tube_rect, color, thickness=2)

        status = "ball lost raw=%d kept=%d maxA=%d" % (raw_n, kept_n, max_area)
        line_text = "tube lost"
        coord_text = "coord lost"
        comm_valid = False
        comm_x_cm = 0.0
        comm_v_cm_s = 0.0
        comm_predicted = False
        comm_detected = ball is not None

        if tube_rect is not None:
            if USE_FIXED_REFERENCE:
                line_text = "tube fixed area=%d refs=%d" % (tube_area, len(ref_lines))
            else:
                line_text = "tube pix=%d area=%d refs=%d" % (tube_pixels, tube_area, len(ref_lines))

        if coord_ball is not None:
            if ball is not None:
                draw_cross_disp(pl.osd_img, ball["cx"], ball["cy"], (0, 255, 255), size=10, thickness=2)
            draw_cross_disp(pl.osd_img, coord_ball["cx"], coord_ball["cy"], (255, 255, 0), size=12, thickness=2)
            draw_text(pl.osd_img, int(coord_ball["cx"]) + 6, int(coord_ball["cy"]) - 12, "P", color=(255, 255, 0), size=22)

            ball_ref = (disp_to_ref_x(coord_ball["cx"]), disp_to_ref_y(coord_ball["cy"]))
            if tube_rect is not None and geometry is not None:
                seg = line_rect_segment(ball_ref, geometry["ref_dir"], tube_rect)
                if seg is not None:
                    draw_line_ref(pl.osd_img, seg[0], seg[1], (0, 255, 255), thickness=3)

                pos_px = dot(ball_ref, geometry["origin"], geometry["tube_axis"])
                pos_text = "pos_px=%.1f" % pos_px
                coord_text = "raw_px=%.1f" % pos_px
                if geometry["scale_cm_per_px"] is not None:
                    pos_cm = pos_px * geometry["scale_cm_per_px"]
                    coord_text = "raw_px=%.1f raw_cm=%.2f span=%.1f" % (
                        pos_px,
                        pos_cm,
                        geometry["red_span_px"] if geometry["red_span_px"] is not None else -1,
                    )
                    if pos_f is None:
                        pos_f = pos_cm
                    else:
                        pos_f = POS_ALPHA * pos_cm + (1.0 - POS_ALPHA) * pos_f
                    if last_t_ms is not None and last_pos is not None:
                        dt_ms = utime.ticks_diff(now_ms, last_t_ms)
                        if dt_ms > 0:
                            vel = (pos_f - last_pos) * 1000.0 / float(dt_ms)
                            vel_f = VEL_ALPHA * vel + (1.0 - VEL_ALPHA) * vel_f
                    last_pos = pos_f
                    last_t_ms = now_ms
                    last_valid_raw_x_cm = pos_f
                    comm_valid = True
                    comm_x_cm = pos_f - pos_zero_offset_cm
                    comm_v_cm_s = vel_f
                    comm_predicted = "predicted" in coord_ball and coord_ball["predicted"]
                    pos_text = "x=%.2fcm v=%.2fcm/s" % (comm_x_cm, vel_f)
                if ball is not None:
                    pred_w = coord_ball["pred_weight"] if "pred_weight" in coord_ball else 1.0
                    status = "%s score=%.2f area=%d pw=%.2f" % (
                        pos_text,
                        ball["score"],
                        int(ball["area"]),
                        pred_w,
                    )
                else:
                    status = "%s predicted miss=%d" % (pos_text, missed_ball_frames)
            else:
                last_t_ms = None
                last_pos = None
                if ball is not None:
                    status = "P=(%.1f,%.1f) score=%.2f area=%d" % (
                        coord_ball["cx"],
                        coord_ball["cy"],
                        ball["score"],
                        int(ball["area"]),
                    )
                else:
                    status = "P=(%.1f,%.1f) predicted miss=%d" % (
                        coord_ball["cx"],
                        coord_ball["cy"],
                        missed_ball_frames,
                    )
        else:
            last_t_ms = None
            last_pos = None
            if ball_locked:
                status = "ball miss %d/%d raw=%d kept=%d maxA=%d" % (
                    missed_ball_frames,
                    MAX_MISSED_FRAMES,
                    raw_n,
                    kept_n,
                    max_area,
                )

        if uart_stream_enabled and frame_count % UART_SEND_EVERY_N_FRAMES == 0:
            uart_send_ball(
                uart,
                uart_seq,
                comm_valid,
                comm_predicted,
                comm_detected,
                comm_x_cm,
                comm_v_cm_s,
                clock.fps(),
            )
            uart_seq = (uart_seq + 1) & 0xFF

        draw_text(pl.osd_img, 5, 5, status)
        draw_text(pl.osd_img, 5, 32, line_text)
        draw_text(pl.osd_img, 5, 59, coord_text)
        draw_text(pl.osd_img, 5, 86, "FPS %.1f red=%d blue=%d" % (
            clock.fps(),
            0 if geometry is None else geometry["red_count"],
            0 if geometry is None else geometry["blue_count"],
        ))

        if frame_count % PRINT_EVERY_N_FRAMES == 0:
            print(
                "%s %s %s raw=%d kept=%d max_area=%d best_score=%.3f tube_rect=%s refs=%d fps=%.1f"
                % (
                    status,
                    line_text,
                    coord_text,
                    raw_n,
                    kept_n,
                    max_area,
                    best_score,
                    str(tube_rect),
                    len(ref_lines),
                    clock.fps(),
                )
            )

        pl.show_image()
        if frame_count % 5 == 0:
            gc.collect()

except BaseException as e:
    print("exception:", repr(e))
    print(e)

finally:
    try:
        if uart is not None:
            uart.deinit()
    except Exception:
        pass
    try:
        if yolo is not None:
            yolo.deinit()
    except Exception:
        pass
    try:
        if pl is not None:
            pl.destroy()
    except Exception:
        pass
