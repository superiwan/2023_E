"""MaixVision 入口：红色运动目标系统。"""

import config
from geometry_cache import GeometryCache
from rectangle_lock import RectangleLock
from touch_controls import TouchControls
from tracker_state import RedMode, RedTargetStateMachine, RunState
from trajectory import generate_waypoints, rectangle_center
from shared.blob_tracking import choose_blob
from shared.protocol import FrameParser, MessageType, encode_frame


def _new_rectangle_lock(mode=RedMode.A4_BORDER):
    screen_mode = mode in (RedMode.ORIGIN, RedMode.SCREEN_BORDER)
    return RectangleLock(
        config.FRAME_WIDTH,
        config.FRAME_HEIGHT,
        config.RECT_STABLE_FRAMES,
        config.SCREEN_PAIR_MISS_FRAMES if screen_mode else config.PAIR_MISS_FRAMES,
        config.MAX_CORNER_JITTER,
        config.MIN_RECT_AREA_RATIO,
        config.MAX_RECT_AREA_CHANGE_RATIO,
        config.SCREEN_ASPECT_RATIO if screen_mode else config.A4_ASPECT_RATIO,
        config.SCREEN_ASPECT_TOLERANCE if screen_mode else config.A4_ASPECT_TOLERANCE,
        config.MIN_INNER_OUTER_AREA_RATIO,
        not screen_mode,
        config.SCREEN_EXPECTED_AREA_RATIO if screen_mode else None,
    )


def _reset_acquisition(mode):
    return _new_rectangle_lock(mode), None, None


def _scale_corners(corners, source_width, source_height, target_width, target_height):
    scale_x = target_width / source_width
    scale_y = target_height / source_height
    return [(round(x * scale_x), round(y * scale_y)) for x, y in corners]


def _frame_due(frame_index, interval):
    return frame_index % interval == 0


def _update_visible_candidates(previous, current, missed_detections):
    if current:
        return current, 0
    missed_detections += 1
    return (previous if missed_detections == 1 else []), missed_detections


def _rectangle_edges(corners):
    return [(corners[index], corners[(index + 1) % 4]) for index in range(4)]


def _expanded_roi(corners):
    margin = config.RECT_ROI_MARGIN
    x0 = max(0, min(point[0] for point in corners) - margin)
    y0 = max(0, min(point[1] for point in corners) - margin)
    x1 = min(config.FRAME_WIDTH, max(point[0] for point in corners) + margin + 1)
    y1 = min(config.FRAME_HEIGHT, max(point[1] for point in corners) + margin + 1)
    return [x0, y0, x1 - x0, y1 - y0]


def _laser_roi(corners, mode):
    if mode == RedMode.ORIGIN:
        return [0, 0, config.FRAME_WIDTH, config.FRAME_HEIGHT]
    return _expanded_roi(corners)


def _restore_cached_trajectory(machine, geometry_cache):
    cached = geometry_cache.get(machine.mode)
    if cached is None:
        return None
    corners, confidence = cached
    if machine.mode == RedMode.ORIGIN:
        waypoints = [rectangle_center(corners)]
    else:
        waypoints = generate_waypoints(corners, config.EDGE_SEGMENTS)
    machine.set_trajectory(waypoints, confidence)
    return corners


def _draw_buttons(frame, image, active_mode):
    mode_width = config.FRAME_WIDTH // 3
    for index, (label, mode) in enumerate(
        (("ORIGIN", RedMode.ORIGIN), ("SCREEN", RedMode.SCREEN_BORDER), ("A4", RedMode.A4_BORDER))
    ):
        x = index * mode_width
        color = image.COLOR_GREEN if mode == active_mode else image.COLOR_WHITE
        frame.draw_rect(x, config.MODE_BUTTON_TOP, mode_width, config.MODE_BUTTON_HEIGHT, color, 2)
        frame.draw_string(x + 8, config.MODE_BUTTON_TOP + 15, label, color)

    top = config.FRAME_HEIGHT - config.RUNTIME_BUTTON_HEIGHT
    width = config.FRAME_WIDTH // 3
    for index, label in enumerate(("REACQUIRE", "START/RESUME", "PAUSE")):
        x = index * width
        frame.draw_rect(x, top, width, config.RUNTIME_BUTTON_HEIGHT, image.COLOR_WHITE, 2)
        frame.draw_string(x + 8, top + 20, label, image.COLOR_WHITE)


def _draw_center_marker(frame, center, color):
    x, y = center
    frame.draw_circle(x, y, 8, color, 2)
    frame.draw_line(x - 12, y, x + 12, y, color, 2)
    frame.draw_line(x, y - 12, x, y + 12, color, 2)


def _draw_rectangles(frame, rectangles, color):
    for corners in rectangles:
        for start, end in _rectangle_edges(corners):
            frame.draw_line(start[0], start[1], end[0], end[1], color, 2)


def _draw_cached_geometry(frame, geometry_cache, image):
    screen = geometry_cache.get(RedMode.SCREEN_BORDER)
    if screen is not None:
        screen_corners, _ = screen
        _draw_rectangles(frame, [screen_corners], image.COLOR_GREEN)
        _draw_center_marker(frame, rectangle_center(screen_corners), image.COLOR_WHITE)

    a4 = geometry_cache.get(RedMode.A4_BORDER)
    if a4 is not None:
        a4_corners, _ = a4
        _draw_rectangles(frame, [a4_corners], image.COLOR_BLUE)


def run():
    from maix import app, camera, display, err, image, pinmap, time, touchscreen, uart

    err.check_raise(pinmap.set_pin_function(config.UART_TX_PIN, config.UART_TX_FUNCTION), "UART TX pin mapping failed")
    err.check_raise(pinmap.set_pin_function(config.UART_RX_PIN, config.UART_RX_FUNCTION), "UART RX pin mapping failed")
    serial = uart.UART(config.UART_DEVICE, config.UART_BAUDRATE)
    cam = camera.Camera(
        config.FRAME_WIDTH,
        config.FRAME_HEIGHT,
        fps=config.CAMERA_FPS,
        buff_num=config.CAMERA_BUFFER_COUNT,
    )
    disp = display.Display()
    touch = touchscreen.TouchScreen()
    controls = TouchControls(
        config.FRAME_WIDTH,
        config.FRAME_HEIGHT,
        config.RUNTIME_BUTTON_HEIGHT,
        config.MODE_BUTTON_TOP,
        config.MODE_BUTTON_HEIGHT,
        config.TOUCH_DEBOUNCE_MS,
    )
    parser = FrameParser()
    machine = RedTargetStateMachine(config.ARRIVE_RADIUS, config.STABLE_FRAMES, config.WAYPOINT_SEND_INTERVAL_MS)
    geometry_cache = GeometryCache()
    rect_lock = _new_rectangle_lock(machine.mode)
    laser_roi = None
    previous_laser = None
    rectangle_candidates = []
    rectangle_candidate_misses = 0
    last_frame_ms = time.ticks_ms()
    fps = 0.0
    frame_index = 0

    def activate_mode():
        nonlocal rect_lock, laser_roi, previous_laser, rectangle_candidates, rectangle_candidate_misses
        rect_lock, laser_roi, previous_laser = _reset_acquisition(machine.mode)
        rectangle_candidates = []
        rectangle_candidate_misses = 0
        corners = _restore_cached_trajectory(machine, geometry_cache)
        if corners is not None:
            laser_roi = _laser_roi(corners, machine.mode)

    def apply_control(command, index):
        reset_required = machine.handle_command(command, index)
        if not reset_required:
            return
        if command == MessageType.REACQUIRE:
            geometry_cache.clear(machine.mode)
        activate_mode()

    while not app.need_exit():
        frame = cam.read()
        now_ms = time.ticks_ms()
        frame_ms = max(1, now_ms - last_frame_ms)
        last_frame_ms = now_ms
        fps = fps * 0.8 + 200.0 / frame_ms

        incoming = serial.read()
        if incoming:
            for message_type, index, _, _ in parser.feed(incoming):
                if message_type in (
                    MessageType.START_RESUME,
                    MessageType.PAUSE,
                    MessageType.REACQUIRE,
                    MessageType.SELECT_MODE,
                ):
                    apply_control(message_type, index)

        x, y, pressed = touch.read()
        event = controls.update(x, y, pressed, now_ms)
        if event is not None:
            command, index = event
            apply_control(command, index)
            serial.write(encode_frame(command, index))

        if machine.state == RunState.ACQUIRE and _frame_due(frame_index, config.RECT_DETECT_INTERVAL_FRAMES):
            detection_frame = frame.resize(config.RECT_DETECTION_WIDTH, config.RECT_DETECTION_HEIGHT)
            detected_rectangles = [
                _scale_corners(
                    item.corners(),
                    config.RECT_DETECTION_WIDTH,
                    config.RECT_DETECTION_HEIGHT,
                    config.FRAME_WIDTH,
                    config.FRAME_HEIGHT,
                )
                for item in detection_frame.find_rects(threshold=config.RECT_THRESHOLD)
            ]
            del detection_frame
            rectangle_candidates, rectangle_candidate_misses = _update_visible_candidates(
                rectangle_candidates,
                rect_lock.filtered_candidates(detected_rectangles),
                rectangle_candidate_misses,
            )
            lock_result = rect_lock.observe(detected_rectangles)
            if lock_result is not None:
                geometry_cache.store(machine.mode, lock_result.corners, lock_result.confidence.value)
                corners = _restore_cached_trajectory(machine, geometry_cache)
                laser_roi = _laser_roi(corners, machine.mode)

        laser = None
        if laser_roi is not None:
            blobs = frame.find_blobs(
                config.RED_LAB_THRESHOLDS,
                roi=laser_roi,
                area_threshold=config.LASER_MIN_PIXELS,
                pixels_threshold=config.LASER_MIN_PIXELS,
            )
            laser = choose_blob(
                blobs,
                previous_laser,
                config.LASER_MIN_PIXELS,
                config.LASER_MAX_PIXELS,
                config.LASER_MAX_JUMP_PX,
                config.LASER_MIN_ROUNDNESS,
            )
            if laser is not None:
                previous_laser = laser

        for message_type, index, data0, data1 in machine.update(now_ms, laser):
            serial.write(encode_frame(message_type, index, data0, data1))

        if _frame_due(frame_index, config.DISPLAY_INTERVAL_FRAMES):
            if machine.state == RunState.ACQUIRE:
                _draw_rectangles(frame, rectangle_candidates, image.COLOR_YELLOW)
            _draw_cached_geometry(frame, geometry_cache, image)
            if machine.waypoints:
                for index, point in enumerate(machine.waypoints):
                    active = index == machine.current_index and machine.state != RunState.DONE
                    frame.draw_circle(point[0], point[1], 3, image.COLOR_YELLOW if active else image.COLOR_BLUE, -1)
            if laser is not None:
                frame.draw_circle(laser[0], laser[1], 5, image.COLOR_RED, 2)
            frame.draw_string(
                2,
                2,
                "{} RECT {} CAND {}".format(
                    "{} {}".format(machine.mode.name, machine.state.value),
                    machine.rect_confidence or "-",
                    len(rectangle_candidates),
                ),
                image.COLOR_GREEN,
            )
            frame.draw_string(2, 20, "FPS {:.1f}".format(fps), image.COLOR_GREEN)
            _draw_buttons(frame, image, machine.mode)
            disp.show(frame)

        frame_index += 1


if __name__ == "__main__":
    run()
