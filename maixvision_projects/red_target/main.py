"""MaixVision 入口：红色运动目标系统。"""

import config
from rectangle_lock import RectangleLock
from touch_controls import TouchControls
from tracker_state import RedTargetStateMachine, RunState
from trajectory import generate_waypoints
from shared.blob_tracking import choose_blob
from shared.protocol import FrameParser, MessageType, encode_frame


def _new_rectangle_lock():
    return RectangleLock(
        config.FRAME_WIDTH,
        config.FRAME_HEIGHT,
        config.RECT_STABLE_FRAMES,
        config.PAIR_MISS_FRAMES,
        config.MAX_CORNER_JITTER,
        config.MIN_RECT_AREA_RATIO,
        config.MAX_RECT_AREA_CHANGE_RATIO,
        config.A4_ASPECT_RATIO,
        config.A4_ASPECT_TOLERANCE,
        config.MIN_INNER_OUTER_AREA_RATIO,
    )


def _reset_acquisition():
    return _new_rectangle_lock(), None, None


def _scale_corners(corners, source_width, source_height, target_width, target_height):
    scale_x = target_width / source_width
    scale_y = target_height / source_height
    return [(round(x * scale_x), round(y * scale_y)) for x, y in corners]


def _expanded_roi(corners):
    margin = config.RECT_ROI_MARGIN
    x0 = max(0, min(point[0] for point in corners) - margin)
    y0 = max(0, min(point[1] for point in corners) - margin)
    x1 = min(config.FRAME_WIDTH, max(point[0] for point in corners) + margin + 1)
    y1 = min(config.FRAME_HEIGHT, max(point[1] for point in corners) + margin + 1)
    return [x0, y0, x1 - x0, y1 - y0]


def _draw_buttons(frame, image):
    top = config.FRAME_HEIGHT - 60
    width = config.FRAME_WIDTH // 3
    for index, label in enumerate(("REACQUIRE", "START/RESUME", "PAUSE")):
        x = index * width
        frame.draw_rect(x, top, width, 60, image.COLOR_WHITE, 2)
        frame.draw_string(x + 8, top + 20, label, image.COLOR_WHITE)


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
    controls = TouchControls(config.FRAME_WIDTH, config.FRAME_HEIGHT, 60, config.TOUCH_DEBOUNCE_MS)
    parser = FrameParser()
    machine = RedTargetStateMachine(config.ARRIVE_RADIUS, config.STABLE_FRAMES, config.WAYPOINT_SEND_INTERVAL_MS)
    rect_lock = _new_rectangle_lock()
    laser_roi = None
    previous_laser = None
    last_frame_ms = time.ticks_ms()
    fps = 0.0

    while not app.need_exit():
        frame = cam.read()
        now_ms = time.ticks_ms()
        frame_ms = max(1, now_ms - last_frame_ms)
        last_frame_ms = now_ms
        fps = fps * 0.8 + 200.0 / frame_ms

        incoming = serial.read()
        if incoming:
            for message_type, _, _, _ in parser.feed(incoming):
                if message_type in (MessageType.START_RESUME, MessageType.PAUSE, MessageType.REACQUIRE):
                    machine.handle_command(message_type)
                    if message_type == MessageType.REACQUIRE:
                        rect_lock, laser_roi, previous_laser = _reset_acquisition()

        x, y, pressed = touch.read()
        command = controls.update(x, y, pressed, now_ms)
        if command is not None:
            machine.handle_command(command)
            serial.write(encode_frame(command))
            if command == MessageType.REACQUIRE:
                rect_lock, laser_roi, previous_laser = _reset_acquisition()

        if machine.state == RunState.ACQUIRE:
            detection_frame = frame.resize(config.RECT_DETECTION_WIDTH, config.RECT_DETECTION_HEIGHT)
            rectangles = [
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
            lock_result = rect_lock.observe(rectangles)
            if lock_result is not None:
                machine.set_trajectory(
                    generate_waypoints(lock_result.corners, config.EDGE_SEGMENTS),
                    lock_result.confidence.value,
                )
                laser_roi = _expanded_roi(lock_result.corners)

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

        if machine.waypoints:
            for index, point in enumerate(machine.waypoints):
                active = index == machine.current_index and machine.state != RunState.DONE
                frame.draw_circle(point[0], point[1], 3, image.COLOR_YELLOW if active else image.COLOR_BLUE, -1)
        if laser is not None:
            frame.draw_circle(laser[0], laser[1], 5, image.COLOR_RED, 2)
        frame.draw_string(2, 2, "{} RECT {}".format(machine.state.value, machine.rect_confidence or "-"), image.COLOR_GREEN)
        frame.draw_string(2, 20, "FPS {:.1f}".format(fps), image.COLOR_GREEN)
        _draw_buttons(frame, image)
        disp.show(frame)


if __name__ == "__main__":
    run()
