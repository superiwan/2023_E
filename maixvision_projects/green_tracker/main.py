"""MaixVision 入口：绿色自动追踪系统。"""

import config
from tracker_state import GreenTrackerState
from shared.blob_tracking import choose_blob
from shared.protocol import encode_frame


def run():
    from maix import app, camera, display, err, image, pinmap, time, uart

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
    tracker = GreenTrackerState(
        config.TRACK_SUCCESS_RADIUS_PX,
        config.TRACK_STABLE_FRAMES,
        config.STATUS_INTERVAL_MS,
    )
    previous_red = None
    previous_green = None
    last_frame_ms = time.ticks_ms()
    fps = 0.0

    while not app.need_exit():
        frame = cam.read()
        now_ms = time.ticks_ms()
        frame_ms = max(1, now_ms - last_frame_ms)
        last_frame_ms = now_ms
        fps = fps * 0.8 + 200.0 / frame_ms

        blobs = frame.find_blobs(
            [config.RED_LAB_THRESHOLD, config.GREEN_LAB_THRESHOLD],
            area_threshold=config.BLOB_MIN_PIXELS,
            pixels_threshold=config.BLOB_MIN_PIXELS,
        )
        red = choose_blob(
            [blob for blob in blobs if blob.code() & 0x01],
            previous_red,
            config.BLOB_MIN_PIXELS,
            config.BLOB_MAX_PIXELS,
            config.BLOB_MAX_JUMP_PX,
            config.BLOB_MIN_ROUNDNESS,
        )
        green = choose_blob(
            [blob for blob in blobs if blob.code() & 0x02],
            previous_green,
            config.BLOB_MIN_PIXELS,
            config.BLOB_MAX_PIXELS,
            config.BLOB_MAX_JUMP_PX,
            config.BLOB_MIN_ROUNDNESS,
        )
        if red is not None:
            previous_red = red
            frame.draw_circle(red[0], red[1], 5, image.COLOR_RED, 2)
        if green is not None:
            previous_green = green
            frame.draw_circle(green[0], green[1], 5, image.COLOR_GREEN, 2)

        for message_type, index, data0, data1 in tracker.update(now_ms, red, green):
            serial.write(encode_frame(message_type, index, data0, data1))

        if red is None and green is None:
            state = "LOST BOTH"
        elif red is None:
            state = "LOST RED"
        elif green is None:
            state = "LOST GREEN"
        else:
            state = "TRACKED" if tracker.tracked else "TRACKING"
        frame.draw_string(2, 2, state, image.COLOR_GREEN if tracker.tracked else image.COLOR_YELLOW)
        frame.draw_string(2, 20, "FPS {:.1f}".format(fps), image.COLOR_GREEN)
        disp.show(frame)


if __name__ == "__main__":
    run()
