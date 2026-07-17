"""将触摸按键转换为运行状态命令。"""

from shared.protocol import MessageType
from tracker_state import RedMode


class TouchControls:
    def __init__(self, width, height, runtime_button_height, mode_top, mode_height, debounce_ms):
        self.width = width
        self.runtime_top = height - runtime_button_height
        self.mode_top = mode_top
        self.mode_bottom = mode_top + mode_height
        self.debounce_ms = debounce_ms
        self._was_pressed = False
        self._last_emit_ms = None

    def update(self, x, y, pressed, now_ms):
        rising_edge = pressed and not self._was_pressed
        self._was_pressed = pressed
        if not rising_edge:
            return None
        if self._last_emit_ms is not None and now_ms - self._last_emit_ms < self.debounce_ms:
            return None

        third = self.width / 3
        if self.mode_top <= y < self.mode_bottom:
            if x < third:
                event = (MessageType.SELECT_MODE, RedMode.ORIGIN)
            elif x < third * 2:
                event = (MessageType.SELECT_MODE, RedMode.SCREEN_BORDER)
            elif x < self.width:
                event = (MessageType.SELECT_MODE, RedMode.A4_BORDER)
            else:
                return None
        elif y >= self.runtime_top:
            if x < third:
                event = (MessageType.REACQUIRE, 0)
            elif x < third * 2:
                event = (MessageType.START_RESUME, 0)
            elif x < self.width:
                event = (MessageType.PAUSE, 0)
            else:
                return None
        else:
            return None
        self._last_emit_ms = now_ms
        return event
