"""将触摸按键转换为运行状态命令。"""

from shared.protocol import MessageType


class TouchControls:
    def __init__(self, width, height, button_height, debounce_ms):
        self.width = width
        self.top = height - button_height
        self.debounce_ms = debounce_ms
        self._was_pressed = False
        self._last_emit_ms = None

    def update(self, x, y, pressed, now_ms):
        rising_edge = pressed and not self._was_pressed
        self._was_pressed = pressed
        if not rising_edge or y < self.top:
            return None
        if self._last_emit_ms is not None and now_ms - self._last_emit_ms < self.debounce_ms:
            return None

        third = self.width / 3
        if x < third:
            command = MessageType.REACQUIRE
        elif x < third * 2:
            command = MessageType.START_RESUME
        elif x < self.width:
            command = MessageType.PAUSE
        else:
            return None
        self._last_emit_ms = now_ms
        return command

