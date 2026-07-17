"""缓存屏幕正方形与 A4 矩形，供模式切换复用。"""

from tracker_state import RedMode


class GeometryCache:
    def __init__(self):
        self._items = {}

    @staticmethod
    def _key(mode):
        mode = RedMode(mode)
        if mode in (RedMode.ORIGIN, RedMode.SCREEN_BORDER):
            return RedMode.SCREEN_BORDER
        return RedMode.A4_BORDER

    def get(self, mode):
        return self._items.get(self._key(mode))

    def store(self, mode, corners, confidence):
        self._items[self._key(mode)] = (list(corners), confidence)

    def clear(self, mode):
        self._items.pop(self._key(mode), None)
