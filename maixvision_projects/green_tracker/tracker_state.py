"""绿色自动追踪系统的误差和追踪成功状态。"""

from shared.protocol import MessageType


class GreenTrackerState:
    def __init__(self, success_radius, stable_frames, status_interval_ms):
        self.success_radius_squared = success_radius * success_radius
        self.required_stable_frames = stable_frames
        self.status_interval_ms = status_interval_ms
        self.tracked = False
        self.stable_count = 0
        self._last_status_ms = None

    def _status_due(self, now_ms, changed):
        return changed or self._last_status_ms is None or now_ms - self._last_status_ms >= self.status_interval_ms

    def update(self, now_ms, red, green):
        messages = []
        previous_tracked = self.tracked
        if red is None or green is None:
            lost_mask = (1 if red is None else 0) | (2 if green is None else 0)
            messages.append((MessageType.SPOT_LOST, lost_mask, 0, 0))
            self.stable_count = 0
            self.tracked = False
        else:
            dx = red[0] - green[0]
            dy = red[1] - green[1]
            messages.append((MessageType.VISION_ERROR, 0, dx, dy))
            if dx * dx + dy * dy <= self.success_radius_squared:
                self.stable_count += 1
                if self.stable_count >= self.required_stable_frames:
                    self.tracked = True
            else:
                self.stable_count = 0
                self.tracked = False

        changed = self.tracked != previous_tracked
        if self._status_due(now_ms, changed):
            messages.append((MessageType.TRACK_STATUS, 0, 1 if self.tracked else 0, 0))
            self._last_status_ms = now_ms
        return messages
