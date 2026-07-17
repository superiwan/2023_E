"""红色运动目标系统的航点与运行状态机。"""

from enum import Enum, IntEnum

from shared.protocol import MessageType


class RunState(Enum):
    ACQUIRE = "ACQUIRE"
    WAIT_START = "WAIT_START"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    DONE = "DONE"


class RedMode(IntEnum):
    ORIGIN = 0
    SCREEN_BORDER = 1
    A4_BORDER = 2


class RedTargetStateMachine:
    def __init__(self, arrive_radius, stable_frames, waypoint_interval_ms):
        self.arrive_radius_squared = arrive_radius * arrive_radius
        self.required_stable_frames = stable_frames
        self.waypoint_interval_ms = waypoint_interval_ms
        self.mode = RedMode.A4_BORDER
        self.state = RunState.ACQUIRE
        self.waypoints = []
        self.rect_confidence = None
        self.current_index = 0
        self.stable_count = 0
        self._last_waypoint_ms = None
        self._start_requested = False

    @property
    def current_waypoint(self):
        if not self.waypoints or self.state in (RunState.ACQUIRE, RunState.DONE):
            return None
        return self.waypoints[self.current_index]

    def set_trajectory(self, waypoints, confidence):
        if self.state != RunState.ACQUIRE:
            return
        if not waypoints:
            raise ValueError("航点不能为空")
        self.waypoints = list(waypoints)
        self.rect_confidence = confidence
        self.current_index = 0
        self.stable_count = 0
        self._last_waypoint_ms = None
        self.state = RunState.RUNNING if self._start_requested else RunState.WAIT_START
        self._start_requested = False

    def _reset_acquisition(self):
        self.state = RunState.ACQUIRE
        self.waypoints = []
        self.rect_confidence = None
        self.current_index = 0
        self.stable_count = 0
        self._last_waypoint_ms = None
        self._start_requested = False

    def handle_command(self, command, index=0):
        command = MessageType(command)
        if command == MessageType.SELECT_MODE:
            try:
                self.mode = RedMode(index)
            except ValueError:
                return False
            self._reset_acquisition()
            return True
        elif command == MessageType.REACQUIRE:
            self._reset_acquisition()
            return True
        elif command == MessageType.START_RESUME and self.state == RunState.ACQUIRE:
            self._start_requested = True
        elif command == MessageType.START_RESUME and self.state in (RunState.WAIT_START, RunState.PAUSED):
            self.state = RunState.RUNNING
            self.stable_count = 0
            self._last_waypoint_ms = None
        elif command == MessageType.PAUSE and self.state == RunState.RUNNING:
            self.state = RunState.PAUSED
            self.stable_count = 0
        elif command == MessageType.PAUSE and self.state == RunState.ACQUIRE:
            self._start_requested = False
        return False

    def update(self, now_ms, laser):
        if self.state != RunState.RUNNING:
            return []

        target = self.current_waypoint
        messages = []
        if self._last_waypoint_ms is None or now_ms - self._last_waypoint_ms >= self.waypoint_interval_ms:
            messages.append((MessageType.WAYPOINT, self.current_index, target[0], target[1]))
            self._last_waypoint_ms = now_ms

        if laser is None:
            self.stable_count = 0
            messages.append((MessageType.SPOT_LOST, self.current_index, 0, 0))
            return messages

        dx = target[0] - laser[0]
        dy = target[1] - laser[1]
        messages.append((MessageType.VISION_ERROR, self.current_index, dx, dy))
        if dx * dx + dy * dy <= self.arrive_radius_squared:
            self.stable_count += 1
        else:
            self.stable_count = 0

        if self.stable_count < self.required_stable_frames:
            return messages

        self.stable_count = 0
        self.current_index += 1
        self._last_waypoint_ms = None
        if self.current_index == len(self.waypoints):
            self.state = RunState.DONE
            messages.append((MessageType.COMPLETE, 0xFF, 0, 0))
        return messages
