"""A4 靶纸内外沿优先、单矩形退化的跨帧锁定器。"""

from enum import Enum

from trajectory import (
    aspect_ratio,
    average_corners,
    order_corners_clockwise,
    polygon_area,
)


class RectConfidence(Enum):
    HIGH = "HIGH"
    LOW = "LOW"


class LockResult:
    def __init__(self, corners, confidence):
        self.corners = corners
        self.confidence = confidence


def _point_in_convex(point, polygon):
    signs = []
    for index, start in enumerate(polygon):
        end = polygon[(index + 1) % 4]
        cross = (end[0] - start[0]) * (point[1] - start[1]) - (end[1] - start[1]) * (point[0] - start[0])
        if cross:
            signs.append(cross > 0)
    return not signs or all(sign == signs[0] for sign in signs)


class RectangleLock:
    def __init__(
        self,
        frame_width,
        frame_height,
        stable_frames,
        pair_miss_frames,
        max_corner_jitter,
        min_area_ratio,
        max_area_change_ratio,
        aspect_ratio,
        aspect_tolerance,
        min_inner_outer_area_ratio=0.50,
        prefer_pair=True,
        target_area_ratio=None,
    ):
        self.frame_area = frame_width * frame_height
        self.stable_frames = stable_frames
        self.pair_miss_frames = pair_miss_frames
        self.max_corner_jitter = max_corner_jitter
        self.min_area_ratio = min_area_ratio
        self.max_area_change_ratio = max_area_change_ratio
        self.expected_aspect_ratio = aspect_ratio
        self.aspect_tolerance = aspect_tolerance
        self.min_inner_outer_area_ratio = min_inner_outer_area_ratio
        self.prefer_pair = prefer_pair
        self.target_area_ratio = target_area_ratio
        self.pair_misses = 0
        self._samples = []
        self._sample_confidence = None
        self._locked = None

    def _valid_candidates(self, rectangles):
        valid = []
        for corners in rectangles:
            ordered = order_corners_clockwise(corners)
            area = polygon_area(ordered)
            ratio = aspect_ratio(ordered)
            if area < self.frame_area * self.min_area_ratio:
                continue
            if abs(ratio - self.expected_aspect_ratio) > self.aspect_tolerance:
                continue
            valid.append((area, ordered))
        return valid

    def filtered_candidates(self, rectangles):
        return [corners for _, corners in self._valid_candidates(rectangles)]

    def _best_pair(self, candidates):
        pairs = []
        for outer_area, outer in candidates:
            for inner_area, inner in candidates:
                if inner_area >= outer_area:
                    continue
                if inner_area / outer_area < self.min_inner_outer_area_ratio:
                    continue
                if not all(_point_in_convex(point, outer) for point in inner):
                    continue
                centerline = average_corners([outer, inner])
                pairs.append((outer_area - inner_area, centerline))
        return min(pairs, default=(None, None), key=lambda item: item[0])[1]

    def _stable_with_previous(self, corners):
        previous = self._samples[-1]
        jitter = max(
            max(abs(current[0] - old[0]), abs(current[1] - old[1]))
            for current, old in zip(corners, previous)
        )
        previous_area = polygon_area(previous)
        area_change = abs(polygon_area(corners) - previous_area) / previous_area
        return jitter <= self.max_corner_jitter and area_change <= self.max_area_change_ratio

    def _accumulate(self, corners, confidence):
        if self._sample_confidence != confidence or (self._samples and not self._stable_with_previous(corners)):
            self._samples = []
        self._sample_confidence = confidence
        self._samples.append(corners)
        if len(self._samples) < self.stable_frames:
            return None
        self._locked = LockResult(average_corners(self._samples), confidence)
        return self._locked

    def observe(self, rectangles):
        if self._locked is not None:
            return self._locked

        candidates = self._valid_candidates(rectangles)
        pair = self._best_pair(candidates) if self.prefer_pair else None
        if pair is not None:
            self.pair_misses = 0
            return self._accumulate(pair, RectConfidence.HIGH)

        self.pair_misses += 1
        if self.pair_misses < self.pair_miss_frames or not candidates:
            self._samples = []
            self._sample_confidence = None
            return None
        if self.target_area_ratio is None:
            selected = max(candidates, key=lambda item: item[0])[1]
        else:
            target_area = self.frame_area * self.target_area_ratio
            selected = min(candidates, key=lambda item: abs(item[0] - target_area))[1]
        return self._accumulate(selected, RectConfidence.LOW)
