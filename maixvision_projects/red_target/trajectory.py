"""A4 靶纸角点规范化与顺时针航点规划。"""

from math import atan2, hypot


def order_corners_clockwise(corners):
    if len(corners) != 4:
        raise ValueError("A4 靶纸必须恰好有四个角点")
    points = [(int(point[0]), int(point[1])) for point in corners]
    center_x = sum(point[0] for point in points) / 4
    center_y = sum(point[1] for point in points) / 4
    points.sort(key=lambda point: atan2(point[1] - center_y, point[0] - center_x))
    start = min(range(4), key=lambda index: points[index][0] + points[index][1])
    return points[start:] + points[:start]


def polygon_area(corners):
    points = order_corners_clockwise(corners)
    twice_area = 0
    for index, point in enumerate(points):
        next_point = points[(index + 1) % 4]
        twice_area += point[0] * next_point[1] - next_point[0] * point[1]
    return abs(twice_area) / 2


def aspect_ratio(corners):
    points = order_corners_clockwise(corners)
    lengths = [
        hypot(
            points[(index + 1) % 4][0] - points[index][0],
            points[(index + 1) % 4][1] - points[index][1],
        )
        for index in range(4)
    ]
    opposite_averages = ((lengths[0] + lengths[2]) / 2, (lengths[1] + lengths[3]) / 2)
    short_side = min(opposite_averages)
    if short_side == 0:
        return 0
    return max(opposite_averages) / short_side


def average_corners(samples):
    ordered_samples = [order_corners_clockwise(sample) for sample in samples]
    if not ordered_samples:
        raise ValueError("角点样本不能为空")
    return [
        (
            round(sum(sample[index][0] for sample in ordered_samples) / len(ordered_samples)),
            round(sum(sample[index][1] for sample in ordered_samples) / len(ordered_samples)),
        )
        for index in range(4)
    ]


def generate_waypoints(corners, segments_per_edge=5):
    if segments_per_edge <= 0:
        raise ValueError("每边分段数必须大于零")
    ordered = order_corners_clockwise(corners)
    points = []
    for edge_index, start in enumerate(ordered):
        end = ordered[(edge_index + 1) % 4]
        for step in range(1, segments_per_edge + 1):
            points.append(
                (
                    round(start[0] + (end[0] - start[0]) * step / segments_per_edge),
                    round(start[1] + (end[1] - start[1]) * step / segments_per_edge),
                )
            )
    return points

