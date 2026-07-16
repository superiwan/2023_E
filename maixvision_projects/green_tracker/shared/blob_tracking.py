"""红绿应用共用的光斑候选筛选与跨帧连续性判断。"""


def choose_blob(
    blobs,
    previous,
    min_pixels,
    max_pixels,
    max_jump_px,
    min_roundness=0.0,
):
    candidates = [
        blob
        for blob in blobs
        if min_pixels <= blob.pixels() <= max_pixels
        and blob.roundness() >= min_roundness
    ]
    if not candidates:
        return None
    if previous is None:
        selected = max(candidates, key=lambda blob: blob.pixels())
        return selected.cx(), selected.cy()

    max_jump_squared = max_jump_px * max_jump_px
    nearby = [
        blob
        for blob in candidates
        if (blob.cx() - previous[0]) ** 2 + (blob.cy() - previous[1]) ** 2 <= max_jump_squared
    ]
    if not nearby:
        return None
    selected = min(
        nearby,
        key=lambda blob: (blob.cx() - previous[0]) ** 2 + (blob.cy() - previous[1]) ** 2,
    )
    return selected.cx(), selected.cy()

