from collections import deque
from typing import Optional, Tuple

import numpy as np


def build_polygon_mask(width: int, height: int, points: list[dict]) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    if len(points) < 3:
        return mask

    poly = np.array([[float(p["x"]), float(p["y"])] for p in points], dtype=np.float64)
    xs = np.arange(width, dtype=np.float64) + 0.5
    ys = np.arange(height, dtype=np.float64) + 0.5
    xx, yy = np.meshgrid(xs, ys)
    inside = np.zeros((height, width), dtype=bool)
    xj, yj = poly[-1]
    for xi, yi in poly:
        cross_y = (yi > yy) != (yj > yy)
        x_intersect = (xj - xi) * (yy - yi) / ((yj - yi) + 1.0e-12) + xi
        inside ^= cross_y & (xx < x_intersect)
        xj, yj = xi, yi
    mask[inside] = 255
    return mask


def _kernel(size: int) -> np.ndarray:
    size = max(1, int(size))
    if size % 2 == 0:
        size += 1
    r = size // 2
    if r == 0:
        return np.ones((1, 1), dtype=bool)
    yy, xx = np.ogrid[-r : r + 1, -r : r + 1]
    return (xx * xx + yy * yy) <= r * r


def _erode(mask: np.ndarray, kernel: np.ndarray) -> np.ndarray:
    h, w = mask.shape
    kh, kw = kernel.shape
    ph, pw = kh // 2, kw // 2
    padded = np.pad(mask, ((ph, ph), (pw, pw)), mode="constant", constant_values=False)
    out = np.ones_like(mask, dtype=bool)
    ys, xs = np.where(kernel)
    for dy, dx in zip(ys, xs):
        out &= padded[dy : dy + h, dx : dx + w]
    return out


def _dilate(mask: np.ndarray, kernel: np.ndarray) -> np.ndarray:
    h, w = mask.shape
    kh, kw = kernel.shape
    ph, pw = kh // 2, kw // 2
    padded = np.pad(mask, ((ph, ph), (pw, pw)), mode="constant", constant_values=False)
    out = np.zeros_like(mask, dtype=bool)
    ys, xs = np.where(kernel)
    for dy, dx in zip(ys, xs):
        out |= padded[dy : dy + h, dx : dx + w]
    return out


def preprocess_mask(mask: np.ndarray, close_kernel: int, open_kernel: int) -> np.ndarray:
    binary = mask > 0
    close_k = _kernel(close_kernel)
    open_k = _kernel(open_kernel)
    binary = _erode(_dilate(binary, close_k), close_k)
    binary = _dilate(_erode(binary, open_k), open_k)
    return binary.astype(np.uint8) * 255


def _largest_component(mask: np.ndarray) -> np.ndarray:
    binary = mask > 0
    h, w = binary.shape
    visited = np.zeros_like(binary, dtype=bool)
    best: list[tuple[int, int]] = []
    for sy, sx in zip(*np.where(binary)):
        if visited[sy, sx]:
            continue
        queue = deque([(int(sy), int(sx))])
        visited[sy, sx] = True
        current: list[tuple[int, int]] = []
        while queue:
            y, x = queue.popleft()
            current.append((y, x))
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and binary[ny, nx] and not visited[ny, nx]:
                    visited[ny, nx] = True
                    queue.append((ny, nx))
        if len(current) > len(best):
            best = current
    out = np.zeros_like(mask, dtype=np.uint8)
    for y, x in best:
        out[y, x] = 255
    return out


def _skeleton(mask: np.ndarray) -> np.ndarray:
    work = mask > 0
    result = np.zeros_like(work, dtype=bool)
    cross = np.array([[False, True, False], [True, True, True], [False, True, False]], dtype=bool)
    while np.any(work):
        eroded = _erode(work, cross)
        opened = _dilate(eroded, cross)
        result |= work & ~opened
        work = eroded
    return result


def select_pixel(width: int, height: int, points: list[dict], method: str, close_kernel: int, open_kernel: int) -> tuple[tuple[float, float], np.ndarray]:
    mask = build_polygon_mask(width, height, points)
    mask = _largest_component(preprocess_mask(mask, close_kernel, open_kernel))
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        raise RuntimeError("ROI mask is empty")

    center = np.array([float(xs.mean()), float(ys.mean())])
    if method == "centroid":
        return (float(center[0]), float(center[1])), mask

    skeleton = _skeleton(mask)
    sy, sx = np.where(skeleton)
    if len(sx) == 0:
        return (float(center[0]), float(center[1])), mask
    distance = (sx - center[0]) ** 2 + (sy - center[1]) ** 2
    index = int(np.argmin(distance))
    return (float(sx[index]), float(sy[index])), mask
