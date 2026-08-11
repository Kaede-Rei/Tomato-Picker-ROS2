import numpy as np

from tomato_picker_gui.selection import preprocess_mask


def depth_to_point(u: int, v: int, depth_m: float, k: np.ndarray) -> tuple[float, float, float]:
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    return ((u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m)


def _trimmed_median(values: np.ndarray, ratio: float) -> float:
    values = np.sort(values.astype(np.float64))
    if values.size == 0:
        return 0.0
    ratio = min(0.45, max(0.0, float(ratio)))
    trim = int(values.size * ratio)
    if trim > 0 and values.size > 2 * trim:
        values = values[trim:-trim]
    return float(np.median(values))


def valid_depth(depth_mm: np.ndarray, u: int, v: int, mask: np.ndarray, kernel_size: int, trim_ratio: float, close_kernel: int, open_kernel: int) -> float:
    if depth_mm.shape[:2] != mask.shape[:2]:
        raise RuntimeError("aligned depth and color image sizes differ")

    mask = preprocess_mask(mask, close_kernel, open_kernel)
    kernel_size = max(3, int(kernel_size))
    if kernel_size % 2 == 0:
        kernel_size += 1
    half = kernel_size // 2
    h, w = depth_mm.shape[:2]
    x0, x1 = max(0, u - half), min(w, u + half + 1)
    y0, y1 = max(0, v - half), min(h, v + half + 1)

    roi_depth = depth_mm[y0:y1, x0:x1]
    roi_mask = mask[y0:y1, x0:x1]
    values = roi_depth[(roi_mask > 0) & np.isfinite(roi_depth) & (roi_depth > 0)]
    if values.size >= 3:
        return _trimmed_median(values, trim_ratio)

    values = depth_mm[(mask > 0) & np.isfinite(depth_mm) & (depth_mm > 0)]
    return _trimmed_median(values, trim_ratio)
