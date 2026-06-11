import numpy as np


def get_median_depth(depth_image, u, v, window_size=7, depth_scale=0.001):
    """
    Get robust depth value around pixel (u, v).

    For RealSense ROS depth image:
    - usually uint16 in millimeters
    - depth_scale=0.001 converts mm to meters
    """

    height, width = depth_image.shape[:2]

    half = window_size // 2

    u_min = max(0, u - half)
    u_max = min(width, u + half + 1)

    v_min = max(0, v - half)
    v_max = min(height, v + half + 1)

    patch = depth_image[v_min:v_max, u_min:u_max].astype(np.float32)

    valid_depth = patch[patch > 0]

    if valid_depth.size == 0:
        return None

    depth_m = np.median(valid_depth) * depth_scale

    return float(depth_m)


def pixel_to_3d(u, v, depth_m, fx, fy, cx, cy):
    """
    Convert pixel coordinate and depth to 3D camera coordinate.

    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    Z = depth
    """

    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m

    return x, y, z
