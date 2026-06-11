import numpy as np
import open3d as o3d


def bbox_depth_to_pointcloud(
    rgb_image_bgr,
    depth_image,
    bbox,
    fx,
    fy,
    cx,
    cy,
    depth_scale=0.001,
    stride=2,
    max_depth_m=2.0,
):
    """
    Convert pixels inside YOLO bounding box into a colored Open3D point cloud.

    rgb_image_bgr: OpenCV BGR image
    depth_image: aligned depth image, usually uint16 in mm
    bbox: [x1, y1, x2, y2]
    fx, fy, cx, cy: camera intrinsics
    depth_scale: mm to meter conversion
    stride: skip pixels for lighter point cloud
    max_depth_m: ignore far/noisy points
    """

    x1, y1, x2, y2 = bbox

    height, width = depth_image.shape[:2]

    x1 = max(0, int(x1))
    y1 = max(0, int(y1))
    x2 = min(width - 1, int(x2))
    y2 = min(height - 1, int(y2))

    points = []
    colors = []

    for v in range(y1, y2, stride):
        for u in range(x1, x2, stride):
            depth_raw = depth_image[v, u]

            if depth_raw == 0:
                continue

            z = float(depth_raw) * depth_scale

            if z <= 0.0 or z > max_depth_m:
                continue

            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            b, g, r = rgb_image_bgr[v, u]
            color = [r / 255.0, g / 255.0, b / 255.0]

            points.append([x, y, z])
            colors.append(color)

    pcd = o3d.geometry.PointCloud()

    if len(points) == 0:
        return pcd

    pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
    pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))

    return pcd


def voxel_downsample_pointcloud(pcd, voxel_size=0.005):
    """
    Downsample point cloud using voxel grid.
    Smaller voxel_size keeps more detail.
    """

    if len(pcd.points) == 0:
        return pcd

    return pcd.voxel_down_sample(voxel_size=voxel_size)
