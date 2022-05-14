import numpy as np
import open3d as o3d
import cv2


def load_transform(t_array):
    transform = np.identity(4, dtype = float)
    transform[0:3,0:3] = t_array[:9].reshape(3,3)
    transform[0:3,3] = t_array[9:]
    return transform

def cam2world(points, rig2cam, rig2world):
    homog_points = np.hstack((points, np.ones((points.shape[0], 1))))
    cam2world_transform = rig2world @ np.linalg.inv(rig2cam)
    world_points = cam2world_transform @ homog_points.T
    return world_points.T[:, :3], cam2world_transform
    
def cam2rig(points, rig2cam):
    homog_points = np.hstack((points, np.ones((points.shape[0], 1))))
    cam2rig_transform = np.linalg.inv(rig2cam)
    rig_points = cam2rig_transform @ homog_points.T
    return rig_points.T[:, :3], cam2rig_transform

def save_ply(output_path, points, rgb=None, cam2world_transform=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if rgb is not None:
        pcd.colors = o3d.utility.Vector3dVector(rgb)
    pcd.estimate_normals()
    if cam2world_transform is not None:
        # Camera center
        camera_center = (cam2world_transform) @ np.array([0, 0, 0, 1])
        o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd, camera_center[:3])

    o3d.io.write_point_cloud(output_path, pcd)

def get_points_in_cam_space(img, lut):
    img = np.tile(img.flatten().reshape((-1, 1)), (1, 3))
    points = img * lut
    remove_ids = np.where(np.sum(points, axis=1) < 1e-6)[0]
    points = np.delete(points, remove_ids, axis=0)
    points /= 1000.
    return points

def match_timestamp(target, all_timestamps):
    return np.argmin([abs(x - target) for x in all_timestamps])

def project_on_pv(points, pv_img, pv2world_transform, focal_length, principal_point):
    height, width, _ = pv_img.shape

    homog_points = np.hstack((points, np.ones(len(points)).reshape((-1, 1))))
    world2pv_transform = np.linalg.inv(pv2world_transform)
    points_pv = (world2pv_transform @ homog_points.T).T[:, :3]

    intrinsic_matrix = np.array([[focal_length[0], 0, principal_point[0]], [
        0, focal_length[1], principal_point[1]], [0, 0, 1]])
    rvec = np.zeros(3)
    tvec = np.zeros(3)
    xy, _ = cv2.projectPoints(points_pv, rvec, tvec, intrinsic_matrix, None)
    xy = np.squeeze(xy)
    xy = np.around(xy).astype(int)

    rgb = np.zeros_like(points)
    width_check = np.logical_and(0 <= xy[:, 0], xy[:, 0] < width)
    height_check = np.logical_and(0 <= xy[:, 1], xy[:, 1] < height)
    valid_ids = np.where(np.logical_and(width_check, height_check))[0]

    z = points_pv[valid_ids, 2]
    xy = xy[valid_ids, :]

    depth_image = np.zeros((height, width), dtype=np.float32)
    for i, p in enumerate(xy):
        depth_image[p[1], p[0]] = z[i]
        # print(z[i])

    colors = pv_img[xy[:, 1], xy[:, 0], :]
    rgb[valid_ids, :] = colors[:, ::-1] / 255.

    return rgb, depth_image


def project_on_depth(points, rgb, intrinsic_matrix, width, height):
    rvec = np.zeros(3)
    tvec = np.zeros(3)
    xy, _ = cv2.projectPoints(points, rvec, tvec, intrinsic_matrix, None)
    xy = np.squeeze(xy)
    xy = np.around(xy).astype(int)

    width_check = np.logical_and(0 <= xy[:, 0], xy[:, 0] < width)
    height_check = np.logical_and(0 <= xy[:, 1], xy[:, 1] < height)
    valid_ids = np.where(np.logical_and(width_check, height_check))[0]
    xy = xy[valid_ids, :]

    z = points[valid_ids, 2]
    depth_image = np.zeros((height, width))
    image = np.zeros((height, width, 3))
    rgb = rgb[valid_ids, :]
    rgb = rgb[:, ::-1]
    for i, p in enumerate(xy):
        depth_image[p[1], p[0]] = z[i]
        image[p[1], p[0]] = rgb[i]

    image = image * 255.

    return image, depth_image