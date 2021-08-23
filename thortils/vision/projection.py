import math
import numpy as np
from thortils.utils import to_rad, R_euler, R_quat, T
import open3d as o3d

def extrinsic(camera_pose):
    """
    Given a camera_pose, returns a 4x4 extrinsic matrix that can transform
    a point in the world frame to the camera frame.

    The pose = (position, rotation), where position = (x,y,z)
    and rotation should be of length 3. It is specified
    by euler angles (degrees), same format as ai2thor rotation in pose.

    Note that we expect the camera_pose to be in Ai2thor's format,
    that means the rotation if of length 3 is (pitch, yaw, roll).

    The extrinsic matrix is a 2D array:

    R(3x3) | t(2x1)
    --------------
    0(3x1) | 1
    """
    pos, rot = camera_pose
    pitch, yaw, roll = rot   # ai2thor convention
    Rmat = R_euler(pitch, yaw, roll, affine=True)
    return np.dot(T(*pos), Rmat)     # first rotate, then translate

def extrinsic_inv(camera_pose):
    # Somehow simply taking the inverse of extrinsic(camera_pose) does
    # not work when stitching point clouds using open3D. My thought process
    # of the way that works:
    #
    # Here, we are asked to produce a inverse extrinsic that transforms points
    # from the camera frame to the world frame. We think about this in a similar
    # way as transforming from world frame to camera frame. First, rotate the
    # camera frame 'back', and then translate it 'back'. (Recall that when
    # going world->camera, the frame is first rotated, and then translated.
    # The rotations are always with respect to the origin; when we are going
    # camera->world, the origin has become the camera frame's origin.
    pos, rot = camera_pose
    x, y, z = pos
    pitch, yaw, roll = rot
    Rmat = R_euler(pitch, yaw, roll, affine=True)
    Rinv_mat = np.linalg.inv(Rmat)  # rotate 'back'
    Tmat = T(-x, -y, -z)  # translate 'back'
    return np.dot(Tmat, Rinv_mat)  # first rotate, then translate

def intrinsic(fov, width, height):
    """This intrinsic matrix works based on open3d visualization"""
    focal_length = 0.5 * width * math.tan(to_rad(fov/2))
    fx, fy, cx, cy = (focal_length/2,
                      focal_length/2,
                      width/2,
                      height/2)
    return fx, fy, cx, cy

def inverse_projection(u, v, d, intrinsic, extrinsic_inv=None):
    """
    Converts image coordinate (u,v) and depth d to x,y,z location in the world,
    given camera intrinsic (fx, fy, cx, cy); Can optionally specify
    extrinsic_inv, a 4x4 matrix, if want to project back to world frame.
    """
    fx, fy, cx, cy = intrinsic
    x_c = (u - cx) * d / fx
    y_c = (v - cy) * d / fy
    z_c = d

    if extrinsic_inv is not None:
        x_w, y_w, z_w, _ =\
            np.matmul(extrinsic_inv,
                      np.asarray([x_c, y_c, z_c, 1]).transpose())
        return (x_w, y_w, z_w)
    else:
        return (x_c, y_c, z_c)

def pcd_from_rgbd_open3d(color, depth, width, height, fx, fy, cx, cy):
    """
    color (np.array): rgb image
    depth (np.array): depth image
    width, height, fx, fy, cx, cy: parameters of camera intrinsic
    """
    depth = depth / np.max(depth)
    depth_img = o3d.geometry.Image(depth)
    color_img = o3d.geometry.Image(color.astype(np.uint8))
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    pcd.transform([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [0, 0, 0, 1]])
    return pcd
