import random
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
    Tmat = T(x, -y, -z)  # translate 'back'; **No idea why it's x not -x; -x causes shift**
    return np.dot(Tmat, Rinv_mat)  # first rotate, then translate

def pinhole_intrinsic(fov, width, height):
    """This intrinsic matrix works based on open3d visualization"""
    focal_length = 0.5 * width * math.tan(to_rad(fov/2))
    fx, fy, cx, cy = (focal_length,
                      focal_length,
                      width/2,
                      height/2)
    return width, height, fx, fy, cx, cy

def inverse_projection(u, v, d, intrinsic, camera_pose_or_extrinsic_inv=None):
    """
    Converts image coordinate (u,v) and depth d to x,y,z location in the world,
    given camera intrinsic (fx, fy, cx, cy); Can optionally specify
    extrinsic_inv, a 4x4 matrix, if want to project back to world frame.

    camera_pose_or_extrinsic_inv can either be a camera_pose or a
    matrix that is the result of extrinsic_inv()
    """
    _, _, fx, fy, cx, cy = intrinsic
    x_c = (u - cx) * d / fx
    y_c = -(v - cy) * d / fy  # this is becasue image y axis and ai2thor's y axis are reversed
    z_c = -d   # the image plane is behind the camera aperture, but the camera looks at +z in ai2thor;
               # so depth is mirrored to make z

    if camera_pose_or_extrinsic_inv is not None:
        if type(camera_pose_or_extrinsic_inv) == tuple:
            camera_pose = camera_pose_or_extrinsic_inv
            e_inv = extrinsic_inv(camera_pose)
        else:
            e_inv = camera_pose_or_extrinsic_inv
        x_w, y_w, z_w, _ =\
            np.dot(e_inv, np.asarray([x_c, y_c, z_c, 1]).transpose())
        return (x_w, y_w, z_w)
    else:
        return (x_c, y_c, z_c)

def open3d_pcd_from_rgbd(color, depth,
                         intrinsic, camera_pose=None,
                         depth_scale=1.0,
                         depth_trunc=7,
                         convert_rgb_to_intensity=False):
    """
    color (np.array): rgb image
    depth (np.array): depth image
    intrinsic: a tuple width, length, fx, fy, cx, cy
    camera_pose: a tuple (position, rotation) of the camera in world frame;
        position and rotation are tuples too.
    depth_scale: depth will be scaled by 1.0 / depth_scale
    depth_trunc: points with depth greater than depth_trunc will be discarded
    """
    width, height, fx, fy, cx, cy = intrinsic
    depth_img = o3d.geometry.Image(depth)
    color_img = o3d.geometry.Image(color.astype(np.uint8))
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_img,
        depth_img,
        depth_scale=depth_scale,
        depth_trunc=depth_trunc,
        convert_rgb_to_intensity=convert_rgb_to_intensity)

    # Get points in camera frame; Note that open3d works with different
    # coordinate conventions w.r.t. ai2thor.
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

    # Therefore we must transform in order for the point cloud to be displayed according to
    # the coordinate system conventions in ai2thor (i.e. y is up).
    pcd.transform([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [0, 0, 0, 1]])

    # After the coordinate system matches ai2thor's convention, we then do
    # the transform to the world frame; Here extrinsic_inv is a matrix
    # that assumes ai2thor's coordinate system convention (therefore we had
    # to do the pcd.transform in the last step.)
    pcd.transform(extrinsic_inv(camera_pose))
    return pcd

def pcd_from_rgbd(color, depth,
                  intrinsic,
                  camera_pose=None,
                  depth_scale=1.0,
                  truncate=7,
                  downsample=0.2):
    einv = extrinsic_inv(camera_pose)
    points = []
    colors = []

    # The procedure is simlar to open3d_pcd_from_rgbd
    for v in range(color.shape[0]):
        for u in range(color.shape[1]):
            if random.uniform(0,1) < (1. - downsample):  # downsample
                d = depth[v,u] / depth_scale
                if d <  0 or d > truncate:
                    continue  # truncate
                point = inverse_projection(u, v, d, intrinsic, einv)
                points.append(point)
                colors.append(color[v,u] / 255.)
    return points, colors
