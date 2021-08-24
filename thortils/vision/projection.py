import random
import math
import numpy as np
import open3d as o3d
from tqdm import tqdm
from thortils.utils import to_rad, R_euler, T, clip
from thortils.controller import thor_controller_param

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
    x, y, z = pos
    pitch, yaw, roll = rot   # ai2thor convention
    Rmat = R_euler(pitch, yaw, roll, affine=True)
    # Note: no idea why it's first translate then rotate; But it works; No idea
    # why -x is needed. It doesn't match what I have learned!
    return np.dot(Rmat, T(-x, y, z))

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

def thor_camera_intrinsic(controller):
    fov = thor_controller_param(controller, "fieldOfView")
    width = thor_controller_param(controller, "width")
    height = thor_controller_param(controller, "height")
    intrinsic = pinhole_intrinsic(fov, width, height)
    return intrinsic

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

def projection(x, y, z, intrinsic, camera_pose_or_extrinsic=None):
    """Converts (x,y,z) in world frame (or camera frame, if
    camera_pose_or_extrinsic is None) to the image plane.
    Also returns the depth, which is mirrored.
    """
    if camera_pose_or_extrinsic is not None:
        if type(camera_pose_or_extrinsic) == tuple:
            camera_pose = camera_pose_or_extrinsic
            e = extrinsic(camera_pose)
        else:
            e = camera_pose_or_extrinsic
        x_c, y_c, z_c, _ =\
            np.dot(e, np.asarray([x, y, z, 1.]).transpose())
    else:
        x_c, y_c, z_c = x, y, z

    _, _, fx, fy, cx, cy = intrinsic
    u, v, w = np.dot([[fx, 0., cx, 0.],
                      [0., -fy, cy, 0.],
                      [0., 0., 1, 0.]], np.array([x_c, y_c, z_c,1]).transpose())
    u /= z_c
    v /= z_c
    d = -z_c
    return u, v, d

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
    depth_img = o3d.geometry.Image(depth.astype(np.float32))
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
                  depth_trunc=7,
                  downsample=0.2,
                  show_progress=False):
    """
    Given rgbd, return a list of points in 3D and corresponding list of colors.

    Args:
        color, depth: np.array
        intrinsic (tuple): output of pinhole_intrinsic
        camera_pose: (position, rotation) provides extrinsics. If None, then the
            resulting points will be in camera frame
    Returns:
        points, colors: list
    """
    einv = extrinsic_inv(camera_pose)
    points = []
    colors = []

    if show_progress:
        enumerator = tqdm(np.ndindex(depth.shape))
    else:
        enumerator = np.ndindex(depth.shape)

    # The procedure is simlar to open3d_pcd_from_rgbd
    for v, u in enumerator:
        if random.uniform(0,1) < (1. - downsample):  # downsample
            d = depth[v,u] / depth_scale
            if d <  0 or d > depth_trunc:
                continue  # truncate
            point = inverse_projection(u, v, d, intrinsic, einv)
            points.append(point)
            colors.append(color[v,u])
    return points, colors

def pcd_from_depth(depth,
                   intrinsic,
                   **params):
    fake_colors = np.zeros((*depth.shape, 3))
    points, _ = pcd_from_rgbd(fake_colors, depth, intrinsic, **params)
    return points

def rgbd_from_pcd(points, colors,
                  intrinsic, camera_pose=None,
                  depth_scale=1.0, show_progress=False):
    """
    Args:
        points (list): List of 3D points
        colors (list): List of corresponding colors (0-255)
        intrinsic (tuple): output of pinhole_intrinsic
        camera_pose: (position, rotation) provides extrinsics.
            If None, then assume the given points to be in camera frame already.
    Returns:
        color, depth: np.ndarray
    """
    if show_progress:
        enumerator = tqdm(points)
    else:
        enumerator = points

    ex = None
    if camera_pose is not None:
        ex = extrinsic(camera_pose)

    width, height = intrinsic[:2]
    outd = np.zeros((width, height))
    outrgb = np.full((width, height, 3), [255., 255., 255.])
    for i, p in enumerate(enumerator):
        x, y, z = p
        u, v, d = projection(x, y, z, intrinsic, ex)
        u = clip(int(round(u)), 0, width-1)
        v = clip(int(round(v)), 0, height-1)
        outd[v, u] = d * depth_scale
        outrgb[v, u] = colors[i]
    outrgb = np.flip(outrgb, 0)
    outrgb = np.flip(outrgb, 1)
    outd = np.flip(outd, 0)
    outd = np.flip(outd, 1)
    return outrgb, outd
