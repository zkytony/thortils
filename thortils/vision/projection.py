import random
import math
import numpy as np
import open3d as o3d
from tqdm import tqdm
from thortils.utils.math import to_rad, R_euler, T, clip
from thortils.controller import thor_controller_param
from thortils.utils.colors import mean_rgb

def extrinsic(camera_pose):
    """Given a camera_pose, returns a 4x4 extrinsic matrix that can transform
    a point in the world frame to the camera frame.

    The pose = (position, rotation), where position = (x,y,z)
    and rotation should be of length 3. It is specified
    by euler angles (degrees), same format as ai2thor rotation in pose.

    The job of the extrinsic matrix is to re-express the coordinates
    of a point in the world frame with respect to the camera frame.
    That means, the origin of the camera frame, which is at camera_pose,
    will become the new (0,0,0). The orientation of the camera frame,
    after recentering, should match its rotation in the world frame.

    This means the extrinsic matrix first applies translation to undo
    the position in the camera_pose, then applies the rotation specified
    in the camera pose.
    """
    pos, rot = camera_pose
    x, y, z = pos
    pitch, yaw, roll = rot   # ai2thor convention
    # Unity documentation: In Unity these rotations are performed around the Z
    # axis, the X axis, and the Y axis, **in that order**; To express this
    # order in scipy rotation, we have to do 'yxz' (the order of matrix multiplication)
    Rmat = R_euler(yaw, pitch, roll, order='yxz').as_matrix()
    ex = np.zeros((4, 4))
    ex[:3, :3] = Rmat
    # not sure why z shouldn't be inverted. Perhaps another Unity thing.
    ex[:3, 3] = np.dot(Rmat, np.array([-x, -y, z]))
    ex[3, 3] = 1
    return ex

def extrinsic_inv(camera_pose):
    """Here, we are asked to produce a inverse extrinsic that transforms points
    from the camera frame to the world frame."""
    return np.linalg.inv(extrinsic(camera_pose))

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

    Note that the output location, if in world frame, is with respect to the following
    coordinate frame. Note that the origin is located at the top-left for the scene's
    top-down view: (TODO: This should probably be straightened out - why doesn't the
    output coordinate frame match the frame given in the camera_pose_or_extrinsic_inv?
    This results in the exact same behavior as open3d in visualization; Perhaps it is
    an open3d <-> ai2hor coordinate frame thing?)

        +y ---> +x
        |       Scene
        |    Top-Down-View
        v +z

    However, ai2thor uses the below one:

        ^ +z
        |      Scene
        |   Top-Down-View
        +y ---> +x

    Not only is the z axis flipped, the origin is at the bottom-left of the scene's
    top-down view. This has implications for converting the output pose to the actual
    ai2thor pose, because the length of the scene's top-down view is needed to convert
    the z axis properly.

    Args:
        u, v, d (float): specifies the location on the image plane (u, v) with depth d
        intrinsic (tuple): The output of the pinhole_intrinsic function
        camera_pose_or_extrinsic_inv (tuple or np.ndarray): either a camera pose (ai2thor
            pose convention), or an inverse extrinsic matrix (4x4). This is optional.
    Returns:
        tuple: (x_w, y_w, z_w): point in world frame, if camera_pose_or_extrinsic_inv is provided
           OR  (x_c, y_c, z_c): point in camera frame, if camera_pose_or_extrinsic_inv is NOT provided
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

def inverse_projection_to_grid(u, v, d, intrinsic, grid_map, camera_pose_or_extrinsic_inv):
    """Maps the given image plane location (u,v) and depth d to a location on the
    given grid map (GridMap). The GridMap is 2D. Note
    """
    x_w, y_w, z_w = inverse_projection(u, v, d, intrinsic, camera_pose_or_extrinsic_inv)
    grid_x, grid_y = grid_map.to_grid_pos(x_w, z_w)
    # Because of the z-axis inversion documented in inverse_projection, we need to revert z coordinate
    grid_y = grid_map.length - grid_y
    return grid_x, grid_y

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
    if camera_pose is not None:
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

def project_bbox_to_grids(bbox, depth, intrinsic, grid_map, rgb=None):
    width, height = intrinsic[:2]
    x1, y1, x2, y2 = bbox
    points = []
    for bv in tqdm(range(y1, y2)):
        for bu in range(x1, x2):
            if random.uniform(0,1) < 0.05: # only keep 5% of pixels
                v = clip(bv, 0, height-1)
                u = clip(bu, 0, width-1)
                d = depth[v, u]
                x, y = inverse_projection_to_grid(u, v, d,
                                                  intrinsic,
                                                  grid_map,
                                                  einv)
                points.append((x, y))
    if rgb is not None:
        color = mean_rgb(rgb[y1:y2, x1:x2]).tolist()
        return points, color
    else:
        return points
