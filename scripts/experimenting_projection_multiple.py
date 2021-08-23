import time
import numpy as np
import math
import argparse
import thortils
from thortils import constants
from thortils.utils import R_euler, T, to_rad
from thortils.controller import thor_controller_param
from thortils.vision import projection as pj
from thortils.agent import thor_agent_pose, thor_camera_pose
import open3d as o3d

from kbcontrol import print_controls

def get_pcd(event, intrinsic):
    _start = time.time()
    rgb = event.frame
    color = o3d.geometry.Image(rgb.astype(np.uint8))

    d = event.depth_frame
    depth = o3d.geometry.Image(d)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth,
                                                              depth_scale=1.0,
                                                              depth_trunc=7,
                                                              convert_rgb_to_intensity=False)
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
    # the transform to the world frame; Here pj.extrinsic_inv is a matrix
    # that assumes ai2thor's coordinate system convention (therefore we had
    # to do the pcd.transform in the last step.)
    camera_pose = thor_camera_pose(event, as_tuple=True)
    pcd.transform(pj.extrinsic_inv(camera_pose))
    print('Created point cloud in {:.3f}'.format(time.time() - _start))
    return pcd


def main():
    parser = argparse.ArgumentParser(
        description="Keyboard control of agent in ai2thor")
    parser.add_argument("-s", "--scene",
                        type=str, help="scene. E.g. FloorPlan1",
                        default="FloorPlan1")
    args = parser.parse_args()

    # launch controller
    controller = thortils.launch_controller({**constants.CONFIG, **{"scene": args.scene}})

    # Obtain camera intrinsics (virtual camera, so can estimate intrinsics from fov).
    fov = thor_controller_param(controller, "fieldOfView")
    width = thor_controller_param(controller, "width")
    height = thor_controller_param(controller, "height")
    focal_length = 0.5 * width * math.tan(to_rad(fov/2))
    fx, fy, cx, cy = (focal_length, focal_length, width/2, height/2)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

    viz = o3d.visualization.Visualizer()
    viz.create_window()

    event = controller.step(action='Pass')
    pcd = get_pcd(event, intrinsic)
    viz.add_geometry(pcd)

    controller.step(action="RotateLeft")
    event = controller.step(action="Pass")
    pcd = get_pcd(event, intrinsic)
    viz.add_geometry(pcd)

    controller.step(action="RotateLeft")
    event = controller.step(action="Pass")
    pcd = get_pcd(event, intrinsic)
    viz.add_geometry(pcd)

    controller.step(action="RotateLeft")
    event = controller.step(action="Pass")
    pcd = get_pcd(event, intrinsic)
    viz.add_geometry(pcd)

    controller.step(action="RotateLeft")
    event = controller.step(action="Pass")
    pcd = get_pcd(event, intrinsic)
    viz.add_geometry(pcd)

    opt = viz.get_render_option()
    opt.show_coordinate_frame = True
    viz.run()
    viz.destroy_window()

if __name__ == "__main__":
    main()
