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

def do_step(controller, action, intrinsic, viz):
    print(action)
    controller.step(action=action)
    event = controller.step(action="Pass")
    camera_pose = thor_camera_pose(event, as_tuple=True)
    pcd = pj.open3d_pcd_from_rgbd(event.frame, event.depth_frame, intrinsic,
                                  camera_pose=camera_pose)
    viz.add_geometry(pcd)

def main_open3d():
    # Testing our method to use open3d to get point cloud

    print("This function will show a point cloud visualization"
          "that is the result of fusing multiple point clouds"
          "obtained from different robot poses; The RGBD->point cloud"
          "is performed using open3d's internal method")

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
    intrinsic = pj.pinhole_intrinsic(fov, width, height)

    viz = o3d.visualization.Visualizer()
    viz.create_window()

    do_step(controller, "Pass", intrinsic, viz)
    do_step(controller, "RotateLeft", intrinsic, viz)
    do_step(controller, "MoveAhead", intrinsic, viz)
    do_step(controller, "RotateRight", intrinsic, viz)
    do_step(controller, "RotateRight", intrinsic, viz)
    do_step(controller, "MoveAhead", intrinsic, viz)
    do_step(controller, "RotateLeft", intrinsic, viz)
    do_step(controller, "RotateLeft", intrinsic, viz)
    do_step(controller, "RotateLeft", intrinsic, viz)
    do_step(controller, "MoveAhead", intrinsic, viz)
    do_step(controller, "MoveBack", intrinsic, viz)

    opt = viz.get_render_option()
    opt.show_coordinate_frame = True
    viz.run()
    viz.destroy_window()

if __name__ == "__main__":
    main_open3d()
