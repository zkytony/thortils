import random
import math
from thortils.utils import R_euler, T, to_rad
from thortils.controller import thor_controller_param
from thortils.agent import thor_place_agent_randomly
import open3d as o3d

import math
import numpy as np
import open3d as o3d
from ai2thor.controller import Controller


def main():
    from thortils.controller import launch_controller
    from thortils.object import thor_object_poses
    from thortils.agent import thor_agent_pose
    controller = launch_controller({'scene': "FloorPlan4"})

    # controller.step(action="RotateLeft", degrees=45)
    event = controller.step(action="Pass")

    fov = thor_controller_param(controller, "fieldOfView")
    width = thor_controller_param(controller, "width")
    height = thor_controller_param(controller, "height")

    # Convert fov to focal length in normalized coordinates
    focal_length = 0.5 * width * math.tan(to_rad(fov/2))

    rgb = event.frame
    color = o3d.geometry.Image(rgb.astype(np.uint8))

    d = event.depth_frame
    d /= np.max(d)
    depth = o3d.geometry.Image(d)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth,
                                                              depth_scale=1.0,
                                                              depth_trunc=0.7,
                                                              convert_rgb_to_intensity=False)

    fx, fy, cx, cy = (focal_length,
                      focal_length,
                      width/2, height/2)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    # pcd.transform([[1, 0, 0, 0],
    #                [0, -1, 0, 0],
    #                [0, 0, -1, 0],
    #                [0, 0, 0, 1]])
    print(np.asarray(pcd.points))
    viz = o3d.visualization.Visualizer()
    viz.create_window()
    viz.add_geometry(pcd)
    opt = viz.get_render_option()
    opt.show_coordinate_frame = True
    viz.run()
    viz.destroy_window()
    # o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    main()
