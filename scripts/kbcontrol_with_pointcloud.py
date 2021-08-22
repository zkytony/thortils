import time
import numpy as np
import math
from thortils.utils import R_euler, T, to_rad
from thortils.controller import thor_controller_param
import open3d as o3d

from kbcontrol import main

def init_func(controller):
    fov = thor_controller_param(controller, "fieldOfView")
    width = thor_controller_param(controller, "width")
    height = thor_controller_param(controller, "height")

    # Convert fov to focal length in normalized coordinates
    focal_length = 0.5 * width * math.tan(to_rad(fov/2))

    # viz = o3d.visualization.Visualizer()
    # viz.create_window()

    config = dict(intrinsics=(focal_length,
                              focal_length,
                              width/2, height/2),
                  width=width,
                  height=height)
                  # visualizer=viz)
    return config

def step_func(event, config):
    fx, fy, cx, cy = config["intrinsics"]
    width = config["width"]
    height = config["height"]
    # viz = config["visualizer"]

    _start = time.time()
    rgb = event.frame
    color = o3d.geometry.Image(rgb.astype(np.uint8))

    d = event.depth_frame
    d /= np.max(d)
    depth = o3d.geometry.Image(d)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth,
                                                              depth_scale=1.0,
                                                              depth_trunc=0.5,
                                                              convert_rgb_to_intensity=False)
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    pcd.transform([[1, 0, 0, 0],
                   [0, -1, 0, 0],
                   [0, 0, -1, 0],
                   [0, 0, 0, 1]])
    print('Created point cloud in {:.3f}'.format(time.time() - _start))
    # if 'last_geometry' in config:
    #     viz.remove_geometry(config['last_geometry'])
    # viz.add_geometry(pcd)
    o3d.visualization.draw_geometries([pcd])
    config['last_geometry'] = pcd

    # viz.poll_events()
    # viz.update_renderer()

if __name__ == "__main__":
    main(init_func, step_func)
