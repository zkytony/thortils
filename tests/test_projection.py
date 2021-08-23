# This tests our own inverse_projection function, not using open3d
from thortils.vision import projection as pj

from thortils.controller import launch_controller, thor_controller_param
from thortils.object import thor_object_poses
from thortils.agent import thor_camera_pose
import numpy as np
import open3d as o3d
import random


def test_inverse_project(controller):
    # See if I can construct a point cloud using my method
    fov = thor_controller_param(controller, "fieldOfView")
    width = thor_controller_param(controller, "width")
    height = thor_controller_param(controller, "height")

    intrinsic = pj.pinhole_intrinsic(fov, width, height)

    event = controller.step(action="Pass")
    rgb = event.frame
    depth = event.depth_frame
    camera_pose = thor_camera_pose(event, as_tuple=True)
    print("Computing inverse projections...")
    points, colors = pj.pcd_from_rgbd(rgb, depth, intrinsic,
                                      camera_pose=camera_pose)

    print("Making point cloud...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
    pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))

    print("Visualizing")
    viz = o3d.visualization.Visualizer()
    viz.create_window()
    viz.add_geometry(pcd)
    opt = viz.get_render_option()
    opt.show_coordinate_frame = True
    viz.run()
    viz.destroy_window()

    # o3d.visualization.draw_geometries([pcd])


def test_inverse_project_with_extrinsics(controller):
    def pcd_after(controller, action, intrinsic, **params):
        controller.step(action=action)
        event = controller.step(action="Pass")
        camera_pose = thor_camera_pose(event, as_tuple=True)
        extrinsic_inv = pj.extrinsic_inv(camera_pose)
        rgb = event.frame
        depth = event.depth_frame
        # depth = depth / np.max(depth)
        points, colors = pj.pcd_from_rgbd(rgb, depth, intrinsic,
                                          camera_pose=camera_pose,
                                          truncate=7, **params)
        return points, colors

    fov = thor_controller_param(controller, "fieldOfView")
    width = thor_controller_param(controller, "width")
    height = thor_controller_param(controller, "height")
    intrinsic = pj.pinhole_intrinsic(fov, width, height)

    points = []
    colors = []
    for action in ["Pass", "RotateLeft"]:#, "RotateLeft"]:#, "RotateRight", "MoveAhead"]:
        print(action)
        p, c = pcd_after(controller, action, intrinsic, downsample=0.5)
        points.extend(p)
        colors.extend(c)

    print("Making point cloud...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
    pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors))
    # pcd.transform(Rt(R_euler(0, 0, -90, affine=True), (0, 0, 0)))

    print("Visualizing")
    viz = o3d.visualization.Visualizer()
    viz.create_window()
    viz.add_geometry(pcd)
    opt = viz.get_render_option()
    opt.show_coordinate_frame = True
    viz.run()
    viz.destroy_window()
    # draw_geometries(geometries)



if __name__ == "__main__":
    controller = launch_controller({'scene': "FloorPlan4"})
    # test_inverse_project(controller)
    test_inverse_project_with_extrinsics(controller)
