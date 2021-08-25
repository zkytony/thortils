# This tests our own inverse_projection function, not using open3d
from thortils.vision import projection as pj

from thortils.controller import launch_controller, thor_controller_param
from thortils.object import thor_object_poses
from thortils.agent import thor_camera_pose
from thortils.utils import clip
from thortils import constants
import numpy as np
import open3d as o3d
import random
from tqdm import tqdm
from matplotlib import pyplot as plt

def test_inverse_project_open3d(controller):
    # Testing our method to use open3d to get point cloud
    print("""test_inverse_project_open3d This function will show a point cloud
          visualization that is the result of a point cloud obtained from. The
          RGBD->point cloud is performed using open3d's internal method""")

    # Obtain camera intrinsics (virtual camera, so can estimate intrinsics from fov).
    fov = thor_controller_param(controller, "fieldOfView")
    width = thor_controller_param(controller, "width")
    height = thor_controller_param(controller, "height")
    intrinsic = pj.pinhole_intrinsic(fov, width, height)

    viz = o3d.visualization.Visualizer()
    viz.create_window()

    controller.step(action="RotateLeft")
    event = controller.step(action="Pass")
    camera_pose = thor_camera_pose(event, as_tuple=True)
    pcd = pj.open3d_pcd_from_rgbd(event.frame, event.depth_frame, intrinsic)
    viz.add_geometry(pcd)
    opt = viz.get_render_option()
    opt.show_coordinate_frame = True
    viz.run()
    viz.destroy_window()


def test_inverse_project_multiple_open3d(controller):
    # Testing our method to use open3d to get point cloud
    def do_step(controller, action, intrinsic, viz):
        print(action)
        controller.step(action=action)
        event = controller.step(action="Pass")
        camera_pose = thor_camera_pose(event, as_tuple=True)
        pcd = pj.open3d_pcd_from_rgbd(event.frame, event.depth_frame, intrinsic,
                                      camera_pose=camera_pose)
        viz.add_geometry(pcd)

    print("""test_inverse_project_multiple_open3d
          This function will show a point cloud visualization
          that is the result of fusing multiple point clouds
          obtained from different robot poses; The RGBD->point cloud
          is performed using open3d's internal method""")

    # Obtain camera intrinsics (virtual camera, so can estimate intrinsics from fov).
    fov = thor_controller_param(controller, "fieldOfView")
    width = thor_controller_param(controller, "width")
    height = thor_controller_param(controller, "height")
    intrinsic = pj.pinhole_intrinsic(fov, width, height)

    viz = o3d.visualization.Visualizer()
    viz.create_window()

    do_step(controller, "LookDown", intrinsic, viz)
    do_step(controller, "LookDown", intrinsic, viz)
    do_step(controller, "MoveAhead", intrinsic, viz)
    do_step(controller, "RotateRight", intrinsic, viz)
    # do_step(controller, "LookUp", intrinsic, viz)
    # do_step(controller, "MoveAhead", intrinsic, viz)
    # do_step(controller, "RotateLeft", intrinsic, viz)
    # do_step(controller, "LookDown", intrinsic, viz)
    # do_step(controller, "LookDown", intrinsic, viz)
    # do_step(controller, "MoveAhead", intrinsic, viz)
    # do_step(controller, "MoveAhead", intrinsic, viz)
    # do_step(controller, "RotateLeft", intrinsic, viz)
    # do_step(controller, "MoveAhead", intrinsic, viz)
    # do_step(controller, "MoveBack", intrinsic, viz)

    opt = viz.get_render_option()
    opt.show_coordinate_frame = True
    viz.run()
    viz.destroy_window()


def test_inverse_project(controller):
    print("""test_inverse_project
          This function tests converting a single
          RGBD image to a point cloud, using our method.""")

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
                                      camera_pose=camera_pose,
                                      downsample=0.5,
                                      show_progress=True)

    print("Making point cloud...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
    pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors) / 255.)

    print("Visualizing")
    viz = o3d.visualization.Visualizer()
    viz.create_window()
    viz.add_geometry(pcd)
    opt = viz.get_render_option()
    opt.show_coordinate_frame = True
    viz.run()
    viz.destroy_window()

    # o3d.visualization.draw_geometries([pcd])


def test_inverse_project_multiple(controller):
    def pcd_after(controller, action, intrinsic, **params):
        controller.step(action=action)
        event = controller.step(action="Pass")
        camera_pose = thor_camera_pose(event, as_tuple=True)
        extrinsic_inv = pj.extrinsic_inv(camera_pose)
        rgb = event.frame
        depth = event.depth_frame
        # Testing our own pcd_from_rgbd method
        points, colors = pj.pcd_from_rgbd(rgb, depth, intrinsic,
                                          camera_pose=camera_pose,
                                          depth_trunc=7,
                                          show_progress=True,
                                          **params)
        return points, colors

    print("""test_inverse_project_multiple
          This function will show a point cloud visualization
          that is the result of fusing multiple point clouds
          obtained from different robot poses; The RGBD->point cloud
          is performed using thortil's native pcd_from_rgbd method""")

    fov = thor_controller_param(controller, "fieldOfView")
    width = thor_controller_param(controller, "width")
    height = thor_controller_param(controller, "height")
    intrinsic = pj.pinhole_intrinsic(fov, width, height)

    points = []
    colors = []
    for action in ["RotateLeft", "MoveAhead", "LookUp", "RotateLeft", "LookDown", "LookDown", "RotateRight"]:
        print(action)
        p, c = pcd_after(controller, action, intrinsic, downsample=0.5)
        points.extend(p)
        colors.extend(c)

    print("Making point cloud...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points))
    pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors) / 255.)
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


def test_project(controller):
    print("""test_project
           This function tests converting a
           point cloud to an RGBD image, using our method;
           To verify the integrity of this RGBD image, ,
           it will be converted to another point cloud and visualized.""")

    # Obtain camera intrinsics (virtual camera, so can estimate intrinsics from fov).
    fov = thor_controller_param(controller, "fieldOfView")
    width = thor_controller_param(controller, "width")
    height = thor_controller_param(controller, "height")
    intrinsic = pj.pinhole_intrinsic(fov, width, height)

    controller.step(action="RotateLeft")
    controller.step(action="LookDown")
    event = controller.step(action="Pass")
    camera_pose = thor_camera_pose(event, as_tuple=True)

    color = event.frame
    depth = event.depth_frame
    pcd = pj.open3d_pcd_from_rgbd(color, depth, intrinsic,
                                  camera_pose=camera_pose)
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) * 255
    outrgb, outd = pj.rgbd_from_pcd(points, colors, intrinsic, camera_pose, show_progress=True)

    plt.imshow(outd, interpolation='nearest', cmap='gray')
    plt.colorbar()
    plt.show()
    plt.clf()

    # Get the point cloud again and render it
    pcd = pj.open3d_pcd_from_rgbd(outrgb, outd,
                                  intrinsic, camera_pose=camera_pose,
                                  depth_scale=1.0,
                                  depth_trunc=7,
                                  convert_rgb_to_intensity=False)
    viz = o3d.visualization.Visualizer()
    viz.create_window()
    viz.add_geometry(pcd)
    opt = viz.get_render_option()
    opt.show_coordinate_frame = True
    viz.run()
    viz.destroy_window()


def rgbd_to_pcd_double(event, intrinsic, camera_pose):
    """Helper method for test_project_multiple"""
    # For testing; convert rgb and depth in the event to point cloud,
    # then convert this pcd to rgb, then convert this rgb to pcd.
    # This is to test the correctness of our rgbd_from_pcd method.
    color1 = event.frame
    depth1 = event.depth_frame
    # using the open3d method because it is faster
    points1, colors1 = pj.pcd_from_rgbd(color1, depth1, intrinsic,
                                        camera_pose=camera_pose,
                                        depth_scale=1.0,
                                        depth_trunc=7, downsample=0.8,
                                        show_progress=True)
    color2, depth2 = pj.rgbd_from_pcd(points1, colors1,
                                      intrinsic, camera_pose, show_progress=True)
    # Get the point cloud again and render it
    pcd2 = pj.open3d_pcd_from_rgbd(color2, depth2, intrinsic,
                                   camera_pose=camera_pose, depth_scale=1.0,
                                   depth_trunc=10, convert_rgb_to_intensity=False)
    return pcd2


def test_project_multiple(controller):
    def pcd_after(controller, action, intrinsic, **params):
        """returns pcd after given action is executed"""
        controller.step(action=action)
        event = controller.step(action="Pass")
        camera_pose = thor_camera_pose(event, as_tuple=True)
        return rgbd_to_pcd_double(event, intrinsic, camera_pose)

    print("""test_project_multiple
          This function will show a point cloud visualization
          that is the result of fusing the point clouds from
          multiple poses; At each pose, (1) obtain RGBD image,
          (2) obtain point cloud given the RGBD image
          (3) Obtain RGBD image given point cloud, using **our pj.rgbd_from_pcd**
          method; (4) Obtain point cloud again but using the generated
          RGBD image from step 3. This verifies that both
          projection and inverse projection work properly""")

    fov = thor_controller_param(controller, "fieldOfView")
    width = thor_controller_param(controller, "width")
    height = thor_controller_param(controller, "height")
    intrinsic = pj.pinhole_intrinsic(fov, width, height)

    geom = []
    print("Making point cloud...")
    for action in ["Pass", "RotateLeft", "LookDown", "RotateLeft", "RotateRight", "MoveAhead"]:
        print(action)
        pcd = pcd_after(controller, action, intrinsic)
        geom.append(pcd)

    print("Visualizing")
    viz = o3d.visualization.Visualizer()
    viz.create_window()
    for pcd in geom:
        viz.add_geometry(pcd)
    opt = viz.get_render_option()
    opt.show_coordinate_frame = True
    viz.run()
    viz.destroy_window()
    # draw_geometries(geometries)



if __name__ == "__main__":
    controller = launch_controller({**constants.CONFIG, **{'scene': "FloorPlan1"}})
    test_inverse_project_open3d(controller)
    test_inverse_project_multiple_open3d(controller)
    test_inverse_project(controller)
    test_inverse_project_multiple(controller)
    test_project(controller)
    test_project_multiple(controller)
