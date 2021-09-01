import random
import open3d as o3d
import numpy as np
import time
from tqdm import tqdm

import thortils.vision.projection as pj
import thortils as tt

from thortils.vision import thor_rgbd



from thortils.agent import (thor_camera_pose,
                            thor_place_agent_randomly)
from thortils import constants
from thortils.utils.math import sep_spatial_sample


class Map3D:

    """A 3D map represented as a collection of 3D points.
    Note that this is currently designed for ai2thor RGBD images,
    which are noise-free; Therefore, a map is simply the result
    of the union of multiple point clouds converted from RGBD
    under the same world frame.

    The coordinate system matches the projection of RGBD to point cloud
    in thortils.vision.projection.py; That is:

        +y ---> +x
        |       Scene
        |    Top-Down-View
        v +z

    Note that ai2thor actually uses

        ^ +z
        |      Scene
        |   Top-Down-View
        +y ---> +x

    This class provides methods to project the 3D map down to 2D (i.e. GridMap),
    as well as methods to incrementally construct the map
    """

    def __init__(self, points=None, colors=None):
        if points is None:
            points = []
        if colors is None:
            colors = []
        self.points = points
        self.colors = colors

    def add(self, points, colors):
        self.points.extend(points)
        self.colors.extend(colors)

    def clear(self):
        self.points = []
        self.colors = []

    def add_from_rgbd(self, color, depth, intrinsic, camera_pose, **kwargs):
        """
        intrinsic (tuple): output of pinhole_intrinsic; The intrinsic of the camera
            that will supply the RGBD images. A tuple width, length, fx, fy, cx, cy.
        camera_pose: a tuple (position, rotation) of the camera in world frame;
            position and rotation are tuples too. This can be a camera pose directly
            obtained from thor_camera_pose.
        kwargs: See thortils.vision.projection.open3d_pcd_from_rgbd
        """
        pcd = pj.open3d_pcd_from_rgbd(color, depth, intrinsic, camera_pose,
                                      **kwargs)

        # We could merge point clouds like this becasue ai2thor's RGBD and camera_pose is noise-free
        self.add(np.asarray(pcd.points),
                 np.asarray(pcd.colors))

    def visualize(self, duration=None):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.asarray(self.points))
        pcd.colors = o3d.utility.Vector3dVector(np.asarray(self.colors))
        viz = o3d.visualization.Visualizer()
        viz.create_window()
        viz.add_geometry(pcd)
        opt = viz.get_render_option()
        opt.show_coordinate_frame = True
        if duration is None:
            viz.run()
        else:
            _start = time.time()
            while time.time() - _start <= duration:
                viz.poll_events()
                viz.update_renderer()
        viz.destroy_window()


class Mapper3D:
    """This is intended to be a convenience object for building
    a 3D map with one set of camera intrinsic parameters, and
    interfacing directly with ai2thor events."""
    def __init__(self, intrinsic, **config):
        self.intrinsic = intrinsic
        self.config = config
        self._map = Map3D()

    @property
    def map(self):
        return self._map

    def update(self, event):
        """Given an ai2thor event, update the map using the contained rgbd image,
        as well as agent state."""
        color, depth = tt.vision.thor_rgbd(event)
        camera_pose = tt.thor_camera_pose(event, as_tuple=True)
        self._map.add_from_rgbd(color, depth, self.intrinsic, camera_pose, **self.config)

    @staticmethod
    def automate(controller, num_stops=20, num_rotates=3,
                 v_angles=constants.V_ANGLES,
                 h_angles=constants.H_ANGLES,
                 rnd=random, sep=1.25, **kwargs):
        """Automatically build a map, by randomly placing
        the agent in the environment, taking RGBD images,
        and then update the map;

        num_stops: Number of places the agent will be placed
        num_rotates: Number of random rotations at each place
        sep: the minimum separation the sampled agent locations should have

        kwargs: See thortils.vision.projection.open3d_pcd_from_rgbd"""
        reachable_positions = tt.thor_reachable_positions(controller)
        placements = sep_spatial_sample(reachable_positions, sep, num_stops)

        intrinsic = pj.thor_camera_intrinsic(controller)
        mapper = Mapper3D(intrinsic, **kwargs)
        for pos in tqdm(placements, desc="Building map"):
            for _ in range(num_rotates):
                event = tt.thor_place_agent_randomly(controller,
                                                     pos=pos,
                                                     v_angles=v_angles,
                                                     h_angles=h_angles,
                                                     rnd=rnd)
                mapper.update(event)
        return mapper
