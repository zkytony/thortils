import random
import open3d as o3d
import numpy as np
import time
from tqdm import tqdm

import thortils.vision.projection as pj
import thortils as tt

from thortils.vision import thor_rgbd

from thortils import constants
from thortils.utils.math import sep_spatial_sample, remap, euclidean_dist
from thortils.grid_map import GridMap


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

    def __init__(self):
        self.pcd = o3d.geometry.PointCloud()

    def add(self, points, colors):
        self.pcd.points.extend(np.asarray(points))
        self.pcd.colors.extend(np.asarray(colors))

    def add_pcd(self, pcd):
        self.pcd.points.extend(pcd.points)
        self.pcd.colors.extend(pcd.colors)

    def add_from_rgbd(self, color, depth, intrinsic, camera_pose, **kwargs):
        """
        intrinsic (tuple): output of pinhole_intrinsic; The intrinsic of the camera
            that will supply the RGBD images. A tuple width, length, fx, fy, cx, cy.
        camera_pose: a tuple (position, rotation) of the camera in world frame;
            position and rotation are tuples too. This can be a camera pose directly
            obtained from thor_camera_pose.
        kwargs: See thortils.vision.projection.open3d_pcd_from_rgbd
        """
        # Using pj.pcd_from_rgbd gives the same result, but it is much slower
        pcd = pj.open3d_pcd_from_rgbd(color, depth, intrinsic, camera_pose,
                                      **kwargs)
        # We could merge point clouds like this becasue ai2thor's RGBD and
        # camera_pose is noise-free
        self.add_pcd(pcd)

    def visualize(self, duration=None):
        viz = o3d.visualization.Visualizer()
        viz.create_window()
        viz.add_geometry(self.pcd)
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

    @property
    def points(self):
        return np.asarray(self.pcd.points)

    @property
    def colors(self):
        return np.asarray(self.pcd.colors)

    def downsample(self):
        self.pcd = self.pcd.voxel_down_sample(voxel_size=0.05)

    def to_grid_map(self,
                    ceiling_cut=1.0,
                    floor_cut=0.1,
                    grid_size=0.25,
                    thor_reachable_positions=None,
                    scene=None,
                    debug=False):
        """Converts the 3D map of point clouds to a GridMap which is 2D.

        Args:
            ceiling_cut (float): The points within `ceiling_cut` range (in meters) from the ymax
                will be regarded as "ceiling"; You may want to set this number so
                that the hanging lights are excluded.
            floor_cut: same as ceiling_cut, but for floors. Set this to 0.4 for FloorPlan201
            debug_args (dict): Things to pass in for use in debuging;

            thor_reachable_positions: If reachable_positions is not None,
                then it should be reachable_positions in thor coordinates, returned by
                thor_reachable_positions; This is used to make sure the free locations
                in the resulting grid map are only the reachable locations.

        Returns:
            GridMap
        """
        downpcd = self.pcd.voxel_down_sample(voxel_size=0.05)
        points = np.asarray(downpcd.points)

        xmax, ymax, zmax = np.max(points, axis=0)
        xmin, ymin, zmin = np.min(points, axis=0)

        # Boundary points;
        # Note: aggressively cutting ceiling and floor points;
        # This may not be desirable if you only want to exclude
        # points corresponding to the lights (this might be achievable
        # by a combination of semantic segmantation and projection;
        # left as a todo).
        floor_points_filter = np.isclose(points[:,1], ymin, atol=floor_cut)
        ceiling_points_filter = np.isclose(points[:,1], ymax, atol=ceiling_cut)
        xwalls_min_filter = np.isclose(points[:,0], xmin, atol=0.05)
        xwalls_max_filter = np.isclose(points[:,0], xmax, atol=0.05)
        zwalls_min_filter = np.isclose(points[:,2], zmin, atol=0.05)
        zwalls_max_filter = np.isclose(points[:,2], zmax, atol=0.05)
        boundary_filter = np.any([floor_points_filter,
                                  ceiling_points_filter,
                                  xwalls_min_filter,
                                  xwalls_max_filter,
                                  zwalls_min_filter,
                                  zwalls_max_filter], axis=0)
        not_boundary_filter = np.logical_not(boundary_filter)


        # For Debugging
        if debug:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.asarray(points[boundary_filter]))
            o3d.visualization.draw_geometries([pcd])

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.asarray(points[not_boundary_filter]))
            o3d.visualization.draw_geometries([pcd])

        # The simplest 2D grid map is Floor + Non-boundary points in 2D
        # The floor points will be reachable, and the other ones are not.
        # Points that will map to grid locations, but origin is NOT at (0,0);
        # A lot of this code is borrowed from thortils.scene.convert_scene_to_grid_map.
        map_points_filter = np.any([floor_points_filter,
                                    not_boundary_filter], axis=0)
        thor_grid_points = (points[map_points_filter] / grid_size).astype(int)
        thor_gx = thor_grid_points[:,0]
        thor_gy = thor_grid_points[:,2]
        width = max(thor_gx) - min(thor_gx) + 1
        length = max(thor_gy) - min(thor_gy) + 1
        # Because of the axis-flip coordinate issue [**]
        thor_gy = -thor_gy
        thor_gx_range = (min(thor_gx), max(thor_gx) + 1)
        thor_gy_range = (min(thor_gy), max(thor_gy) + 1)
        # remap coordinates to be nonnegative (origin AT (0,0))
        gx = remap(thor_gx, thor_gx_range[0], thor_gx_range[1], 0, width).astype(int)
        gy = remap(thor_gy, thor_gy_range[0], thor_gy_range[1], 0, length).astype(int)

        gx_range = (min(gx), max(gx)+1)
        gy_range = (min(gy), max(gy)+1)

        # Little test: can convert back
        try:
            assert all(remap(gx, gx_range[0], gx_range[1], thor_gx_range[0], thor_gx_range[1]).astype(int) == thor_gx)
            assert all(remap(gy, gy_range[0], gy_range[1], thor_gy_range[0], thor_gy_range[1]).astype(int) == thor_gy)
        except AssertionError as ex:
            print("Unable to remap coordinates")
            raise ex

        thor_reachable_points = points[floor_points_filter]
        thor_reachable_points[:,1] = 0
        thor_reachable_grid_points = (thor_reachable_points / grid_size).astype(int)
        thor_reachable_gx = thor_reachable_grid_points[:,0]
        thor_reachable_gy = thor_reachable_grid_points[:,2]
        thor_reachable_gy = -thor_reachable_gy  # see [**] #length

        thor_obstacle_points = points[not_boundary_filter]
        thor_obstacle_points[:,1] = 0
        thor_obstacle_grid_points = (thor_obstacle_points / grid_size).astype(int)
        thor_obstacle_gx = thor_obstacle_grid_points[:,0]
        thor_obstacle_gy = thor_obstacle_grid_points[:,2]
        thor_obstacle_gy = -thor_obstacle_gy  # see [**] length

        # For Debugging
        if debug:
            reachable_colors = np.full((thor_reachable_points.shape[0], 3), (0.7, 0.7, 0.7))
            obstacle_colors = np.full((thor_obstacle_points.shape[0], 3), (0.2, 0.2, 0.2))

            # We now grab points
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.asarray(thor_reachable_points))
            pcd.colors = o3d.utility.Vector3dVector(np.asarray(reachable_colors))

            # We now grab points
            pcd2 = o3d.geometry.PointCloud()
            pcd2.points = o3d.utility.Vector3dVector(np.asarray(thor_obstacle_points))
            pcd2.colors = o3d.utility.Vector3dVector(np.asarray(obstacle_colors))
            o3d.visualization.draw_geometries([pcd, pcd2])

        gx_reachable = remap(thor_reachable_gx, thor_gx_range[0], thor_gx_range[1], 0, width).astype(int)
        gy_reachable = remap(thor_reachable_gy, thor_gy_range[0], thor_gy_range[1], 0, length).astype(int)
        gx_obstacles = remap(thor_obstacle_gx, thor_gx_range[0], thor_gx_range[1], 0, width).astype(int)
        gy_obstacles = remap(thor_obstacle_gy, thor_gy_range[0], thor_gy_range[1], 0, length).astype(int)

        all_positions = set((x,y) for x in range(width) for y in range(length))
        grid_map_reachable_positions = set(zip(gx_reachable, gy_reachable))
        grid_map_obstacle_positions = set(zip(gx_obstacles, gy_obstacles))

        grid_map = GridMap(width, length,
                           grid_map_obstacle_positions,
                           unknown=(all_positions\
                                    - grid_map_obstacle_positions\
                                    - grid_map_reachable_positions),
                           name=scene,
                           ranges_in_thor=(thor_gx_range, thor_gy_range),
                           grid_size=grid_size)

        if thor_reachable_positions is not None:
            obstacles = grid_map.obstacles
            correct_reachable_positions = set()
            for thor_pos in thor_reachable_positions:
                grid_pos = grid_map.to_grid_pos(*thor_pos)
                correct_reachable_positions.add(grid_pos)
                if grid_pos in obstacles:
                    obstacles.remove(grid_pos)

            for pos in grid_map.free_locations:
                if pos not in correct_reachable_positions:
                    obstacles.add(pos)
            grid_map.update(obstacles, unknown=grid_map.unknown)
        return grid_map


class Mapper3D:
    """This is intended to be a convenience object for building
    a 3D map with one set of camera intrinsic parameters, and
    interfacing directly with ai2thor events."""
    def __init__(self, controller):
        self.controller = controller
        self.intrinsic = pj.thor_camera_intrinsic(controller)
        self._map = Map3D()

    @property
    def map(self):
        return self._map

    def update(self, event, **kwargs):
        """Given an ai2thor event, update the map using the contained rgbd image,
        as well as agent state.
        kwargs: See thortils.vision.projection.open3d_pcd_from_rgbd"""
        color, depth = tt.vision.thor_rgbd(event)
        camera_pose = tt.thor_camera_pose(event, as_tuple=True)
        self._map.add_from_rgbd(color, depth, self.intrinsic, camera_pose, **kwargs)

    def automate(self, num_stops=20, num_rotates=4,
                 v_angles=constants.V_ANGLES,
                 h_angles=constants.H_ANGLES,
                 seed=1000, sep=1.25,
                 downsample=True, **kwargs):
        """Automatically build a map, by randomly placing
        the agent in the environment, taking RGBD images,
        and then update the map;

        num_stops: Number of places the agent will be placed
        num_rotates: Number of random rotations at each place
        sep: the minimum separation the sampled agent locations should have

        kwargs: See thortils.vision.projection.open3d_pcd_from_rgbd;

        Will reset the agent to the initial pose after finish.

        After finish, you can access the map through the map attribute."""
        rnd = random.Random(seed)

        initial_agent_pose = tt.thor_agent_pose(self.controller)
        initial_horizon = tt.thor_camera_horizon(self.controller.last_event)

        reachable_positions = tt.thor_reachable_positions(self.controller)
        placements = sep_spatial_sample(reachable_positions, sep, num_stops,
                                        rnd=rnd)

        for pos in tqdm(placements, desc="Building map"):
            for _ in range(num_rotates):
                event = tt.thor_place_agent_randomly(self.controller,
                                                     pos=pos,
                                                     v_angles=v_angles,
                                                     h_angles=h_angles,
                                                     rnd=rnd)
                self.update(event, **kwargs)
        if downsample:
            self.map.downsample()
        tt.thor_teleport(self.controller,
                         initial_agent_pose[0],
                         initial_agent_pose[1],
                         initial_horizon)

        return self.map

    def get_grid_map(self, **kwargs):
        """Obtain the GridMap from current 3D map.
        Note: do not need to pass in grid_size or scene. Will
            obtain that from self.controller.

        Example call:
            grid_map = mapper.get_grid_map(floor_cut=floor_cut,
                                           ceiling_cut=ceiling_cut)
        """
        scene = tt.thor_scene_from_controller(self.controller)
        grid_size = tt.thor_grid_size_from_controller(self.controller)
        kwargs["scene"] = scene
        kwargs["grid_size"] = grid_size

        reachable_positions = tt.thor_reachable_positions(self.controller)
        return self.map.to_grid_map(thor_reachable_positions=reachable_positions,
                                    **kwargs)
