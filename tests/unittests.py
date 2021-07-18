import unittest
import time
import random
import math
import numpy as np
from corrsearch.models import *
from corrsearch.utils import *
from corrsearch.objects import ObjectState, JointState
from corrsearch.objects.template import Object
from corrsearch.experiments.domains.thor.problem import *
from corrsearch.experiments.domains.thor.grid_map import *
from corrsearch.experiments.domains.thor.thor import *
from corrsearch.experiments.domains.thor.detector import *
from corrsearch.experiments.domains.thor.visualizer import *
from corrsearch.experiments.domains.thor.transition import *
from corrsearch.experiments.domains.thor.spatial_corr import *
import matplotlib.pyplot as plt


class DummyProblem(SearchProblem):
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.env = None

    def obj(self, objid):
        return {}


@unittest.SkipTest
class TestThorTransition(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        config = {
            "scene_name": "FloorPlan_Train1_1",
            "width": 400,
            "height": 400,
            "grid_size": 0.25
        }
        controller = launch_controller(config)
        scene_info = load_scene_info(scene_name)
        grid_map = convert_scene_to_grid_map(controller, scene_info, config["grid_size"])
        controller.grid_size = config["grid_size"]

        cls.controller = controller
        cls.grid_map = grid_map
        cls.config = config

    # @unittest.SkipTest
    def test_thor_visualize(self):
        cls = TestThorTransition
        controller, grid_map, config =\
            cls.controller, cls.grid_map, cls.config
        robot_id = 0
        init_robot_pose = (*random.sample(grid_map.free_locations, 1)[0], 0.0)
        state = JointState({robot_id: RobotState(robot_id, {"pose": init_robot_pose,
                                                            "energy":0.0})})
        region = grid_map.free_region(*init_robot_pose[:2])

        problem = DummyProblem(robot_id)
        problem.grid_map = grid_map

        viz = ThorViz(problem)
        viz.visualize(state)
        time.sleep(3)
        controller.stop()


    def test_thor_viz_highlight(self):
        cls = TestThorTransition
        controller, grid_map, config =\
            cls.controller, cls.grid_map, cls.config

        robot_id = 0
        init_robot_pose = (*random.sample(grid_map.free_locations, 1)[0], 0.0)
        region = grid_map.free_region(*init_robot_pose[:2])
        boundary = grid_map.boundary_cells(thickness=2)

        problem = DummyProblem(robot_id)
        problem.grid_map = grid_map

        viz = ThorViz(problem)
        viz.highlight(region)
        time.sleep(2)
        viz.highlight(boundary)
        time.sleep(2)
        controller.stop()


    # @unittest.SkipTest
    def test_pose_conversion(self):
        cls = TestThorTransition
        controller, grid_map, config =\
            cls.controller, cls.grid_map, cls.config

        # Get current thor pose
        thor_pose2d = thor_agent_pose2d(controller)

        # Get grid pose from thor pose
        grid_loc2d = grid_map.to_grid_pos(thor_pose2d[0], thor_pose2d[1],
                                          grid_size=config["grid_size"])
        grid_pose = (*grid_loc2d, to_rad(thor_pose2d[2]))

        # Get thor pose from grid pose
        predicted_thor_pose2d = grid_map.to_thor_pose(*grid_pose,
                                                      grid_size=config["grid_size"])
        self.assertEqual(predicted_thor_pose2d,
                         thor_pose2d)



    def check_pose(self, grid_map_pose, grid_map, controller, grid_size):
        next_thor_robot_pose = grid_map.to_thor_pose(*grid_map_pose,
                                                     grid_size=grid_size)
        thor_apply_pose(controller, next_thor_robot_pose)

        actual_thor_pose = thor_agent_pose2d(controller)
        for i in range(len(actual_thor_pose)):
            self.assertAlmostEqual(actual_thor_pose[i], next_thor_robot_pose[i], places=4)


    # @unittest.SkipTest
    def test_thor_moving(self):
        cls = TestThorTransition
        controller, grid_map, config =\
            cls.controller, cls.grid_map, cls.config

        robot_id = 0
        problem = DummyProblem(robot_id)
        problem.grid_map = grid_map

        pos, rot = thor_agent_pose(controller)

        # This tells me that 0 degree is facing up (positive z direction)
        for i, angle in enumerate([0, 15, 30, 45, 60, 75, 90]):
            controller.step('TeleportFull',
                            x=pos["x"], y=pos["y"], z=pos["z"],
                            rotation=dict(y=angle))
            time.sleep(0.05)
        controller.step('TeleportFull',
                        x=pos["x"], y=pos["y"], z=pos["z"],
                        rotation=dict(y=0.0))
        # controller.step("ToggleMapView")

        thor_pose2d = thor_agent_pose2d(controller)
        print(thor_pose2d)
        grid_loc2d = grid_map.to_grid_pos(thor_pose2d[0], thor_pose2d[1],
                                          grid_size=config["grid_size"])
        init_robot_pose = (*grid_loc2d, to_rad(thor_pose2d[2]))
        print(init_robot_pose)
        state = JointState({robot_id: RobotState(robot_id, {"pose": init_robot_pose,
                                                            "energy":0.0})})
        viz = ThorViz(problem)
        viz.visualize(state)
        time.sleep(0.5)

        trans_model = DetRobotTrans(robot_id, grid_map)
        forward = Move((1.0, 0.0), "forward")
        backward = Move((-1.0, 0.0), "backward")
        left = Move((0.0, -math.pi/4), "left")
        right = Move((0.0, math.pi/4), "right")

        # Forward
        print("Pose before move", thor_agent_pose2d(controller))
        next_state = JointState({robot_id: trans_model.sample(state, forward)})
        self.assertEqual(next_state[robot_id].pose, state[robot_id].pose)
        print("Pose After move", thor_agent_pose2d(controller))

        # backward
        next_state = JointState({robot_id: trans_model.sample(state, backward)})
        self.assertNotEqual(next_state[robot_id].pose, state[robot_id].pose)
        self.check_pose(next_state[robot_id].pose, grid_map, controller, config["grid_size"])
        viz.visualize(next_state)
        time.sleep(2)

        # Left
        next_state = JointState({robot_id: trans_model.sample(state, left)})
        self.assertNotEqual(next_state[robot_id].pose, state[robot_id].pose)
        self.check_pose(next_state[robot_id].pose, grid_map, controller, config["grid_size"])
        viz.visualize(next_state)
        time.sleep(2)

        # Right
        next_state = JointState({robot_id: trans_model.sample(state, right)})
        self.assertNotEqual(next_state[robot_id].pose, state[robot_id].pose)
        self.check_pose(next_state[robot_id].pose, grid_map, controller, config["grid_size"])
        viz.visualize(next_state)
        time.sleep(2)

@unittest.SkipTest
class TestGridMap(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        config = {
            "scene_name": "FloorPlan_Train1_1",
            "width": 400,
            "height": 400,
            "grid_size": 0.25
        }
        controller = launch_controller(config)
        scene_info = load_scene_info(scene_name)
        grid_map = convert_scene_to_grid_map(controller, scene_info, config["grid_size"])
        controller.grid_size = config["grid_size"]

        cls.controller = controller
        cls.grid_map = grid_map
        cls.config = config


    def test_geodesic_distance(self):
        cls = TestGridMap
        controller, grid_map, config =\
            cls.controller, cls.grid_map, cls.config

        robot_id = 0
        init_robot_pose = (*random.sample(grid_map.free_locations, 1)[0], 0.0)
        problem = DummyProblem(robot_id)
        problem.grid_map = grid_map

        point1 = init_robot_pose[:2]
        point2 = random.sample(grid_map.free_locations, 1)[0]
        path = grid_map.shortest_path(point1, point2)
        self.assertGreaterEqual(len(path), euclidean_dist(point1, point2))
        self.assertEqual(len(path), grid_map.geodesic_distance(point1, point2))

        viz = ThorViz(problem)
        viz.highlight(path)
        time.sleep(2)
        controller.stop()



# @unittest.SkipTest
class TestThorDetector(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        config = {
            "scene_name": "FloorPlan_Train1_2",
            "width": 400,
            "height": 400,
            "grid_size": 0.25
        }
        controller = launch_controller(config)
        scene_info = load_scene_info(scene_name)
        grid_map = convert_scene_to_grid_map(controller, scene_info, config["grid_size"])
        controller.grid_size = config["grid_size"]

        cls.controller = controller
        cls.grid_map = grid_map
        cls.config = config

    def robot_grid_pose(self):
        cls = TestThorDetector
        thor_pose2d = thor_agent_pose2d(cls.controller)
        grid_loc2d = cls.grid_map.to_grid_pos(thor_pose2d[0], thor_pose2d[1],
                                              grid_size=cls.config["grid_size"])
        robot_pose = (*grid_loc2d, to_rad(thor_pose2d[2]))
        return robot_pose


    def test_laser_sensor_geometry(self):
        cls = TestThorDetector
        sensor = FanSensorThor(fov=75, min_range=0, max_range=2,
                               grid_map=cls.grid_map)
        robot_id = 0
        problem = DummyProblem(robot_id)
        problem.grid_map = cls.grid_map
        problem.target_id = None

        init_robot_pose = self.robot_grid_pose()
        state = JointState({robot_id: RobotState(robot_id, {"pose": init_robot_pose,
                                                            "energy":0.0})})

        viz = ThorViz(problem)
        viz.visualize(state, sensor=sensor)
        time.sleep(2)

        next_thor_robot_pose = cls.grid_map.to_thor_pose(*(7, 12, 0.0),
                                                         grid_size=cls.config["grid_size"])
        thor_apply_pose(cls.controller, next_thor_robot_pose)
        robot_pose = self.robot_grid_pose()
        next_state = JointState({robot_id: RobotState(robot_id, {"pose": robot_pose,
                                                                 "energy":0.0})})

        viz.visualize(next_state, sensor=sensor)
        time.sleep(2)

        next_thor_robot_pose = cls.grid_map.to_thor_pose(*(7, 12, math.pi/2.0),
                                                         grid_size=cls.config["grid_size"])
        thor_apply_pose(cls.controller, next_thor_robot_pose)
        robot_pose = self.robot_grid_pose()
        next_state = JointState({robot_id: RobotState(robot_id, {"pose": robot_pose,
                                                                 "energy":0.0})})

        viz.visualize(next_state, sensor=sensor)
        time.sleep(2)

        next_thor_robot_pose = cls.grid_map.to_thor_pose(*(7, 12, math.pi),
                                                         grid_size=cls.config["grid_size"])
        thor_apply_pose(cls.controller, next_thor_robot_pose)
        robot_pose = self.robot_grid_pose()
        next_state = JointState({robot_id: RobotState(robot_id, {"pose": robot_pose,
                                                                 "energy":0.0})})

        viz.visualize(next_state, sensor=sensor)
        time.sleep(2)


@unittest.SkipTest
class TestThorEnv(unittest.TestCase):

    def test_env_basic(self):
        config = {
            "scene_name": "FloorPlan_Train1_1",
            "width": 400,
            "height": 400,
            "grid_size": 0.25
        }
        robot_id = 0
        target_object = (100, "Laptop")
        scene_info = load_scene_info(config["scene_name"])
        env = ThorEnv(robot_id, target_object, config, scene_info)

        # Problem is still dummy now
        problem = DummyProblem(robot_id)
        problem.grid_map = env.grid_map
        problem.target_id = 100

        viz = ThorViz(problem)
        viz.visualize(env.state)
        time.sleep(2)

        forward = Move((1.0, 0.0), "forward")
        backward = Move((-1.0, 0.0), "backward")
        left = Move((0.0, -math.pi/4), "left")
        right = Move((0.0, math.pi/4), "right")

        action_sequence = [
            (right, True),
            (forward, True),
            (left, True),
            (forward, False),
            (left, True),
            (left, True),
            (forward, True),
            (forward, True),
            (forward, True),
            (forward, False),
            (right, True),
            (right, True),
            (right, True),
            (backward, False),
            (left, True),
            (left, True),
            (backward, True)]
        # env.controller.step("ToggleMapView")
        for action, changes_pose in action_sequence:
            print("Taking action: {}".format(action))
            prev_pose = env.state[robot_id].pose
            env.state_transition(action, execute=True)
            after_pose = env.state[robot_id].pose
            self.assertEqual(prev_pose != after_pose, changes_pose)
            viz.visualize(env.state)
            time.sleep(1)


@unittest.SkipTest
class TestThorProblem(unittest.TestCase):

    def test_problem_basic(self):
        scene_name = "FloorPlan_Train1_1"
        scene_info = load_scene_info(scene_name)
        grid_size = 0.25
        robot_id = 0
        target_object = (100, "Laptop")
        problem = ThorSearch(robot_id,
                             target_object,
                             scene_name,
                             scene_info,
                             detectors_spec_path="./config/detectors_spec.yaml",
                             grid_size=0.25)
        problem.instantiate(init_belief="uniform")




if __name__ == "__main__":
    # test_thor_visualize()
    # test_thor_moving()
    unittest.main(exit=False)
{"mode":"full","isActive":false}
