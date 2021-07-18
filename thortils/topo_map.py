import json

from .graph import Node, Edge, Graph
from .agent import thor_camera_pose
from ..actions import ThorAction

class PoseNode(Node):
    """
    PoseNode represents one pose (position, rotation) the agent can reach.
    Note that this should be the camera pose.
    """
    def __init__(self, id, position, rotation):
        """
        Args:
           id (int): ID of the node
           position (tuple): tuple (x, y, z); ai2thor uses (x, z) for robot base
           rotation (tuple): tuple (x, y, z); pitch, yaw, roll.
              Not doing quaternion because in ai2thor the mobile robot
              can only do two of the rotation axes so there's no problem using
              Euclidean.  Will restrict the angles to be between 0 to 360.
              Units in degrees (to be consistent with ai2thor).

              yaw refers to rotation of the agent's body.
              pitch refers to rotation of the camera up and down.
        """
        self.position = position
        self.rotation = rotation
        super().__init__(id)


class ActionEdge(Edge):
    def __init__(self, id, node1, node2, action, params):
        """
        Args:
            action (str): Action name string in Thor
            params (dict): Parameters for the action
        """
        self.action = action
        self.params = params
        super().__init__(self, id, node1, node2, data=self.action)


class NavTopoMap(Graph):
    """
    NavTopoMap: a graph where nodes are poses, and edges are navigation actions.
    """

    def __init__(self, scene, edges):
        """
        Args:
            scene (str): e.g. "FloorPlan2"
            edges (set or dict): ActionEdge objects.
        """
        self.scene = scene
        super().__init__(edges)
