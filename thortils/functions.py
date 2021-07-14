"""
Functions related to THOR simulation
"""
import numpy as np
import math
import os
import pickle
import yaml
import types
from ai2thor.controller import Controller
from .grid_map import GridMap
from .object_interactions import *
from .. import constants
from ..constants import get_acceptable_thor_actions
from ..utils import remap, to_rad, euclidean_dist

def reachable_thor_loc2d(controller):
    """
    Returns a tuple (x, z) where x and z are lists corresponding to x/z coordinates.
    You can obtain a set of 2d positions tuples by:
        `set(zip(x, z))`
    """
    # get reachable positions
    event = controller.step(action="GetReachablePositions")
    positions = event.metadata["actionReturn"]
    x = np.array([p['x'] for p in positions])
    y = np.array([p['y'] for p in positions])
    z = np.array([p['z'] for p in positions])
    return x, z


def launch_controller(config):
    controller = Controller(
        scene                      = config.get("scene"),
        agentMode                  = config.get("AGENT_MODE"                   ,constants.AGENT_MODE),
        gridSize                   = config.get("GRID_SIZE"                    ,constants.GRID_SIZE),
        visibilityDistance         = config.get("VISIBILITY_DISTANCE"          ,constants.VISIBILITY_DISTANCE),
        snapToGrid                 = config.get("SNAP_TO_GRID"                 ,constants.SNAP_TO_GRID),
        renderDepthImage           = config.get("RENDER_DEPTH"                 ,constants.RENDER_DEPTH),
        renderInstanceSegmentation = config.get("RENDER_INSTANCE_SEGMENTATION" ,constants.RENDER_INSTANCE_SEGMENTATION),
        width                      = config.get("IMAGE_WIDTH"                  ,constants.IMAGE_WIDTH),
        height                     = config.get("IMAGE_HEIGHT"                 ,constants.IMAGE_HEIGHT),
        fieldOfView                = config.get("FOV"                          ,constants.FOV),
        rotateStepDegrees          = config.get("H_ROTATION"                   ,constants.H_ROTATION),
        x_display                  = config.get("x_display"                    , None),
        host                       = config.get("host"                         , "127.0.0.1"),
        port                       = config.get("port"                         , 0),
        headless                   = config.get("headless"                     , False))
    return controller


def thor_get(event, *keys):
    """Get the true environment state, which is the metadata in the event returned
    by the controller. If you would like a particular state variable's value,
    pass in a sequence of string keys to retrieve that value.
    For example, to get agent pose, you call:

    env.state("agent", "position")"""
    if len(keys) > 0:
        d = event.metadata
        for k in keys:
            d = d[k]
        return d
    else:
        return event.metadata

def _resolve(event_or_controller):
    """Returns an event, whether the given parameter is an event (already)
    or a controller"""
    if isinstance(event_or_controller, Controller):
        return event_or_controller.step(action="Pass")
    else:
        return event_or_controller  # it's just an event

def thor_agent_pose2d(event_or_controller):
    """Returns a tuple (x, y, th), a 2D pose
    """
    event = _resolve(event_or_controller)
    position = thor_get(event, "agent", "position")
    rotation = thor_get(event, "agent", "rotation")
    return position["x"], position["z"], rotation["y"]

def thor_object_poses(event_or_controller, object_type):
    """Returns a dictionary id->pose
    for the objects of given type. An object pose is
    a 3D (x,y,z) tuple"""
    event = _resolve(event_or_controller)
    thor_objects = thor_get(controller, "objects")
    objposes = {}
    for obj in thor_objects:
        if obj["objectType"] == object_type:
            pose = (obj["position"]["x"], obj["position"]["y"], obj["position"]["z"])
            objposes[obj["objectId"]] = pose
    return objposes

def thor_visible_objects(event_or_controller):
    event = _resolve(event_or_controller)
    thor_objects = thor_get(event, "objects")
    result = []
    for obj in thor_objects:
        if obj["visible"]:
            result.append(obj)
    return result

def thor_interactable_objects(event_or_controller, visible_objects,
                              interaction_distance):
    event = _resolve(event_or_controller)
    pos, rot = thor_agent_pose(event)
    posvec = pos["x"], pos["y"], pos["z"]
    result = []
    for obj in visible_objects:
        objposvec = (obj["position"]["x"], obj["position"]["y"], obj["position"]["z"])
        if euclidean_dist(posvec, objposvec) <= interaction_distance:
            result.append(obj)
    return result


def robothor_scene_names(scene_type="Train", levels=None, nums=None):
    scenes = []
    if scene_type == "Train":
        if levels is None:
            levels = range(1, 13)
        if nums is None:
            nums = range(1, 6)
    elif scene_type == "Val":
        if levels is None:
            levels = range(1, 4)
        if nums is None:
            nums = range(1, 6)
    else:
        raise ValueError("RoboThor has no scene type {}".format(scene_type))

    for i in levels:
        for j in nums:
            scene = "FloorPlan_{}{}_{}".format(scene_type, i, j)
            scenes.append(scene)
    return scenes

def ithor_scene_names(scene_type="kitchen", levels=None):
    """
    Returns a list of scene names
    """
    scenes = dict(
        kitchen = [f"FloorPlan{i}" for i in range(1, 31)],
        living_room = [f"FloorPlan{200 + i}" for i in range(1, 31)],
        bedroom = [f"FloorPlan{300 + i}" for i in range(1, 31)],
        bathroom = [f"FloorPlan{400 + i}" for i in range(1, 31)]
    )
    if scene_type.lower() in scenes:
        if levels is None:
            return scenes[scene_type]
        else:
            return [scenes[scene_type][i-1] for i in levels]
    raise ValueError("Unknown scene type {}".format(scene_type))


def convert_scene_to_grid_map(controller, scene_info, grid_size):
    """Converts an Ai2Thor scene to a GridMap"""
    x, z = reachable_thor_loc2d(controller)

    # obtain grid indices for coordinates  (origin NOT at (0,0))
    thor_gx = np.round(x / grid_size).astype(int)
    thor_gy = np.round(z / grid_size).astype(int)
    width = max(thor_gx) - min(thor_gx) + 1
    length = max(thor_gy) - min(thor_gy) + 1

    # save these for later use
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

    # grid map positions
    positions = set(zip(gx, gy))

    # grid map dimensions
    # obstacles: locations that do not fall into valid positions
    obstacles = {(x,y)
                 for x in gx
                 for y in gy
                 if (x,y) not in positions}

    grid_map = GridMap(width, length, obstacles,
                       name=scene_info.scene_name,
                       ranges_in_thor=(thor_gx_range, thor_gy_range))

    return grid_map

def get_object_interactions(event_or_controller,
                            properties=constants.INTERACTION_PROPERTIES,
                            interaction_distance=constants.INTERACTION_DISTANCE):
    """
    Args:
        controller (Controller)
        properties (list):
            Specifies what interaction properties we care about.
            Ai2Thor objects have interaction properties, such as "pickupable",
            "openable", "toggleable", etc. Each property allows for
            several interaction types. For example, a 'pickupable' object
            allows "PickupObject" and "DropObject".

            Note that whether an interaction can be applied depends not only
            on the property being True but also on other checks. For example,
            "PickupObject" can be applied if the object is "pickupable" AND
            the object is Not "isPickedUp".

            properties is a list of tuples. Each tuple

                 (property_name, function(obj))

            specifies a property we care about, and a function that takes in the object
            (a dictionary) and returns a list of interactions that can be performed.
            Example:

                 ("pickupable", lambda obj: "PickupObject" if not obj["isPickedUp"] else "DropObject")

    Returns:
        tuple: objects, interactions

        objects: Mapping from object id to object dict
        interactions: Mapping from object id to list of interaction actions
            (i.e. action strings such as "PickupObject")

    """
    event = _resolve(event_or_controller)
    visible_objects = thor_visible_objects(event)
    interactable_objects = thor_interactable_objects(event, visible_objects,
                                                     interaction_distance)

    objects = {}
    interactions = {}
    for obj in interactable_objects:
        objects[obj["objectId"]] = obj
        interactions[obj["objectId"]] = []

        for item in properties:
            assert len(item) == 2,\
                "properties must be list of (property_name, function) tuples"
            assert isinstance(item[0], str),\
                "properties must be list of (**property_name**, function) tuples"
            assert callable(item[1]),\
                "properties must be list of (property_name, **function**) tuples"

            prop, func = item
            if obj[prop]:
                action_names = func(obj)  # func returns a list of interactions
                                          # (i.e. action strings) that can be
                                          # performed
                interactions[obj["objectId"]].extend(action_names)

    return objects, interactions


def get_object_mask_pixels(event_or_controller, objects=None, center_only=False):
    """
    Returns a mapping from object id to either a list of pixel locations or
    the location of the center (if `center_only` is True).
    """
    event = _resolve(event_or_controller)
    bboxes2D = event.instance_detections2D

    if objects is not None:
        bboxes2D = {object_id: bboxes2D[object_id]
                    for object_id in bboxes2D
                    if object_id in objects}
    if center_only:
        # Replace the bounding box with a center point
        for object_id in bboxes2D:
            #[Upper Left x, Upper Left y, Lower Right x, Lower Right y]
            x1, y1, x2, y2 = bboxes2D[object_id]
            x = (x1 + x2) / 2
            y = (y1 + y2) / 2
            bboxes2D[object_id] = [x, y]

    # Convert ndarray tolist
    for object_id in bboxes2D:
        bboxes2D[object_id] = bboxes2D[object_id].tolist()

    return bboxes2D
