# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

# Utility functions regarding objects
#
# Note: For all functions that return pose,
# the returned pose is by default a dictionary
# e.g. with (x=, y=, z=) keys. The `as_typle`
# option should be False by default.

import math
from typing import Tuple, Dict, List, Set, Union, Any, Optional, Mapping, cast

from . import constants
from .controller import thor_get, _resolve
from .agent import thor_agent_pose, thor_agent_position
from .utils import euclidean_dist

import re

def thor_object_type(object_id):
    """By convention, thor object id are <object_type>|.|.|."""
    # I notice that sometimes you'll get e.g. ButterKnife1
    return re.match("[A-Za-z]+", object_id.split("|")[0]).group()

def thor_all_object_types(event_or_controller):
    """Returns a set of all object types in the given event or controller"""
    event = _resolve(event_or_controller)
    thor_objects = thor_get(event, "objects")
    return set(obj['objectType'] for obj in thor_objects)

def thor_object_with_id(event_or_controller, object_id):
    event = _resolve(event_or_controller)
    thor_objects = thor_get(event, "objects")
    try:
        obj = list(filter(lambda obj: obj["objectId"] == object_id, thor_objects))[0]
        return obj
    except IndexError as ex:
        # Object does not exist
        return None

def thor_object_pose(event_or_controller, object_id, as_tuple=False):
    """For objects, pose == position"""
    obj = thor_object_with_id(event_or_controller, object_id)
    if obj is None:
        # Unknown because object_id is not valid.
        return None
    p = obj["position"]
    if as_tuple:
        return (p["x"], p["y"], p["z"])
    else:
        return p

def thor_object_position(event_or_controller, object_id, as_tuple=False):
    return thor_object_pose(event_or_controller, object_id, as_tuple=as_tuple)

def thor_object_poses(event_or_controller, object_type):
    """Returns a dictionary id->pose
    for the objects of given type. An object pose is
    a 3D (x,y,z) tuple"""
    event = _resolve(event_or_controller)
    thor_objects = thor_get(event, "objects")
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


def get_object_bboxes2D(event_or_controller, objects=None, center_only=False):
    """
    Returns a mapping from object id to either a list bounding boxes or
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

def thor_closest_object_with_properties(event_or_controller, properties):
    """
    Exactly the same as https://github.com/allenai/allenact/blob/6b98c325251d5629c01c463792683c40279f5821/allenact_plugins/ithor_plugin/ithor_environment.py#L766

    Returns the object closest to the given pose that
    satisfy the given properties. `properties` is a dictionary
    mapping from attribute name to value
    """
    event = _resolve(event_or_controller)
    agent_pos = thor_agent_position(event)
    min_dist = float("inf")
    closest = None
    for o in thor_get(event, "objects"):
        satisfies_all = True
        for k, v in properties.items():
            if o[k] != v:
                satisfies_all = False
                break
        if satisfies_all:
            d = position_dist(agent_pos, o["position"])
            if d < min_dist:
                min_dist = d
                closest = o
    return closest

def thor_closest_object_of_type(event_or_controller, object_type):
    """
    https://github.com/allenai/allenact/blob/6b98c325251d5629c01c463792683c40279f5821/allenact_plugins/ithor_plugin/ithor_environment.py#L795
    """
    properties = {"objectType": object_type}
    return thor_closest_object_with_properties(event_or_controller,
                                               properties)

def thor_closest_object_of_type_position(event_or_controller, object_type, as_tuple=False):
    event = _resolve(event_or_controller)
    obj = thor_closest_object_of_type(event, object_type)
    return thor_object_position(event, obj["objectId"], as_tuple=as_tuple)


def position_dist(
    p0: Mapping[str, Any],
    p1: Mapping[str, Any],
    ignore_y: bool = False,
    l1_dist: bool = False,
) -> float:
    """Distance between two points of the form {"x": x, "y":y, "z":z"}.
    https://github.com/allenai/allenact/blob/6b98c325251d5629c01c463792683c40279f5821/allenact_plugins/ithor_plugin/ithor_environment.py#L726"""
    if l1_dist:
        return (
            abs(p0["x"] - p1["x"])
            + (0 if ignore_y else abs(p0["y"] - p1["y"]))
            + abs(p0["z"] - p1["z"])
        )
    else:
        return math.sqrt(
            (p0["x"] - p1["x"]) ** 2
            + (0 if ignore_y else (p0["y"] - p1["y"]) ** 2)
            + (p0["z"] - p1["z"]) ** 2
        )


def thor_object_in_fov(event_or_controller, object_id):
    """Returns True if object with given id is within
    the current field of view"""
    visible_objects = thor_visible_objects(event_or_controller)
    for obj in visible_objects:
        if obj["objectId"] == object_id:
            return True
    return False

def thor_object_of_type_in_fov(event_or_controller, object_type):
    """Returns True if object with given type is within
    the current field of view"""
    visible_objects = thor_visible_objects(event_or_controller)
    for obj in visible_objects:
        if obj["objectType"] == object_type:
            return True
    return False

def thor_object_in_receptacle(object_id, possible_receptacle):
    """
    Uses the "receptacleObjectIds" field to check.
    Args:
        object_id (str): id of object to be checked
        possible_receptacle (dict): Object dictionary returned by Thor

    Returns:
        True if object is in receptacle
    """
    if possible_receptacle["receptacle"]:
        return object_id in possible_receptacle["receptacleObjectIds"]
    return False

def thor_object_receptors(event_or_controller, object_id,
                          openable_only=False):
    """Given an object id, returns a list of receptacles
    that contain this object. Returns a list because it
    may be the case that an object is in a container
    and the container is in another container.

    The receptacles in the returned list will be ordered
    from inner most to outer most. The list contains receptacle object dictionaries.

    If `openable_only` is True, then only return receptors that can be opened."""
    event = _resolve(event_or_controller)
    thor_objects = thor_get(event, "objects")
    receptors = {}  # maps from receptor ID to its list of contained objects
    for obj in thor_objects:
        if thor_object_in_receptacle(object_id, obj):
            receptors[obj["objectId"]] = (obj["receptacleObjectIds"], obj)
    # create a list from receptors in the inner-outer order. If receptor A, B
    # are in `receptors` dict and A contains more objects than B, then A must be
    # more outer than B.
    sorted_receptor_ids = sorted(receptors, key=lambda r: len(receptors[r][0]))
    if openable_only:
        return [receptors[objid][1] for objid in sorted_receptor_ids
                if receptors[objid][1]["openable"] is True]
    else:
        return [receptors[objid][1] for objid in sorted_receptor_ids]


def thor_distances_in_scene(event_or_controller, class1, class2, in_2d=True):
    """Returns a list of euclidean distances between
    all instances of class1 and all instances of class2"""
    class1_poses = thor_object_poses(event_or_controller, class1)
    class2_poses = thor_object_poses(event_or_controller, class2)
    dists = []
    for objid1 in class1_poses:
        p1 = class1_poses[objid1]
        if in_2d:
            p1 = (p1[0], p1[2])
        for objid2 in class2_poses:
            p2 = class2_poses[objid2]
            if in_2d:
                p2 = (p2[0], p2[2])
            dists.append(euclidean_dist(p1, p2))
    return dists


def thor_objects_height_range(event_or_controller):
    """
    Returns a tuple (min_height, max_height), in thor coordinates,
    that represents the minimum and maximum height any object could
    be in the given event or controller
    """
    event = _resolve(event_or_controller)
    thor_objects = thor_get(event, "objects")

    object_y_coords = [obj["position"]["y"]
                       for obj in thor_objects]
    return min(object_y_coords), max(object_y_coords)
