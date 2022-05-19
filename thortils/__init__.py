# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

from . import constants
from .controller import (_resolve,
                         thor_get,
                         launch_controller,
                         thor_scene_from_controller,
                         thor_grid_size_from_controller)
from .object import (thor_all_object_types,
                     thor_object_with_id,
                     thor_object_type,
                     thor_object_pose,
                     thor_object_poses,
                     thor_object_position,
                     thor_visible_objects,
                     thor_interactable_objects,
                     get_object_interactions,
                     get_object_bboxes2D,
                     thor_closest_object_of_type,
                     thor_closest_object_with_properties,
                     thor_closest_object_of_type_position,
                     thor_object_in_fov,
                     thor_object_of_type_in_fov,
                     thor_object_in_receptacle,
                     thor_object_receptors,
                     thor_distances_in_scene,
                     thor_objects_height_range)
from .agent import (thor_reachable_positions,
                    thor_agent_pose,
                    thor_agent_position,
                    thor_camera_horizon,
                    thor_camera_pose,
                    thor_camera_position,
                    thor_place_agent_randomly,
                    thor_teleport,
                    thor_teleport2d,
                    thor_pose_as_tuple,
                    thor_pose_as_dict)
from .interactions import (OpenObject,
                           CloseObject,
                           PickupObject,
                           DropObject,
                           ToggleObjectOn,
                           ToggleObjectOff,
                           PushObjectLeft,
                           PushObjectRight,
                           PushObjectForward,
                           PullObject,
                           RemoveFromScene)

from .scene import (robothor_scene_names,
                    ithor_scene_names,
                    ithor_scene_type,
                    convert_scene_to_grid_map,
                    proper_convert_scene_to_grid_map)

from .navigation import compute_spl, spl_ratio

from .grid_map import GridMap

from . import vision

def ai2thor_version():
    import os
    abs_path = os.path.abspath(os.path.dirname(__file__))
    with open(os.path.join(abs_path, "../AI2THOR_VERSION")) as f:
        version = f.readlines()[0].strip()
    return version
