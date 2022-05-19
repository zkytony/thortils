# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

import numpy as np
import random
from ai2thor.controller import Controller
from .controller import thor_get, _resolve
from .constants import V_ANGLES, H_ANGLES


def _reachable_thor_loc2d(controller):
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

def thor_reachable_positions(controller, by_axes=False):
    """
    If `by_axes` is True, then returns x, z
    where x and z are both numpy arrays corresponding
    to the coordinates of the reachable positions.

    Otherwise, returns [(x,z) ... ] where x and z are
    floats for individual reachable position coordinates.
    """
    x, z = _reachable_thor_loc2d(controller)
    if by_axes:
        return x, z
    else:
        return [(x[i], z[i]) for i in range(len(x))]

def thor_agent_pose(event_or_controller, as_tuple=False):
    """Returns a tuple (pos, rot),
    pos: dict (x=, y=, z=)
    rot: dict (x=, y=, z=)
    The angles are in degrees and between 0 to 360 (ai2thor convention)
    """
    event = _resolve(event_or_controller)
    p = thor_get(event, "agent", "position")
    r = thor_get(event, "agent", "rotation")
    if as_tuple:
        return (p["x"], p["y"], p["z"]), (r["x"], r["y"], r["z"])
    else:
        return p, r

def thor_agent_position(event_or_controller, as_tuple=False):
    """Returns a tuple (pos, rot),
    pos: dict (x=, y=, z=)
    """
    event = _resolve(event_or_controller)
    position = thor_get(event, "agent", "position")
    if as_tuple:
        return (position["x"], position["y"], position["z"])
    else:
        return position

def thor_teleport2d(controller, pose):
    """Given a 2d pose (x,y,th), teleport the agent to that pose"""
    pos, rot = thor_agent_pose(controller)
    x, z, th = pose
    thor_teleport(controller,
                  dict(x=x, y=pos["y"], z=z),
                  dict(x=rot['x'], y=th, z=rot['z']),
                  horizon=thor_camera_horizon(controller))

def thor_teleport(controller, position, rotation, horizon):
    """Calls the Teleport function with relevant parameters."""
    controller.step(action="Teleport",
                           position=position,
                           rotation=rotation,
                           horizon=horizon,
                           standing=True)  # we don't deal with this
    return controller.step(action="Pass")

def thor_camera_position(event_or_controller, as_tuple=False):
    event = _resolve(event_or_controller)
    pos = thor_get(event, "cameraPosition")
    if as_tuple:
        return (pos['x'], pos['y'], pos['z'])
    return pos

def thor_camera_pose(event_or_controller, as_tuple=False):
    """
    This is exactly the same as thor_agent_pose
    except that the pitch of the rotation is set
    to camera horizon. Everything else is the same.
    """
    event = _resolve(event_or_controller)
    position = thor_camera_position(event)
    rotation = thor_get(event, "agent", "rotation")
    assert abs(rotation["z"]) < 1e-3  # assert that there is no roll
    cameraHorizon = thor_get(event, "agent", "cameraHorizon")
    if as_tuple:
        return (position["x"], position["y"], position["z"]),\
            (cameraHorizon, rotation["y"], 0)
    else:
        return position, dict(x=cameraHorizon, y=rotation["y"], z=0)


def thor_camera_horizon(event_or_controller):
    event = _resolve(event_or_controller)
    cameraHorizon = thor_get(event, "agent", "cameraHorizon")
    return cameraHorizon


def thor_place_agent_randomly(controller,
                              v_angles=V_ANGLES,
                              h_angles=H_ANGLES,
                              rnd=random,
                              pos=None):
    """Place the agent randomly in an environment;
    Both the position and rotation will be random,
    but valid.

    Args:
       controller_or_reachable_positions (list or or Controller)
       v_angles (list): List of valid pitch (tilt) angles
       h_angles (list): List of valid yaw (rotation) angles
       pos (x,z): If provided, will place the agent there, but
            randomize the angles.
    """
    reachable_positions = thor_reachable_positions(controller)
    agent_pose = thor_agent_pose(controller.last_event, as_tuple=False)
    if pos is None:
        pos = rnd.sample(reachable_positions, 1)[0]
    pitch = rnd.sample(v_angles, 1)[0]
    yaw = rnd.sample(h_angles, 1)[0]
    return controller.step(action="Teleport",
                           position=dict(x=pos[0], y=agent_pose[0]['y'], z=pos[1]),
                           rotation=dict(x=agent_pose[1]['x'], y=yaw, z=agent_pose[1]['z']),
                           horizon=pitch,
                           standing=True)

# Pose type changes
def thor_pose_as_tuple(pose_or_component):
    """
    Returns tuple representation of given pose
    or pose component (position or rotation).
    """
    if type(pose_or_component) == tuple:
        position, rotation = pose_or_component
        return (position["x"], position["y"], position["z"]),\
            (rotation["x"], rotation["y"], rotation["z"])
    else:
        return (pose_or_component["x"],
                pose_or_component["y"],
                pose_or_component["z"])

def thor_pose_as_dict(pose_or_component):
    if len(pose_or_component) == 2:
        position, rotation = pose_or_component
        x,y,z = position
        pitch,yaw,roll = rotation
        return dict(x=x, y=y, z=z), dict(x=pitch, y=yaw, z=roll)
    else:
        x,y,z = pose_or_component # works both for positation and rotation
        return dict(x=x,y=y,z=z)
