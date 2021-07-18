import numpy as np
from .controller import thor_get, _resolve


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

def thor_reachable_positions(controller):
    return reachable_thor_loc2d(controller)

def thor_agent_pose(event_or_controller, as_tuple=False):
    """Returns a tuple (pos, rot),
    pos: dict (x=, y=, z=)
    rot: dict (x=, y=, z=)
    """
    event = _resolve(event_or_controller)
    p = thor_get(event, "agent", "position")
    r = thor_get(event, "agent", "rotation")
    if as_tuple:
        return (p["x"], p["y"], p["z"]), (r["x"], r["y"], r["z"])
    else:
        return p, r

def thor_agent_position(event_or_controller):
    """Returns a tuple (pos, rot),
    pos: dict (x=, y=, z=)
    """
    event = _resolve(event_or_controller)
    position = thor_get(event, "agent", "position")
    return position

def thor_apply_pose(controller, pose):
    """Given a 2d pose (x,y,th), teleport the agent to that pose"""
    pos, rot = thor_agent_pose(controller)
    x, z, th = pose
    # if th != 0.0:
    #     import pdb; pdb.set_trace()
    controller.step("TeleportFull",
                    x=x, y=pos["y"], z=z,
                    rotation=dict(y=th))
    controller.step(action="Pass")  #https://github.com/allenai/ai2thor/issues/538


def thor_camera_pose(event_or_controller, get_tuples=False):
    """
    This is exactly the same as thor_agent_pose
    except that the pitch of the rotation is set
    to camera horizon. Everything else is the same.
    """
    event = _resolve(event_or_controller)
    position = thor_get(event, "agent", "position")
    rotation = thor_get(event, "agent", "rotation")
    assert abs(rotation["z"]) < 1e-3  # assert that there is no roll
    cameraHorizon = thor_get(event, "agent", "cameraHorizon")
    if get_tuples:
        return (position["x"], position["y"], position["z"]),\
            (cameraHorizon, rotation["y"], 0)
    else:
        return position, dict(x=cameraHorizon, y=rotation["y"], z=0)
