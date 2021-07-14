from . import thor_get, _resolve

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

def thor_agent_pose(event_or_controller):
    """Returns a tuple (pos, rot),
    pos: dict (x=, y=, z=)
    rot: dict (x=, y=, z=)
    """
    event = _resolve(event_or_controller)
    position = thor_get(event, "agent", "position")
    rotation = thor_get(event, "agent", "rotation")
    return position, rotation

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
