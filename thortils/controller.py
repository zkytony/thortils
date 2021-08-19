from ai2thor.controller import Controller
from . import constants

def _resolve(event_or_controller):
    """Returns an event, whether the given parameter is an event (already)
    or a controller"""
    if isinstance(event_or_controller, Controller):
        return event_or_controller.step(dict(action="Pass"))
    else:
        return event_or_controller  # it's just an event


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

def thor_step(controller, action, params={}):
    return controller.step(dict(action=action, **params))

def thor_pass(controller):
    return controller.step(dict(action="Pass"))

def thor_scene_from_controller(controller):
    return controller.scene.split("_")[0]

def thor_grid_size_from_controller(controller):
    return controller.initialization_parameters["gridSize"]

def launch_controller(config):
    rotateStepDegrees = config.get("H_ROTATION", constants.H_ROTATION)
    if rotateStepDegrees < 90:
        print("WARNING: Ai2thor version 1.0.1 does not support rotation < 90 degrees.")

    controller = Controller()
    controller.start()
    controller.reset(config["scene"])
    controller.scene = config["scene"]

    grid_size = config.get("GRID_SIZE", constants.GRID_SIZE)
    fieldOfView = config.get("FOV", constants.FOV)
    controller.initialization_parameters = {
        "gridSize": grid_size
    }
    controller.step(dict(action="Initialize",
                         gridSize=grid_size,
                         fieldOfView=fieldOfView,
                         renderObjectImage=True))

    # controller = Controller(
    #     scene                      = config["scene"],
    #     agentMode                  = config.get("AGENT_MODE"                   ,constants.AGENT_MODE),
    #     gridSize                   = config.get("GRID_SIZE"                    ,constants.GRID_SIZE),
    #     visibilityDistance         = config.get("VISIBILITY_DISTANCE"          ,constants.VISIBILITY_DISTANCE),
    #     snapToGrid                 = config.get("SNAP_TO_GRID"                 ,constants.SNAP_TO_GRID),
    #     renderDepthImage           = config.get("RENDER_DEPTH"                 ,constants.RENDER_DEPTH),
    #     renderInstanceSegmentation = config.get("RENDER_INSTANCE_SEGMENTATION" ,constants.RENDER_INSTANCE_SEGMENTATION),
    #     width                      = config.get("IMAGE_WIDTH"                  ,constants.IMAGE_WIDTH),
    #     height                     = config.get("IMAGE_HEIGHT"                 ,constants.IMAGE_HEIGHT),
    #     fieldOfView                = config.get("FOV"                          ,constants.FOV),
    #     rotateStepDegrees          = config.get("H_ROTATION"                   ,constants.H_ROTATION),
    #     x_display                  = config.get("x_display"                    , None),
    #     host                       = config.get("host"                         , "127.0.0.1"),
    #     port                       = config.get("port"                         , 0),
    #     headless                   = config.get("headless"                     , False))
    return controller


if __name__ == "__main__":
    # python -m thortils.controller
    import sys
    scene = "FloorPlan1"
    debug = False
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            if arg == "--debug":
                debug = True
            if arg.startswith("Floor"):
                scene = arg
    controller = launch_controller({"scene": scene})
    if debug:
        # Enters debugger with an event object to play with
        event = controller.step(action="Pass")
        import pdb; pdb.set_trace()
