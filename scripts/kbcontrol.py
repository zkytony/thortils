# Keyboard control of Ai2Thor

import thortils
import thortils.constants as constants
from thortils.utils import getch
import argparse

def print_controls(controls):
    reverse = {controls[k]:k for k in controls}
    ss =f"""
            {reverse['MoveAhead']}
        (MoveAhead)

    {reverse['RotateLeft']}                 {reverse['RotateRight']}
(RotateLeft)     (RotateRight)

    {reverse['LookUp']}
(LookUp)

    {reverse['LookDown']}
(LookDown)

    q
(quit)
    """
    print(ss)


def main():
    parser = argparse.ArgumentParser(
        description="Keyboard control of agent in ai2thor")
    parser.add_argument("-s", "--scene",
                        type=str, help="scene. E.g. FloorPlan1",
                        default="FloorPlan1")
    args = parser.parse_args()

    controls = {
        "w": "MoveAhead",
        "a": "RotateLeft",
        "d": "RotateRight",
        "e": "LookUp",
        "c": "LookDown"
    }
    print_controls(controls)

    controller = thortils.launch_controller({**constants.CONFIG, **{"scene": args.scene}})

    while True:
        k = getch()
        if k == "q":
            print("bye.")
            break

        if k in controls:
            action = controls[k]
            params = constants.MOVEMENT_PARAMS[action]
            controller.step(action=action, **params)

            print("{} | Agent pose: {}".format(k, thortils.thor_agent_pose(controller, as_tuple=True)))

if __name__ == "__main__":
    main()
