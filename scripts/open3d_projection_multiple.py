import time
import numpy as np
import math
import argparse
import thortils
from thortils import constants

import sys
import os
ABS_PATH = os.path.dirname(os.path.abspath(__file__))

# just so we can import mjolnir stuff
sys.path.append(os.path.join(ABS_PATH, '../tests'))
from tests.test_projection import test_inverse_project_multiple_open3d

def main_open3d():
    # Testing our method to use open3d to get point cloud
    parser = argparse.ArgumentParser(
        description="Keyboard control of agent in ai2thor")
    parser.add_argument("-s", "--scene",
                        type=str, help="scene. E.g. FloorPlan1",
                        default="FloorPlan1")
    args = parser.parse_args()

    # launch controller
    controller = thortils.launch_controller({**constants.CONFIG, **{"scene": args.scene}})
    test_inverse_project_multiple_open3d(controller)

if __name__ == "__main__":
    main_open3d()
