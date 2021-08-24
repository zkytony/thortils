import argparse
import random
import time
import thortils
from tqdm import tqdm
from thortils import constants, thor_object_type
from thortils.vision import thor_object_bboxes
from thortils.vision import projection as pj
from thortils.agent import thor_camera_pose, thor_agent_pose
from thortils.utils import clip
from thortils.utils.colors import mean_rgb

from thortils.utils.visual import GridMapVizualizer

# If take no action
CLASSES = {"Fridge", "CoffeeMachine", "Toaster"}

# If rotate three times
CLASSES = {"Book", "CounterTop"}

def test_project_object_detection():
    parser = argparse.ArgumentParser(
        description="Keyboard control of agent in ai2thor")
    parser.add_argument("-s", "--scene",
                        type=str, help="scene. E.g. FloorPlan1",
                        default="FloorPlan1")
    args = parser.parse_args()
    controller = thortils.launch_controller({**constants.CONFIG, **{"scene": args.scene}})
    controller.step(action="RotateLeft")
    controller.step(action="RotateLeft")
    controller.step(action="RotateLeft")
    event = controller.step(action="Pass")

    grid_map = thortils.convert_scene_to_grid_map(controller, args.scene, constants.GRID_SIZE)
    # note that obstacles are simply locations not in the set of reachable locations; they do not imply exact locations of objects
    viz = GridMapVizualizer(grid_map=grid_map, obstacle_color=(230, 230, 230))

    intrinsic = pj.thor_camera_intrinsic(controller)
    width, height = intrinsic[:2]

    # Get groundtruth object detections
    rgb = event.frame
    depth = event.depth_frame

    camera_pose = thor_camera_pose(event, as_tuple=True)
    agent_pose = thor_agent_pose(event, as_tuple=True)

    bboxes = thor_object_bboxes(event)
    einv = pj.extrinsic_inv(camera_pose)
    detgrids = {}  # detection grids; maps from objid to (cls, color, points2D)
    for objectId in bboxes:
        cls = thor_object_type(objectId)
        if cls not in CLASSES:
            continue

        print("Projecting bounding box for {}".format(cls))
        x1, y1, x2, y2 = bboxes[objectId]
        x_center = int(round((x1 + x2) / 2))
        y_center = int(round((y1 + y2) / 2))

        color = mean_rgb(rgb[y1:y2, x1:x2]).tolist()
        points = []
        for bv in tqdm(range(y1, y2)):
            for bu in range(x1, x2):
                if random.uniform(0,1) < 0.05: # only keep 5% of pixels
                    v = clip(bv, 0, height-1)
                    u = clip(bu, 0, width-1)
                    d = depth[v, u]
                    x, y = pj.inverse_projection_to_grid(u, v, d, intrinsic, grid_map, einv)
                    points.append((x, y))
        detgrids[objectId] = (cls, color, points)

    # highlight
    img = viz.render()
    for objectId in detgrids:
        cls, color, points = detgrids[objectId]
        img = viz.highlight(img, points, color=color, alpha=0.05,
                            show_progress=True)  # each cell gets very small alpha

    gx, gy, gth = grid_map.to_grid_pose(agent_pose[0][0], agent_pose[0][2], agent_pose[1][1])
    img = viz.draw_robot(img, gx, gy, gth, color=(200, 140, 194))

    viz.show_img(img)
    time.sleep(5)

if __name__ == "__main__":
    test_project_object_detection()
