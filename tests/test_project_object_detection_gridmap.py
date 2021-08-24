import thortils
from thortils import constants, thor_object_type
from thortils.vision import thor_object_bboxes
from thortils.vision import projection as pj
from thortils.agent import thor_camera_pose
from thortils.utils import clip
import argparse
import random
import time

from cospomdp_apps.thor.visual import GridMapVizualizer

CLASSES = {"CounterTop"} #"Apple",

def main():
    parser = argparse.ArgumentParser(
        description="Keyboard control of agent in ai2thor")
    parser.add_argument("-s", "--scene",
                        type=str, help="scene. E.g. FloorPlan1",
                        default="FloorPlan1")
    args = parser.parse_args()
    controller = thortils.launch_controller({**constants.CONFIG, **{"scene": args.scene}})
    controller.step(action="RotateLeft")
    controller.step(action="RotateLeft")
    controller.step(action="Pass")

    grid_map = thortils.convert_scene_to_grid_map(controller, args.scene, constants.GRID_SIZE)
    viz = GridMapVizualizer(grid_map=grid_map)

    intrinsic = pj.thor_camera_intrinsic(controller)
    width, height = intrinsic[:2]

    # Get groundtruth object detections
    event = controller.step(action="Pass")
    rgb = event.cv2img
    depth = event.depth_frame
    camera_pose = thor_camera_pose(event, as_tuple=True)

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

        color = rgb[x_center, y_center].tolist()
        points = set()
        for bv in range(y1, y2):
            for bu in range(x1, x2):
                # if random.uniform(0,1) < 0.7:
                # if bv == y_center and bu == x_center:
                #     import pdb; pdb.set_trace()
                v = clip(bv, 0, height-1)
                u = clip(bu, 0, width-1)
                d = depth[v, u]
                thor_x, thor_y, thor_z = pj.inverse_projection(u, v, d, intrinsic, einv)
                x, y = grid_map.to_grid_pos(thor_x, thor_z)
                y = grid_map.length - y
                points.add((x, y))
        detgrids[objectId] = (cls, color, points)

    # highlight
    img = viz.render()
    for objectId in detgrids:
        cls, color, points = detgrids[objectId]
        img = viz.highlight(img, points, color=color)

    viz.show_img(img)
    time.sleep(5)

if __name__ == "__main__":
    main()
