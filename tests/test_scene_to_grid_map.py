# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

import os
import time
from thortils import (launch_controller,
                      convert_scene_to_grid_map)
from thortils.scene import SceneDataset
from thortils.utils.visual import GridMapVisualizer
from thortils.agent import thor_reachable_positions
from thortils.grid_map import GridMap


def test_scene_to_grid_map():
    floor_plan = "FloorPlan1"
    scene_info = SceneDataset.load_single("../scenes", floor_plan)
    controller = launch_controller({"scene":floor_plan})
    grid_map = convert_scene_to_grid_map(controller, floor_plan, 0.25)

    print(floor_plan)
    for y in range(grid_map.length):
        row = []
        for x in range(grid_map.width):
            if (x,y) in grid_map.free_locations:
                row.append(".")
            else:
                assert (x,y) in grid_map.obstacles
                row.append("x")
        print("".join(row))

    # Highlight reachable positions
    reachable_positions = thor_reachable_positions(controller)
    highlights = []
    for thor_pos in reachable_positions:
        highlights.append(grid_map.to_grid_pos(*thor_pos))
    viz = GridMapVisualizer(grid_map=grid_map, res=30)
    img = viz.render()
    img = viz.highlight(img, highlights,
                        color=(25, 214, 224), show_progress=True)
    viz.show_img(img)
    time.sleep(10)

def test_grid_map_save_load():
    floor_plan = "FloorPlan1"
    scene_info = SceneDataset.load_single("../scenes", floor_plan)
    controller = launch_controller({"scene":floor_plan})
    grid_map = convert_scene_to_grid_map(controller, floor_plan, 0.25)
    grid_map.save("temp-grid-map.json")

    grid_map2 = GridMap.load("temp-grid-map.json")

    assert grid_map.free_locations == grid_map2.free_locations
    assert grid_map.width == grid_map2.width
    assert grid_map.length == grid_map2.length
    assert grid_map.grid_size == grid_map2.grid_size

    os.remove("temp-grid-map.json")




if __name__ == "__main__":
    # test_scene_to_grid_map()
    test_grid_map_save_load()
