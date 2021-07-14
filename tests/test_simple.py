from thortils import (launch_controller,
                      load_scene_info,
                      convert_scene_to_grid_map)

floor_plan = "FloorPlan22"
scene_info = load_scene_info(floor_plan)
controller = launch_controller({"scene":floor_plan})
grid_map = convert_scene_to_grid_map(controller, scene_info, 0.25)

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
