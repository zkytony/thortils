import time
import thortils as tt
from thortils import constants
from thortils.controller import launch_controller, thor_controller_param
from thortils.map3d import Map3D, Mapper3D
from thortils.utils.visual import GridMapVisualizer
from thortils.agent import thor_reachable_positions


def test_mapper(scene, floor_cut=0.1):
    controller = launch_controller({**constants.CONFIG, **{'scene': scene}})
    mapper = Mapper3D(controller)

    mapper.automate(num_stops=20, sep=1.5)
    grid_map = mapper.get_grid_map(floor_cut=floor_cut, debug=False)

    # Visualize reachable positions obtained from controller
    reachable_positions = thor_reachable_positions(controller)
    highlights = []
    for thor_pos in reachable_positions:
        highlights.append(grid_map.to_grid_pos(*thor_pos))
    # show grid map
    viz = GridMapVisualizer(grid_map=grid_map, res=30)
    img = viz.render()
    img = viz.highlight(img, highlights,
                        color=(25, 214, 224), show_progress=True)
    viz.show_img(img)
    time.sleep(5)
    viz.on_cleanup()
    controller.stop()



if __name__ == "__main__":
    test_mapper("FloorPlan2")
    test_mapper("FloorPlan1")
    test_mapper("FloorPlan3")
    test_mapper("FloorPlan4")
    test_mapper("FloorPlan201", floor_cut=0.3)
    test_mapper("FloorPlan202")
    test_mapper("FloorPlan301")
    test_mapper("FloorPlan302")
    test_mapper("FloorPlan303")
    test_mapper("FloorPlan401")
    test_mapper("FloorPlan402")
    test_mapper("FloorPlan403")
