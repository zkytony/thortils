from thortils import constants
from thortils.controller import launch_controller, thor_controller_param
from thortils.map3d import Map3D, Mapper3D


def test_mapper(scene, floor_cut=0.1):
    controller = launch_controller({**constants.CONFIG, **{'scene': scene}})
    mapper = Mapper3D.automate(controller )
    mapper.map.visualize()

    # Convert the map into a 2D grid map
    mapper.map.to_grid_map(floor_cut=floor_cut)


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
