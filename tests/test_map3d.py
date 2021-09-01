from thortils import constants
from thortils.controller import launch_controller, thor_controller_param
from thortils.map3d import Map3D, Mapper3D


def test_mapper(scene):
    controller = launch_controller({**constants.CONFIG, **{'scene': scene}})
    mapper = Mapper3D.automate(controller )
    mapper.map.visualize()

    # Convert the map into a 2D grid map
    mapper.map.to_grid_map()


    controller.stop()



if __name__ == "__main__":
    test_mapper("FloorPlan2")
    test_mapper("FloorPlan301")
