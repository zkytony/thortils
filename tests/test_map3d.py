from thortils import constants
from thortils.controller import launch_controller, thor_controller_param
from thortils.map3d import Map3D, Mapper3D


def test_mapper(scene):
    controller = launch_controller({**constants.CONFIG, **{'scene': scene}})
    mapper = Mapper3D.automate(controller)
    mapper.map.visualize(5)
    controller.stop()

if __name__ == "__main__":
    test_mapper("FloorPlan2")
    test_mapper("FloorPlan301")
