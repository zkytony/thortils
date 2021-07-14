import argparse
from thortils.scene import SceneDataset, ithor_scene_names
import thortils.constants as constants


def build_scene_dataset(rootdir):
    """Saves scene info files (.pkl files, see ThorSceneInfo)."""
    scene_names = []
    for scene_type in constants.LEVELS:
        scene_names.extend(ithor_scene_names(scene_type,
                                             levels=constants.LEVELS[scene_type]))
    SceneDataset.create(rootdir, scene_names)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Build scene dataset.')
    parser.add_argument('rootdir', type=str, help='Root directory of dataset')
    args = parser.parse_args()
    confirm = input("Build scene dataset under {}? [y] ".format(args.rootdir))
    if confirm.lower().startswith("y"):
        build_scene_dataset(args.rootdir)
