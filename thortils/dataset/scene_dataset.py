# Ai2Thor scene dataset.  The scene dataset stores initial configuration of
# scenes (object data, including initial poses).  Uses the random spawn
# functionality to generate scenes.  Note that you can use "SetObjectPoses"
# action to configure object poses in a scene (doc:
# https://allenai.github.io/ai2thor-v2.1.0-documentation/actions/initialization;
# Note: there does seem to be bugs related to SetObject Poses:
#  - https://github.com/allenai/ai2thor/issues/619 (high integrity scene restoration)
#     -- this is fixed in release 2.7.3
#  - https://github.com/allenai/ai2thor/pull/758
# ===> We are currently using ai2thor v2.7.2 (NEED TO UPGRADE. TODO)
import os
import re
import pickle
import dls.constants as constants
import json
import matplotlib.pyplot as plt
import random
from ..controller import launch_controller
from ..scene import ithor_scene_names
from dls.utils import euclidean_dist

def match_scene_name(scene_name):
    """
    Args:
        scene_name (str): FloorPlanXX-<random_seed(int) | default>

    Returns a tuple of scene name and seed (or 'default') if we
    can match the format."""
    m = re.match("^FloorPlan[0-9]+-([0-9]+|default)$", scene_name)
    if m is not None:
        return m.group().split("-")
    return None


class ThorSceneInfo:
    def __init__(self, scene_name_with_seed, objects):
        """
        A ThorSceneInfo is constructed by the name of the scene
        and the objects (mapping from object id to object data) in the scene.

        Args:
            scene_name_with_seed (str): name of scene.
                Format: FloorPlanXX-<random_seed(int) | default>

            objects (dict): Map from object id (ai2thor format) to attributes
        """
        self.scene_name, self.seed = match_scene_name(scene_name_with_seed)
        self.objects = objects

        # builds a map from object type to set of object ids
        self._type_to_objid = {}
        for objid in self.objects:
            objtype = self.objtype(objid)
            if objtype not in self._type_to_objid:
                self._type_to_objid[objtype] = set()
            self._type_to_objid[objtype].add(objid)

    def objtype(self, objid):
        return self.objects[objid]["objectType"]

    def obj(self, objid):
        """Returns the dict of thor data structure given pomdp objid"""
        return self.objects[objid]

    def has_object_type(self, objtype):
        return objtype in self._type_to_objid

    def all_object_types(self):
        return list(sorted(self._type_to_objid.keys()))

    def objects_of_type(self, objtype):
        """Returns set of object ids of given type"""
        return self._type_to_objid[objtype]

    @property
    def floor(self):
        return self.scene_name.split("-")[0]

    def to_json(self):
        """Converts to json string"""
        return {"scene_name_with_seed": "{}-{}".format(self.scene_name, self.seed),
                "objects": self.objects}

    @classmethod
    def from_json(cls, json_str_or_obj):
        if type(json_str_or_obj) == str:
            jsonobj = json.loads(json_str_or_obj)
        else:
            jsonobj = json_str_or_obj

        if "scene_name" in jsonobj:
            # Older version
            scene_name_with_seed = jsonobj["scene_name"]
        else:
            scene_name_with_seed = jsonobj["scene_name_with_seed"]
        return ThorSceneInfo(scene_name_with_seed, jsonobj["objects"])

    def pose2d(self, objid):
        obj = self.objects[objid]
        thor_pose = obj["position"]["x"], obj["position"]["z"]
        return thor_pose

    def plot_scatter(self, ax=None):
        """Plot object locations"""
        if ax is None:
            fig, ax = plt.subplots(figsize=(4, 4))
        dirpath = os.path.join("data", "plots")
        os.makedirs(dirpath, exist_ok=True)

        x = []
        z = []
        texts = []

        for objid in self.objects:
            if self.objtype(objid) in constants.OBJTYPES_ON_PLOT:
                thor_x, thor_z = self.pose2d(objid)

                add = True
                for i in range(len(x)):
                    if euclidean_dist((x[i], z[i]),
                                      (thor_x, thor_z)) < constants.SCATTER_GRANULARITY:
                        add = False
                        break
                if add:
                    x.append(thor_x)
                    z.append(thor_z)
                    texts.append(self.objtype(objid)) #ax.text(x[-1], z[-1],

        ax.scatter(x, z)
        for i, label in enumerate(texts):
            if label in constants.LARGE_RECEPTABLES:
                ax.text(x[i], z[i], label, rotation=30, weight='bold')
            else:
                ax.text(x[i], z[i], label, rotation=30)
        ax.set_title("{} (seed={})".format(self.scene_name, self.seed))



class SceneDataset:
    """A scene dataset consists of multiple scenes (i.e. ThorSceneInfo objects)"""
    def __init__(self, scene_infos):
        """
        Args:
            scene_infos (dict): Maps from scene_name to
                                      {random_seed | "default" -> ThorSceneInfo }
        """
        self._infos = scene_infos

    def scene_info(self, scene_name, seed="default"):
        return self._infos[scene_name][seed]

    def has_scene(self, scene_name):
        return scene_name in self._infos

    @classmethod
    def load(cls, rootdir):
        """
        Args:
            rootdir (str): The directory where this dataset is stored

                The structure of this directory is simply:
                    <scene_name_with_seed>/
                        scene_info-<scene_name_with_seed>.pkl

                where <scene_name_with_seed> is "FloorPlanXX-<random_seed(int) | default>"
        """
        scenes = {}
        for scene_name_with_seed in os.listdir(rootdir):
            m = match_scene_name(scene_name_with_seed)
            if m is None:
                continue
            scene_name, init_poses_seed = m
            scene_pickle_file = SceneDataset.scene_info_file(rootdir, scene_name_with_seed)
            try:
                with open(scene_pickle_file, "rb") as f:
                    scene_info = ThorSceneInfo.from_json(pickle.load(f))

                if scene_name not in scenes:
                    scenes[scene_name] = {}
                scenes[scene_name][init_poses_seed] = scene_info

            except FileNotFoundError:
                print("{} not found".format(scene_pickle_file))
                pass
        return SceneDataset(scenes)

    @staticmethod
    def scene_info_file(rootdir, scene_name):
        return os.path.join(rootdir, scene_name, "scene_info-{}.pkl".format(scene_name))

    @classmethod
    def create(cls, rootdir, scene_names):
        """
        Creates a dataset of scene infos. Each scene info will be a pickle file that
        contains a JSON string of scene information (using pickle file for faster loading).

        Args:
            scene_names (list or dict):
                Either a list of scene_names,
                or a dict of "scene_name -> [seeds]"
            rootdir (str): Path to save the data
        """
        data = {}
        for scene_name in scene_names:
            # Get seeds. Use "default" if no seed provided
            if type(scene_names) == dict:
                seeds = scene_names[scene_name]
            else:
                seeds = ["default"]

            # Controller configs
            config = {
                "scene_name": scene_name,
                "agent_mode": constants.AGENT_MODE,
                "visibility_distance": constants.VISIBILITY_DISTANCE,
                "fov": constants.FOV,
                "grid_size": constants.GRID_SIZE
            }

            controller = launch_controller(config)
            for seed in seeds:
                # see docs here: https://ai2thor.allenai.org/ithor/documentation/objects/domain-randomization
                if seed != "default":
                    controller.step(action="InitialRandomSpawn",
                                    randomSeed=int(seed),
                                    forceVisible=constants.FORCE_VISIBLE,
                                    placeStationary=constants.PLACE_STATIONARY)
                # Obtain scene objects info
                scene_name_with_seed = "{}-{}".format(scene_name, seed)
                event = controller.step(action="Pass")
                objects = {obj["objectId"] : obj
                           for obj in event.metadata["objects"]}
                scene_info_json = ThorSceneInfo(scene_name_with_seed, objects).to_json()

                os.makedirs(os.path.join(rootdir, scene_name_with_seed), exist_ok=True)
                scene_info_file = SceneDataset.scene_info_file(rootdir, scene_name_with_seed)
                with open(scene_info_file, "wb") as f:
                    pickle.dump(scene_info_json, f)
            controller.stop()

    def plot_scatter_plots(self, outdir, scenes=None):
        os.makedirs(outdir, exist_ok=True)
        for scene_name in sorted(self._infos):
            if scenes is not None and scene_name not in scenes:
                continue
            for seed in sorted(self._infos[scene_name]):
                scene_info = self.scene_info(scene_name, seed=seed)
                print("Plotting {}-{}".format(scene_name, seed))
                scene_info.plot_scatter()

                scene_name_with_seed = "{}-{}".format(scene_name, seed)
                plot_filename = scene_name_with_seed + ".png"
                plt.savefig(os.path.join(outdir, plot_filename), dpi=300)
                print("saved {}".format(plot_filename))
                plt.clf()


# def build_scene_dataset(rootdir):
#     scene_names = []
#     for scene_type in constants.LEVELS:
#         scene_names.extend(ithor_scene_names(scene_type,
#                                              levels=constants.LEVELS[scene_type]))
#     SceneDataset.create(rootdir, scene_names)

# if __name__ == "__main__":
#     rootdir = "./scenes"
#     confirm = input("Build scene dataset under {}? [y] ".format(rootdir))
#     if confirm.lower().startswith("y"):
#         build_scene_dataset(rootdir)


# ######## TEMPORARY CODE
# rootdir = "./scenes_json"
# os.makedirs(rootdir, exist_ok=True)
# for root, dirs, files in os.walk("./scenes"):
#     scene_name = os.path.basename(root)
#     m = match_scene_name(scene_name)
#     if m is not None:
#         scene_name = "-".join(m)
#         print(scene_name)

#         for filename in files:
#             if filename.endswith(".pkl"):
#                 with open(os.path.join(root, filename), "rb") as f:
#                     scene_info = pickle.load(f)
#                 scene_info_json = scene_info.to_json()
#                 os.makedirs(os.path.join(rootdir, scene_name), exist_ok=True)
#                 with open(os.path.join(rootdir, scene_name, filename), "wb") as f:
#                     pickle.dump(scene_info_json, f)
#             print("   ", filename)

########## Used to create scatter plots
def create_scatter_plots():
    dataset = SceneDataset.load("./scenes")
    dataset.plot_scatter_plots("./scene_scatter_plots")

if __name__ == "__main__":
    create_scatter_plots()
