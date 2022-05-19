# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

import re
import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from ai2thor.controller import Controller
from . import constants
from .grid_map import GridMap
from .controller import (launch_controller,
                         thor_scene_from_controller,
                         thor_grid_size_from_controller)
from .agent import thor_reachable_positions
from .map3d import Mapper3D
from .utils import remap, euclidean_dist


def robothor_scene_names(scene_type="Train", levels=None, nums=None):
    scenes = []
    if scene_type == "Train":
        if levels is None:
            levels = range(1, 13)
        if nums is None:
            nums = range(1, 6)
    elif scene_type == "Val":
        if levels is None:
            levels = range(1, 4)
        if nums is None:
            nums = range(1, 6)
    else:
        raise ValueError("RoboThor has no scene type {}".format(scene_type))

    for i in levels:
        for j in nums:
            scene = "FloorPlan_{}{}_{}".format(scene_type, i, j)
            scenes.append(scene)
    return scenes

def ithor_scene_names(scene_type="kitchen", levels=None):
    """
    Returns a list of scene names.

    Args:
        scene_type (str): type of scene e.g. kitchen
        levels (enumerable): the levels you want to include.
            Note that this should always contain numbers greater than
            or equal to 1 and less than or equal to 30,
            regardless of scene_type.
    """
    if levels is not None:
        if max(levels) > 30 or min(levels) < 1:
            raise ValueError("Invalid levels. Must be >= 1 and < 31")
    scenes = dict(
        kitchen = [f"FloorPlan{i}" for i in range(1, 31)],
        living_room = [f"FloorPlan{200 + i}" for i in range(1, 31)],
        bedroom = [f"FloorPlan{300 + i}" for i in range(1, 31)],
        bathroom = [f"FloorPlan{400 + i}" for i in range(1, 31)]
    )
    if scene_type.lower() in scenes:
        if levels is None:
            return scenes[scene_type]
        else:
            return [scenes[scene_type][i-1] for i in levels]
    raise ValueError("Unknown scene type {}".format(scene_type))

def ithor_scene_type(scene):
    if not scene.startswith("FloorPlan"):
        raise ValueError("invalid scene name", scene)
    number = int(scene.split("FloorPlan")[1].strip())
    if 1 <= number <= 30:
        return "kitchen"
    elif 201 <= number <= 230:
        return "living_room"
    elif 301 <= number <= 330:
        return "bedroom"
    elif 401 <= number <= 430:
        return "bathroom"
    else:
        raise ValueError("Unknown scene", scene)


def convert_scene_to_grid_map(controller_or_reachable_positions,
                              scene_name, grid_size):
    """Converts an Ai2Thor scene to a GridMap;
    Args:
        controller_or_reachable_positions: Either a Controller,
        or reachable positions (output of thor_reachable_positions)"""
    if isinstance(controller_or_reachable_positions, Controller):
        controller = controller_or_reachable_positions
        x, z = thor_reachable_positions(controller, by_axes=True)
    else:
        reachable_positions = controller_or_reachable_positions
        if type(reachable_positions) == list or type(reachable_positions) == set:
            x, z = map(np.array, zip(*reachable_positions))
        elif type(reachable_positions) == tuple and len(reachable_positions) == 2:
            x, z = reachable_positions
        else:
            raise ValueError("Cannot understand first argument"
                             "{}".format(controller_or_reachable_positions))

    # obtain grid indices for coordinates  (origin NOT at (0,0))
    thor_gx = np.round(x / grid_size).astype(int)
    thor_gy = np.round(z / grid_size).astype(int)
    width = max(thor_gx) - min(thor_gx) + 1
    length = max(thor_gy) - min(thor_gy) + 1

    # save these for later use
    thor_gx_range = (min(thor_gx), max(thor_gx) + 1)
    thor_gy_range = (min(thor_gy), max(thor_gy) + 1)

    # remap coordinates to be nonnegative (origin AT (0,0))
    gx = remap(thor_gx, thor_gx_range[0], thor_gx_range[1], 0, width).astype(int)
    gy = remap(thor_gy, thor_gy_range[0], thor_gy_range[1], 0, length).astype(int)

    gx_range = (min(gx), max(gx)+1)
    gy_range = (min(gy), max(gy)+1)

    # Little test: can convert back
    try:
        assert all(remap(gx, gx_range[0], gx_range[1], thor_gx_range[0], thor_gx_range[1]).astype(int) == thor_gx)
        assert all(remap(gy, gy_range[0], gy_range[1], thor_gy_range[0], thor_gy_range[1]).astype(int) == thor_gy)
    except AssertionError as ex:
        print("Unable to remap coordinates")
        raise ex

    # grid map positions
    positions = set(zip(gx, gy))

    # grid map dimensions
    # obstacles: locations that do not fall into valid positions
    obstacles = {(x,y)
                 for x in gx
                 for y in gy
                 if (x,y) not in positions}

    grid_map = GridMap(width, length, obstacles,
                       name=scene_name,
                       ranges_in_thor=(thor_gx_range, thor_gy_range),
                       grid_size=grid_size)

    return grid_map


def proper_convert_scene_to_grid_map(controller,
                                     floor_cut=0.1,
                                     ceiling_cut=1.0,
                                     **kwargs):
    """
    kwargs are optional arguments for Mapper3D.automate.
    Includes:
        num_stops=20,
        num_rotates=4,
        sep=1.25,
        downsample=True,
        **kwargs):
    """
    # Will use Mapper3D
    mapper = Mapper3D(controller)
    mapper.automate(**kwargs)
    grid_map = mapper.get_grid_map(floor_cut=floor_cut,
                                   ceiling_cut=ceiling_cut)
    return grid_map


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
    def load_single(cls, rootdir, scene_name_with_seed):
        if "-" not in scene_name_with_seed:
            # There is no seed in the supplied scene name
            scene_name_with_seed += "-default"

        scene_pickle_file = SceneDataset.scene_info_file(rootdir, scene_name_with_seed)
        try:
            with open(scene_pickle_file, "rb") as f:
                scene_info = ThorSceneInfo.from_json(pickle.load(f))
            return scene_info
        except FileNotFoundError as ex:
            print("{} not found".format(scene_pickle_file))
            raise ex

    @classmethod
    def load(cls, rootdir, pass_if_not_found=True):
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
            try:
                scene_info = cls.load_single(rootdir, scene_name_with_seed)
                if scene_name not in scenes:
                    scenes[scene_name] = {}
                scenes[scene_name][init_poses_seed] = scene_info
            except FileNotFoundError:
                if pass_if_not_found:
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
            print("Gathering scene data for {}".format(scene_name))
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
