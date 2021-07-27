# Print objects in openable containers

from thortils.constants import SCENE_TYPES
from thortils.scene import SceneDataset, ithor_scene_names
from thortils.object import thor_object_type
import json

SCENE_DATA_PATH = "../scenes"
OUTPUT_PATH = "../info/containment.json"

SCENE_DATASET = SceneDataset.load(SCENE_DATA_PATH)

DATA = {}
for scene_type in SCENE_TYPES:
    print(":{}:".format(scene_type))
    DATA[scene_type] = {}
    print(scene_type)
    for scene_name in ithor_scene_names(scene_type):
        try:
            SCENE_INFO = SCENE_DATASET.scene_info(scene_name)
        except KeyError:
            print("Info for {} not found".format(scene_name))
            continue

        DATA[scene_type][scene_name] = {}

        print("---------- {} -----------".format(scene_name))
        for object_id in SCENE_INFO.objects:
            obj = SCENE_INFO.obj(object_id)
            if obj["openable"] and obj["receptacle"] and len(obj["receptacleObjectIds"]) > 0:
                obj_in_containers = list(map(thor_object_type, obj["receptacleObjectIds"]))
                print("{} contains {}".format(
                    obj["objectType"], obj_in_containers))
                container = obj["objectType"]
                DATA[scene_type][scene_name][container] = obj_in_containers
        print() # blank line

with open(OUTPUT_PATH, "w") as f:
    json.dump(DATA, f, indent=4, sort_keys=True)
