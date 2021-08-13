from thortils.constants import SCENE_TYPES
from thortils.scene import SceneDataset, ithor_scene_names
import pandas as pd
import scipy.spatial as ss
import numpy as np

def compute_volume(bbox3D):
    """bbox3D: object oriented bounding box"""
    hull = ss.ConvexHull(bbox3D)
    return hull.volume

SCENE_DATA_PATH = "../scenes"
OUTPUT_PATH = "../info/object_properties.csv"

SCENE_DATASET = SceneDataset.load(SCENE_DATA_PATH)
# Each row in the data is:
#  object_type, receptacle, openable, scenes_present, average_volume
DATA = []
OBJECTS = {}
for scene_type in SCENE_TYPES:
    for scene_name in ithor_scene_names(scene_type):
        try:
            scene_info = SCENE_DATASET.scene_info(scene_name)
        except KeyError:
            print("Info for {} not found".format(scene_name))
            continue

        for object_id in scene_info.objects:
            obj = scene_info.obj(object_id)
            obj_info = dict(receptacle=obj["receptacle"],
                            openable=obj["openable"])
            objtype = obj["objectType"]
            if obj["objectType"] in OBJECTS:
                try:
                    assert OBJECTS[objtype]["info"] == obj_info
                except AssertionError:
                    pass
            else:
                OBJECTS[objtype] = {}
                OBJECTS[objtype]["scenes"] = []
                OBJECTS[objtype]["volumes"] = []
                OBJECTS[objtype]["info"] = obj_info

            OBJECTS[objtype]["scenes"].append(scene_name)
            OBJECTS[objtype]["volumes"].append(compute_volume(obj["axisAlignedBoundingBox"]["cornerPoints"]))

for objectType in OBJECTS:
    d = OBJECTS[objtype]
    info = d["info"]
    row = [objectType,
           info['receptacle'],
           info['openable'],
           d["scenes"],
           np.mean(d["volumes"])]
    DATA.append(row)

df = pd.DataFrame(data=DATA,
                  columns=["objectType", "receptacle", "openable", "scenes", "avgVolume"])
df.to_csv(OUTPUT_PATH)
print(df)
