from tqdm import tqdm
from pprint import pprint
import thortils as tt

if __name__ == "__main__":
    for catg in ["kitchen", "living_room", "bedroom", "bathroom"]:
        print("Checking {}".format(catg))
        shared_types = set()
        for scene in tqdm(tt.ithor_scene_names(scene_type=catg)):
            controller = tt.launch_controller({"scene": scene})
            object_types = tt.thor_all_object_types(controller)
            if len(shared_types) == 0:
                shared_types = object_types
            else:
                for t in set(shared_types):
                    if t not in object_types:
                        shared_types.remove(t)
            controller.stop()
        print("Shared types in {}:\n".format(shared_types))
