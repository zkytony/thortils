import argparse
import thortils
from thortils.constants import CONFIG
from thortils.utils import euclidean_dist
from operator import itemgetter
from pprint import pprint

def main():
    parser = argparse.ArgumentParser(
        description="Check whether it is possible to navigate to the goal")
    parser.add_argument("-s", "--scene", type=str, help="scene",
                        default="FloorPlan1")
    parser.add_argument("-t", "--target-class", type=str, default="PepperShaker")
    parser.add_argument("-g", "--goal-dist", type=int, default=1.0)
    args = parser.parse_args()

    controller = thortils.launch_controller({**CONFIG, **{'scene':args.scene}})
    reachable_positions = thortils.thor_reachable_positions(controller)

    obj = thortils.thor_closest_object_of_type(controller, args.target_class)
    obj_pos = thortils.thor_object_position(controller.last_event, obj["objectId"], as_tuple=True)
    obj_pos = itemgetter(0,2)(obj_pos)

    closest_reachable_pos = min(reachable_positions, key=lambda p: euclidean_dist(p, obj_pos))
    acceptable_poss = list(filter(lambda p: euclidean_dist(p, obj_pos) <= args.goal_dist,
                                  reachable_positions))

    print("For {}, the closest position the robot can be is {} with distance {}"\
          .format(args.target_class, closest_reachable_pos, euclidean_dist(closest_reachable_pos, obj_pos)))
    print("There are totally {} positions the robot can be at to be close enough to the target:".format(len(acceptable_poss)))
    pprint(acceptable_poss)

if __name__ == "__main__":
    main()
