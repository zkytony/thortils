 <img src="https://user-images.githubusercontent.com/7720184/167218713-33243dc8-3e57-45e5-8264-2192377f6654.png" width="800px">


# thortils
This is a repository that contains utility functions when working with [Ai2-THOR](https://ai2thor.allenai.org/), an open-source simulator of embodied
agents in household environments. The **idea** of this repository is that even though Ai2-THOR updates its version rather
frequently with potential changes to its API, **_thortils_ will always provide the SAME API** for commonly useful
functionalities one would need ("Do one thing once"). This includes, for example:

- [Launching a controller](https://github.com/zkytony/thortils/blob/v3.3.4/thortils/controller.py#L44)
   ```python
   import thortils as tt
   
   controller = tt.launch_controller({"scene": "FloorPlan1"})
   ```

- [Get visible objects](https://github.com/zkytony/thortils/blob/v3.3.4/thortils/object.py#L68)
   ```python
   import thortils as tt
   
   controller = tt.launch_controller({"scene": "FloorPlan1"})
   event = controller.step(action="Pass")
   result = tt.thor_visible_objects(event)
   ```
   The result is a list, where each element is a dictionary that contains metadata about an object (from the `event`):
   ```
   >>> result[0]
   {'name': 'Cabinet_5e0161e9', 'position': {'x': -1.8499999046325684, 'y': 2.015000104904175, 'z': 0.3799999952316284}, 'rotation': {'x': -0.0, 'y': 90.0, 'z': -0.0}, 'visible': True, 'obstructed': False, 'receptacle': True, ...
   ```

- [Construct a 3D map of a scene as a point cloud](https://github.com/zkytony/thortils/blob/v3.3.4/tests/test_map3d.py#L10), using Open3D
  ```python
  import thortils as tt
  
  controller = tt.launch_controller({"scene": "FloorPlan1"})
  mapper = tt.map3d.Mapper3D(controller)
  mapper.automate(num_stops=20, sep=1.5)
  mapper.map.visualize()
  ```
  The output looks like:
  
  <img src='https://user-images.githubusercontent.com/7720184/167222497-74fc3f2e-9a6c-4b65-bb82-57d194bf467e.png' width='500px'>

- [Construct a **proper** 2D map by projecting the 3D map](https://github.com/zkytony/thortils/blob/v3.3.4/thortils/scene.py#L143)
  
  ```python
  # continuing from the above example
  grid_map = mapper.get_grid_map(floor_cut=0.1)  # treat bottom 0.1m as floor
  viz = tt.utils.visual.GridMapVisualizer(grid_map=grid_map, res=30)
  img = viz.render()
  viz.show_img(img)
  ```
  The output looks like
  
  <img src='https://user-images.githubusercontent.com/7720184/167223125-4258ebd2-7bbb-456f-93e9-1b2607eb6910.png' width='200px'>

  See more at [test_mapper.py](https://github.com/zkytony/thortils/blob/v3.3.4/tests/test_map3d.py#L10)


- [Projection of object detection bounding boxes onto the 2D grid map](https://github.com/zkytony/thortils/blob/v3.3.4/tests/test_project_object_detection_gridmap.py#L21)

   For code, please refer to the test `tests/test_project_object_detection_gridmap.py` linked above.
   The result looks like:
   
   <img src='https://user-images.githubusercontent.com/7720184/167223511-175a39dc-2d71-4dfb-bbee-429cbe0a61f7.png' width='500px'>

- [Get shortest path to object](https://github.com/zkytony/thortils/blob/v3.3.4/thortils/navigation.py#L342). Please refer to the linked function for details.
 


## Versions
The branches of thortils are named after the version it is built for. Currently, the version on this branch is 3.3.4. For later versions of Ai2-THOR, 
you can create a branch on top of this one, and run tests under `tests/`, and fix bugs due to the Ai2-THOR version upgrade. The API of thortils 
should stay the same or could be expanded.

## Projects that use thortils

* [COS-POMDP](https://github.com/zkytony/cos-pomdp): Code for "Towards Optimal Correlational Object Search" (ICRA 2022)
* [ai2thor-web](https://github.com/zkytony/ai2thor-web): Running AI2-THOR in browser to conduct user studies with non-technical, remote participants.


## Citation
If you find this package useful, please cite the paper "[_Towards Optimal Correlational Object Search_](https://kaiyuzheng.me/documents/papers/cospomdp-icra22-full.pdf), International Conference on Robotics and Automation (ICRA), 2022.
```
@inproceedings{zheng2022towards,
  title={Towards Optimal Correlational Object Search,
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  author={Zheng, Kaiyu and Chitnis, Rohan and Sung, Yoonchang and Konidaris, George and Tellex, Stefanie},
  year={2022}
}
```
The codebase for this paper, a good example of using this package, is here: https://github.com/zkytony/cos-pomdp


## Proper 2D Grid Map

   Ai2-THOR by default provides a "GetReachablePositions" function. You might want to use this to construct a grid map of the scene,
   but that is actually incorrect. Because there are many places in the scene that are not reachable and will be excluded. As an example
   to illustrate the problem, below is a screenshot of a kitchen scene. The left shows the first-person view, and the right shows the
   grid map obtained based on the "GetReachablePositions" function. The black cell corresponds to an occupied place, such as on the
   table or the counter. (Ignore the colors and the graph for now)
   
   <img src="https://user-images.githubusercontent.com/7720184/167220311-8e306292-4fc8-4664-b87d-8ff2c93989f1.png" width="500px">
   
   The problem is that clearly there are more occupied place that is not included in this grid map. Also, some we may not
   want to distinguish some occupied places, such as the areas inside a fridge or a cabinet. We cannot do it using this method.


   Instead, thortils provides a method [`proper_convert_scene_to_grid_map`](https://github.com/zkytony/thortils/blob/v3.3.4/thortils/scene.py#L143) which
   obtains the 2D grid map by projecting a 3D map of the scene constructed from a sequence of RGBD images collected at sampled viewpoints within the scene. This 2D grid map is an **occupancy grid map**, which means a grid cell is either _free_, _occupied_, or _unknown_. This accounts for places
   that could be inside a container (like a fridge).
   
   **The constructed 3D map (left); the ceilling and floor (middle); the walls and furnitures (right)**
   
   <img src="https://user-images.githubusercontent.com/7720184/167218713-33243dc8-3e57-45e5-8264-2192377f6654.png" width="500px">

   **The 2D projection (left) and the corresponding 2D grid map (right).** Black indicates obstacle (or occupied), gray indicates unknown (e.g. inside fridge), and cyan indicates free space (the robot can access)
   
   <img src="https://user-images.githubusercontent.com/7720184/167220947-f7c5281d-ed1f-4b97-aec7-db89193d43e6.png" width="500px">

    Note that the coordinates of this grid map are 0-based integers (instead of metric), which
   can be more convenient to work with. The granularity depends on the `grid_size` setting of the Ai2-THOR controller.

## Installation

1. Clone the repository and then install it by:
   ```
   pip install -e .
   ```

     
2. Run a little test

   ```
   cd tests
   python test_scene_to_grid_map.py
   ```
   Expected output:
   ```
   FloorPlan22
   xxx............xxxx
   xxx............xxxx
   xxx............xxxx
   xxx...........xxxxx
   xxx..........xxxxxx
   xxx.........xxxxxxx
   xxx........xxxxxxxx
   xxx.......xxxxxxxxx
   xx........xxxxxxxxx
   xx........xxxxxxx..
   ..........xxxxxxx..
   ...................
   ...................
   ...................
   ```
    (note: this is only a test; this grid map is not actually an accurate reflection of the scene.)
    
   If this works, then you should be good to go. Try running the other tests under `tests/`.
    
3. Optionally, obtain a scene dataset. You can either:

   - Download [scenes.zip](https://drive.google.com/file/d/1WcIfUusWBfrGeDw-tVQqlcdnQiRKQyE4/view?usp=sharing)
     and [scene_scatter_plots.zip](https://drive.google.com/file/d/1d3PRWkqjH6YaBvw39MFWtmUB722-DYIQ/view?usp=sharing)
     and decompress them in the root directory of this repository, or

   - Run the following scripts to generate these two datasets:
     ```
     cd scripts
     python build_scene_dataset.py ../scenes
     python create_scatter_plots.py ../scenes/ ../scene_scatter_plots
     ```
    
   This is only necessary if you would like to use the functions provided by [SceneDataset](https://github.com/zkytony/thortils/blob/v3.3.4/thortils/scene.py#L280).
   

## Organization

Inside thortils/:

* agent.py:  Functions related to the agent (e.g. pose)
* controller.py: Launching the controller, and the `thor_get` function.
* object.py: Functions related to the objects (e.g. visible objects)
* scene.py: Functions related to scenes (e.g. scene names, convert scene to grid map, ThorSceneInfo, SceneDataset)
* grid_map.py: The `GridMap` class (0-based index of coordinates). Can be converted from an Ai2thor scene
* interactions.py: Functions that correspond to calling different interaction actions in Thor (e.g. `OpenObject` means calling `controller.step(action="OpenObject")`).
* constants.py: The configuration, including parameters used as default when launching controllers.
* utils.py: Non-Thor related utility functions


## Notes on the Codebase

### Poses

In ai2thor, a pose is typically a tuple `(position, rotation)`.
Although ai2thor likes to use dictionary, we often use tuples in this codebase:

* `position` (tuple): tuple `(x, y, z)`; ai2thor uses (x, z) for robot base
* `rotation` (tuple): tuple `(x, y, z)`; pitch, yaw, roll.

   Not doing quaternion because in ai2thor the mobile robot
   can only do two of the rotation axes so there's no problem using
   Euclidean.  Will use DEGREES. Will restrict the angles to be
   between 0 to 360 (same as ai2thor).

   **yaw** refers to rotation of the agent's body.
   **pitch** refers to rotation of the camera up and down.

There are two kinds of pose representations throughout the code in this repo:

* **Full pose**: refers to a tuple `(position, rotation)`, defined below.
* **simplified pose** refers to (x, z, pitch, yaw)

### Actions

When specifying actions in ai2thor, you supply an action name and a dictionary
of parameters. For navigation actions, we also use a format as follows:
```
(action_name, (forward, h_angle, v_angle))
```
We sometimes call variables "action\_delta" or "delta" to refer to `(forward, h_angle, v_angle)`


## Command Line Usage
This is only a few functions among all that you can run on the command line.

### Start controller
```
python -m thortils.controller
```
You can specify a scene
```
python -m thortils.controller FloorPlan2
```
The following enters debugger with an event object to play with
```
python -m thortils.controller --debug
python -m thortils.controller FloorPlan2 --debug
```


### Keyboard control
In `scripts/` there is a utility program that starts a controller,
and allows you to control the agent with keyboard to navigate around.

```
python scripts/kbcontrol.py
```

Example output:
```
            w
        (MoveAhead)

    a                 d
(RotateLeft)     (RotateRight)

    e
(LookUp)

    c
(LookDown)

    q
(quit)

w | Agent pose: ((-1.25, 0.9009995460510254, 1.0), (-0.0, 270.0, 0.0))
w | Agent pose: ((-1.25, 0.9009995460510254, 1.0), (-0.0, 270.0, 0.0))
a | Agent pose: ((-1.25, 0.9009995460510254, 1.0), (-0.0, 225.0, 0.0))
d | Agent pose: ((-1.25, 0.9009995460510254, 1.0), (-0.0, 270.0, 0.0))
a | Agent pose: ((-1.25, 0.9009995460510254, 1.0), (-0.0, 225.0, 0.0))
```

## Contributor

* [Kaiyu Zheng](https://kaiyuzheng.me/)

Feel free to open issues about mistakes, or contribute directly by sending pull requests (to this REAdME documentation or to the codebase in general).
