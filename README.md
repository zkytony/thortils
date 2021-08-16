# thortils
Code related to Ai2-Thor. Try to do one thing once.


## Setup

1. Clone the repository and then install it by:
   ```
   pip install -e .
   ```

2. Obtain scene dataset. You can either:

   - Download [scenes.zip](https://drive.google.com/file/d/1WcIfUusWBfrGeDw-tVQqlcdnQiRKQyE4/view?usp=sharing)
     and [scene_scatter_plots.zip](https://drive.google.com/file/d/1d3PRWkqjH6YaBvw39MFWtmUB722-DYIQ/view?usp=sharing)
     and decompress them in the root directory of this repository, or

   - Run the following scripts to generate these two datasets:
     ```
     cd scripts
     python build_scene_dataset.py ../scenes
     python create_scatter_plots.py ../scenes/ ../scene_scatter_plots
     ```
3. Run a little test

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


## Documentations

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


## Tips

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
