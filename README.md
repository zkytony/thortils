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
