# ROS1 Nav Stack for robots

Tested with Kyon (+MPC) and CENTAURO.

Input: Nav Target as PostStamped
Output: /cmd_vel [Twist]

### Dependencies:
- sudo apt-get install ros-noetic-move-base
- sudo apt-get install ros-noetic-octomap-server
- sudo apt-get install ros-noetic-dwa-local-planner
- sudo apt-get install ros-noetic-ros-noetic-teb-local-planner

### Config Folder
This folder contains the configuration files used by Nav Stack for `centauro` and `kyon`.

More specifically, the main params that may be modified are the following:

- costmap_common_params: contains frame information and footprint
- global_costmap_params: here there is the map_topic and the point clouds used to mark occupied/free areas
- local_costmap_params: same as for the global_costmap_params
- teb_local_planner and dwa_local_planner: are the config files related to the planner used at a local scale. Here info like min-max velocities, local footprint, tolerance are described.

### Run
By running `centuaro_nav.launch` you will have:
- move_base with the Nav Stack framework (planners, costmaps)
- Octomap with dynamic occupancy grid or a staitc one loaded (specifying the one you want) 
