# ROS2 - Nav2 for robots

## Dependencies
- Nav2
- perception_utils [github: aled96]

Tested with CENTAURO.

Input: Nav Target as PostStamped
Output: /cmd_vel [Twist]

### Config Folder
This folder contains the configuration files used by Nav Stack for `centauro`.

More specifically, the main params that may be modified are the following:

- costmap_common_params: contains frame information and footprint
- global_costmap_params: here there is the map_topic and the point clouds used to mark occupied/free areas
- local_costmap_params: same as for the global_costmap_params
- teb_local_planner and dwa_local_planner: are the config files related to the planner used at a local scale. Here info like min-max velocities, local footprint, tolerance are described.

### Run
By running `centuaro_nav.launch.py` you will launch:
- Nav2 nodes (controller, planner, bt_behavior, ..)
- static transform map -> odom (identity)
- rviz2
- Octomap with dynamic 2D occupancy grid based on velodyne point cloud
- valid_target_selector_node: used to adjust the nav target in case it is close/on occupied elements (which would result in planning failures)

NOTE: Valid Target Selector Node is executed only when the nav target is sent via 'set_candidate_nav_target' service. If you send a target to Nav2 with the 2D Goal of Rviz, then valid target selector is bypassed.

# Required
- odom -> pelvis
- xbot2-core running for robot's tf
- velodyne point cloud