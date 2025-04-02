# ROS2 - Nav2 for robots

## Dependencies
- Nav2
- perception_utils [github: aled96]

Tested in ROS2 with CENTAURO.

Input: Nav Target [geometry_msgs::msg::PoseStamped]
Output: /cmd_vel [geometry_msgs::msg::TwistStamped]

### Packages
This repo contains two ROS2 pacakges:
- centauro_ros_nav
- centauro_ros_nav_srvs

#### _centauro_ros_nav_
More specifically:
- _behavior_trees_: containes the BT used by Nav2 to define the behavior.
- _config/centauro_nav_params.yaml_: contains the parameters for all the nodes running within the Nav2 framework, i.e. local and global costmaps, planner, controller etc. If you want to change them, this is the config file you have to modify. 
- _launch/centauro_nav.launch.py_: launch Nav2 and other custom nodes
- _maps_: Empty at the moment.

#### _centauro_ros_nav_srvs_
ROS2 package for the definition of a custom service: `centauro_ros_nav_srvs::srv::SendCandidateNavTarget`.

### Run
By running `centuaro_nav.launch.py` you will launch:
- Nav2 nodes (controller, planner, bt_behavior, ..)
- static transform map -> odom (identity)
- rviz2
- Octomap with dynamic 2D occupancy grid based on velodyne point cloud
- Octomap 3D based on 'D435i_camera/points_filtered'
- valid_target_selector_node: used to adjust the nav target in case it is close/on occupied elements (which would result in planning failures)

NOTE: Valid Target Selector Node is executed only when the nav target is sent via 'set_candidate_nav_target' service. If you send a target to Nav2 with the 2D Goal of Rviz, then valid target selector is bypassed.
Valid Target Selector is used only if the target is sent through the `centauro_ros_nav_srvs::srv::SendCandidateNavTarget` service.

# Required
- odom -> pelvis
- xbot2-core running for robot's tf
- velodyne point cloud
- camera-pelvis TF