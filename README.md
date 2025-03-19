# ROS Nav Stack for robots

Input: Nav Target as PostStamped
Output: /cmd_vel [Twist]

### Config Folder
This folder contains the configuration files used by Nav Stack for `kyon`.

More specifically, the main params that may be modified are the following:

- costmap_common_params: contains frame information and footprint
- global_costmap_params: here there is the map_topic and the point clouds used to mark occupied/free areas
- local_costmap_params: same as for the global_costmap_params
- teb_local_planner and dwa_local_planner: are the config files related to the planner used at a local scale. Here info like min-max velocities, local footprint, tolerance are described.

### Run
By running `kyon_nav.launch` you will have:
- move_base with the Nav Stack framework (planners, costmaps)
- Octomap with dynamic occupancy grid or a staitc one loaded (specifying the one you want) 

# Dependencies
- kyon_gazebo_odom


# Launch KYON
Start the simulation in Gazebo with Kyon and an environment (with sensors running) + XBotCore
- roslaunch kyon_controller simulator.launch arms:=false feet:=true sensors:=true

Move the robot in homing pose
- rosservice call /xbotcore/homing/switch 1

Run Gazebo Odometry + RViz
- roslaunch kyon_gazebo_odom gazebo_odom.launch

Run the controller for trotting
- roslaunch kyon_controller controller.launch joy:=false xbot:=true closed_loop:=true

Run ROS Nav Stack
- roslaunch kyon_ros_nav kyon_nav.launch

Start trotting:
- rosservice call /horizon/trot/switch 1

Now you can send targets from RViz with the "2D Nav Goal" button!