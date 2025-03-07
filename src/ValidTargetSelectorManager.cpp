#include <centauro_ros_nav/ValidTargetSelectorManager.h>

ValidTargetSelectorManager::ValidTargetSelectorManager( std::string ns, double rate )
//   : nh_ ( ns ),
{
    //initialization ROS node
    
    ROS_INFO_STREAM ("I am initializing the Node...");
    initROSNode(rate);

    ROS_INFO_STREAM ("Init Params...");
    initParams();    
        
    //initialization done
    ROS_INFO_STREAM ("Initialization done.");
}

void ValidTargetSelectorManager::initROSNode(double rate){
    // init ROS node
    rate_ = rate;
    period_ = 1.0 / rate_;
    timer_ = nh_.createTimer(ros::Duration(period_), &ValidTargetSelectorManager::main_loop, this, false, false);
    time_ = 0.0;
    
    get_candidate_target_srv_ = nh_.advertiseService("/set_candidate_nav_target", &ValidTargetSelectorManager::setCandidateTarget, this);
    send_nav_target_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    ros::param::get("~/map_topic_name", occupancy_map_topic_);
    occupancy_sub_ = nh_.subscribe(occupancy_map_topic_, 10, &ValidTargetSelectorManager::occupancyCallback, this);
}

void ValidTargetSelectorManager::initParams(){
    ros::param::get("~/footprint_param_name", footprint_param_name_);

    footprint_radius_ = 0.0;

    //Get footprint
    XmlRpc::XmlRpcValue param;
    if (nh_.getParam(footprint_param_name_, param)) {
        if (param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < param.size(); ++i) {
                if (param[i].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                    std::vector<double> inner_list;
                    for (int j = 0; j < param[i].size(); ++j) {
                        if (param[i][j].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                            inner_list.push_back(static_cast<double>(param[i][j]));

                            if(inner_list[inner_list.size()-1] > footprint_radius_)
                                footprint_radius_ = inner_list[inner_list.size()-1];

                        } else {
                            ROS_ERROR("Invalid data type inside list.");
                        }
                    }
                    footprint_.push_back({inner_list[0], inner_list[1]});
                }
            }
        }
    }
}

void ValidTargetSelectorManager::occupancyCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    occupancy_ = msg;
}

bool ValidTargetSelectorManager::setCandidateTarget(centauro_ros_nav::SendCandidateNavTarget::Request  &req,
                                                    centauro_ros_nav::SendCandidateNavTarget::Response &res)
{
    res.success = defineValidTarget(req.target_pose, req.robot_pose);

    if(res.success)
        send_nav_target_.publish(nav_target_);

    return res.success;
}

bool ValidTargetSelectorManager::defineValidTarget(geometry_msgs::PoseStamped& target, geometry_msgs::Pose& robot){
    ROS_WARN("SERVICE CALLED");
    
    if(occupancy_ == nullptr){
        ROS_WARN("occupancy is null");
        return false;
    }

    // Define needed values in grid cell
    candidate_pos_ = static_cast<int>((target.pose.position.x - occupancy_->info.origin.position.x)/occupancy_->info.resolution) +
                     static_cast<int>((target.pose.position.y - occupancy_->info.origin.position.y)/occupancy_->info.resolution)*occupancy_->info.width;
        
    radius_grid_ = 1 + static_cast<int>(footprint_radius_/occupancy_->info.resolution);
    ROS_WARN("Radius: %d", radius_grid_);
    //Default one is the candidate
    nav_target_ = target;

    //Check farthest obstacle in the occupancy (within footprint_radius_ distance from target)
    //Increase distance from center
    for(int i = radius_grid_; i > 0; i--){
        if(checkCollisionRadius(i, robot.position)){

            ROS_WARN("Robot Pose: %f %f", robot.position.x, robot.position.y);
            ROS_WARN("Colliding point: %f %f", colliding_point_[0], colliding_point_[1]);
            ROS_WARN("TargetCell: %d", candidate_pos_);
            ROS_WARN("CollidCell: %d", colliding_cell_);

            ROS_WARN("Dist^2 coll-robot: %f", min_dist_robot_);
            //Evalute the valid point in the opposite direction
            //TODO FastAtan
            angle_ = atan2(robot.position.y - colliding_point_[1],
                           robot.position.x - colliding_point_[0]);
            
            // Set new target pose (x, y, theta)
            // Difference if obstacle is between robot and target or if the obstacle is on the other side
            // Add 2 to be safe
            // if(min_dist_robot_ > pow(nav_target_.pose.position.x - robot.position.x, 2) + 
            //                      pow(nav_target_.pose.position.y - robot.position.y, 2))
            // {
            //     ROS_WARN("MOD: %f", static_cast<double>(2 + radius_grid_ - i)*occupancy_->info.resolution);    
            //     nav_target_.pose.position.x += static_cast<double>(2 + radius_grid_ - i)*occupancy_->info.resolution*cos(angle_);
            //     nav_target_.pose.position.y += static_cast<double>(2 + radius_grid_ - i)*occupancy_->info.resolution*sin(angle_);
            // }
            // else
            // {
            //     ROS_WARN("MOD2: %f", static_cast<double>(2 + 2*radius_grid_ - i)*occupancy_->info.resolution);
            //     nav_target_.pose.position.x += static_cast<double>(2 + radius_grid_ + i)*occupancy_->info.resolution*cos(angle_);
            //     nav_target_.pose.position.y += static_cast<double>(2 + radius_grid_ + i)*occupancy_->info.resolution*sin(angle_);
            // }
            nav_target_.pose.position.x = colliding_point_[0] + static_cast<double>(2 + radius_grid_)*occupancy_->info.resolution*cos(angle_);
            nav_target_.pose.position.y = colliding_point_[1] + static_cast<double>(2 + radius_grid_)*occupancy_->info.resolution*sin(angle_);
           
            
            nav_target_.pose.orientation.z = sin(angle_/2);
            nav_target_.pose.orientation.w = cos(angle_/2);
            
            break;
        }
    }
    
    ROS_WARN("TARGET: %f %f - %f", nav_target_.pose.position.x, nav_target_.pose.position.y, angle_);
    
    return true;
}

bool ValidTargetSelectorManager::checkCollisionRadius(int depth, geometry_msgs::Point robot){

    // Look for the occupied elements closest to the robot
    min_dist_robot_ = 100000.0;
    colliding_cell_ = -1;

    for(int j = -depth; j <= depth; j++){
        for(int k = -depth; k <= depth; k+=((abs(j) == depth)?1:2*depth)){
            temp_cell_ = candidate_pos_ + k +j*occupancy_->info.width;

            if(temp_cell_ < 0 || temp_cell_ >= occupancy_->info.width*occupancy_->info.height ||
               occupancy_->data[temp_cell_] >= 100)
            {
                colliding_point_[0] = (temp_cell_%occupancy_->info.width)*occupancy_->info.resolution + occupancy_->info.origin.position.x;
                colliding_point_[1] = (temp_cell_/occupancy_->info.width)*occupancy_->info.resolution + occupancy_->info.origin.position.y;
            
                temp_dist_ = pow(colliding_point_[0] - robot.x, 2) + 
                             pow(colliding_point_[1] - robot.y, 2);

                if(temp_dist_ < min_dist_robot_){
                    min_dist_robot_ = temp_dist_;
                    colliding_cell_ = temp_cell_;
                }
            }
        }
    }

    return colliding_cell_ >= 0;
}

void ValidTargetSelectorManager::main_loop(const ros::TimerEvent& timer){
    time_ += period_; // update time
}

void ValidTargetSelectorManager::spin(){
    timer_.start();
    //ROS_INFO_STREAM("ForceEstim started looping time " << 1.0/_period << "Hz");
    ros::spin();
}

ValidTargetSelectorManager::~ValidTargetSelectorManager(){
    
}