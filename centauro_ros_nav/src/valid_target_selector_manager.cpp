#include "centauro_ros_nav/valid_target_selector_manager.hpp"

using namespace valid_target_selector;

ValidTargetSelectorManager::ValidTargetSelectorManager()
: Node("valid_target_selector")
{
    initNode();
}

void ValidTargetSelectorManager::initNode(){
    // Declare params
    this->declare_parameter("map_topic_name", "");
    occupancy_map_topic_ = this->get_parameter("map_topic_name").as_string();

    RCLCPP_INFO(this->get_logger(), "I READ: %s", occupancy_map_topic_.c_str());

    footprint_radius_ = 0.0;

    // Service Server for candidate target correction
    get_candidate_target_srv_ = this->create_service<centauro_ros_nav_srvs::srv::SendCandidateNavTarget>("/set_candidate_nav_target",
                                    std::bind(&ValidTargetSelectorManager::setCandidateTarget, this, std::placeholders::_1, std::placeholders::_2));

    // Publisher
    send_nav_target_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose",
                        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

    // Subscriber
    auto occupancyCallback =
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) -> void {
         occupancy_ = msg;
      };

    occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(occupancy_map_topic_, 10, occupancyCallback);


    //Get Footprint Parameter from /global_costmap/global_costmap node
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "global_costmap/global_costmap");
    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto parameters = parameters_client->get_parameters({"footprint"});
    
    RCLCPP_INFO(this->get_logger(), "OK");

    std::string footprint_str;
    // Get a few of the parameters just set.
    footprint_str = parameters[0].value_to_string();
    RCLCPP_INFO(this->get_logger(), "Params %s", footprint_str.c_str());

    std::regex numberRegex(R"([-+]?\d*\.?\d+)");  // Regex to match floating-point numbers
    std::sregex_iterator it(footprint_str.begin(), footprint_str.end(), numberRegex);
    std::sregex_iterator end;

    std::vector<double> numbers;
    while (it != end) {
        numbers.push_back(std::stof(it->str()));  // Convert to float and store
        ++it;
    }

    // Group numbers into pairs and store them as arrays in the result vector
    for (size_t i = 0; i < numbers.size(); i += 2) {
        footprint_.push_back({numbers[i], numbers[i + 1]});
    }

    footprint_radius_ = 0.5;
}

void ValidTargetSelectorManager::setCandidateTarget (const std::shared_ptr<centauro_ros_nav_srvs::srv::SendCandidateNavTarget::Request>  request,
                                                           std::shared_ptr<centauro_ros_nav_srvs::srv::SendCandidateNavTarget::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "DefineValidTarget CALLED");
    
    if(occupancy_ == nullptr){
        RCLCPP_INFO(this->get_logger(), "occupancy is null");
        response->success = false;
        return ;
    }

    // Define needed values in grid cell
    candidate_pos_ = static_cast<int>((request->target_pose.position.x - occupancy_->info.origin.position.x)/occupancy_->info.resolution) +
                     static_cast<int>((request->target_pose.position.y - occupancy_->info.origin.position.y)/occupancy_->info.resolution)*occupancy_->info.width;
        
    radius_grid_ = 1 + static_cast<int>(footprint_radius_/occupancy_->info.resolution);
    RCLCPP_DEBUG(this->get_logger(), "Radius: %d", radius_grid_);
    //Default one is the candidate
    nav_target_.header.stamp = this->get_clock()->now();
    nav_target_.header.frame_id = request->reference_frame;
    nav_target_.pose = request->target_pose;

    //Check farthest obstacle in the occupancy (within footprint_radius_ distance from target)
    //Increase distance from center
    for(int i = radius_grid_; i >= 0; i--){
        if(checkCollisionRadius(i, request->robot_pose.position)){

            RCLCPP_DEBUG(this->get_logger(), "Robot Pose: %f %f", request->robot_pose.position.x, request->robot_pose.position.y);
            RCLCPP_DEBUG(this->get_logger(), "Colliding point: %f %f", colliding_point_[0], colliding_point_[1]);
            RCLCPP_DEBUG(this->get_logger(), "TargetCell: %d", candidate_pos_);
            RCLCPP_DEBUG(this->get_logger(), "CollidCell: %d", colliding_cell_);

            RCLCPP_DEBUG(this->get_logger(), "Dist^2 coll-robot: %f", min_dist_robot_);
            //Evalute the valid point in the opposite direction
            //TODO FastAtan
            angle_ = atan2(request->target_pose.position.y - colliding_point_[1],
                           request->target_pose.position.x - colliding_point_[0]);
            
            // Set new target pose (x, y, theta)
            // Difference if obstacle is between robot and target or if the obstacle is on the other side
            // Add 2 to be safe
            // if(min_dist_robot_ > pow(nav_target_.pose.position.x - robot.position.x, 2) + 
            //                      pow(nav_target_.pose.position.y - robot.position.y, 2))
            // {
            //     RCLCPP_INFO(this->get_logger(), "MOD: %f", static_cast<double>(2 + radius_grid_ - i)*occupancy_->info.resolution);    
            //     nav_target_.pose.position.x += static_cast<double>(2 + radius_grid_ - i)*occupancy_->info.resolution*cos(angle_);
            //     nav_target_.pose.position.y += static_cast<double>(2 + radius_grid_ - i)*occupancy_->info.resolution*sin(angle_);
            // }
            // else
            // {
            //     RCLCPP_INFO(this->get_logger(), "MOD2: %f", static_cast<double>(2 + 2*radius_grid_ - i)*occupancy_->info.resolution);
            //     nav_target_.pose.position.x += static_cast<double>(2 + radius_grid_ + i)*occupancy_->info.resolution*cos(angle_);
            //     nav_target_.pose.position.y += static_cast<double>(2 + radius_grid_ + i)*occupancy_->info.resolution*sin(angle_);
            // }
            nav_target_.pose.position.x = colliding_point_[0] + static_cast<double>(2 + radius_grid_)*occupancy_->info.resolution*cos(angle_);
            nav_target_.pose.position.y = colliding_point_[1] + static_cast<double>(2 + radius_grid_)*occupancy_->info.resolution*sin(angle_);
           
            //If rotate_to_point --> Change target angle to update final orientation
            if(request->rotate_to_point){
                angle_ = atan2(nav_target_.pose.position.y - request->point_to_face.y,
                               nav_target_.pose.position.x - request->point_to_face.x);
            }

            nav_target_.pose.orientation.z = sin(angle_/2);
            nav_target_.pose.orientation.w = cos(angle_/2);
                        
            break;
        }
    }

    response->new_target = nav_target_.pose;
    
    RCLCPP_INFO(this->get_logger(), "NEW  TARGET: %f %f - %f", nav_target_.pose.position.x,
                                                               nav_target_.pose.position.y,
                                                               angle_);
    
    response->success = true;
    send_nav_target_->publish(nav_target_);

}

bool ValidTargetSelectorManager::checkCollisionRadius(int depth, geometry_msgs::msg::Point robot){

    // Look for the occupied elements closest to the robot
    min_dist_robot_ = 100000.0;
    colliding_cell_ = -1;

    for(int j = -depth; j <= depth; j++){
        for(int k = -depth; k <= depth; k+=((abs(j) == depth)?1:2*depth)){
            temp_cell_ = candidate_pos_ + k +j*occupancy_->info.width;

            if(temp_cell_ < 0 || temp_cell_ >= static_cast<int>(occupancy_->info.width*occupancy_->info.height) ||
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

ValidTargetSelectorManager::~ValidTargetSelectorManager(){
    
}