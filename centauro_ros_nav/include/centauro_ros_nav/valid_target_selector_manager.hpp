#ifndef __VALID_TARGET_SELECT__
#define __VALID_TARGET_SELECT__

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <fstream>
#include <regex>

//#include <XmlRpcValue.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "centauro_ros_nav_srv/srv/send_candidate_nav_target.hpp"

using namespace std::chrono_literals;

namespace valid_target_selector{
    
class ValidTargetSelectorManager : public rclcpp::Node {

    public:
        ValidTargetSelectorManager();
        ~ValidTargetSelectorManager();

    private:
        
        rclcpp::TimerBase::SharedPtr timer_pub_pose_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr send_nav_target_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
        
        rclcpp::Service<centauro_ros_nav_srv::srv::SendCandidateNavTarget>::SharedPtr get_candidate_target_srv_;

        nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_;

        geometry_msgs::msg::PoseStamped nav_target_;
        std::string occupancy_map_topic_;

        std::vector<std::array<double,2>> footprint_;
        double footprint_radius_; //Overestimate

        int candidate_pos_, temp_cell_, radius_grid_, colliding_cell_;
        std::array<double, 2> colliding_point_;

        double angle_, temp_dist_, min_dist_robot_;

        //-------------------------------------------
        void initNode();
        
        bool defineValidTarget(geometry_msgs::msg::PoseStamped& target, geometry_msgs::msg::Pose& robot);

        void setCandidateTarget (const std::shared_ptr<centauro_ros_nav_srv::srv::SendCandidateNavTarget::Request> request,
                                 std::shared_ptr<centauro_ros_nav_srv::srv::SendCandidateNavTarget::Response>      response);

        bool checkCollisionRadius(int depth, geometry_msgs::msg::Point robot);
};

}

#endif
