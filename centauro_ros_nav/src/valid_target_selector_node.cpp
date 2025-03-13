#include <iostream> // cout, cerr
#include <fstream> // ifstream
#include <sstream> // stringstream

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "centauro_ros_nav/valid_target_selector_manager.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<valid_target_selector::ValidTargetSelectorManager>());
  rclcpp::shutdown();
  return 0;
}