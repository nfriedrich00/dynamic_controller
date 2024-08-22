#ifndef DYNAMIC_CONTROLLER_HPP // include guard
#define DYNAMIC_CONTROLLER_HPP

#include <memory>
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
//#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace dynamic_controller
{

class DynamicController : public nav2_core::Controller
{
public:
  DynamicController();// = default;
  ~DynamicController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker) override;

  void setPlan(const nav_msgs::msg::Path &path) override;
  void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

private:
  void changeControllerToRPP(); // todo: change to universal changeController function
  void changeControllerToMPPI();
  void timerCallback();

  // Dynamically loaded controller
  // chatgpt:
  //std::shared_ptr<nav2_core::Controller> active_controller_;
  nav2_core::Controller::Ptr active_controller_;
  nav2_core::Controller::Ptr primary_controller_;
  nav2_core::Controller::Ptr secondary_controller_;
  pluginlib::ClassLoader<nav2_core::Controller> controller_loader_;
  rclcpp::Logger logger_{rclcpp::get_logger("DynamicController")};

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool use_rpp_{true};
};

}  // namespace dynamic_controller
#endif /* DYNAMIC_CONTROLLER_HPP */
