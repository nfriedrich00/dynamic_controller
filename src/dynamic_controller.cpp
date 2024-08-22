#include "dynamic_controller/dynamic_controller.hpp"

namespace dynamic_controller
{

DynamicController::DynamicController()
: controller_loader_("nav2_core", "nav2_core::Controller")
{
}

void DynamicController::changeControllerToRPP()
{
  if (active_controller_) {
    active_controller_->deactivate();
    active_controller_->cleanup();
    active_controller_.reset();
  }
  RCLCPP_INFO(logger_, "Switching to RPP Controller");
  active_controller_ = controller_loader_.createUniqueInstance("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");
  auto name = "RegulatedPurePursuitController";
  //this is the name for the paramters in the config file!
  active_controller_->configure(parent_, name, tf_, costmap_ros_);
  active_controller_->activate();
  RCLCPP_INFO(logger_, "Switched to RPP Controller");
}

void DynamicController::changeControllerToMPPI()
{
  if (active_controller_) {
    active_controller_->deactivate();
    active_controller_->cleanup();
    active_controller_.reset();
  }
  RCLCPP_INFO(logger_, "Switching to MPPI Controller");
  active_controller_ = controller_loader_.createUniqueInstance("nav2_mppi_controller::MPPIController");
  auto name = "MPPIController";
  active_controller_->configure(parent_, name, tf_, costmap_ros_);
  active_controller_->activate();
  RCLCPP_INFO(logger_, "Switched to MPPI Controller");
}

void DynamicController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock parent node in DynamicController::configure");
  }
  try {
    primary_controller_ = controller_loader_.createUniqueInstance("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");
    secondary_controller_ = controller_loader_.createUniqueInstance("nav2_mppi_controller::MPPIController");

    primary_controller_->configure(parent, "RegulatedPurePursuitController", tf, costmap_ros);
    secondary_controller_->configure(parent, "MPPIController", tf, costmap_ros);

    // Load the regulated pure pursuit controller
//    active_controller_ = controller_loader_.createUniqueInstance("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");
//    active_controller_->configure(parent, name, tf, costmap_ros);
  } catch (const pluginlib::PluginlibException &ex) {
    RCLCPP_FATAL(logger_, "Failed to load the controller plugin. Error: %s", ex.what());
    throw std::runtime_error("Failed to load controller plugin.");
  }

  timer_ = node->create_wall_timer(
    std::chrono::seconds(30),
    std::bind(&DynamicController::timerCallback, this)
  );
}

void DynamicController::timerCallback()
{
  /*
  if (use_rpp_) {
    changeControllerToMPPI();
  } else {
    changeControllerToRPP();
  }
  */
  use_rpp_ = !use_rpp_;
}

void DynamicController::cleanup()
{
//  if (active_controller_) {
//    active_controller_->cleanup();
//  }
  if (primary_controller_) {
    primary_controller_->cleanup();
  }
  if (secondary_controller_) {
    secondary_controller_->cleanup();
  }
}

void DynamicController::activate()
{
//  if (active_controller_) {
//    active_controller_->activate();
//  }
  if (primary_controller_) {
    primary_controller_->activate();
  }
  if (secondary_controller_) {
    secondary_controller_->activate();
  }
}

void DynamicController::deactivate()
{
//  if (active_controller_) {
//    active_controller_->deactivate();
//  }
  if (primary_controller_) {
    primary_controller_->deactivate();
  }
  if (secondary_controller_) {
    secondary_controller_->deactivate();
  }
}

geometry_msgs::msg::TwistStamped DynamicController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped &pose,
  const geometry_msgs::msg::Twist &velocity,
  nav2_core::GoalChecker *goal_checker)
{

  if (use_rpp_) {
    if (primary_controller_) {
      return primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
    }
  }
  else {
    if (secondary_controller_) {
      return secondary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
    }
  }
  /*
  if (active_controller_) {
    return active_controller_->computeVelocityCommands(pose, velocity, goal_checker);
  }
*/
  throw std::runtime_error("No active controller is loaded");
}

void DynamicController::setPlan(const nav_msgs::msg::Path &path)
{
//  if (active_controller_) {
//    active_controller_->setPlan(path);
//  }
  if (primary_controller_) {
    primary_controller_->setPlan(path);
  }
  if (secondary_controller_) {
    secondary_controller_->setPlan(path);
  }
}

void DynamicController::setSpeedLimit(const double &speed_limit, const bool &percentage)
{
//  if (active_controller_) {
//    active_controller_->setSpeedLimit(speed_limit, percentage);
//  }
  if (primary_controller_) {
    primary_controller_->setSpeedLimit(speed_limit, percentage);
  }
  if (secondary_controller_) {
    secondary_controller_->setSpeedLimit(speed_limit, percentage);
  }
}

}  // namespace dynamic_controller

PLUGINLIB_EXPORT_CLASS(
  dynamic_controller::DynamicController,
  nav2_core::Controller)
