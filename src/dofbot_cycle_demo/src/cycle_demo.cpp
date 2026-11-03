#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

using MoveGroup = moveit::planning_interface::MoveGroupInterface;

namespace
{
bool plan_and_execute(MoveGroup& group, const std::string& goal, const std::string& step,
                      const rclcpp::Logger& logger)
{
  if (!group.getCurrentState(30.0))
  {
    RCLCPP_ERROR(logger, "%s: no current joint state was received", step.c_str());
    return false;
  }

  group.setStartStateToCurrentState();
  if (!group.setNamedTarget(goal))
  {
    RCLCPP_ERROR(logger, "%s: named target '%s' does not exist", step.c_str(), goal.c_str());
    return false;
  }

  MoveGroup::Plan plan;
  RCLCPP_INFO(logger, "%s: planning to '%s'", step.c_str(), goal.c_str());
  if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "%s: planning failed; stopping the cycle", step.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "%s: executing", step.c_str());
  if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(logger, "%s: execution failed; stopping the cycle", step.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "%s: complete", step.c_str());
  return true;
}

int run(const rclcpp::Node::SharedPtr& node)
{
  const auto logger = node->get_logger();
  const auto arm_target = node->get_parameter_or<std::string>("arm_target", "init");
  const auto arm_home = node->get_parameter_or<std::string>("arm_home", "up");
  const auto gripper_closed = node->get_parameter_or<std::string>("gripper_closed", "close");
  const auto gripper_open = node->get_parameter_or<std::string>("gripper_open", "open");
  const auto repeat_count = node->get_parameter_or<int64_t>("repeat_count", 3);
  const auto arm_speed = node->get_parameter_or<double>("arm_speed", 0.1);
  const auto gripper_speed = node->get_parameter_or<double>("gripper_speed", 0.1);
  const auto pause_seconds = node->get_parameter_or<double>("pause_seconds", 0.5);

  if (repeat_count < 0 || arm_speed <= 0.0 || arm_speed > 1.0 || gripper_speed <= 0.0 ||
      gripper_speed > 1.0 || pause_seconds < 0.0)
  {
    RCLCPP_ERROR(logger, "Invalid parameters: repeat_count >= 0, speeds in (0, 1], pause_seconds >= 0");
    return 2;
  }

  MoveGroup arm(node, "arm_group");
  MoveGroup gripper(node, "grip_group");
  arm.setPlanningTime(10.0);
  gripper.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(10);
  gripper.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(arm_speed);
  arm.setMaxAccelerationScalingFactor(arm_speed);
  gripper.setMaxVelocityScalingFactor(gripper_speed);
  gripper.setMaxAccelerationScalingFactor(gripper_speed);

  RCLCPP_INFO(logger, "Ready: arm %s -> gripper %s -> arm %s -> gripper %s; repeat_count=%ld",
              arm_target.c_str(), gripper_closed.c_str(), arm_home.c_str(), gripper_open.c_str(),
              static_cast<long>(repeat_count));

  for (int64_t cycle = 1; rclcpp::ok() && (repeat_count == 0 || cycle <= repeat_count); ++cycle)
  {
    RCLCPP_INFO(logger, "Starting cycle %ld", static_cast<long>(cycle));
    if (!plan_and_execute(arm, arm_target, "arm to target", logger) ||
        !plan_and_execute(gripper, gripper_closed, "close gripper", logger) ||
        !plan_and_execute(arm, arm_home, "arm to home", logger) ||
        !plan_and_execute(gripper, gripper_open, "open gripper", logger))
    {
      return 1;
    }

    RCLCPP_INFO(logger, "Cycle %ld complete", static_cast<long>(cycle));
    if (pause_seconds > 0.0 && rclcpp::ok())
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(pause_seconds));
    }
  }

  RCLCPP_INFO(logger, "All requested cycles complete");
  return 0;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("dofbot_cycle_demo", options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  int exit_code = 1;
  try
  {
    exit_code = run(node);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "Cycle stopped: %s", error.what());
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
