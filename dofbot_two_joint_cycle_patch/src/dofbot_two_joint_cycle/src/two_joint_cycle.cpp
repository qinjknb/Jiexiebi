#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using MoveGroup = moveit::planning_interface::MoveGroupInterface;

namespace
{
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

int64_t to_nanoseconds(const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<int64_t>(duration.sec) * kNanosecondsPerSecond + duration.nanosec;
}

void set_duration(builtin_interfaces::msg::Duration& duration, int64_t nanoseconds)
{
  duration.sec = static_cast<int32_t>(nanoseconds / kNanosecondsPerSecond);
  duration.nanosec = static_cast<uint32_t>(nanoseconds % kNanosecondsPerSecond);
}

std::map<std::string, double> current_joint_positions(MoveGroup& group, double wait_seconds = 30.0)
{
  const auto state = group.getCurrentState(wait_seconds);
  if (!state)
  {
    return {};
  }
  return { { "Arm1_Joint", state->getVariablePosition("Arm1_Joint") },
           { "Arm2_Joint", state->getVariablePosition("Arm2_Joint") } };
}

bool near_positions(const std::map<std::string, double>& actual,
                    const std::map<std::string, double>& expected, double tolerance)
{
  for (const auto& joint : expected)
  {
    const auto found = actual.find(joint.first);
    if (found == actual.end() || !std::isfinite(found->second) ||
        std::abs(found->second - joint.second) > tolerance)
    {
      return false;
    }
  }
  return true;
}

bool wait_for_positions(MoveGroup& group, const std::map<std::string, double>& expected,
                        double tolerance)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
  {
    if (near_positions(current_joint_positions(group, 0.2), expected, tolerance))
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

std::map<std::string, double> point_positions(
    const trajectory_msgs::msg::JointTrajectory& trajectory, std::size_t index)
{
  std::map<std::string, double> result;
  for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i)
  {
    result[trajectory.joint_names[i]] = trajectory.points[index].positions[i];
  }
  return result;
}

bool validate_forward(const MoveGroup::Plan& plan,
                      const std::map<std::string, double>& cycle_start,
                      double tolerance, const rclcpp::Logger& logger)
{
  const auto& joint_trajectory = plan.trajectory.joint_trajectory;
  const std::set<std::string> expected_joints = { "Arm1_Joint", "Arm2_Joint" };
  const std::set<std::string> trajectory_joints(joint_trajectory.joint_names.begin(),
                                                joint_trajectory.joint_names.end());
  if (trajectory_joints != expected_joints || joint_trajectory.points.size() < 2 ||
      !plan.trajectory.multi_dof_joint_trajectory.points.empty())
  {
    RCLCPP_ERROR(logger, "Forward trajectory must contain only Arm1_Joint and Arm2_Joint, with at least two points");
    return false;
  }

  int64_t previous_time = -1;
  for (const auto& point : joint_trajectory.points)
  {
    const int64_t time = to_nanoseconds(point.time_from_start);
    if (point.positions.size() != 2 || time < 0 || time < previous_time)
    {
      RCLCPP_ERROR(logger, "Forward trajectory has invalid positions or timestamps");
      return false;
    }
    for (const double position : point.positions)
    {
      if (!std::isfinite(position))
      {
        RCLCPP_ERROR(logger, "Forward trajectory contains a non-finite position");
        return false;
      }
    }
    if ((!point.velocities.empty() && point.velocities.size() != 2) ||
        (!point.accelerations.empty() && point.accelerations.size() != 2))
    {
      RCLCPP_ERROR(logger, "Forward trajectory has inconsistent velocity or acceleration fields");
      return false;
    }
    previous_time = time;
  }
  if (previous_time <= 0 ||
      !near_positions(point_positions(joint_trajectory, 0), cycle_start, tolerance))
  {
    RCLCPP_ERROR(logger, "Forward trajectory does not begin at the current joint state");
    return false;
  }
  for (const std::size_t index : { std::size_t(0), joint_trajectory.points.size() - 1 })
  {
    for (const double velocity : joint_trajectory.points[index].velocities)
    {
      if (!std::isfinite(velocity) || std::abs(velocity) > 0.001)
      {
        RCLCPP_ERROR(logger, "Trajectory endpoints must have zero velocity before reversing");
        return false;
      }
    }
  }
  return true;
}

MoveGroup::Plan make_reverse_plan(const MoveGroup::Plan& forward,
                                  const moveit::core::RobotState& current_state)
{
  MoveGroup::Plan reverse = forward;
  moveit::core::robotStateToRobotStateMsg(current_state, reverse.start_state);
  auto& trajectory = reverse.trajectory.joint_trajectory;
  const int64_t total = to_nanoseconds(trajectory.points.back().time_from_start);
  std::reverse(trajectory.points.begin(), trajectory.points.end());
  for (auto& point : trajectory.points)
  {
    const int64_t original_time = to_nanoseconds(point.time_from_start);
    set_duration(point.time_from_start, total - original_time);
    for (double& velocity : point.velocities)
    {
      velocity = -velocity;
    }
    // Under time reversal, joint acceleration keeps the same sign.
  }
  trajectory.header.stamp.sec = 0;
  trajectory.header.stamp.nanosec = 0;
  return reverse;
}

int run(const rclcpp::Node::SharedPtr& node)
{
  const auto logger = node->get_logger();
  const auto goal_name = node->get_parameter_or<std::string>("goal_name", "test");
  const auto repeat_count = node->get_parameter_or<int64_t>("repeat_count", 3);
  const double speed = node->get_parameter_or<double>("speed", 0.1);
  const double pause_seconds = node->get_parameter_or<double>("pause_seconds", 0.5);
  const double endpoint_tolerance = node->get_parameter_or<double>("endpoint_tolerance", 0.01);
  if (repeat_count < 0 || !std::isfinite(speed) || speed <= 0.0 || speed > 1.0 ||
      !std::isfinite(pause_seconds) || pause_seconds < 0.0 ||
      !std::isfinite(endpoint_tolerance) || endpoint_tolerance <= 0.0 || endpoint_tolerance > 0.2)
  {
    RCLCPP_ERROR(logger, "Invalid parameters: repeat_count >= 0, speed in (0,1], pause >= 0");
    return 2;
  }

  MoveGroup arm(node, "arm_group");
  const auto active_names = arm.getActiveJoints();
  const std::set<std::string> active_joints(active_names.begin(), active_names.end());
  if (active_joints != std::set<std::string>{ "Arm1_Joint", "Arm2_Joint" })
  {
    RCLCPP_ERROR(logger, "arm_group must contain exactly Arm1_Joint and Arm2_Joint");
    return 2;
  }
  const auto goal = arm.getNamedTargetValues(goal_name);
  if (goal.size() != 2 || !goal.count("Arm1_Joint") || !goal.count("Arm2_Joint"))
  {
    RCLCPP_ERROR(logger, "Named goal '%s' must specify both arm joints", goal_name.c_str());
    return 2;
  }
  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(speed);
  arm.setMaxAccelerationScalingFactor(speed);

  const auto initial = current_joint_positions(arm);
  if (initial.size() != 2)
  {
    RCLCPP_ERROR(logger, "No initial /joint_states received");
    return 1;
  }
  RCLCPP_INFO(logger, "Captured initial Arm1=%.4f, Arm2=%.4f rad; goal '%s' Arm1=%.4f, Arm2=%.4f rad",
              initial.at("Arm1_Joint"), initial.at("Arm2_Joint"), goal_name.c_str(),
              goal.at("Arm1_Joint"), goal.at("Arm2_Joint"));
  if (near_positions(initial, goal, 0.001))
  {
    RCLCPP_ERROR(logger, "Initial state already equals goal; there is no cycle to execute");
    return 2;
  }

  for (int64_t cycle = 1; rclcpp::ok() && (repeat_count == 0 || cycle <= repeat_count); ++cycle)
  {
    const auto cycle_start = current_joint_positions(arm);
    if (!near_positions(cycle_start, initial, endpoint_tolerance))
    {
      RCLCPP_ERROR(logger, "Cycle %ld: robot is no longer at the captured initial state; stopping",
                   static_cast<long>(cycle));
      return 1;
    }

    arm.setStartStateToCurrentState();
    if (!arm.setNamedTarget(goal_name))
    {
      RCLCPP_ERROR(logger, "Cycle %ld: goal '%s' is unavailable", static_cast<long>(cycle), goal_name.c_str());
      return 1;
    }
    MoveGroup::Plan forward;
    RCLCPP_INFO(logger, "Cycle %ld: planning outbound", static_cast<long>(cycle));
    if (arm.plan(forward) != moveit::core::MoveItErrorCode::SUCCESS ||
        !validate_forward(forward, cycle_start, endpoint_tolerance, logger))
    {
      RCLCPP_ERROR(logger, "Cycle %ld: outbound planning failed; stopping", static_cast<long>(cycle));
      return 1;
    }
    const auto endpoint = point_positions(forward.trajectory.joint_trajectory,
                                          forward.trajectory.joint_trajectory.points.size() - 1);
    RCLCPP_INFO(logger, "Cycle %ld: executing outbound", static_cast<long>(cycle));
    if (arm.execute(forward) != moveit::core::MoveItErrorCode::SUCCESS ||
        !wait_for_positions(arm, endpoint, endpoint_tolerance))
    {
      RCLCPP_ERROR(logger, "Cycle %ld: outbound execution did not reach its endpoint; stopping",
                   static_cast<long>(cycle));
      return 1;
    }

    if (pause_seconds > 0.0)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(pause_seconds));
    }
    const auto state_at_goal = arm.getCurrentState(30.0);
    if (!state_at_goal)
    {
      RCLCPP_ERROR(logger, "Cycle %ld: lost joint states at goal", static_cast<long>(cycle));
      return 1;
    }
    MoveGroup::Plan reverse = make_reverse_plan(forward, *state_at_goal);
    RCLCPP_INFO(logger, "Cycle %ld: executing the exact joint path in reverse", static_cast<long>(cycle));
    if (arm.execute(reverse) != moveit::core::MoveItErrorCode::SUCCESS ||
        !wait_for_positions(arm, initial, endpoint_tolerance))
    {
      RCLCPP_ERROR(logger, "Cycle %ld: return did not reach the initial state; stopping",
                   static_cast<long>(cycle));
      return 1;
    }
    RCLCPP_INFO(logger, "Cycle %ld complete: one outbound and one return", static_cast<long>(cycle));
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
  auto node = std::make_shared<rclcpp::Node>("dofbot_two_joint_cycle", options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  int result = 1;
  try
  {
    result = run(node);
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "Cycle stopped: %s", error.what());
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return result;
}
