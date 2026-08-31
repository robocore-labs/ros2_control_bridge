#pragma once
// A ros2_control SystemInterface that talks to a motor driver over ROS topics
// instead of a serial bus.
//
// NOTHING HERE KNOWS WHAT ROBOT IT IS ON. Joints are sorted into groups by a
// per-joint `group` parameter in the URDF's <ros2_control> block, each group
// gets one Float64MultiArray publisher, and a group's command interface is
// whatever its joints declare. Add a group by naming it on a joint; there is no
// list of group names in this file.
//
// It replaced a version with three hard-coded groups — base (velocity), arm
// (position) and camera (position) — where "camera" was decided by testing
// whether the joint's name contained the substring "camera". A pan/tilt joint
// called `hn_pan_joint` silently joined the arm group and got published on the
// arm's topic, in the arm's array, at an index the arm's driver owned.
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ros2_control_bridge {

// One command topic, one array, one interface type. `joints` is in URDF order,
// and that order IS the array layout the driver on the other end indexes into —
// it is the contract between this plugin and the motor driver's
// `command_indices`, so reordering joints in the URDF reorders the wire format.
struct Group
{
  std::string name;
  std::string topic;
  std::string interface;                 // "position" or "velocity"
  std::vector<std::string> joints;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub;
};

class TopicBridge : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TopicBridge)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // -- parameters --
  // Derived from the <ros2_control> system name in on_init, so two instances
  // in one controller_manager do not collide.
  std::string node_name_{"ros2_control_topic_bridge"};
  std::string state_topic_{"/motor_manager/joint_states"};
  std::string cmd_topic_prefix_{"/motor_manager"};
  std::string default_group_{"default"};
  bool state_best_effort_{true};

  // -- structure, all derived from the URDF --
  std::vector<Group> groups_;

  // -- buffers, keyed by joint name --
  std::unordered_map<std::string, double> cmd_;         // rad, or rad/s
  std::unordered_map<std::string, double> pos_state_;
  std::unordered_map<std::string, double> vel_state_;

  // SEEDING IS A SAFETY INTERLOCK, NOT A CONVENIENCE. A position group must not
  // be published before the real joint positions are known, or the first
  // message commands whatever the command interfaces happen to hold (0.0) and
  // the arm snaps to its zero pose. `seen_` records which joints have appeared
  // in a JointState; once every owned joint has, commands are seeded from the
  // measured positions and publishing begins.
  //
  // The version this replaced waited instead for any arm joint to report a
  // position more than 0.005 rad from zero. An arm genuinely parked at its zero
  // pose never satisfied that, and the bridge held position forever.
  std::unordered_map<std::string, bool> seen_;
  bool seeded_{false};

  // -- ROS --
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::mutex state_mtx_;

  void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  std::string param_or(const std::string & key, const std::string & fallback) const;
};

}  // namespace ros2_control_bridge
