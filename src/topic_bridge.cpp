#include "ros2_control_bridge/topic_bridge.hpp"

#include <cctype>

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace ros2_control_bridge {

std::string TopicBridge::param_or(const std::string & key, const std::string & fallback) const
{
  auto it = info_.hardware_parameters.find(key);
  return it == info_.hardware_parameters.end() ? fallback : it->second;
}

// A node name must be a valid ROS name: alphanumerics and underscores, not
// starting with a digit. System names come from the URDF and are conventionally
// already fine, but they are author-supplied, so anything else is folded to an
// underscore rather than trusted.
static std::string sanitize_node_name(const std::string & in)
{
  std::string out;
  out.reserve(in.size());
  for (char c : in) {
    out.push_back((std::isalnum(static_cast<unsigned char>(c)) || c == '_') ? c : '_');
  }
  if (out.empty() || std::isdigit(static_cast<unsigned char>(out.front()))) {
    out.insert(out.begin(), '_');
  }
  return out;
}

CallbackReturn TopicBridge::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  node_name_ = sanitize_node_name(info.name.empty() ? "ros2_control_topic_bridge"
                                                    : info.name + "_bridge");

  state_topic_      = param_or("state_topic", state_topic_);
  cmd_topic_prefix_ = param_or("cmd_topic_prefix", cmd_topic_prefix_);
  default_group_    = param_or("default_group", default_group_);
  const std::string be = param_or("state_best_effort", "true");
  state_best_effort_ = (be == "true" || be == "1");

  auto logger = rclcpp::get_logger("ros2_control_bridge");

  for (const auto & j : info_.joints) {
    // One command interface per joint, and it decides the group's wire format.
    // Two on one joint would make the group's array ambiguous — which of the
    // two does index i carry? — so it is rejected rather than guessed at.
    if (j.command_interfaces.size() != 1) {
      RCLCPP_ERROR(logger,
        "joint '%s' declares %zu command interfaces; this bridge supports exactly one "
        "(the group's array carries one value per joint)",
        j.name.c_str(), j.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    const std::string iface = j.command_interfaces[0].name;
    if (iface != HW_IF_POSITION && iface != HW_IF_VELOCITY) {
      RCLCPP_ERROR(logger, "joint '%s': unsupported command interface '%s' (want position or velocity)",
                   j.name.c_str(), iface.c_str());
      return CallbackReturn::ERROR;
    }

    auto gp = j.parameters.find("group");
    const std::string gname = (gp == j.parameters.end()) ? default_group_ : gp->second;

    auto git = std::find_if(groups_.begin(), groups_.end(),
                            [&](const Group & g) { return g.name == gname; });
    if (git == groups_.end()) {
      Group g;
      g.name = gname;
      g.interface = iface;
      // Per-group override, else <prefix>/<name>_cmd. The convention is the
      // motor driver's default too, so a matching pair needs no topic
      // parameters at all on either side.
      g.topic = param_or("group." + gname + ".topic", cmd_topic_prefix_ + "/" + gname + "_cmd");
      groups_.push_back(g);
      git = std::prev(groups_.end());
    } else if (git->interface != iface) {
      RCLCPP_ERROR(logger,
        "group '%s' mixes interfaces: joint '%s' is '%s' but the group is '%s'. One array "
        "cannot be both positions and velocities — split them into two groups.",
        gname.c_str(), j.name.c_str(), iface.c_str(), git->interface.c_str());
      return CallbackReturn::ERROR;
    }

    git->joints.push_back(j.name);
    cmd_[j.name] = 0.0;
    pos_state_[j.name] = 0.0;
    vel_state_[j.name] = 0.0;
    seen_[j.name] = false;
  }

  if (groups_.empty()) {
    RCLCPP_ERROR(logger, "no joints declared — nothing to bridge");
    return CallbackReturn::ERROR;
  }

  // Log the whole mapping. This is the one place the robot's shape becomes
  // visible, and a joint in the wrong group is otherwise silent until it moves.
  for (const auto & g : groups_) {
    std::ostringstream js;
    for (size_t i = 0; i < g.joints.size(); ++i) js << (i ? ", " : "") << i << ":" << g.joints[i];
    RCLCPP_INFO(logger, "group '%s' [%s] -> %s  {%s}",
                g.name.c_str(), g.interface.c_str(), g.topic.c_str(), js.str().c_str());
  }
  RCLCPP_INFO(logger, "state topic %s (%s)", state_topic_.c_str(),
              state_best_effort_ ? "best effort" : "reliable");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TopicBridge::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> v;
  // Position AND velocity for every joint regardless of its command interface:
  // joint_trajectory_controller asks for velocity state even on a
  // position-commanded joint, and refuses to configure without it.
  for (const auto & g : groups_) {
    for (const auto & n : g.joints) {
      v.emplace_back(n, HW_IF_POSITION, &pos_state_[n]);
      v.emplace_back(n, HW_IF_VELOCITY, &vel_state_[n]);
    }
  }
  return v;
}

std::vector<hardware_interface::CommandInterface> TopicBridge::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> v;
  for (const auto & g : groups_) {
    for (const auto & n : g.joints) {
      v.emplace_back(n, g.interface, &cmd_[n]);
    }
  }
  return v;
}

CallbackReturn TopicBridge::on_configure(const rclcpp_lifecycle::State &)
{
  // Named after the <ros2_control> system, not a constant. Several systems can
  // each load their own instance — an end-effector on the same bus as the arm
  // is the normal case — and a fixed name gives them all the SAME node name.
  // ROS 2 permits that and then warns about it, `ros2 node list` shows one
  // entry twice with no way to tell which is which, and anything addressing a
  // node by name gets whichever answered first.
  node_ = std::make_shared<rclcpp::Node>(node_name_);

  for (auto & g : groups_) {
    g.pub = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      g.topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
  if (state_best_effort_) qos.best_effort(); else qos.reliable();
  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, qos,
    std::bind(&TopicBridge::state_callback, this, std::placeholders::_1));

  exec_.add_node(node_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn TopicBridge::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(),
              "activated — holding commands until every joint has reported a state");
  return CallbackReturn::SUCCESS;
}

CallbackReturn TopicBridge::on_deactivate(const rclcpp_lifecycle::State &)
{
  exec_.remove_node(node_);
  state_sub_.reset();
  for (auto & g : groups_) g.pub.reset();
  node_.reset();
  return CallbackReturn::SUCCESS;
}

void TopicBridge::state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::scoped_lock<std::mutex> lk(state_mtx_);
  const size_t n = msg->name.size();
  for (size_t i = 0; i < n; ++i) {
    const auto & name = msg->name[i];
    auto p = pos_state_.find(name);
    if (p == pos_state_.end()) continue;          // a joint owned by someone else
    if (i < msg->position.size()) {
      p->second = msg->position[i];
      seen_[name] = true;
    }
    if (i < msg->velocity.size()) vel_state_[name] = msg->velocity[i];
  }
}

return_type TopicBridge::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  exec_.spin_some();

  if (!seeded_) {
    std::scoped_lock<std::mutex> lk(state_mtx_);
    const bool all = std::all_of(seen_.begin(), seen_.end(),
                                 [](const auto & kv) { return kv.second; });
    if (all) {
      // Start from where the robot actually is, so the first published command
      // is the current pose rather than a step to it.
      for (const auto & g : groups_) {
        if (g.interface != HW_IF_POSITION) continue;
        for (const auto & n : g.joints) cmd_[n] = pos_state_[n];
      }
      seeded_ = true;
      RCLCPP_INFO(node_->get_logger(), "all joints reported — commands seeded, publishing");
    } else {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "waiting for joint states on %s", state_topic_.c_str());
    }
  }
  return return_type::OK;
}

return_type TopicBridge::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (const auto & g : groups_) {
    // Velocity groups publish from the start: zero velocity is a safe command
    // and withholding it would leave a wheel running through the seeding wait.
    // Position groups stay silent until seeded, because zero is a POSE there.
    if (g.interface == HW_IF_POSITION && !seeded_) continue;

    std_msgs::msg::Float64MultiArray msg;
    msg.data.reserve(g.joints.size());
    bool finite = true;
    for (const auto & n : g.joints) {
      const double v = cmd_.at(n);
      if (!std::isfinite(v)) { finite = false; break; }
      msg.data.push_back(v);
    }
    if (!finite) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "group '%s' holds a non-finite command; not publishing", g.name.c_str());
      continue;
    }
    g.pub->publish(msg);
  }
  return return_type::OK;
}

}  // namespace ros2_control_bridge

PLUGINLIB_EXPORT_CLASS(ros2_control_bridge::TopicBridge, hardware_interface::SystemInterface)
