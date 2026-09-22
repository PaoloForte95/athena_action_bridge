#include <algorithm>
#include <memory>
#include <string>

#include <material_handler_msgs/action/dump_material.hpp>
#include <material_handler_msgs/action/load_material.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <standard_msgs/action/dump.hpp>
#include <standard_msgs/action/load.hpp>
#include <standard_msgs/action/move_to_pose.hpp>

#include "athena_action_bridge/action_bridge.hpp"

namespace athena_action_bridge
{

using MoveToPose = standard_msgs::action::MoveToPose;
using Load = standard_msgs::action::Load;
using Dump = standard_msgs::action::Dump;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using LoadMaterial = material_handler_msgs::action::LoadMaterial;
using DumpMaterial = material_handler_msgs::action::DumpMaterial;
using rclcpp_action::ResultCode;

namespace
{

std::string navigationMessage(ResultCode code, bool rejected)
{
  if (rejected) {
    return "Navigation goal was rejected";
  }
  switch (code) {
    case ResultCode::SUCCEEDED:
      return "Goal reached";
    case ResultCode::CANCELED:
      return "Navigation canceled";
    case ResultCode::ABORTED:
      return "Navigation aborted";
    default:
      return "Navigation ended with an unknown result";
  }
}

float progressOf(double done, double requested)
{
  if (requested <= 0.0) {
    return 0.0f;
  }
  return static_cast<float>(std::clamp(done / requested, 0.0, 1.0));
}

}  // namespace

class ActionBridgeNode : public rclcpp::Node
{
public:
  explicit ActionBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("action_bridge", options)
  {
    const auto move_server = declare_parameter<std::string>("move_to_pose.server", "move_to_pose");
    const auto move_client =
      declare_parameter<std::string>("move_to_pose.client", "navigate_to_pose");
    const auto frame_id = declare_parameter<std::string>("move_to_pose.frame_id", "map");
    const auto behavior_tree = declare_parameter<std::string>("move_to_pose.behavior_tree", "");

    const auto load_server = declare_parameter<std::string>("load.server", "load");
    const auto load_client = declare_parameter<std::string>("load.client", "load_material");

    const auto dump_server = declare_parameter<std::string>("dump.server", "dump");
    const auto dump_client = declare_parameter<std::string>("dump.client", "dump_material");

    move_to_pose_ = std::make_unique<ActionBridge<MoveToPose, NavigateToPose>>(
      this, move_server, move_client,
      [frame_id, behavior_tree](const MoveToPose::Goal & in) {
        NavigateToPose::Goal out;
        out.pose = in.target_pose;
        if (out.pose.header.frame_id.empty()) {
          out.pose.header.frame_id = frame_id;
        }
        out.behavior_tree = behavior_tree;
        return out;
      },
      [](const MoveToPose::Goal &, const NavigateToPose::Feedback & in,
      MoveToPose::Feedback & out) {
        out.current_pose = in.current_pose;
        out.distance_remaining = in.distance_remaining;
      },
      [](const MoveToPose::Goal &, ResultCode code, const NavigateToPose::Result * in,
      MoveToPose::Result & out) {
        out.success = code == ResultCode::SUCCEEDED;
        out.message = navigationMessage(code, in == nullptr);
        return out.success;
      });

    load_ = std::make_unique<ActionBridge<Load, LoadMaterial>>(
      this, load_server, load_client,
      [](const Load::Goal & in) {
        LoadMaterial::Goal out;
        out.name = in.target;
        out.location = in.location;
        out.amount = in.amount;
        return out;
      },
      [](const Load::Goal & goal, const LoadMaterial::Feedback & in, Load::Feedback & out) {
        out.phase = "loading";
        out.progress = progressOf(in.amount_loaded, goal.amount);
      },
      [](const Load::Goal & goal, ResultCode code, const LoadMaterial::Result * in,
      Load::Result & out) {
        out.success = code == ResultCode::SUCCEEDED && in && in->material_loaded;
        out.payload = out.success ? goal.amount : 0.0;
        return out.success;
      });

    dump_ = std::make_unique<ActionBridge<Dump, DumpMaterial>>(
      this, dump_server, dump_client,
      [this](const Dump::Goal & in) {
        DumpMaterial::Goal out;
        out.header.stamp = now();
        out.name = in.target;
        out.location = in.location;
        out.amount = in.amount;
        return out;
      },
      [](const Dump::Goal & goal, const DumpMaterial::Feedback & in, Dump::Feedback & out) {
        out.phase = "dumping";
        out.progress = progressOf(in.amount_dumped, goal.amount);
      },
      [](const Dump::Goal & goal, ResultCode code, const DumpMaterial::Result * in,
      Dump::Result & out) {
        out.success = code == ResultCode::SUCCEEDED && in && in->material_dumped;
        out.payload = out.success ? goal.amount : 0.0;
        return out.success;
      });
  }

private:
  std::unique_ptr<ActionBridge<MoveToPose, NavigateToPose>> move_to_pose_;
  std::unique_ptr<ActionBridge<Load, LoadMaterial>> load_;
  std::unique_ptr<ActionBridge<Dump, DumpMaterial>> dump_;
};

}  // namespace athena_action_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<athena_action_bridge::ActionBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
