#include <memory>
#include <string>

#include <standard_msgs/action/dump.hpp>
#include <standard_msgs/action/load.hpp>
#include <standard_msgs/action/move_to_pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include "athena_action_bridge/action_bridge.hpp"

namespace athena_action_bridge
{

using MoveToPose = standard_msgs::action::MoveToPose;
using Load = standard_msgs::action::Load;
using Dump = standard_msgs::action::Dump;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
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

template<typename ActionT>
std::unique_ptr<ActionBridge<ActionT, ActionT>> makeForwarder(
  rclcpp::Node * node, const std::string & server, const std::string & client)
{
  return std::make_unique<ActionBridge<ActionT, ActionT>>(
    node, server, client,
    [](const typename ActionT::Goal & goal) {
      return goal;
    },
    [](const typename ActionT::Feedback & in, typename ActionT::Feedback & out) {
      out = in;
    },
    [](ResultCode code, const typename ActionT::Result * in, typename ActionT::Result & out) {
      if (in) {
        out = *in;
      }
      if (code != ResultCode::SUCCEEDED) {
        out.success = false;
      }
    });
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
    const auto load_client = declare_parameter<std::string>("load.client", "machine/load");

    const auto dump_server = declare_parameter<std::string>("dump.server", "dump");
    const auto dump_client = declare_parameter<std::string>("dump.client", "machine/dump");

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
      [](const NavigateToPose::Feedback & in, MoveToPose::Feedback & out) {
        out.current_pose = in.current_pose;
        out.distance_remaining = in.distance_remaining;
      },
      [](ResultCode code, const NavigateToPose::Result * in, MoveToPose::Result & out) {
        out.success = code == ResultCode::SUCCEEDED;
        out.message = navigationMessage(code, in == nullptr);
      });

    load_ = makeForwarder<Load>(this, load_server, load_client);
    dump_ = makeForwarder<Dump>(this, dump_server, dump_client);
  }

private:
  std::unique_ptr<ActionBridge<MoveToPose, NavigateToPose>> move_to_pose_;
  std::unique_ptr<ActionBridge<Load, Load>> load_;
  std::unique_ptr<ActionBridge<Dump, Dump>> dump_;
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
