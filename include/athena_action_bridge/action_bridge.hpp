#ifndef ATHENA_ACTION_BRIDGE__ACTION_BRIDGE_HPP_
#define ATHENA_ACTION_BRIDGE__ACTION_BRIDGE_HPP_

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace athena_action_bridge
{

/**
 * @brief Exposes an action server of type FromT and forwards every goal to an
 * action server of type ToT. Feedback and results are converted back.
 * Cancel requests are forwarded to the downstream goal.
 */
template<typename FromT, typename ToT>
class ActionBridge
{
public:
  using ServerHandle = rclcpp_action::ServerGoalHandle<FromT>;
  using ClientHandle = rclcpp_action::ClientGoalHandle<ToT>;

  using GoalConverter = std::function<typename ToT::Goal(const typename FromT::Goal &)>;
  using FeedbackConverter =
    std::function<void(const typename ToT::Feedback &, typename FromT::Feedback &)>;
  using ResultConverter = std::function<void(
        rclcpp_action::ResultCode, const typename ToT::Result *, typename FromT::Result &)>;

  ActionBridge(
    rclcpp::Node * node,
    const std::string & server_name,
    const std::string & client_name,
    GoalConverter convert_goal,
    FeedbackConverter convert_feedback,
    ResultConverter convert_result)
  : node_(node),
    server_name_(server_name),
    client_name_(client_name),
    convert_goal_(std::move(convert_goal)),
    convert_feedback_(std::move(convert_feedback)),
    convert_result_(std::move(convert_result))
  {
    client_ = rclcpp_action::create_client<ToT>(node_, client_name_);

    server_ = rclcpp_action::create_server<FromT>(
      node_, server_name_,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename FromT::Goal>) {
        return handleGoal();
      },
      [this](const std::shared_ptr<ServerHandle> handle) {
        return handleCancel(handle);
      },
      [this](const std::shared_ptr<ServerHandle> handle) {
        handleAccepted(handle);
      });

    RCLCPP_INFO(
      node_->get_logger(), "Bridge ready: %s -> %s",
      server_name_.c_str(), client_name_.c_str());
  }

private:
  rclcpp_action::GoalResponse handleGoal()
  {
    if (!client_->action_server_is_ready()) {
      RCLCPP_WARN(
        node_->get_logger(), "[%s] Goal rejected: %s is not available",
        server_name_.c_str(), client_name_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<ServerHandle> handle)
  {
    typename ClientHandle::SharedPtr downstream;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto it = active_.find(handle->get_goal_id());
      if (it != active_.end()) {
        downstream = it->second;
      }
    }

    RCLCPP_INFO(node_->get_logger(), "[%s] Cancel requested", server_name_.c_str());
    if (downstream) {
      client_->async_cancel_goal(downstream);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<ServerHandle> handle)
  {
    typename rclcpp_action::Client<ToT>::SendGoalOptions options;

    options.goal_response_callback =
      [this, handle](typename ClientHandle::SharedPtr downstream) {
        if (!downstream) {
          RCLCPP_WARN(
            node_->get_logger(), "[%s] Goal rejected by %s",
            server_name_.c_str(), client_name_.c_str());
          finish(handle, rclcpp_action::ResultCode::ABORTED, nullptr);
          return;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          active_[handle->get_goal_id()] = downstream;
        }
        if (handle->is_canceling()) {
          client_->async_cancel_goal(downstream);
        }
      };

    options.feedback_callback =
      [this, handle](
      typename ClientHandle::SharedPtr,
      const std::shared_ptr<const typename ToT::Feedback> feedback) {
        if (!handle->is_active()) {
          return;
        }
        auto out = std::make_shared<typename FromT::Feedback>();
        convert_feedback_(*feedback, *out);
        handle->publish_feedback(out);
      };

    options.result_callback =
      [this, handle](const typename ClientHandle::WrappedResult & wrapped) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          active_.erase(handle->get_goal_id());
        }
        finish(handle, wrapped.code, wrapped.result.get());
      };

    RCLCPP_INFO(
      node_->get_logger(), "[%s] Forwarding goal to %s",
      server_name_.c_str(), client_name_.c_str());
    client_->async_send_goal(convert_goal_(*handle->get_goal()), options);
  }

  void finish(
    const std::shared_ptr<ServerHandle> & handle,
    rclcpp_action::ResultCode code,
    const typename ToT::Result * result)
  {
    if (!handle->is_active()) {
      return;
    }

    auto out = std::make_shared<typename FromT::Result>();
    convert_result_(code, result, *out);

    if (code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "[%s] Goal succeeded", server_name_.c_str());
      handle->succeed(out);
    } else if (code == rclcpp_action::ResultCode::CANCELED && handle->is_canceling()) {
      RCLCPP_INFO(node_->get_logger(), "[%s] Goal canceled", server_name_.c_str());
      handle->canceled(out);
    } else {
      RCLCPP_WARN(node_->get_logger(), "[%s] Goal aborted", server_name_.c_str());
      handle->abort(out);
    }
  }

  rclcpp::Node * node_;
  std::string server_name_;
  std::string client_name_;
  GoalConverter convert_goal_;
  FeedbackConverter convert_feedback_;
  ResultConverter convert_result_;

  typename rclcpp_action::Server<FromT>::SharedPtr server_;
  typename rclcpp_action::Client<ToT>::SharedPtr client_;

  std::mutex mutex_;
  std::map<rclcpp_action::GoalUUID, typename ClientHandle::SharedPtr> active_;
};

}  // namespace athena_action_bridge

#endif  // ATHENA_ACTION_BRIDGE__ACTION_BRIDGE_HPP_
