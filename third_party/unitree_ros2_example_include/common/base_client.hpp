#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "nlohmann/json.hpp"
#include "time_tools.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"
#include "ut_errror.hpp"

class BaseClient {
  using Request = unitree_api::msg::Request;
  using Response = unitree_api::msg::Response;
  using IdentityId =
      decltype(std::declval<Request>().header.identity.id);

  rclcpp::Node* node_;
  std::string topic_name_request_;
  std::string topic_name_response_;

  rclcpp::Publisher<Request>::SharedPtr req_puber_;

  // Only one synchronous Call() may be active at a time.
  std::mutex call_mutex_;

  // State shared by Call() and the persistent response callback.
  std::mutex response_mutex_;
  std::condition_variable response_cv_;
  IdentityId pending_identity_id_{};
  std::shared_ptr<const Response> pending_response_;
  bool waiting_for_response_{false};

  // Declared last so it is destroyed first during BaseClient destruction.
  rclcpp::Subscription<Response>::SharedPtr resp_suber_;

 public:
  BaseClient(rclcpp::Node* node,
             const std::string& topic_name_request,
             std::string topic_name_response)
      : node_(node),
        topic_name_request_(topic_name_request),
        topic_name_response_(std::move(topic_name_response)) {
    // Create the response subscription once and keep it for the complete
    // lifetime of this BaseClient. This avoids losing a fast response while a
    // newly-created DDS subscription is still being discovered.
    resp_suber_ = node_->create_subscription<Response>(
        topic_name_response_, rclcpp::QoS(32),
        [this](const std::shared_ptr<const Response> data) {
          bool matched = false;

          {
            std::lock_guard<std::mutex> lock(response_mutex_);

            if (waiting_for_response_ && data &&
                data->header.identity.id == pending_identity_id_) {
              pending_response_ = data;
              matched = true;
            }
          }

          if (matched) {
            response_cv_.notify_one();
          }
        });

    req_puber_ =
        node_->create_publisher<Request>(topic_name_request_, rclcpp::QoS(10));
  }

  int32_t Call(Request req, nlohmann::json& js) {
    // BaseClient maintains one pending-response slot, so synchronous calls
    // must be serialized.
    std::lock_guard<std::mutex> call_lock(call_mutex_);

    req.header.identity.id =
        unitree::common::GetSystemUptimeInNanoseconds();
    const IdentityId identity_id = req.header.identity.id;

    {
      std::lock_guard<std::mutex> response_lock(response_mutex_);
      pending_identity_id_ = identity_id;
      pending_response_.reset();
      waiting_for_response_ = true;
    }

    req_puber_->publish(req);

    Response response;

    {
      std::unique_lock<std::mutex> response_lock(response_mutex_);

      const bool received = response_cv_.wait_for(
          response_lock, std::chrono::seconds(5),
          [this, identity_id]() {
            return pending_response_ &&
                   pending_response_->header.identity.id == identity_id;
          });

      if (!received) {
        waiting_for_response_ = false;
        pending_identity_id_ = IdentityId{};
        pending_response_.reset();
        return UT_ROBOT_TASK_TIMEOUT;
      }

      response = *pending_response_;

      waiting_for_response_ = false;
      pending_identity_id_ = IdentityId{};
      pending_response_.reset();
    }

    if (response.header.status.code != 0) {
      std::cout << "error code: " << response.header.status.code << std::endl;
      return response.header.status.code;
    }

    try {
      js = nlohmann::json::parse(response.data.data());
    } catch (const nlohmann::detail::exception&) {
    }

    return UT_ROBOT_SUCCESS;
  }

  int32_t Call(Request req) {
    nlohmann::json js;
    return Call(std::move(req), js);
  }
};
