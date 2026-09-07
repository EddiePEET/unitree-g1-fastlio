#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "g1_loco_bridge/dry_run_backend.hpp"
#include "g1_loco_bridge/unitree_ros2_backend.hpp"
#include "g1_loco_bridge/velocity_backend.hpp"

using namespace std::chrono_literals;

class G1CmdVelDryRun : public rclcpp::Node
{
public:
  G1CmdVelDryRun()
  : Node("g1_cmd_vel_dry_run")
  {
    cmd_vel_topic_ = declare_parameter<std::string>(
      "cmd_vel_topic",
      "/cmd_vel_dry_test");

    enable_service_name_ = declare_parameter<std::string>(
      "enable_service_name",
      "/g1_loco_bridge/enable");

    dry_run_ = declare_parameter<bool>(
      "dry_run",
      true);

    allow_reverse_ = declare_parameter<bool>(
      "allow_reverse",
      false);

    allow_lateral_ = declare_parameter<bool>(
      "allow_lateral",
      false);

    unitree_command_duration_sec_ = declare_parameter<double>(
      "unitree_command_duration_sec",
      0.30);

    unitree_resend_period_sec_ = declare_parameter<double>(
      "unitree_resend_period_sec",
      0.10);

    command_timeout_sec_ = declare_parameter<double>(
      "command_timeout_sec",
      0.5);

    control_period_sec_ = declare_parameter<double>(
      "control_period_sec",
      0.05);

    report_period_sec_ = declare_parameter<double>(
      "report_period_sec",
      0.5);

    max_linear_x_ = declare_parameter<double>(
      "max_linear_x",
      0.20);

    max_linear_y_ = declare_parameter<double>(
      "max_linear_y",
      0.10);

    max_angular_z_ = declare_parameter<double>(
      "max_angular_z",
      0.30);

    linear_deadband_ = declare_parameter<double>(
      "linear_deadband",
      0.01);

    angular_deadband_ = declare_parameter<double>(
      "angular_deadband",
      0.02);

    min_effective_linear_x_ = declare_parameter<double>(
      "min_effective_linear_x",
      0.20);

    min_effective_angular_z_ = declare_parameter<double>(
      "min_effective_angular_z",
      0.40);

    max_linear_accel_ = declare_parameter<double>(
      "max_linear_accel",
      0.20);

    max_linear_decel_ = declare_parameter<double>(
      "max_linear_decel",
      0.40);

    max_angular_accel_ = declare_parameter<double>(
      "max_angular_accel",
      0.40);

    max_angular_decel_ = declare_parameter<double>(
      "max_angular_decel",
      0.80);

    reject_abs_linear_x_ = declare_parameter<double>(
      "reject_abs_linear_x",
      1.00);

    reject_abs_linear_y_ = declare_parameter<double>(
      "reject_abs_linear_y",
      0.50);

    reject_abs_angular_z_ = declare_parameter<double>(
      "reject_abs_angular_z",
      1.50);

    validate_parameters();

    if (dry_run_) {
      backend_ =
        std::make_unique<g1_loco_bridge::DryRunBackend>(
          get_logger(),
          report_period_sec_);
    } else {
      backend_ =
        std::make_unique<g1_loco_bridge::UnitreeRos2Backend>(
          this,
          get_logger(),
          unitree_command_duration_sec_,
          unitree_resend_period_sec_);
    }

    subscription_ =
      create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_,
        rclcpp::QoS(10),
        std::bind(
          &G1CmdVelDryRun::cmd_vel_callback,
          this,
          std::placeholders::_1));

    enable_service_ =
      create_service<std_srvs::srv::SetBool>(
        enable_service_name_,
        std::bind(
          &G1CmdVelDryRun::enable_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    auto control_period =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(control_period_sec_));

    if (control_period < 1ms) {
      control_period = 1ms;
    }

    const auto now = std::chrono::steady_clock::now();

    last_control_time_ = now;
    last_report_time_ = now;

    control_timer_ =
      create_wall_timer(
        control_period,
        std::bind(
          &G1CmdVelDryRun::control_callback,
          this));

    RCLCPP_WARN(
      get_logger(),
      "Selected backend: %s, real_backend=%s.",
      backend_->name(),
      backend_->is_real() ? "true" : "false");

    if (backend_->is_real()) {
      RCLCPP_WARN(
        get_logger(),
        "REAL UNITREE ROS 2 BACKEND SELECTED. "
        "Both safety gates start DISABLED. "
        "No motion is permitted until explicitly enabled.");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "DRY RUN ONLY: the selected backend cannot control the robot.");
    }

    RCLCPP_WARN(
      get_logger(),
      "Safety gate starts DISABLED. Enable manually with service: %s",
      enable_service_name_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Input topic: %s",
      cmd_vel_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Limits: linear.x=+-%.3f m/s, linear.y=+-%.3f m/s, "
      "angular.z=+-%.3f rad/s",
      max_linear_x_,
      max_linear_y_,
      max_angular_z_);

    RCLCPP_INFO(
      get_logger(),
      "Reject thresholds: |linear.x|>%.3f, |linear.y|>%.3f, "
      "|angular.z|>%.3f",
      reject_abs_linear_x_,
      reject_abs_linear_y_,
      reject_abs_angular_z_);

    RCLCPP_INFO(
      get_logger(),
      "Timeout=%.3f s, control period=%.3f s, report period=%.3f s",
      command_timeout_sec_,
      control_period_sec_,
      report_period_sec_);
  }

  ~G1CmdVelDryRun() override
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      force_zero_locked();
      enabled_ = false;
    }

    /*
     * Explicitly finish the backend safe-stop and worker shutdown
     * before reporting that the bridge has stopped. Do not rely on
     * automatic member destruction after this destructor body.
     */
    if (backend_) {
      (void)backend_->set_enabled(false);
      backend_->shutdown();
    }

    RCLCPP_WARN(
      get_logger(),
      "Bridge stopped after backend shutdown completed. "
      "Safety state is zero. Backend: %s.",
      backend_ ? backend_->name() : "none");
  }

private:
  static double clamp_symmetric(
    const double value,
    const double absolute_limit)
  {
    return std::clamp(
      value,
      -absolute_limit,
      absolute_limit);
  }

  static double apply_deadband(
    const double value,
    const double deadband)
  {
    if (std::abs(value) < deadband) {
      return 0.0;
    }

    return value;
  }

  static double apply_minimum_magnitude(
    const double value,
    const double minimum_magnitude)
  {
    if (value == 0.0) {
      return 0.0;
    }

    if (std::abs(value) >= minimum_magnitude) {
      return value;
    }

    return std::copysign(
      minimum_magnitude,
      value);
  }

  static bool signs_are_opposite(
    const double first,
    const double second)
  {
    return
      (first > 0.0 && second < 0.0) ||
      (first < 0.0 && second > 0.0);
  }

  static double slew_axis(
    const double current,
    const double target,
    const double accel_limit,
    const double decel_limit,
    const double dt_sec)
  {
    double step_target = target;
    double rate_limit = decel_limit;

    /*
     * When direction changes, first decelerate to zero.
     * Only after reaching zero may the value accelerate in
     * the opposite direction.
     */
    if (signs_are_opposite(current, target)) {
      step_target = 0.0;
      rate_limit = decel_limit;
    } else if (std::abs(target) > std::abs(current)) {
      rate_limit = accel_limit;
    } else {
      rate_limit = decel_limit;
    }

    const double max_change =
      std::max(0.0, rate_limit * dt_sec);

    const double requested_change =
      step_target - current;

    const double limited_change =
      std::clamp(
        requested_change,
        -max_change,
        max_change);

    return current + limited_change;
  }

  bool twist_is_finite(
    const geometry_msgs::msg::Twist & message) const
  {
    return
      std::isfinite(message.linear.x) &&
      std::isfinite(message.linear.y) &&
      std::isfinite(message.linear.z) &&
      std::isfinite(message.angular.x) &&
      std::isfinite(message.angular.y) &&
      std::isfinite(message.angular.z);
  }

  bool unsupported_axes_are_zero(
    const geometry_msgs::msg::Twist & message) const
  {
    constexpr double epsilon = 1.0e-6;

    return
      std::abs(message.linear.z) <= epsilon &&
      std::abs(message.angular.x) <= epsilon &&
      std::abs(message.angular.y) <= epsilon;
  }

  bool raw_command_within_reject_limits(
    const geometry_msgs::msg::Twist & message) const
  {
    return
      std::abs(message.linear.x) <= reject_abs_linear_x_ &&
      std::abs(message.linear.y) <= reject_abs_linear_y_ &&
      std::abs(message.angular.z) <= reject_abs_angular_z_;
  }

  geometry_msgs::msg::Twist sanitize_command(
    const geometry_msgs::msg::Twist & input) const
  {
    geometry_msgs::msg::Twist output;

    /*
     * First real-motion phase:
     * - reverse motion is blocked unless explicitly permitted;
     * - lateral motion is blocked unless explicitly permitted.
     *
     * These gates are applied before the normal deadband,
     * hard limits and slew-rate processing.
     */
    const double requested_linear_x =
      allow_reverse_ ?
      input.linear.x :
      std::max(0.0, input.linear.x);

    const double requested_linear_y =
      allow_lateral_ ?
      input.linear.y :
      0.0;

    output.linear.x =
      apply_minimum_magnitude(
        apply_deadband(
          clamp_symmetric(
            requested_linear_x,
            max_linear_x_),
          linear_deadband_),
        min_effective_linear_x_);

    output.linear.y =
      apply_deadband(
        clamp_symmetric(
          requested_linear_y,
          max_linear_y_),
        linear_deadband_);

    output.angular.z =
      apply_minimum_magnitude(
        apply_deadband(
          clamp_symmetric(
            input.angular.z,
            max_angular_z_),
          angular_deadband_),
        min_effective_angular_z_);

    return output;
  }

  void force_zero_locked()
  {
    raw_command_ = geometry_msgs::msg::Twist();
    target_command_ = geometry_msgs::msg::Twist();
    safe_output_ = geometry_msgs::msg::Twist();

    command_received_ = false;
  }

  void validate_parameters()
  {
    auto reject =
      [this](const std::string & reason)
      {
        RCLCPP_FATAL(
          get_logger(),
          "Invalid safety parameter: %s",
          reason.c_str());

        throw std::invalid_argument(reason);
      };

    auto require_positive =
      [&reject](
        const char * name,
        const double value)
      {
        if (!std::isfinite(value) || value <= 0.0) {
          reject(
            std::string(name) +
            " must be finite and greater than zero.");
        }
      };

    auto require_nonnegative =
      [&reject](
        const char * name,
        const double value)
      {
        if (!std::isfinite(value) || value < 0.0) {
          reject(
            std::string(name) +
            " must be finite and nonnegative.");
        }
      };

    if (cmd_vel_topic_.empty()) {
      reject("cmd_vel_topic must not be empty.");
    }

    if (enable_service_name_.empty()) {
      reject("enable_service_name must not be empty.");
    }

    require_positive(
      "command_timeout_sec",
      command_timeout_sec_);

    require_positive(
      "control_period_sec",
      control_period_sec_);

    require_positive(
      "report_period_sec",
      report_period_sec_);

    require_positive(
      "max_linear_x",
      max_linear_x_);

    require_positive(
      "max_linear_y",
      max_linear_y_);

    require_positive(
      "max_angular_z",
      max_angular_z_);

    require_nonnegative(
      "linear_deadband",
      linear_deadband_);

    require_nonnegative(
      "angular_deadband",
      angular_deadband_);

    require_positive(
      "min_effective_linear_x",
      min_effective_linear_x_);

    require_positive(
      "min_effective_angular_z",
      min_effective_angular_z_);

    require_positive(
      "max_linear_accel",
      max_linear_accel_);

    require_positive(
      "max_linear_decel",
      max_linear_decel_);

    require_positive(
      "max_angular_accel",
      max_angular_accel_);

    require_positive(
      "max_angular_decel",
      max_angular_decel_);

    require_positive(
      "reject_abs_linear_x",
      reject_abs_linear_x_);

    require_positive(
      "reject_abs_linear_y",
      reject_abs_linear_y_);

    require_positive(
      "reject_abs_angular_z",
      reject_abs_angular_z_);

    if (linear_deadband_ >= max_linear_x_) {
      reject(
        "linear_deadband must be smaller than max_linear_x.");
    }

    if (linear_deadband_ >= max_linear_y_) {
      reject(
        "linear_deadband must be smaller than max_linear_y.");
    }

    if (angular_deadband_ >= max_angular_z_) {
      reject(
        "angular_deadband must be smaller than max_angular_z.");
    }

    if (min_effective_linear_x_ <= linear_deadband_) {
      reject(
        "min_effective_linear_x must be greater than "
        "linear_deadband.");
    }

    if (min_effective_linear_x_ > max_linear_x_) {
      reject(
        "min_effective_linear_x must not exceed "
        "max_linear_x.");
    }

    if (min_effective_angular_z_ <= angular_deadband_) {
      reject(
        "min_effective_angular_z must be greater than "
        "angular_deadband.");
    }

    if (min_effective_angular_z_ > max_angular_z_) {
      reject(
        "min_effective_angular_z must not exceed "
        "max_angular_z.");
    }

    if (reject_abs_linear_x_ <= max_linear_x_) {
      reject(
        "reject_abs_linear_x must be greater than "
        "max_linear_x.");
    }

    if (reject_abs_linear_y_ <= max_linear_y_) {
      reject(
        "reject_abs_linear_y must be greater than "
        "max_linear_y.");
    }

    if (reject_abs_angular_z_ <= max_angular_z_) {
      reject(
        "reject_abs_angular_z must be greater than "
        "max_angular_z.");
    }

    if (control_period_sec_ >= command_timeout_sec_) {
      reject(
        "control_period_sec must be smaller than "
        "command_timeout_sec.");
    }

    require_positive(
      "unitree_command_duration_sec",
      unitree_command_duration_sec_);

    require_positive(
      "unitree_resend_period_sec",
      unitree_resend_period_sec_);

    if (unitree_command_duration_sec_ > 1.0) {
      reject(
        "unitree_command_duration_sec must not exceed "
        "1.0 second.");
    }

    if (unitree_resend_period_sec_ >=
      unitree_command_duration_sec_)
    {
      reject(
        "unitree_resend_period_sec must be smaller than "
        "unitree_command_duration_sec.");
    }
  }

  void cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!twist_is_finite(*message)) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        force_zero_locked();
        invalid_command_count_++;
        timeout_reported_ = false;
      }

      RCLCPP_ERROR(
        get_logger(),
        "Rejected Twist containing NaN or Inf. "
        "Safety output was forced to zero.");

      if (backend_) {
        backend_->request_stop(
          "non-finite Twist rejected by safety layer");
      }

      return;
    }

    if (!unsupported_axes_are_zero(*message)) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        force_zero_locked();
        invalid_command_count_++;
        timeout_reported_ = false;
      }

      RCLCPP_ERROR(
        get_logger(),
        "Rejected unsupported Twist axes. "
        "Only linear.x, linear.y and angular.z are accepted. "
        "Safety output was forced to zero.");

      if (backend_) {
        backend_->request_stop(
          "unsupported Twist axes rejected by safety layer");
      }

      return;
    }

    if (!raw_command_within_reject_limits(*message)) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);

        force_zero_locked();
        invalid_command_count_++;
        timeout_reported_ = false;
      }

      RCLCPP_ERROR(
        get_logger(),
        "Rejected abnormally large Twist: "
        "raw[x=%.3f y=%.3f yaw=%.3f], "
        "thresholds[x=%.3f y=%.3f yaw=%.3f]. "
        "Safety output was forced to zero.",
        message->linear.x,
        message->linear.y,
        message->angular.z,
        reject_abs_linear_x_,
        reject_abs_linear_y_,
        reject_abs_angular_z_);

      if (backend_) {
        backend_->request_stop(
          "abnormally large Twist rejected by safety layer");
      }

      return;
    }

    const auto sanitized =
      sanitize_command(*message);

    bool report_disabled = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      if (!enabled_) {
        force_zero_locked();

        if (!disabled_input_reported_) {
          disabled_input_reported_ = true;
          report_disabled = true;
        }
      } else {
        raw_command_ = *message;
        target_command_ = sanitized;
        last_receive_time_ = std::chrono::steady_clock::now();

        command_received_ = true;
        timeout_reported_ = false;
        disabled_input_reported_ = false;
      }
    }

    if (report_disabled) {
      RCLCPP_WARN(
        get_logger(),
        "Twist ignored because the safety gate is disabled: "
        "raw[x=%.3f y=%.3f yaw=%.3f]",
        message->linear.x,
        message->linear.y,
        message->angular.z);
    }

    if (report_disabled) {
      return;
    }

    /*
     * 调试阶段明确记录 PC2 内部 ROS 输入。
     * 这里收到 Twist 后，control_callback 才会将安全速度
     * 交给 UnitreeRos2Backend，并调用官方 SetVelocity。
     */
    RCLCPP_INFO(
      get_logger(),
      "Bridge accepted fresh Twist: "
      "raw[x=%.3f y=%.3f yaw=%.3f]",
      message->linear.x,
      message->linear.y,
      message->angular.z);
  }

  void enable_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    const bool enable_requested = request->data;

    /*
     * Ask the backend first. A real backend may reject enabling
     * when its read-only validation is incomplete or unsuccessful.
     *
     * No node data mutex is held during this call.
     */
    bool backend_accepted = false;

    if (backend_) {
      backend_accepted =
        backend_->set_enabled(enable_requested);
    } else {
      backend_accepted = !enable_requested;
    }

    const bool node_enabled =
      enable_requested && backend_accepted;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      enabled_ = node_enabled;
      force_zero_locked();

      timeout_reported_ = false;
      disabled_input_reported_ = false;
    }

    if (enable_requested && backend_accepted) {
      response->success = true;
      response->message =
        "Safety gate enabled. Waiting for a fresh Twist.";

      RCLCPP_WARN(
        get_logger(),
        "Bridge safety gate ENABLED. "
        "A fresh valid Twist is required.");

      return;
    }

    if (enable_requested && !backend_accepted) {
      response->success = false;
      response->message =
        "Enable request rejected by the selected backend. "
        "Bridge remains disabled.";

      RCLCPP_ERROR(
        get_logger(),
        "Bridge enable request REJECTED by backend '%s'. "
        "Node safety gate remains DISABLED.",
        backend_ ? backend_->name() : "missing");

      return;
    }

    /*
     * Local disabling is always considered successful because
     * the node gate and all cached commands have already been
     * forced to zero.
     */
    response->success = true;
    response->message =
      "Safety gate disabled and output forced to zero.";

    RCLCPP_WARN(
      get_logger(),
      "Bridge safety gate DISABLED. Output forced to zero.");
  }

  void control_callback()
  {
    geometry_msgs::msg::Twist raw_snapshot;
    geometry_msgs::msg::Twist target_snapshot;
    geometry_msgs::msg::Twist output_snapshot;

    bool report_timeout = false;
    bool report_state = false;
    bool enabled_snapshot = false;
    bool submit_backend_command = false;
    bool request_backend_stop = false;

    g1_loco_bridge::VelocityCommand backend_command;

    double command_age_sec = 0.0;

    const auto now =
      std::chrono::steady_clock::now();

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      double dt_sec =
        std::chrono::duration<double>(
          now - last_control_time_).count();

      last_control_time_ = now;

      /*
       * Prevent a delayed timer callback from producing one very
       * large velocity step.
       */
      dt_sec =
        std::clamp(
          dt_sec,
          0.0,
          0.25);

      enabled_snapshot = enabled_;

      if (!enabled_) {
        force_zero_locked();
        return;
      }

      if (!command_received_) {
        force_zero_locked();
        return;
      }

      command_age_sec =
        std::chrono::duration<double>(
          now - last_receive_time_).count();

      if (command_age_sec > command_timeout_sec_) {
        force_zero_locked();
        request_backend_stop = true;

        if (!timeout_reported_) {
          timeout_reported_ = true;
          report_timeout = true;
        }
      } else {
        safe_output_.linear.x =
          slew_axis(
            safe_output_.linear.x,
            target_command_.linear.x,
            max_linear_accel_,
            max_linear_decel_,
            dt_sec);

        safe_output_.linear.y =
          slew_axis(
            safe_output_.linear.y,
            target_command_.linear.y,
            max_linear_accel_,
            max_linear_decel_,
            dt_sec);

        safe_output_.angular.z =
          slew_axis(
            safe_output_.angular.z,
            target_command_.angular.z,
            max_angular_accel_,
            max_angular_decel_,
            dt_sec);

        backend_command.vx =
          static_cast<float>(safe_output_.linear.x);

        backend_command.vy =
          static_cast<float>(safe_output_.linear.y);

        backend_command.omega =
          static_cast<float>(safe_output_.angular.z);

        submit_backend_command = true;

        const double since_last_report =
          std::chrono::duration<double>(
            now - last_report_time_).count();

        if (since_last_report >= report_period_sec_) {
          raw_snapshot = raw_command_;
          target_snapshot = target_command_;
          output_snapshot = safe_output_;

          last_report_time_ = now;
          report_state = true;
        }
      }
    }

    if (request_backend_stop && backend_) {
      backend_->request_stop("command watchdog timeout");
    } else if (submit_backend_command && backend_) {
      backend_->submit_velocity(backend_command);
    }

    if (report_timeout) {
      RCLCPP_WARN(
        get_logger(),
        "Watchdog timeout after %.3f s. "
        "Safety target and output forced immediately to zero.",
        command_age_sec);
    }

    if (report_state && enabled_snapshot) {
      RCLCPP_INFO(
        get_logger(),
        "SAFETY raw[x=%.3f y=%.3f yaw=%.3f] "
        "limited_target[x=%.3f y=%.3f yaw=%.3f] "
        "safe_output[x=%.3f y=%.3f yaw=%.3f]",
        raw_snapshot.linear.x,
        raw_snapshot.linear.y,
        raw_snapshot.angular.z,
        target_snapshot.linear.x,
        target_snapshot.linear.y,
        target_snapshot.angular.z,
        output_snapshot.linear.x,
        output_snapshot.linear.y,
        output_snapshot.angular.z);
    }
  }

  std::string cmd_vel_topic_;
  std::string enable_service_name_;

  bool dry_run_{true};
  bool allow_reverse_{false};
  bool allow_lateral_{false};

  double unitree_command_duration_sec_{0.30};
  double unitree_resend_period_sec_{0.10};

  double command_timeout_sec_{0.5};
  double control_period_sec_{0.05};
  double report_period_sec_{0.5};

  double max_linear_x_{0.20};
  double max_linear_y_{0.10};
  double max_angular_z_{0.30};

  double linear_deadband_{0.01};
  double angular_deadband_{0.02};

  double min_effective_linear_x_{0.20};
  double min_effective_angular_z_{0.40};

  double max_linear_accel_{0.20};
  double max_linear_decel_{0.40};

  double max_angular_accel_{0.40};
  double max_angular_decel_{0.80};

  double reject_abs_linear_x_{1.00};
  double reject_abs_linear_y_{0.50};
  double reject_abs_angular_z_{1.50};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    subscription_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
    enable_service_;

  rclcpp::TimerBase::SharedPtr
    control_timer_;

  std::unique_ptr<g1_loco_bridge::VelocityBackend>
    backend_;

  std::mutex data_mutex_;

  geometry_msgs::msg::Twist raw_command_;
  geometry_msgs::msg::Twist target_command_;
  geometry_msgs::msg::Twist safe_output_;

  std::chrono::steady_clock::time_point
    last_receive_time_;

  std::chrono::steady_clock::time_point
    last_control_time_;

  std::chrono::steady_clock::time_point
    last_report_time_;

  bool enabled_{false};
  bool command_received_{false};
  bool timeout_reported_{false};
  bool disabled_input_reported_{false};

  std::size_t invalid_command_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<G1CmdVelDryRun>();

  rclcpp::spin(node);

  node.reset();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
