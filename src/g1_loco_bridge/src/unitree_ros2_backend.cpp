#include "g1_loco_bridge/unitree_ros2_backend.hpp"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <type_traits>

#include <rclcpp/logging.hpp>

#include <g1/g1_loco_client.hpp>

namespace g1_loco_bridge
{

static_assert(
  std::is_constructible_v<
    unitree::robot::g1::LocoClient,
    rclcpp::Node *>,
  "Official G1 LocoClient must be constructible from rclcpp::Node*.");

namespace
{

constexpr int32_t kSuccess = 0;
constexpr int32_t kLocoStateNotAvailable = 7301;
constexpr int32_t kTransportException = -2;
constexpr int32_t kUnknownTransportException = -3;

/*
 * 官方 G1 运动模式：
 *   500：Walk Motion
 *   501：Walk Motion with 3-DOF waist
 *   801：Run（部分 29DOF 固件）
 *   802：新版 29DOF ai_sport
 *
 * 本程序不会自动切换 FSM，只检查当前模式。
 */
bool fsm_allows_velocity(const int fsm_id)
{
  switch (fsm_id) {
    case 500:
    case 501:
    case 801:
    case 802:
      return true;

    default:
      return false;
  }
}

}  // namespace

class UnitreeRos2Backend::Impl
{
public:
  Impl(
    rclcpp::Node * node,
    rclcpp::Logger logger,
    const double command_duration_sec,
    const double resend_period_sec)
  : node_(node),
    logger_(std::move(logger)),
    command_duration_sec_(command_duration_sec),
    resend_period_sec_(resend_period_sec)
  {
    validate_parameters();

    client_ =
      std::make_unique<unitree::robot::g1::LocoClient>(
        node_);

    worker_ =
      std::thread(
      [this]()
      {
        worker_loop();
      });

    RCLCPP_WARN(
      logger_,
      "UnitreeRos2Backend created with its independent safety "
      "gate DISABLED. Official LocoClient was constructed. "
      "Motion APIs remain blocked until the backend safety gate is "
      "explicitly enabled after the read-only check.");
  }

  ~Impl()
  {
    shutdown();
  }

  bool set_enabled(const bool enabled)
  {
    bool changed = false;
    bool enable_rejected = false;
    std::string rejection_reason;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (shutdown_requested_) {
        return false;
      }

      if (enabled) {
        if (!read_only_check_complete_) {
          enable_rejected = true;
          rejection_reason =
            "read-only communication check is not complete";
        } else if (!read_only_check_passed_) {
          enable_rejected = true;
          rejection_reason =
            "read-only communication check did not pass";
        } else {
          changed = !enabled_;
          enabled_ = true;

          latest_command_ = VelocityCommand();
          command_pending_ = false;
          stop_requested_ = false;
          stop_reason_.clear();
          disabled_submission_reported_ = false;
        }
      } else {
        changed = enabled_;
        enabled_ = false;

        latest_command_ = VelocityCommand();
        command_pending_ = false;
        disabled_submission_reported_ = false;

        stop_requested_ = true;
        stop_reason_ = "backend safety gate disabled";
      }
    }

    condition_.notify_one();

    if (enable_rejected) {
      RCLCPP_ERROR(
        logger_,
        "UnitreeRos2Backend enable request rejected: %s. "
        "Backend remains DISABLED.",
        rejection_reason.c_str());

      return false;
    }

    if (!changed) {
      return true;
    }

    if (enabled) {
      RCLCPP_WARN(
        logger_,
        "UnitreeRos2Backend safety gate ENABLED after "
        "successful read-only communication validation.");
    } else {
      RCLCPP_WARN(
        logger_,
        "UnitreeRos2Backend DISABLED and reset to zero.");
    }

    return true;
  }

  void submit_velocity(const VelocityCommand & command)
  {
    if (!command_is_finite(command)) {
      request_stop("non-finite command rejected by backend");
      return;
    }

    bool report_disabled = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (shutdown_requested_) {
        return;
      }

      if (!enabled_) {
        latest_command_ = VelocityCommand();
        command_pending_ = false;

        if (!disabled_submission_reported_) {
          disabled_submission_reported_ = true;
          report_disabled = true;
        }
      } else {
        /*
         * This is a one-element mailbox. A newer command replaces
         * an older pending command so stale velocities cannot build
         * up in an unbounded queue.
         */
        latest_command_ = command;
        command_pending_ = true;
        disabled_submission_reported_ = false;
      }
    }

    if (report_disabled) {
      RCLCPP_WARN(
        logger_,
        "UnitreeRos2Backend rejected a command because "
        "its independent safety gate is disabled.");
    }

    condition_.notify_one();
  }

  void request_stop(const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (shutdown_requested_) {
        return;
      }

      latest_command_ = VelocityCommand();
      command_pending_ = false;
      stop_requested_ = true;
      stop_reason_ =
        reason.empty() ? "unspecified safe-stop request" : reason;
    }

    condition_.notify_one();
  }

  void shutdown()
  {
    bool join_worker = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (shutdown_requested_) {
        return;
      }

      enabled_ = false;
      latest_command_ = VelocityCommand();
      command_pending_ = false;
      stop_requested_ = true;
      stop_reason_ = "backend shutdown";
      shutdown_requested_ = true;

      join_worker = worker_.joinable();
    }

    condition_.notify_one();

    if (join_worker) {
      worker_.join();
    }

    RCLCPP_WARN(
      logger_,
      "UnitreeRos2Backend shut down with its safety gate "
      "disabled and command cache reset to zero.");
  }

private:
  void run_read_only_check()
  {
    /*
     * The node constructor finishes before main() begins spinning.
     * Give the executor time to start and discover DDS peers.
     */
    std::this_thread::sleep_for(
      std::chrono::milliseconds(1000));

    int fsm_id = -1;
    int fsm_mode = -1;
    int balance_mode = -1;

    RCLCPP_INFO(
      logger_,
      "Read-only check: calling GetFsmId...");

    const int32_t fsm_id_result =
      client_->GetFsmId(fsm_id);

    RCLCPP_INFO(
      logger_,
      "GetFsmId returned: code=%d, value=%d",
      fsm_id_result,
      fsm_id);

    RCLCPP_INFO(
      logger_,
      "Read-only check: calling GetFsmMode...");

    const int32_t fsm_mode_result =
      client_->GetFsmMode(fsm_mode);

    RCLCPP_INFO(
      logger_,
      "GetFsmMode returned: code=%d, value=%d",
      fsm_mode_result,
      fsm_mode);

    RCLCPP_INFO(
      logger_,
      "Read-only check: calling GetBalanceMode...");

    const int32_t balance_mode_result =
      client_->GetBalanceMode(balance_mode);

    if (balance_mode_result == kSuccess) {
      RCLCPP_INFO(
        logger_,
        "GetBalanceMode returned: code=0, value=%d",
        balance_mode);
    } else if (
      balance_mode_result == kLocoStateNotAvailable)
    {
      RCLCPP_WARN(
        logger_,
        "GetBalanceMode returned code 7301: "
        "LocoState not available.");
    } else {
      RCLCPP_ERROR(
        logger_,
        "GetBalanceMode returned: code=%d",
        balance_mode_result);
    }

    const bool primary_queries_ok =
      fsm_id_result == kSuccess &&
      fsm_mode_result == kSuccess;

    const bool balance_query_acceptable =
      balance_mode_result == kSuccess ||
      balance_mode_result == kLocoStateNotAvailable;

    const bool motion_mode_ok =
      fsm_allows_velocity(fsm_id);

    if (!motion_mode_ok) {
      RCLCPP_ERROR(
        logger_,
        "Current FSM ID=%d is not in the allowed velocity "
        "mode list [500, 501, 801, 802]. "
        "Backend will remain DISABLED. "
        "No automatic SetFsmId call will be made.",
        fsm_id);
    } else {
      RCLCPP_INFO(
        logger_,
        "Current FSM ID=%d allows official velocity control.",
        fsm_id);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);

      read_only_check_complete_ = true;
      read_only_check_passed_ =
        primary_queries_ok &&
        balance_query_acceptable &&
        motion_mode_ok;

      /*
       * Read-only communication never opens the motion gate.
       */
      enabled_ = false;
      latest_command_ = VelocityCommand();
      command_pending_ = false;
    }

    if (primary_queries_ok &&
      balance_query_acceptable &&
      motion_mode_ok)
    {
      RCLCPP_INFO(
        logger_,
        "Official Unitree ROS 2 read-only communication "
        "check passed. Backend remains DISABLED.");
    } else {
      RCLCPP_ERROR(
        logger_,
        "Official Unitree ROS 2 read-only communication "
        "check failed. Backend remains DISABLED.");
    }
  }

  static bool command_is_finite(
    const VelocityCommand & command)
  {
    return
      std::isfinite(command.vx) &&
      std::isfinite(command.vy) &&
      std::isfinite(command.omega);
  }

  void validate_parameters()
  {
    if (node_ == nullptr) {
      throw std::invalid_argument(
              "UnitreeRos2Backend node pointer must not be null.");
    }

    if (!std::isfinite(command_duration_sec_) ||
      command_duration_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "command_duration_sec must be finite and "
              "greater than zero.");
    }

    if (command_duration_sec_ > 1.0) {
      throw std::invalid_argument(
              "command_duration_sec must not exceed 1.0 second.");
    }

    if (!std::isfinite(resend_period_sec_) ||
      resend_period_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "resend_period_sec must be finite and "
              "greater than zero.");
    }

    if (resend_period_sec_ >= command_duration_sec_) {
      throw std::invalid_argument(
              "resend_period_sec must be smaller than "
              "command_duration_sec.");
    }
  }

  static bool command_is_zero(
    const VelocityCommand & command)
  {
    constexpr float epsilon = 1.0e-4F;

    return
      std::abs(command.vx) <= epsilon &&
      std::abs(command.vy) <= epsilon &&
      std::abs(command.omega) <= epsilon;
  }

  /*
   * These functions call the official Unitree ROS 2 motion API.
   * Every velocity request has a finite duration, and every return
   * code or exception is forwarded into the backend safe-stop state.
   */
  int32_t send_velocity_transport(
    const VelocityCommand & command)
  {
    if (!client_) {
      RCLCPP_ERROR(
        logger_,
        "LocoClient is null. Cannot send velocity.");
      return -1;
    }

    try {
      constexpr int32_t max_attempts = 2;

      for (int32_t attempt = 1; attempt <= max_attempts; ++attempt) {
        const int32_t ret =
          client_->SetVelocity(
            static_cast<float>(command.vx),
            static_cast<float>(command.vy),
            static_cast<float>(command.omega),
            static_cast<float>(command_duration_sec_));

        if (ret == kSuccess) {
          if (attempt == 1) {
            /*
             * 调试阶段暂时使用 INFO，明确证明 PC2 已经调用
             * 官方 LocoClient::SetVelocity，且 PC1 服务返回 0。
             */
            RCLCPP_INFO(
              logger_,
              "Official LocoClient::SetVelocity accepted: "
              "code=0 vx=%.3f vy=%.3f "
              "omega=%.3f duration=%.3f",
              command.vx,
              command.vy,
              command.omega,
              command_duration_sec_);
          } else {
            RCLCPP_WARN(
              logger_,
              "SetVelocity retry succeeded after one "
              "transport timeout.");
          }

          return ret;
        }

        if (
          ret == UT_ROBOT_TASK_TIMEOUT &&
          attempt < max_attempts)
        {
          RCLCPP_WARN(
            logger_,
            "SetVelocity timed out once. Retrying the "
            "same finite-duration command one time.");
          continue;
        }

        RCLCPP_ERROR(
          logger_,
          "SetVelocity failed. code=%d attempt=%d/%d",
          ret,
          attempt,
          max_attempts);
        return ret;
      }

      return UT_ROBOT_TASK_TIMEOUT;
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        logger_,
        "SetVelocity threw an exception: %s",
        exception.what());
      return kTransportException;
    } catch (...) {
      RCLCPP_ERROR(
        logger_,
        "SetVelocity threw an unknown exception.");
      return kUnknownTransportException;
    }
  }

  int32_t send_stop_transport()
  {
    if (!client_) {
      RCLCPP_ERROR(
        logger_,
        "LocoClient is null. Cannot stop.");
      return -1;
    }

    try {
      const int32_t ret =
        client_->StopMove();

      if (ret != kSuccess) {
        RCLCPP_ERROR(
          logger_,
          "StopMove failed. code=%d",
          ret);
      } else {
        RCLCPP_WARN(
          logger_,
          "StopMove called.");
      }

      return ret;
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        logger_,
        "StopMove threw an exception: %s",
        exception.what());
      return kTransportException;
    } catch (...) {
      RCLCPP_ERROR(
        logger_,
        "StopMove threw an unknown exception.");
      return kUnknownTransportException;
    }
  }

  void worker_loop()
  {
    try {
      run_read_only_check();
    } catch (const std::exception & exception) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        read_only_check_complete_ = true;
        read_only_check_passed_ = false;
        enabled_ = false;
        latest_command_ = VelocityCommand();
        command_pending_ = false;
        active_command_ = VelocityCommand();
        active_command_valid_ = false;
      }

      RCLCPP_ERROR(
        logger_,
        "Read-only communication check threw an exception: %s. "
        "Backend remains DISABLED.",
        exception.what());
      return;
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        read_only_check_complete_ = true;
        read_only_check_passed_ = false;
        enabled_ = false;
        latest_command_ = VelocityCommand();
        command_pending_ = false;
        active_command_ = VelocityCommand();
        active_command_valid_ = false;
      }

      RCLCPP_ERROR(
        logger_,
        "Read-only communication check threw an unknown exception. "
        "Backend remains DISABLED.");
      return;
    }

    const auto resend_period =
      std::chrono::duration<double>(
      resend_period_sec_);

    for (;;) {
      VelocityCommand command;
      std::string stop_reason;

      bool process_command = false;
      bool process_stop = false;
      bool resend_command = false;
      bool had_motion_command = false;
      bool timed_out = false;

      {
        std::unique_lock<std::mutex> lock(mutex_);

        const auto event_pending =
          [this]()
          {
            return
              shutdown_requested_ ||
              stop_requested_ ||
              command_pending_;
          };

        /*
         * With an active command, wake periodically for finite-
         * duration renewal. Otherwise sleep until a new event.
         */
        if (enabled_ &&
          active_command_valid_ &&
          !event_pending())
        {
          timed_out =
            !condition_.wait_for(
            lock,
            resend_period,
            event_pending);
        } else {
          condition_.wait(
            lock,
            event_pending);
        }

        /*
         * Stop always has priority over shutdown and velocity.
         */
        if (stop_requested_) {
          stop_reason = stop_reason_;
          stop_requested_ = false;

          latest_command_ = VelocityCommand();
          command_pending_ = false;

          active_command_ = VelocityCommand();
          active_command_valid_ = false;

          had_motion_command = motion_command_sent_;
          process_stop = true;
        } else if (shutdown_requested_) {
          break;
        } else if (command_pending_) {
          command = latest_command_;
          command_pending_ = false;

          if (command_is_zero(command)) {
            stop_reason = "zero velocity command";
            active_command_ = VelocityCommand();
            active_command_valid_ = false;

            had_motion_command = motion_command_sent_;
            process_stop = true;
          } else {
            process_command = true;
          }
        } else if (
          timed_out &&
          enabled_ &&
          active_command_valid_)
        {
          command = active_command_;
          process_command = true;
          resend_command = true;
        }
      }

      if (process_stop) {
        if (had_motion_command) {
          const int32_t result =
            send_stop_transport();

          if (result == kSuccess) {
            std::lock_guard<std::mutex> lock(mutex_);
            motion_command_sent_ = false;

            RCLCPP_WARN(
              logger_,
              "Safe-stop completed successfully: %s.",
              stop_reason.c_str());
          } else {
            RCLCPP_ERROR(
              logger_,
              "Safe-stop transport failed: reason=%s, code=%d.",
              stop_reason.c_str(),
              result);
          }
        } else {
          RCLCPP_WARN(
            logger_,
            "Safe-stop state entered with no previously sent "
            "motion command: %s.",
            stop_reason.c_str());
        }

        continue;
      }

      if (process_command) {
        {
          std::lock_guard<std::mutex> lock(mutex_);

          /*
           * Once SetVelocity is attempted, conservatively assume the
           * request may have reached the robot even if the API returns
           * an error or throws after transmission. The following stop
           * path must therefore attempt StopMove.
           */
          motion_command_sent_ = true;
        }

        const int32_t result =
          send_velocity_transport(command);

        if (result == kSuccess) {
          std::lock_guard<std::mutex> lock(mutex_);

          if (enabled_ &&
            !stop_requested_ &&
            !shutdown_requested_)
          {
            active_command_ = command;
            active_command_valid_ = true;
          } else {
            active_command_ = VelocityCommand();
            active_command_valid_ = false;
          }

          if (!resend_command) {
            RCLCPP_DEBUG(
              logger_,
              "Newest safe velocity accepted by official "
              "Unitree transport.");
          }
        } else {
          {
            std::lock_guard<std::mutex> lock(mutex_);

            enabled_ = false;
            latest_command_ = VelocityCommand();
            command_pending_ = false;
            active_command_ = VelocityCommand();
            active_command_valid_ = false;

            stop_requested_ = true;
            stop_reason_ =
              "velocity transport returned a nonzero code";
          }

          condition_.notify_one();

          RCLCPP_ERROR(
            logger_,
            "Velocity transport failed with code=%d. "
            "Backend gate was disabled and safe-stop requested.",
            result);
        }
      }
    }
  }

  rclcpp::Node * node_;
  rclcpp::Logger logger_;

  std::unique_ptr<unitree::robot::g1::LocoClient>
    client_;

  double command_duration_sec_;
  double resend_period_sec_;

  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread worker_;

  VelocityCommand latest_command_;
  VelocityCommand active_command_;
  std::string stop_reason_;

  bool enabled_{false};
  bool read_only_check_complete_{false};
  bool read_only_check_passed_{false};
  bool command_pending_{false};
  bool active_command_valid_{false};
  bool motion_command_sent_{false};
  bool stop_requested_{false};
  bool shutdown_requested_{false};
  bool disabled_submission_reported_{false};
};

UnitreeRos2Backend::UnitreeRos2Backend(
  rclcpp::Node * node,
  rclcpp::Logger logger,
  const double command_duration_sec,
  const double resend_period_sec)
: impl_(
    std::make_unique<Impl>(
      node,
      std::move(logger),
      command_duration_sec,
      resend_period_sec))
{
}

UnitreeRos2Backend::~UnitreeRos2Backend()
{
  shutdown();
}

const char * UnitreeRos2Backend::name() const noexcept
{
  return "unitree_ros2";
}

bool UnitreeRos2Backend::is_real() const noexcept
{
  return true;
}

bool UnitreeRos2Backend::set_enabled(const bool enabled)
{
  return impl_->set_enabled(enabled);
}

void UnitreeRos2Backend::submit_velocity(
  const VelocityCommand & command)
{
  impl_->submit_velocity(command);
}

void UnitreeRos2Backend::request_stop(
  const std::string & reason)
{
  impl_->request_stop(reason);
}

void UnitreeRos2Backend::shutdown()
{
  if (impl_) {
    impl_->shutdown();
  }
}

}  // namespace g1_loco_bridge
