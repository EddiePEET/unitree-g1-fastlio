#include "g1_loco_bridge/dry_run_backend.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <rclcpp/logging.hpp>

namespace g1_loco_bridge
{

DryRunBackend::DryRunBackend(
  rclcpp::Logger logger,
  const double report_period_sec)
: logger_(std::move(logger)),
  last_report_time_(std::chrono::steady_clock::now())
{
  if (!std::isfinite(report_period_sec) ||
    report_period_sec <= 0.0)
  {
    throw std::invalid_argument(
            "DryRunBackend report_period_sec must be "
            "finite and greater than zero.");
  }

  report_period_ =
    std::chrono::duration_cast<
    std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(report_period_sec));

  RCLCPP_WARN(
    logger_,
    "DryRunBackend created. This backend cannot control the robot.");
}

DryRunBackend::~DryRunBackend()
{
  shutdown();
}

const char * DryRunBackend::name() const noexcept
{
  return "dry_run";
}

bool DryRunBackend::is_real() const noexcept
{
  return false;
}

bool DryRunBackend::command_is_finite(
  const VelocityCommand & command)
{
  return
    std::isfinite(command.vx) &&
    std::isfinite(command.vy) &&
    std::isfinite(command.omega);
}

bool DryRunBackend::set_enabled(const bool enabled)
{
  bool changed = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (shutdown_) {
      return false;
    }

    changed = enabled_ != enabled;
    enabled_ = enabled;

    latest_command_ = VelocityCommand();
    disabled_submission_reported_ = false;
    last_report_time_ = std::chrono::steady_clock::now();
  }

  if (!changed) {
    return true;
  }

  if (enabled) {
    RCLCPP_WARN(
      logger_,
      "DryRunBackend ENABLED. "
      "Commands will only be recorded.");
  } else {
    RCLCPP_WARN(
      logger_,
      "DryRunBackend DISABLED and reset to zero.");
  }

  return true;
}

void DryRunBackend::submit_velocity(
  const VelocityCommand & command)
{
  if (!command_is_finite(command)) {
    request_stop("non-finite command rejected by DryRunBackend");
    return;
  }

  bool report_disabled = false;
  bool report_command = false;
  VelocityCommand command_snapshot;

  const auto now = std::chrono::steady_clock::now();

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (shutdown_) {
      return;
    }

    if (!enabled_) {
      latest_command_ = VelocityCommand();

      if (!disabled_submission_reported_) {
        disabled_submission_reported_ = true;
        report_disabled = true;
      }
    } else {
      latest_command_ = command;
      disabled_submission_reported_ = false;

      if ((now - last_report_time_) >= report_period_) {
        command_snapshot = latest_command_;
        last_report_time_ = now;
        report_command = true;
      }
    }
  }

  if (report_disabled) {
    RCLCPP_WARN(
      logger_,
      "DryRunBackend rejected a command because its "
      "independent gate is disabled.");
  }

  if (report_command) {
    RCLCPP_DEBUG(
      logger_,
      "DRY BACKEND safe command: "
      "vx=%.3f vy=%.3f omega=%.3f",
      command_snapshot.vx,
      command_snapshot.vy,
      command_snapshot.omega);
  }
}

void DryRunBackend::request_stop(
  const std::string & reason)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (shutdown_) {
      return;
    }

    latest_command_ = VelocityCommand();
    disabled_submission_reported_ = false;
  }

  RCLCPP_WARN(
    logger_,
    "DryRunBackend entered zero state: %s",
    reason.c_str());
}

void DryRunBackend::shutdown()
{
  bool should_report = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (shutdown_) {
      return;
    }

    enabled_ = false;
    latest_command_ = VelocityCommand();
    disabled_submission_reported_ = false;
    shutdown_ = true;

    should_report = true;
  }

  if (should_report) {
    RCLCPP_WARN(
      logger_,
      "DryRunBackend shut down in zero state. "
      "No robot command was sent.");
  }
}

}  // namespace g1_loco_bridge
