#ifndef G1_LOCO_BRIDGE__DRY_RUN_BACKEND_HPP_
#define G1_LOCO_BRIDGE__DRY_RUN_BACKEND_HPP_

#include <chrono>
#include <mutex>
#include <string>

#include <rclcpp/logger.hpp>

#include "g1_loco_bridge/velocity_backend.hpp"

namespace g1_loco_bridge
{

class DryRunBackend final : public VelocityBackend
{
public:
  DryRunBackend(
    rclcpp::Logger logger,
    double report_period_sec);

  ~DryRunBackend() override;

  const char * name() const noexcept override;

  bool is_real() const noexcept override;

  bool set_enabled(bool enabled) override;

  void submit_velocity(
    const VelocityCommand & command) override;

  void request_stop(
    const std::string & reason) override;

  void shutdown() override;

private:
  static bool command_is_finite(
    const VelocityCommand & command);

  rclcpp::Logger logger_;

  std::chrono::steady_clock::duration report_period_;
  std::chrono::steady_clock::time_point last_report_time_;

  std::mutex mutex_;

  VelocityCommand latest_command_;

  bool enabled_{false};
  bool shutdown_{false};
  bool disabled_submission_reported_{false};
};

}  // namespace g1_loco_bridge

#endif  // G1_LOCO_BRIDGE__DRY_RUN_BACKEND_HPP_
