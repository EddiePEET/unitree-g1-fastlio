#ifndef G1_LOCO_BRIDGE__UNITREE_ROS2_BACKEND_HPP_
#define G1_LOCO_BRIDGE__UNITREE_ROS2_BACKEND_HPP_

#include <memory>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include "g1_loco_bridge/velocity_backend.hpp"

namespace g1_loco_bridge
{

class UnitreeRos2Backend final : public VelocityBackend
{
public:
  /*
   * The implementation uses the supplied ROS node for the official
   * Unitree ROS 2 request and response topics.
   *
   * command_duration_sec:
   *   Finite lifetime attached to every SetVelocity request.
   *
   * resend_period_sec:
   *   Minimum period between successive velocity API calls.
   *
   * Construction must not enable motion. The backend-side safety
   * gate always starts disabled.
   */
  UnitreeRos2Backend(
    rclcpp::Node * node,
    rclcpp::Logger logger,
    double command_duration_sec,
    double resend_period_sec);

  ~UnitreeRos2Backend() override;

  UnitreeRos2Backend(
    const UnitreeRos2Backend &) = delete;

  UnitreeRos2Backend & operator=(
    const UnitreeRos2Backend &) = delete;

  UnitreeRos2Backend(
    UnitreeRos2Backend &&) = delete;

  UnitreeRos2Backend & operator=(
    UnitreeRos2Backend &&) = delete;

  const char * name() const noexcept override;

  bool is_real() const noexcept override;

  /*
   * Enabling the backend only opens its independent safety gate.
   * A fresh valid velocity must still be submitted afterwards.
   */
  bool set_enabled(bool enabled) override;

  /*
   * This function must remain non-blocking for ROS callback threads.
   * The implementation stores only the newest command in a mailbox;
   * an independent worker performs the official API call.
   */
  void submit_velocity(
    const VelocityCommand & command) override;

  /*
   * Requests the common safe-stop state for watchdog timeout,
   * manual disable, invalid input or Unitree API failure.
   */
  void request_stop(
    const std::string & reason) override;

  /*
   * Stops the worker thread and releases the official client.
   * The operation must be idempotent.
   */
  void shutdown() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace g1_loco_bridge

#endif  // G1_LOCO_BRIDGE__UNITREE_ROS2_BACKEND_HPP_
