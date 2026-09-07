#ifndef G1_LOCO_BRIDGE__VELOCITY_BACKEND_HPP_
#define G1_LOCO_BRIDGE__VELOCITY_BACKEND_HPP_

#include <string>

namespace g1_loco_bridge
{

struct VelocityCommand
{
  float vx{0.0F};
  float vy{0.0F};
  float omega{0.0F};
};

class VelocityBackend
{
public:
  virtual ~VelocityBackend() = default;

  /*
   * Human-readable backend name for logs.
   */
  virtual const char * name() const noexcept = 0;

  /*
   * True only for a backend capable of contacting the robot.
   */
  virtual bool is_real() const noexcept = 0;

  /*
   * Independent backend-side safety gate.
   *
   * A real backend must never process a nonzero velocity while
   * this gate is disabled, even if the caller makes a mistake.
   */
  /*
   * Returns true when the backend accepts the requested gate state.
   * A real backend may reject enable=true when its read-only
   * communication check has not completed or has failed.
   */
  virtual bool set_enabled(bool enabled) = 0;

  /*
   * Submit the newest safe velocity without blocking the ROS
   * callback thread.
   *
   * A real backend should keep only the latest pending command,
   * rather than allowing old commands to accumulate in a queue.
   */
  virtual void submit_velocity(
    const VelocityCommand & command) = 0;

  /*
   * Enter the common safe-stop path.
   *
   * Examples include watchdog timeout, manual disable,
   * invalid input and Unitree API errors.
   */
  virtual void request_stop(
    const std::string & reason) = 0;

  /*
   * Stop the backend worker and release its resources.
   * Implementations must make this operation idempotent.
   */
  virtual void shutdown() = 0;
};

}  // namespace g1_loco_bridge

#endif  // G1_LOCO_BRIDGE__VELOCITY_BACKEND_HPP_
