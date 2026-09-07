#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <g1/g1_loco_client.hpp>

class G1UnitreeRos2VelocityProbe : public rclcpp::Node
{
public:
  G1UnitreeRos2VelocityProbe()
  : Node("g1_unitree_ros2_velocity_probe"),
    client_(this)
  {
    RCLCPP_WARN(
      get_logger(),
      "Official one-shot velocity probe. "
      "This program does NOT call Start(), SetFsmId(), "
      "or any mode-switching API.");

    RCLCPP_WARN(
      get_logger(),
      "Command is fixed to vx=0.10 m/s, vy=0, "
      "omega=0, duration=1.0 s.");

    worker_ = std::thread([this]() {
      run_probe();
    });
  }

  ~G1UnitreeRos2VelocityProbe() override
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  int exit_code() const
  {
    return exit_code_.load();
  }

private:
  void finish(const int code)
  {
    exit_code_.store(code);

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void run_probe()
  {
    try {
      // 等待 DDS 请求/响应端完成发现。
      std::this_thread::sleep_for(
        std::chrono::seconds(2));

      int fsm_id = -1;
      int fsm_mode = -1;

      const int32_t id_ret =
        client_.GetFsmId(fsm_id);

      const int32_t mode_ret =
        client_.GetFsmMode(fsm_mode);

      RCLCPP_INFO(
        get_logger(),
        "Read-only state: "
        "GetFsmId code=%d value=%d, "
        "GetFsmMode code=%d value=%d",
        id_ret,
        fsm_id,
        mode_ret,
        fsm_mode);

      if (id_ret != 0 || mode_ret != 0) {
        RCLCPP_ERROR(
          get_logger(),
          "Read-only communication failed. "
          "No velocity command will be sent.");

        finish(2);
        return;
      }

      // 模式由用户手动设置，本程序不判断也不切换。
      for (int count = 3; count >= 1; --count) {
        if (!rclcpp::ok()) {
          finish(3);
          return;
        }

        RCLCPP_WARN(
          get_logger(),
          "Velocity command will be sent in %d second(s).",
          count);

        std::this_thread::sleep_for(
          std::chrono::seconds(1));
      }

      constexpr float vx = 0.10F;
      constexpr float vy = 0.00F;
      constexpr float omega = 0.00F;
      constexpr float duration = 1.00F;

      RCLCPP_WARN(
        get_logger(),
        "Calling official LocoClient::SetVelocity "
        "exactly once.");

      const int32_t move_ret =
        client_.SetVelocity(
          vx,
          vy,
          omega,
          duration);

      RCLCPP_WARN(
        get_logger(),
        "SetVelocity returned code=%d "
        "vx=%.2f vy=%.2f omega=%.2f duration=%.2f",
        move_ret,
        vx,
        vy,
        omega,
        duration);

      if (move_ret != 0) {
        RCLCPP_ERROR(
          get_logger(),
          "Official SetVelocity failed.");

        const int32_t stop_ret =
          client_.StopMove();

        RCLCPP_WARN(
          get_logger(),
          "Emergency StopMove returned code=%d",
          stop_ret);

        finish(4);
        return;
      }

      // 等待命令自身的 duration 结束。
      std::this_thread::sleep_for(
        std::chrono::milliseconds(1300));

      const int32_t stop_ret =
        client_.StopMove();

      RCLCPP_WARN(
        get_logger(),
        "Final StopMove returned code=%d",
        stop_ret);

      if (stop_ret != 0) {
        finish(5);
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "One-shot official velocity probe completed.");

      finish(0);
    } catch (const std::exception & exception) {
      RCLCPP_FATAL(
        get_logger(),
        "Probe exception: %s",
        exception.what());

      finish(6);
    } catch (...) {
      RCLCPP_FATAL(
        get_logger(),
        "Unknown probe exception.");

      finish(7);
    }
  }

  unitree::robot::g1::LocoClient client_;
  std::thread worker_;
  std::atomic<int> exit_code_{1};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<G1UnitreeRos2VelocityProbe>();

  rclcpp::spin(node);

  const int result = node->exit_code();

  node.reset();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return result;
}
