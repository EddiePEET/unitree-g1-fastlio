#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <g1/g1_loco_client.hpp>

namespace
{

constexpr int32_t kSuccess = 0;

bool is_supported_motion_fsm(const int fsm_id)
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

class G1UnitreeRos2StartProbe : public rclcpp::Node
{
public:
  G1UnitreeRos2StartProbe()
  : Node("g1_unitree_ros2_start_probe"),
    client_(this)
  {
    RCLCPP_WARN(
      get_logger(),
      "Official Start-only probe. "
      "This program does not call SetVelocity.");

    worker_ = std::thread([this]() { run_probe(); });
  }

  ~G1UnitreeRos2StartProbe() override
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
      // 等待 DDS 服务发现完成。
      std::this_thread::sleep_for(
        std::chrono::milliseconds(1000));

      int fsm_id = -1;
      int fsm_mode = -1;

      const int32_t id_ret =
        client_.GetFsmId(fsm_id);

      RCLCPP_INFO(
        get_logger(),
        "Before Start: GetFsmId code=%d value=%d",
        id_ret,
        fsm_id);

      const int32_t mode_ret =
        client_.GetFsmMode(fsm_mode);

      RCLCPP_INFO(
        get_logger(),
        "Before Start: GetFsmMode code=%d value=%d",
        mode_ret,
        fsm_mode);

      if (id_ret != kSuccess || mode_ret != kSuccess) {
        RCLCPP_ERROR(
          get_logger(),
          "Initial FSM query failed. Start will not be called.");
        finish(2);
        return;
      }

      if (!is_supported_motion_fsm(fsm_id)) {
        RCLCPP_ERROR(
          get_logger(),
          "FSM ID=%d is not in the allowed list "
          "[500, 501, 801, 802]. Start will not be called.",
          fsm_id);
        finish(3);
        return;
      }

      if (fsm_mode == 1) {
        RCLCPP_WARN(
          get_logger(),
          "Robot is already in dynamic mode. "
          "Start will not be called again.");

        const int32_t stop_ret = client_.StopMove();

        RCLCPP_WARN(
          get_logger(),
          "StopMove returned code=%d",
          stop_ret);

        finish(stop_ret == kSuccess ? 0 : 7);
        return;
      }

      if (fsm_mode != 0) {
        RCLCPP_ERROR(
          get_logger(),
          "Unexpected FSM mode=%d. "
          "Expected 0 (static) or 1 (dynamic).",
          fsm_mode);
        finish(4);
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "Calling official LocoClient::Start() exactly once. "
        "The robot may change posture or enter active standing.");

      const int32_t start_ret = client_.Start();

      RCLCPP_WARN(
        get_logger(),
        "Start returned code=%d",
        start_ret);

      if (start_ret != kSuccess) {
        RCLCPP_ERROR(
          get_logger(),
          "Start failed. No velocity command was sent.");
        finish(5);
        return;
      }

      bool dynamic = false;

      for (int attempt = 1; attempt <= 20; ++attempt) {
        std::this_thread::sleep_for(
          std::chrono::milliseconds(500));

        int current_mode = -1;
        const int32_t ret =
          client_.GetFsmMode(current_mode);

        RCLCPP_INFO(
          get_logger(),
          "After Start check %d/20: code=%d fsm_mode=%d",
          attempt,
          ret,
          current_mode);

        if (ret == kSuccess && current_mode == 1) {
          dynamic = true;
          break;
        }
      }

      const int32_t stop_ret =
        client_.StopMove();

      RCLCPP_WARN(
        get_logger(),
        "StopMove returned code=%d",
        stop_ret);

      if (!dynamic) {
        RCLCPP_ERROR(
          get_logger(),
          "Start returned success, but fsm_mode did not "
          "change from static to dynamic.");
        finish(6);
        return;
      }

      if (stop_ret != kSuccess) {
        RCLCPP_ERROR(
          get_logger(),
          "Dynamic mode was reached, but StopMove failed.");
        finish(7);
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "Start probe passed: fsm_mode changed to 1 "
        "and StopMove returned code=0.");

      finish(0);
    } catch (const std::exception & exception) {
      RCLCPP_FATAL(
        get_logger(),
        "Start probe exception: %s",
        exception.what());
      finish(8);
    } catch (...) {
      RCLCPP_FATAL(
        get_logger(),
        "Start probe unknown exception.");
      finish(9);
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
    std::make_shared<G1UnitreeRos2StartProbe>();

  rclcpp::spin(node);

  const int code = node->exit_code();

  node.reset();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return code;
}
