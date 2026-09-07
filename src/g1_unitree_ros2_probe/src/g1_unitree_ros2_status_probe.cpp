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
constexpr int32_t kLocoStateNotAvailable = 7301;

}  // namespace

class G1UnitreeRos2StatusProbe : public rclcpp::Node
{
public:
  G1UnitreeRos2StatusProbe()
  : Node("g1_unitree_ros2_status_probe"),
    client_(this)
  {
    RCLCPP_WARN(
      get_logger(),
      "Official Unitree ROS 2 read-only probe. "
      "No motion or mode-changing APIs are used.");

    worker_ = std::thread(
      [this]()
      {
        run_queries();
      });
  }

  ~G1UnitreeRos2StatusProbe() override
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
  void run_queries()
  {
    try {
      // Allow the ROS executor time to begin spinning and discover DDS peers.
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      int fsm_id = -1;
      int fsm_mode = -1;
      int balance_mode = -1;

      RCLCPP_INFO(get_logger(), "Calling GetFsmId...");
      const int32_t fsm_id_result =
        client_.GetFsmId(fsm_id);

      RCLCPP_INFO(
        get_logger(),
        "GetFsmId returned: code=%d, value=%d",
        fsm_id_result,
        fsm_id);

      RCLCPP_INFO(get_logger(), "Calling GetFsmMode...");
      const int32_t fsm_mode_result =
        client_.GetFsmMode(fsm_mode);

      RCLCPP_INFO(
        get_logger(),
        "GetFsmMode returned: code=%d, value=%d",
        fsm_mode_result,
        fsm_mode);

      RCLCPP_INFO(get_logger(), "Calling GetBalanceMode...");
      const int32_t balance_mode_result =
        client_.GetBalanceMode(balance_mode);

      if (balance_mode_result == kSuccess) {
        RCLCPP_INFO(
          get_logger(),
          "GetBalanceMode returned: code=0, value=%d",
          balance_mode);
      } else if (balance_mode_result == kLocoStateNotAvailable) {
        RCLCPP_WARN(
          get_logger(),
          "GetBalanceMode returned code 7301: "
          "LocoState not available.");
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "GetBalanceMode returned: code=%d",
          balance_mode_result);
      }

      const bool primary_queries_ok =
        fsm_id_result == kSuccess &&
        fsm_mode_result == kSuccess;

      const bool balance_query_acceptable =
        balance_mode_result == kSuccess ||
        balance_mode_result == kLocoStateNotAvailable;

      if (primary_queries_ok && balance_query_acceptable) {
        exit_code_.store(0);

        RCLCPP_INFO(
          get_logger(),
          "Official Unitree ROS 2 read-only communication check passed.");
      } else {
        exit_code_.store(2);

        RCLCPP_ERROR(
          get_logger(),
          "Official Unitree ROS 2 read-only communication check failed.");
      }
    } catch (const std::exception & exception) {
      exit_code_.store(3);

      RCLCPP_FATAL(
        get_logger(),
        "Read-only worker threw an exception: %s",
        exception.what());
    } catch (...) {
      exit_code_.store(4);

      RCLCPP_FATAL(
        get_logger(),
        "Read-only worker threw an unknown exception.");
    }

    RCLCPP_INFO(
      get_logger(),
      "Read-only worker finished. Requesting ROS shutdown.");

    rclcpp::shutdown();
  }

  unitree::robot::g1::LocoClient client_;
  std::thread worker_;
  std::atomic<int> exit_code_{1};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<G1UnitreeRos2StatusProbe>();

  // The main thread must keep spinning so BaseClient's temporary
  // /api/sport/response subscription can receive the matching response.
  rclcpp::spin(node);

  const int exit_code = node->exit_code();

  node.reset();

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return exit_code;
}
