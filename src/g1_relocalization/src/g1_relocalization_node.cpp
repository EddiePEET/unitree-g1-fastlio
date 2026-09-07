#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

class G1RelocalizationNode : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZI;
  using PointCloudT = pcl::PointCloud<PointT>;

  G1RelocalizationNode()
  : Node("g1_relocalization")
  {
    declareParameters();
    readParameters();
    initializeBodyBaseTransform();

    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/global_pcd_map",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization_pose", 10);

    fitness_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/localization_fitness", 10);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    loadGlobalMap();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &G1RelocalizationNode::cloudCallback,
        this,
        std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &G1RelocalizationNode::odomCallback,
        this,
        std::placeholders::_1));

    initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_,
      10,
      std::bind(
        &G1RelocalizationNode::initialPoseCallback,
        this,
        std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(localization_period_ms_),
      std::bind(&G1RelocalizationNode::localizationTimer, this));

    // Keep TF publishing independent from the slower NDT cycle.
    // This callback group allows the TF timer to run while NDT is computing.
    tf_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    tf_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&G1RelocalizationNode::tfTimer, this),
      tf_callback_group_);

    RCLCPP_INFO(
      get_logger(),
      "G1 relocalization node started. Waiting for /initialpose.");
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("map_path", "");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("odom_frame", "camera_init");
    declare_parameter<std::string>("body_frame", "body");
    declare_parameter<double>("body_roll_deg", 180.0);

    declare_parameter<std::string>(
      "input_cloud_topic", "/cloud_registered_body");
    declare_parameter<std::string>(
      "odometry_topic", "/Odometry");
    declare_parameter<std::string>(
      "initial_pose_topic", "/initialpose");

    declare_parameter<double>("map_voxel_size", 0.25);
    declare_parameter<double>("scan_voxel_size", 0.20);

    declare_parameter<double>("ndt_resolution", 1.0);
    declare_parameter<double>("ndt_step_size", 0.1);
    declare_parameter<double>(
      "ndt_transformation_epsilon", 0.01);
    declare_parameter<int>("ndt_max_iterations", 40);

    declare_parameter<double>("fitness_threshold", 2.0);
    declare_parameter<int>("localization_period_ms", 500);
  }

  void readParameters()
  {
    get_parameter("map_path", map_path_);
    get_parameter("global_frame", global_frame_);
    get_parameter("odom_frame", odom_frame_);
    get_parameter("body_frame", body_frame_);
    get_parameter("body_roll_deg", body_roll_deg_);

    get_parameter("input_cloud_topic", input_cloud_topic_);
    get_parameter("odometry_topic", odometry_topic_);
    get_parameter("initial_pose_topic", initial_pose_topic_);

    get_parameter("map_voxel_size", map_voxel_size_);
    get_parameter("scan_voxel_size", scan_voxel_size_);

    get_parameter("ndt_resolution", ndt_resolution_);
    get_parameter("ndt_step_size", ndt_step_size_);
    get_parameter(
      "ndt_transformation_epsilon",
      ndt_transformation_epsilon_);
    get_parameter("ndt_max_iterations", ndt_max_iterations_);

    get_parameter("fitness_threshold", fitness_threshold_);
    get_parameter(
      "localization_period_ms",
      localization_period_ms_);
  }

  void initializeBodyBaseTransform()
  {
    constexpr double kPi =
      3.14159265358979323846;

    const float roll_rad =
      static_cast<float>(
        body_roll_deg_ * kPi / 180.0);

    // base_T_body：把FAST-LIO倒装body坐标
    // 转换到机器人标准base坐标。
    base_T_body_ =
      Eigen::Matrix4f::Identity();

    base_T_body_.block<3, 3>(0, 0) =
      Eigen::AngleAxisf(
        roll_rad,
        Eigen::Vector3f::UnitX())
      .toRotationMatrix();

    body_T_base_ =
      base_T_body_.inverse();
  }

  void loadGlobalMap()
  {
    if (map_path_.empty()) {
      throw std::runtime_error("Parameter map_path is empty.");
    }

    auto raw_map = PointCloudT::Ptr(new PointCloudT());

    if (pcl::io::loadPCDFile<PointT>(map_path_, *raw_map) < 0) {
      throw std::runtime_error(
        "Failed to load PCD map: " + map_path_);
    }

    map_cloud_ = PointCloudT::Ptr(new PointCloudT());

    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(raw_map);
    voxel_filter.setLeafSize(
      static_cast<float>(map_voxel_size_),
      static_cast<float>(map_voxel_size_),
      static_cast<float>(map_voxel_size_));
    voxel_filter.filter(*map_cloud_);

    if (map_cloud_->empty()) {
      throw std::runtime_error(
        "Global map is empty after voxel filtering.");
    }

    ndt_.setInputTarget(map_cloud_);
    ndt_.setResolution(ndt_resolution_);
    ndt_.setStepSize(ndt_step_size_);
    ndt_.setTransformationEpsilon(
      ndt_transformation_epsilon_);
    ndt_.setMaximumIterations(ndt_max_iterations_);

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud_, map_msg);
    map_msg.header.frame_id = global_frame_;
    map_msg.header.stamp = now();
    map_pub_->publish(map_msg);

    RCLCPP_INFO(
      get_logger(),
      "Loaded map: %s, raw points=%zu, filtered points=%zu",
      map_path_.c_str(),
      raw_map->size(),
      map_cloud_->size());
  }

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto raw_scan = PointCloudT::Ptr(new PointCloudT());
    pcl::fromROSMsg(*msg, *raw_scan);

    if (raw_scan->empty()) {
      return;
    }

    auto filtered_scan = PointCloudT::Ptr(new PointCloudT());

    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(raw_scan);
    voxel_filter.setLeafSize(
      static_cast<float>(scan_voxel_size_),
      static_cast<float>(scan_voxel_size_),
      static_cast<float>(scan_voxel_size_));
    voxel_filter.filter(*filtered_scan);

    if (filtered_scan->size() < 100) {
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_scan_ = filtered_scan;
    latest_scan_stamp_ = msg->header.stamp;
    has_scan_ = true;
  }

  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Eigen::Matrix4f odom_body =
      poseToMatrix(msg->pose.pose);

    std::lock_guard<std::mutex> lock(data_mutex_);
    odom_to_body_ = odom_body;
    has_odom_ = true;
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!msg->header.frame_id.empty() &&
        msg->header.frame_id != global_frame_) {
      RCLCPP_WARN(
        get_logger(),
        "Initial pose frame is '%s', expected '%s'.",
        msg->header.frame_id.c_str(),
        global_frame_.c_str());
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    // RViz的2D Pose Estimate表示map->base_link。
    // NDT源点云位于FAST-LIO倒装body坐标系。
    // 因此必须转换成map->body作为NDT初值。
    initial_guess_map_body_ =
      poseToMatrix(msg->pose.pose)
      * base_T_body_;

    has_initial_pose_ = true;
    localized_ = false;

    RCLCPP_INFO(
      get_logger(),
      "Received initial pose: x=%.3f, y=%.3f, z=%.3f",
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
  }

  void localizationTimer()
  {
    // 在首次NDT成功前，先发布单位变换，使map坐标系在RViz中存在。
    // NDT成功后，map_to_odom_会被真实定位结果更新。
    Eigen::Matrix4f current_map_odom;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      current_map_odom = map_to_odom_;
    }

    builtin_interfaces::msg::Time current_stamp = now();
    // TF is continuously published by tfTimer() at 10 Hz.

    PointCloudT::Ptr scan;
    Eigen::Matrix4f odom_body;
    Eigen::Matrix4f guess;
    builtin_interfaces::msg::Time scan_stamp;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      if (!has_scan_ || !has_odom_) {
        return;
      }

      if (!localized_ && !has_initial_pose_) {
        return;
      }

      scan = latest_scan_;
      odom_body = odom_to_body_;
      scan_stamp = latest_scan_stamp_;

      if (localized_) {
        guess = map_to_odom_ * odom_body;
      } else {
        guess = initial_guess_map_body_;
      }
    }

    ndt_.setInputSource(scan);

    PointCloudT aligned_cloud;
    ndt_.align(aligned_cloud, guess);

    if (!ndt_.hasConverged()) {
      RCLCPP_WARN(
        get_logger(),
        "NDT did not converge.");
      return;
    }

    const double fitness = ndt_.getFitnessScore();

    std_msgs::msg::Float64 fitness_msg;
    fitness_msg.data = fitness;
    fitness_pub_->publish(fitness_msg);

    if (!std::isfinite(fitness) ||
        fitness > fitness_threshold_) {
      RCLCPP_WARN(
        get_logger(),
        "NDT rejected: fitness=%.6f, threshold=%.6f",
        fitness,
        fitness_threshold_);
      return;
    }

    const Eigen::Matrix4f map_body =
      ndt_.getFinalTransformation();

    const Eigen::Matrix4f map_odom =
      map_body * odom_body.inverse();

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      map_to_odom_ = map_odom;
      localized_ = true;
    }

    // map_to_odom_ has been updated above.
    // tfTimer() continuously publishes the latest transform at 10 Hz.
    publishLocalizationPose(map_body, scan_stamp);

    ++successful_match_count_;

    if (successful_match_count_ == 1 ||
        successful_match_count_ % 10 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "NDT success: fitness=%.6f, x=%.3f, y=%.3f, z=%.3f",
        fitness,
        map_body(0, 3),
        map_body(1, 3),
        map_body(2, 3));
    }
  }

  void tfTimer()
  {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      if (!localized_) {
        return;
      }

      transform = map_to_odom_;
    }

    publishTransform(transform);
  }

  void publishTransform(
    const Eigen::Matrix4f & transform)
  {
    geometry_msgs::msg::TransformStamped tf_msg;

    // Always publish the latest transform with a fresh timestamp.
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = global_frame_;
    tf_msg.child_frame_id = odom_frame_;
    tf_msg.transform = matrixToTransform(transform);

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publishLocalizationPose(
    const Eigen::Matrix4f & map_body,
    const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = global_frame_;
    // 对外发布机器人标准base位姿，
    // 而不是倒装IMU的body位姿。
    const Eigen::Matrix4f map_base =
      map_body * body_T_base_;

    pose_msg.pose.pose =
      matrixToPose(map_base);

    pose_msg.pose.covariance.fill(0.0);

    pose_pub_->publish(pose_msg);
  }

  static Eigen::Matrix4f poseToMatrix(
    const geometry_msgs::msg::Pose & pose)
  {
    Eigen::Quaternionf quaternion(
      static_cast<float>(pose.orientation.w),
      static_cast<float>(pose.orientation.x),
      static_cast<float>(pose.orientation.y),
      static_cast<float>(pose.orientation.z));

    if (quaternion.norm() < 1e-6f) {
      quaternion = Eigen::Quaternionf::Identity();
    } else {
      quaternion.normalize();
    }

    Eigen::Matrix4f transform =
      Eigen::Matrix4f::Identity();

    transform.block<3, 3>(0, 0) =
      quaternion.toRotationMatrix();

    transform(0, 3) =
      static_cast<float>(pose.position.x);
    transform(1, 3) =
      static_cast<float>(pose.position.y);
    transform(2, 3) =
      static_cast<float>(pose.position.z);

    return transform;
  }

  static geometry_msgs::msg::Pose matrixToPose(
    const Eigen::Matrix4f & transform)
  {
    geometry_msgs::msg::Pose pose;

    pose.position.x = transform(0, 3);
    pose.position.y = transform(1, 3);
    pose.position.z = transform(2, 3);

    Eigen::Quaternionf quaternion(
      transform.block<3, 3>(0, 0));
    quaternion.normalize();

    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    return pose;
  }

  static geometry_msgs::msg::Transform matrixToTransform(
    const Eigen::Matrix4f & transform)
  {
    geometry_msgs::msg::Transform output;

    output.translation.x = transform(0, 3);
    output.translation.y = transform(1, 3);
    output.translation.z = transform(2, 3);

    Eigen::Quaternionf quaternion(
      transform.block<3, 3>(0, 0));
    quaternion.normalize();

    output.rotation.x = quaternion.x();
    output.rotation.y = quaternion.y();
    output.rotation.z = quaternion.z();
    output.rotation.w = quaternion.w();

    return output;
  }

  std::string map_path_;
  std::string global_frame_;
  std::string odom_frame_;
  std::string body_frame_;

  std::string input_cloud_topic_;
  std::string odometry_topic_;
  std::string initial_pose_topic_;

  double map_voxel_size_{0.25};
  double scan_voxel_size_{0.20};

  double ndt_resolution_{1.0};
  double ndt_step_size_{0.1};
  double ndt_transformation_epsilon_{0.01};
  int ndt_max_iterations_{40};

  double fitness_threshold_{2.0};
  int localization_period_ms_{500};
  double body_roll_deg_{180.0};

  PointCloudT::Ptr map_cloud_;
  PointCloudT::Ptr latest_scan_;

  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;

  Eigen::Matrix4f base_T_body_ =
    Eigen::Matrix4f::Identity();

  Eigen::Matrix4f body_T_base_ =
    Eigen::Matrix4f::Identity();

  Eigen::Matrix4f odom_to_body_ =
    Eigen::Matrix4f::Identity();

  Eigen::Matrix4f initial_guess_map_body_ =
    Eigen::Matrix4f::Identity();

  Eigen::Matrix4f map_to_odom_ =
    Eigen::Matrix4f::Identity();

  builtin_interfaces::msg::Time latest_scan_stamp_;

  bool has_scan_{false};
  bool has_odom_{false};
  bool has_initial_pose_{false};
  bool localized_{false};

  std::size_t successful_match_count_{0};

  std::mutex data_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_sub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    map_pub_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    fitness_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster>
    tf_broadcaster_;

  rclcpp::CallbackGroup::SharedPtr tf_callback_group_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<G1RelocalizationNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("g1_relocalization"),
      "%s",
      error.what());
  }

  rclcpp::shutdown();
  return 0;
}
