# unitree-g1-fastlio

Unitree G1 + Livox MID360 的 ROS 2 建图与导航工作空间快照。当前仓库版本为 **v8.7**，包含 Livox 驱动、FAST-LIO、NDT 重定位、Nav2 规划、安全速度桥、地图和运行脚本。

> 安全提示：真实机器人运动存在碰撞和人身伤害风险。请先完成仿真或 Dry Run 验证，并确保遥控器、急停和安全人员到位。`g1_navigation.sh` 不会直接启用真实运动；真实桥还需要单独运行 `g1_navigation_real.sh` 并显式执行 `enable`。

## 目录结构

```text
.
├── maps/                         # PCD 地图、Nav2 栅格地图和转换脚本
├── script/                       # 建图、导航和真实桥管理脚本
├── src/
│   ├── FAST_LIO_ROS2/            # FAST-LIO ROS 2
│   ├── Livox-SDK2/               # Livox SDK 2 源码
│   ├── livox_ros_driver2/         # Livox ROS 2 驱动
│   ├── g1_relocalization/         # NDT 重定位
│   ├── g1_nav2_bringup/           # Nav2 规划启动与配置
│   ├── g1_loco_bridge/            # G1 速度命令安全桥
│   ├── g1_unitree_ros2_probe/     # Unitree ROS 2 探测工具
│   └── unitree_ros2_interfaces/   # Unitree ROS 2 消息接口
├── third_party/                   # Unitree SDK、头文件、预编译库及许可证
└── g1_cmd_vel_dry_run.yaml        # Dry Run 示例参数
```

当前地图：

- `maps/map.pcd`：重定位使用的点云地图。
- `maps/map_multilayer.pgm` 与 `maps/map_multilayer.yaml`：Nav2 栅格地图。
- `maps/pcd2pgm_multilayer.py`：PCD 到栅格地图的转换脚本。

本次源包中没有独立的 `waypoints/` 或 `task_data/` 航点文件。

## 运行环境

该版本按以下环境组织：

- Ubuntu 20.04
- ROS 2 Foxy
- `colcon` 与 `rosdep`
- PCL、Eigen、Nav2
- Livox MID360
- 真实 G1 控制需要单独配置 Unitree ROS 2/CycloneDDS 环境

仓库包含第三方源码与部分厂商预编译库，支持 `x86_64` 和 `aarch64`。请根据实际主机架构、ROS 2 安装和设备网络配置进行验证。

## 获取与编译

```bash
git clone https://github.com/EddiePEET/unitree-g1-fastlio.git
cd unitree-g1-fastlio

source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

管理脚本会优先从脚本所在位置自动识别工作空间。可用环境变量覆盖外部环境路径：

```bash
export G1_WS="$PWD"
export PCL_SETUP="$HOME/ws_pcl_ros/install/setup.bash"       # 可选，自定义 PCL underlay
export UNITREE_ROS2_SETUP=/path/to/cyclonedds_ws/install/setup.bash
```

`UNITREE_ROS2_SETUP` 在仅建图或规划/Dry Run 时可以不设置；使用真实 G1 控制桥前必须正确设置并验证。

## 建图

```bash
./script/g1_mapping.sh start
./script/g1_mapping.sh status
./script/g1_mapping.sh save
./script/g1_mapping.sh finish
```

常用辅助命令：

```bash
./script/g1_mapping.sh logs
./script/g1_mapping.sh map-path
./script/g1_mapping.sh stop
```

建图脚本会在 `runtime/mapping/` 生成日志、PID、会话信息和临时配置；这些文件不进入 Git。

## 定位与 Nav2

`g1_navigation.sh` 支持规划模式和 Dry Run 安全桥，不直接开放真实运动入口：

```bash
./script/g1_navigation.sh start
./script/g1_navigation.sh continue

# 或启动 Dry Run
./script/g1_navigation.sh start-dry
./script/g1_navigation.sh continue-dry

./script/g1_navigation.sh status
./script/g1_navigation.sh logs
./script/g1_navigation.sh stop
```

关键接口：

- 全局坐标系：`map`
- 里程计坐标系：`camera_init`
- FAST-LIO 机体坐标系：`body`
- 导航底盘坐标系：`nav_base`
- 里程计话题：`/Odometry`
- 局部障碍点云：`/cloud_registered_body`
- 隔离速度输出：`/cmd_vel_nav`

## 真实 G1 控制桥

只有在完成 Dry Run、现场安全检查和 Unitree ROS 2 环境配置后，才运行：

```bash
export UNITREE_ROS2_SETUP=/path/to/cyclonedds_ws/install/setup.bash

./script/g1_navigation_real.sh start
./script/g1_navigation_real.sh enable
./script/g1_navigation_real.sh disable
./script/g1_navigation_real.sh stop
```

`start` 仅启动真实桥并保持安全门关闭；`enable` 才允许 Nav2 命令进入真实机器人后端。

## 配置说明

- MID360/FAST-LIO：`src/FAST_LIO_ROS2/config/mid360.yaml`
- NDT 重定位：`src/g1_relocalization/config/relocalization.yaml`
- Nav2：`src/g1_nav2_bringup/config/nav2_params.yaml`
- Dry Run 与真实桥：`src/g1_loco_bridge/config/`

地图路径在仓库配置中留空，由管理脚本在启动时注入当前工作空间内的地图绝对路径。若绕过管理脚本直接启动节点，请显式传入 `map_file_path` 或 `map_path`。

## 仓库清理说明

此快照已移除可重新生成或仅与原机器有关的内容，包括：

- ROS 2/colcon 的 `build/`、`install/`、`log/`
- `runtime/` 日志、PID 和临时配置
- 嵌套仓库的 `.git/` 历史
- Python 缓存、IDE 配置、编辑器临时文件
- 本地备份目录及 `.bak`、`.before` 等历史副本

## 许可证

第三方组件继续使用各自目录中的许可证文件；例如 FAST-LIO、Livox-SDK2、livox_ros_driver2、Unitree SDK2、CycloneDDS、iceoryx 和 RapidJSON。请在复制、发布或商用前分别核对这些条款。

仓库根目录目前没有为自研部分单独声明统一许可证，因此未明确授权的自研代码默认不应视为开源许可授予。

## 已知限制

- 这是从已运行工作空间整理出的 v8.7 源码快照，不包含编译产物。
- 本次整理环境不是 Ubuntu/ROS 2 主机，未在此环境重新编译或连接实机验收。
- 真实导航必须在目标机器人、雷达、DDS 网络和安全条件齐备后重新验证。
