# reinovo bobac3 · ROS Melodic 服务机器人仿真工程

![ROS Melodic](https://img.shields.io/badge/ROS-Melodic-blue)
![Ubuntu 18.04](https://img.shields.io/badge/Ubuntu-18.04-orange)
![Python 2.7](https://img.shields.io/badge/Python-2.7-yellow)
![Gazebo 9](https://img.shields.io/badge/Gazebo-9-brightgreen)
![License](https://img.shields.io/badge/License-MIT-green)

## 1. 项目概述

本项目是一个基于 ROS Melodic 和 Gazebo 的服务机器人综合仿真工程，源自 reinovo（瑞诺）公司 **bobac3 服务机器人**的算法开发虚拟机，用于模拟和测试服务机器人在室内展馆环境中的**自主建图、定位导航、语音交互和任务执行**能力。

工程覆盖从底层底盘驱动到高层任务调度的完整技术栈：26 个 ROS 功能包组成了「感知 → 定位 → 规划 → 控制 → 任务」的完整闭环，既支持实车部署，也提供 Gazebo 仿真版本，所有核心模块均提供实车版与仿真版两套 launch 文件。

**核心场景**：展馆语音导览机器人——一条命令启动仿真，对着麦克风说「**带我参观一圈**」，机器人即自动依次导航至北京馆、上海馆、广州馆、深圳馆、吉林馆，每到一站播放该馆讲解语音，最后返回出发点。

```bash
roslaunch nav_goal tiaoshi.launch
# 然后对着麦克风说："带我参观一圈"
```

![系统概览](docs/images/01-system-overview.jpg)

---

## 2. 核心功能

* **语音触发全流程导览**：一句「带我参观一圈」即可触发完整的多点导航 + 讲解流程，无需手动发目标点。也支持「去北京馆」等单点指令。
* **三套 SLAM 方案可选**：集成 GMapping、Cartographer、RTabMap 三种建图算法，通过 launch 参数一键切换，实车与仿真版本分离。
* **双导航栈配置**：实车使用 TEB 局部规划器（动态避障平滑），仿真使用 DWA 局部规划器（轻量稳定），全局规划器分别为 navfn 与 global_planner，参数按实车/仿真分目录管理。
* **全向轮里程计模型**：AMCL 采用 `omni-corrected` 里程计模型，专为麦克纳姆轮全向底盘调优。
* **双雷达数据融合**：`rei_lidar_fuse` 通过 nodelet 将前后两路激光雷达融合为单一的 `fuse_scan` 话题，实现 360° 无死角覆盖。
* **语音交互闭环**：集成科大讯飞 AIUI SDK，支持语音唤醒、离线识别、语义理解和 TTS 播报，导航点与语音讲解内容联动。
* **任务序列编排**：通过 YAML 定义任务序列（导航点 + 语音 + 等待 + 相对移动），由 `multi_task` 主控循环执行，无需修改代码即可调整任务流程。
* **AR 视觉定位**：`ar_pose` 支持底座/手臂/头部三个相机安装位姿，用于充电桩对接等高精度定位场景。
* **完整 Gazebo 仿真**：自定义机器人 URDF/Xacro 模型（含底盘、机械臂、多传感器）与 3 个室内展馆 `.world` 场景。

---

## 3. 系统架构

#### 3.1. 总体设计

系统采用分层模块化架构，各功能包通过 ROS 的 Topics / Services / Actions 通信机制解耦。整体分为四层：

| 层级 | 功能包 | 职责 |
|---|---|---|
| **任务层** | `multi_task` | 任务序列编排、语音播报联动、异常处理 |
| **导航层** | `bobac3_navigation`、`amcl`、`global_planner`、`my_planner` | 定位、路径规划、运动控制 |
| **感知层** | `rei_lidar_fuse`、`face_rec`、`ar_pose`、`deep_camera_scan_filter` | 传感器融合与视觉识别 |
| **驱动层** | `rei_robot_base`、`robot_joy`、`robot_audio` | 底盘控制、手柄遥控、语音硬件 |

#### 3.2. 架构图

```mermaid
graph TD
    subgraph "任务编排层 Task Layer"
        NavGoal("nav_goal<br/>tiaoshi_node<br/>★ 语音导览主控")
        MultiTask("multi_task<br/>meals_delivered.py")
        Cruise("rei_robot_cruise<br/>多目标巡航")
    end

    subgraph "导航层 Navigation Layer"
        MoveBase("move_base<br/>全局: navfn/global_planner<br/>局部: TEB / DWA")
        AMCL("amcl<br/>omni-corrected")
        MapServer("map_server<br/>/map")
        Cleaner("jie_ware<br/>costmap_cleaner")
    end

    subgraph "感知层 Perception Layer"
        LidarFuse("rei_lidar_fuse<br/>双雷达融合 → fuse_scan")
        ArPose("ar_pose<br/>AR 标签位姿")
        FaceRec("face_rec<br/>人脸识别")
    end

    subgraph "仿真/驱动层 Simulation & Base Layer"
        Gazebo("bobac3_description<br/>Gazebo 仿真")
        RobotBase("rei_robot_base<br/>底盘驱动")
        Audio("robot_audio<br/>讯飞 AIUI")
    end

    NavGoal -- "MoveBaseGoal (Action)" --> MoveBase
    MultiTask -- "MoveBaseGoal (Action)" --> MoveBase
    Cruise -- "SetGoals (Srv)" --> MoveBase
    MapServer -- "/map" --> AMCL
    MapServer -- "/map" --> MoveBase
    AMCL -- "map→odom TF" --> MoveBase
    LidarFuse -- "/fuse_scan" --> AMCL
    LidarFuse -- "/fuse_scan" --> MoveBase
    Gazebo -- "/scan /odom /tf" --> LidarFuse
    Gazebo -- "/scan /odom /tf" --> AMCL
    MoveBase -- "/cmd_vel" --> RobotBase
    MoveBase -- "/cmd_vel" --> Gazebo
    Cleaner -. "清理代价地图" .-> MoveBase
    Audio -- "/voice_dictation 识别文本" --> NavGoal
    NavGoal -- "/voice_tts 播报" --> Audio
    Audio <-.-> MultiTask
    ArPose -.-> MultiTask
```

#### 3.3. 数据流

1. **环境感知**：Gazebo 仿真插件（或实车驱动）发布激光雷达数据 `/scan`、轮式里程计 `/odom` 和 TF 坐标变换；`rei_lidar_fuse` 将前后双雷达数据融合为 `/fuse_scan`。
2. **地图服务**：`map_server` 加载 `bobac3_navigation/maps/` 下预建的栅格地图（22 个版本），发布到 `/map` 话题。
3. **自主定位**：AMCL 节点订阅 `/fuse_scan`、`/odom`、`/map`，使用 `omni-corrected` 全向里程计模型和 likelihood_field 激光模型，输出 `map → odom` 的 TF 变换和位姿估计。
4. **任务下发**：`multi_task` 读取 `task_sequence.yaml` 任务序列，通过 actionlib 向 `move_base` 发送 `MoveBaseGoal` 目标点。
5. **路径规划与执行**：`move_base` 全局规划器基于静态地图计算最优路径，局部规划器（TEB/DWA）根据实时传感器数据动态避障，输出 `/cmd_vel` 速度指令。
6. **物理执行**：底盘驱动（或 Gazebo 插件）订阅 `/cmd_vel`，转换为麦轮速度并驱动机器人运动。
7. **任务联动**：到达目标点后，`multi_task` 调用 `robot_audio` 服务播放对应的展馆讲解语音，随后执行下一个任务项。

---

## 4. 环境要求

* **操作系统**：Ubuntu 18.04 LTS（Melodic 官方支持版本）
* **ROS 版本**：ROS Melodic Morenia（建议安装 `ros-melodic-desktop-full`）
* **核心依赖包**：

```bash
sudo apt install -y \
    ros-melodic-navigation \
    ros-melodic-gmapping \
    ros-melodic-cartographer-ros \
    ros-melodic-rtabmap-ros \
    ros-melodic-map-server \
    ros-melodic-move-base \
    ros-melodic-amcl \
    ros-melodic-joy \
    ros-melodic-nodelet
```

* **Python**：Python 2.7（Ubuntu 18.04 默认，**勿用 Python 3**）
* **硬件（实车部署时）**：bobac3 底盘、前后双激光雷达、麦克风阵列、USB 相机

---

## 5. 安装与配置

#### 5.1. 获取源码

```bash
git clone https://github.com/kang-jia-ning/reinovo_Melodic.git ~/bobac3_ws
cd ~/bobac3_ws
```

> 项目原始路径为 `/home/bobac3/bobac3_ws`，源码中有 114 处硬编码绝对路径。若使用其他用户名，请执行：
> ```bash
> grep -rl "/home/bobac3" src | xargs sed -i "s|/home/bobac3|$HOME|g"
> ```

#### 5.2. 安装依赖

```bash
cd ~/bobac3_ws
rosdep install --from-paths src --ignore-src -r -y
```

#### 5.3. 编译

```bash
cd ~/bobac3_ws
catkin_make
```

#### 5.4. 配置环境

```bash
echo "source ~/bobac3_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 6. 使用说明

#### 6.1. 查看机器人模型（RViz）

```bash
roslaunch bobac3_description display.launch
```

![RViz 机器人模型](docs/images/04-rviz-robot-model.jpg)

启动后同时打开 `joint_state_publisher_gui` 关节控制面板，可拖动滑块观察底盘轮组与机械臂关节运动。

#### 6.2. 启动 Gazebo 仿真

```bash
roslaunch bobac3_description gazebo.launch
```

![Gazebo 仿真世界](docs/images/05-gazebo-world.jpg)

默认加载 `yes_my_world.world` 展馆场景。可选场景：

| 场景文件 | 说明 |
|---|---|
| `yes_my_world.world` | 标准展馆布局（默认） |
| `bos.world` | 备用展馆场景 |
| `it_is_obs_world.world` | 含额外障碍物场景 |

#### 6.3. SLAM 建图

```bash
# 仿真：GMapping（默认）
roslaunch bobac3_slam bobac3_slam_sim.launch

# 仿真：Cartographer
roslaunch bobac3_slam bobac3_slam_sim.launch slam_methods:=cartographer

# 仿真：RTabMap
roslaunch bobac3_slam bobac3_slam_sim.launch slam_methods:=rtabmap

# 实车：GMapping（使用融合雷达 fuse_scan）
roslaunch bobac3_slam bobac3_slam.launch
```

遥控机器人扫完场景后保存地图：

```bash
roslaunch bobac3_slam rtabmap_saver.launch
# 或手动
rosrun map_server map_saver -f ~/bobac3_ws/src/bobac3_navigation/maps/my_map
```

#### 6.4. 自主导航

```bash
# 仿真导航（Gazebo + map_server + AMCL + move_base + RViz 一键启动）
roslaunch bobac3_navigation demo_nav_2d.launch

# 指定地图与定位方式
roslaunch bobac3_navigation demo_nav_2d.launch \
    map_file_name:=demonew3 \
    location_method:=amcl_sim
```

![导航栈参数构成](docs/images/06-navigation-params.jpg)

启动内容：

| 组件 | 来源 | 说明 |
|---|---|---|
| Gazebo 仿真 | `bobac3_description/gazebo.launch` | 场景 + 机器人 |
| map_server | `map_server.launch` | 加载 `maps/demonew.yaml` |
| AMCL | `amcl_sim.launch` | 全向轮模型，500–2000 粒子 |
| move_base | `move_base_sim.launch` | DWA 局部规划（仿真） |
| costmap_cleaner | `jie_ware` 包 | 定期清理代价地图残留 |

在 RViz 中使用 **2D Nav Goal** 工具发送目标点，机器人即自主规划并移动。

#### 6.5. 运行导览任务（语音触发）

**这是本项目的主演示流程**，一条命令启动完整导览系统：

```bash
roslaunch nav_goal tiaoshi.launch
```

![送餐导览任务链](docs/images/07-task-chain.jpg)

`tiaoshi.launch` 一次性拉起以下组件：

| 组件 | 来源 | 作用 |
|---|---|---|
| Gazebo + 机器人 | `bobac3_description/gazebo.launch` | 仿真环境 |
| map_server | `bobac3_navigation/map_server.launch` | 加载地图 `demonew3` |
| move_base | `bobac3_navigation/move_base_sim.launch` | 导航（DWA） |
| AMCL | `bobac3_navigation/amcl_sim.launch` | 定位 |
| RViz | `bobac3_navigation/rviz/robot_navigation.rviz` | 可视化 |
| **costmap_cleaner** ×2 | `jie_ware` | 代价地图清理 + 速度归零清理 |
| **tiaoshi_node** | `nav_goal` | 语音触发 + 导航点调度主控 |
| **voice_aiui_node** | `robot_audio` | 讯飞语音识别 |
| **voice_collect_node** | `robot_audio` | 麦克风音频采集 |

启动完成后，**对着麦克风说一句话即可触发任务**。程序在 `tiaoshi.cpp` 主循环中持续采集语音、转文字、做关键词匹配：

| 你说的话 | 关键词匹配 | 机器人行为 |
|---|---|---|
| 「**带我参观一圈**」 | `一圈` | **启动完整导览流程**（见下方） |
| 「元宝」 | `元宝` | 播报「什么事呀.wav」 |
| 「去北京馆」 | `北京` | 单独导航到北京馆 + 讲解 |
| 「去上海馆」 | `上海` | 单独导航到上海馆 + 讲解 |
| 「去广州馆」 | `广州` | 单独导航到广州馆 + 讲解 |
| 「去深圳馆」 | `深圳` | 单独导航到深圳馆 + 讲解 |
| 「去吉林馆」 | `吉林` | 单独导航到吉林馆 + 讲解 |

**「一圈」导览流程**（`tiaoshi.cpp:178`）：

```
语音"带我参观一圈"
  → 播报「这就带你参观.wav」
  → goal1 北京馆  → 播报「北京是中国的首都政治中心.wav」
  → goal2 上海馆  → 播报「上海是中国的经济中心.wav」
  → goal3 广州馆  → 播报「广州自古都是我国的商业之都.wav」
  → goal4 深圳馆  → 播报「深圳是中国的科创中心.wav」
  → goal5 吉林馆  → 播报「吉林位于我国东北是人参之都.wav」
  → goal6 返回出发点 → 任务结束
```

每个导航点之间是**阻塞等待**：`ac.sendGoal(goalN)` 后循环 `while(!SUCCEEDED)` 直到 `move_base` 返回成功，才播放该馆讲解语音，然后前往下一个点。语音文件位于 `nav_goal/voices/`（13 个 wav）。

**导航点坐标**在 `nav_goal/config/params.yaml` 中定义（非 `multi_task` 的 yaml）：

```yaml
beijing_x: 2.520      beijing_y: 0.125
shanghai_x: 1.046     shanghai_y: 2.110
guangzhou_x: 2.526    guangzhou_y: 1.105
shenzhen_x: 1.050     shenzhen_y: 1.123
jilin_x: 2.5215       jilin_y: 2.1183
charge_x: 0.592       charge_y: 1.937
```

> ⚠️ 注意 `tiaoshi.launch` 第 40 行有一处**笔误**：`<node .../>` 后面多了一个孤立的字符 `s`。ROS launch 解析器通常容忍行尾多余文本，但严格模式下会告警，可自行删除。

#### 6.6. 其他任务方式

```bash
# 送餐流程（Python 主控 + YAML 任务序列）
roslaunch multi_task meals_delivered.launch

# 多点导航（另一套 C++ 主控）
roslaunch nav_goal nav.launch
```

> `nav.launch` 与 `tiaoshi.launch` 结构几乎相同，区别是主控节点换成 `nav_multigoal_pro_node`，默认地图为 `reicom`。`multi_task` 走的是 Python + YAML 配置化路线，`nav_goal` 走的是 C++ 硬编码路线。

#### 6.6. 单模块测试

```bash
# 相对移动：前进 0.2m
rosservice call /relative_move "frame: base_link
target: [0.2, 0, 0]"

# 自动充电
rosservice call /set_charge "..."

# 多目标巡航
roslaunch rei_robot_cruise example.launch
```

---

## 7. 模块规格详述

#### 7.1. `src/bobac3_description`: 模型与仿真模块

* **职责**：定义 bobac3 机器人的 URDF/Xacro 模型，提供 Gazebo 仿真环境。
* **组件**：
  * `xacro/bobac3_serverbot.xacro`：机器人主模型，包含麦克纳姆轮底盘、6 自由度机械臂、前后双激光雷达、深度相机、超声波传感器、防跌落传感器。
  * `xacro/bobac3_arm.xacro`、`bobac3_base.xacro`：机械臂与底盘子模块。
  * `xacro/gazebo_plugin.xacro`：Gazebo 差速/全向驱动插件、激光雷达插件、相机插件配置。
  * `launch/display.launch`：模型可视化（RViz + joint_state_publisher_gui）。
  * `launch/gazebo.launch`：仿真主启动文件，加载 world 并 spawn 机器人。
  * `meshes/`：50+ 个 STL 网格文件（底盘、轮组、相机支架、配送箱等）。
  * `gazebo_kinematics.cpp`：仿真运动学解算辅助节点。

#### 7.2. `src/bobac3_navigation`: 导航配置模块

* **职责**：集成 ROS 导航栈，管理所有导航参数与地图资源。
* **组件**：
  * `launch/demo_nav_2d.launch`：一键导航入口，串联 Gazebo + map_server + move_base + AMCL + RViz。
  * `launch/move_base.launch`（实车）/**`move_base_sim.launch`（仿真）**：导航核心，参数加载方式相同但调优值不同。
    * **实车全局规划器**：navfn（Dijkstra 实现）
    * **仿真全局规划器**：global_planner（A\* 实现）
    * **实车局部规划器**：TEB（Timed Elastic Band，动态避障平滑）
    * **仿真局部规划器**：DWA（Dynamic Window Approach）
  * `launch/amcl_sim.launch`：AMCL 定位，关键参数——
    * `odom_model_type: omni-corrected`（全向轮专用）
    * `min/max_particles: 500/2000`
    * `laser_max_range: 12.0 m`
    * `update_min_d: 0.25 m`（位置更新阈值）
  * `param/sim/`、`param/`：代价地图、规划器参数，按实车/仿真分目录管理。
  * `maps/`：**22 个地图版本**（`.pgm` + `.yaml`），包含展馆、走廊等多种场景，含 Cartographer 的 `.pbstream` 格式。

#### 7.3. `src/bobac3_slam`: 建图模块

* **职责**：提供三种 SLAM 方案的统一接口。
* **组件**：
  * `bobac3_slam.launch` / `bobac3_slam_sim.launch`：总入口，通过 `slam_methods` 参数切换算法。
  * `bobac3_gmapping.launch`：GMapping 建图。实车版订阅融合话题 `fuse_scan`，仿真版订阅 `scan`。
  * `bobac3_cartographer.launch`：Cartographer 建图，配置文件 `bobac3_2d.lua`（实车）/ `bobac3_2d_sim.lua`（仿真），含 `cartographer_occupancy_grid_node` 栅格转换。
  * `bobac3_rtabmap_sim.launch`：RTabMap RGB-D 建图，含 `rgbd_sync` 同步节点与点云生成。
  * `rtabmap_saver.launch`：地图保存。

#### 7.4. `src/rei_robot_base`: 底盘驱动模块

* **职责**：bobac3 底盘的串口通信、运动学解算与状态反馈。
* **组件**：
  * `base_node.cpp`：主节点，串口收发、状态轮询。
  * `kinematics.cpp`：麦克纳姆轮运动学正逆解（`/cmd_vel` → 四轮转速）。
  * **ROS API**：
    * 订阅：`/cmd_vel`（geometry_msgs/Twist）
    * 发布：`/car_data`（CarData：电池电压、电机电流）、`/bumper_cliff`（BumperCliff：防撞防跌）
    * 服务：`/set_io`（SetIO：IO 口控制）
  * `launch/bobac3_base.launch`：实车底盘启动入口。

#### 7.5. `src/rei_lidar_fuse`: 双雷达融合模块

* **职责**：将前后两路激光雷达的点云融合为单一 360° 扫描数据。
* **组件**：
  * `lidar_fuse_nodelet.cpp`：nodelet 实现，零拷贝降低延迟。
  * `launch/lidar_fuse_nodelet.launch`：加载 nodelet manager 与融合节点。
  * 输出话题：`/fuse_scan`（sensor_msgs/LaserScan），供 AMCL 与 move_base 消费。

#### 7.6. `src/robot_audio`: 语音交互模块

* **职责**：基于科大讯飞 AIUI 的完整语音链路。
* **组件**：
  * `voice_awake`：语音唤醒（离线）。
  * `voice_collect`：音频采集与 VAD 端点检测。
  * `robot_aiui.cpp`：AIUI SDK 封装，在线识别与语义理解。
  * `voice_nav.cpp`：语音导航指令解析（「去上海馆」→ 导航目标）。
  * **服务接口**：`/robot_iat`（语音转文字）、`/robot_tts`（文字转语音）、`/robot_semanteme`（语义理解）、`/awake`（唤醒）等 8 个。
  * `AIUI/`：讯飞 SDK、授权文件与声学模型（**需有效 appid**）。

#### 7.7. `src/multi_task`: 任务调度模块

* **职责**：导览送餐任务的主控逻辑，串联导航、语音、充电、视觉。
* **组件**：
  * `meals_delivered.py`：展馆导览主控（Python 2）。
  * `multipoint_nav.py`：多点循环导航。
  * `param/navigation_point.yaml`：**导航点坐标表**，包含出发点、充电桩、上海馆、深圳馆、吉林馆、广州馆、北京馆等，每项为 `position` + `orientation` 四元数。
  * `param/task_sequence.yaml`：**任务序列定义**，支持的动作类型：

| 动作 | 说明 | 示例 |
|---|---|---|
| `nav_goal` | 导航到指定点 | `point 出发点` |
| `voice_tts` | TTS 播报 | `str 上海是中国的经济中心` |
| `play_audio` | 播放音频文件 | `str 自动充电成功.wav` |
| `wait_voice` | 等待语音指令 | `str 参观` |
| `wait_time` | 定时等待 | `val 5` |
| `relative_move` | 相对移动 | `val [0.2,0,0]` |
| `goto_charging` | 触发自动充电 | `val False` |

#### 7.8. `src/improve_rrt_connect`: 改进采样规划模块

* **职责**：研究性算法包，实现改进的 RRT-Connect 路径规划。
* **组件**：
  * `Improve_RRT_Connect.py`：基础改进版（偏置采样 + 剪枝优化）。
  * `social_Improve_RRT_Connect.py`：社交感知版（避让人群密集区）。
  * `voice_Improve_RRT_Connect.py`：与语音联动的演示版。

#### 7.9. `src/nav_goal`: 语音导览主控模块

* **职责**：**本项目的主演示入口**。用 C++ 实现语音触发 + 多点导航的完整导览逻辑。
* **组件**：
  * `src/tiaoshi.cpp`：导览主控节点（`tiaoshi_node`）。
    * 调用 `robot_audio` 的 `/voice_collect`、`/voice_dictation` 服务做语音采集与识别
    * 调用 `/voice_tts` 服务做 TTS，或用 `system("play xxx.wav")` 播放预录音频
    * 用 `actionlib::SimpleActionClient<MoveBaseAction>` 连接 `move_base`
    * 主循环中对识别文本做 `text.find()` 关键词匹配，分发到不同导航分支
  * `src/tiaoshi(复件).cpp`：历史备份版本，逻辑基本相同
  * `launch/tiaoshi.launch`：**一键导览启动文件**（Gazebo + 导航 + 语音 + RViz）
  * `launch/nav.launch`：多点导航版本（`nav_multigoal_pro_node`，默认地图 `reicom`）
  * `config/params.yaml`：**导航点坐标表**（北京/上海/广州/深圳/吉林馆 + 充电桩 + 出发点）
  * `config/clearer_params.yaml`：速度归零清理器参数（`zero_vel_threshold: 8.0`、`cool_down: 6s`）
  * `voices/`：**13 个预录讲解音频**（`去XX馆.wav`、`XX是中国的...wav`、`这就带你参观.wav`）
* **语音指令集**：

| 关键词 | 分支行为 |
|---|---|
| `一圈` | 完整导览：北京 → 上海 → 广州 → 深圳 → 吉林 → 返回起点，每站播放讲解 |
| `北京` / `上海` / `广州` / `深圳` / `吉林` | 单点导航 + 该馆讲解 |
| `元宝` | 应答「什么事呀」 |

* **依赖**：`robot_audio`（语音服务）、`move_base`（导航 Action）、`jie_ware`（代价地图清理）、`bobac3_navigation`（地图与参数）。

#### 7.10. 其他功能包

| 包 | 职责 | 关键接口 |
|---|---|---|
| `auto_charging` | 红外对桩自动充电 | srv: `/set_charge` |
| `relative_move` | 以机器人坐标系相对移动 | srv: `/set_relative_move` |
| `ar_pose` | AR 标签位姿估计 | msg: `ArPose`；8 个 launch 覆盖三种相机位 |
| `face_rec` | 人脸检测识别 | srv: `/recognition_results` |
| `deep_camera_scan_filter` | 深度相机转伪激光 + 滤波 | `scan_filter.launch` |
| `jie_ware` | 代价地图清理 + 激光定位 | `costmap_cleaner` 节点 |
| `my_planner` | 自研全局规划器 | nav_core 插件 |
| `pid_lib` | PID 控制库 | 纯 C++ 库 |
| `global_planner` | A\*/Dijkstra 实现 | ROS 官方定制版 |
| `amcl` | 蒙特卡洛定位 | ROS 官方定制版 |
| `robot_joy` | 手柄遥控 | `/joy` → `/cmd_vel` |
| `secondary_localization` | 二次精定位 | `track.cpp` |
| `bobac3_audio` | 语音导航封装 | `voice_nav.launch` |
| `roborts_msgs` | 比赛风格消息定义 | GameStatus/Relocate 等 |

---

## 8. 系统工作流程详解

以**语音导览任务**（`roslaunch nav_goal tiaoshi.launch`）为例：

1. **启动与编排**：`tiaoshi.launch` 拉起 Gazebo、map_server、move_base、AMCL、RViz，以及 `tiaoshi_node`（主控）、`voice_aiui_node`（识别）、`voice_collect_node`（采集）和两个 `costmap_cleaner`。
2. **环境加载**：Gazebo 加载 `yes_my_world.world` 展馆场景并 spawn 机器人模型。
3. **地图发布**：`map_server` 加载 `maps/demonew3.yaml`，向全系统发布 `/map`。
4. **传感器就绪**：Gazebo 插件发布 `/scan`、`/odom`、`/tf`；实车时 `rei_lidar_fuse` 将双雷达融合为 `/fuse_scan`。
5. **机器人定位**：AMCL 融合地图与激光数据，收敛粒子群，发布 `map → odom` TF。
6. **等待语音**：`tiaoshi_node` 进入主循环，`voice_collect_node` 持续采集麦克风音频，`voice_aiui_node` 转成文字。
7. **关键词匹配**：`tiaoshi.cpp` 对识别文本做 `text.find()` 匹配。你说「带我参观**一圈**」→ 命中 `一圈` 分支。
8. **语音应答**：`system("play .../这就带你参观.wav")` 播放应答语音。
9. **首个目标**：通过 actionlib 向 `move_base` 发送 `goal1`（北京馆坐标，来自 `config/params.yaml`）。
10. **全局规划**：move_base 基于 `/map` 用 global_planner（A\*）计算最优路径。
11. **局部控制**：DWA 局部规划器根据实时代价地图动态避障，输出 `/cmd_vel`。
12. **物理执行**：Gazebo 插件解算麦轮速度，驱动机器人移动。
13. **到达讲解**：`move_base` 返回 SUCCEEDED，`tiaoshi_node` 播放「北京是中国的首都政治中心.wav」。
14. **循环下一站**：依次执行 上海馆 → 广州馆 → 深圳馆 → 吉林馆，每站都是「导航 → 阻塞等待成功 → 播放讲解」。
15. **返回起点**：发送 `goal6`（出发点），到达后 `task1=false`，导览流程结束，回到等待语音状态。
16. **期间清理**：`costmap_cleaner` 持续清除代价地图中的幽灵障碍，`velocity_clearer` 在机器人静止超时后清理局部代价地图，避免误判。

---

## 9. 已知问题与注意事项

* **Python 2.7 专属**：`meals_delivered.py` 等脚本使用 Python 2 语法，**不能**在 ROS Noetic（Python 3）下运行。
* **硬编码路径**：源码中有 114 处 `/home/bobac3/...` 绝对路径（world 的 STL 引用、语音文件路径、地图 yaml），迁移时需批量替换。`bobac3_navigation/maps/rtab_map.yaml` 指向已不存在的旧路径 `~/ros_workspace/`，使用前需修正。
* **讯飞授权**：`robot_audio/AIUI/` 需要有效 appid 才能在线识别，仓库中的配置仅在原作者环境有效。
* **实车专用包**：`rei_robot_base`、`rei_lidar_fuse`、`auto_charging` 依赖 bobac3 硬件（串口协议、雷达型号、红外对桩），脱离实车无法运行。
* **同名包覆盖**：`amcl`、`global_planner` 与 ROS 官方包同名，`rospack` 解析时工作空间版本优先；如需使用官方版本，请注意 `ROS_PACKAGE_PATH` 顺序。

---

## 10. 免责声明

本仓库为**个人学习用途的源码归档**，采集自本地 VMware 虚拟机。

- 涉及的讯飞 AIUI SDK、ALVAR 库归其各自权利人所有
- 硬件驱动仅供研究与学习，生产使用请自行评估
- 不包含任何商业授权

---

## 附：文档截图

所有配图均为从原虚拟机**实机截取**的真实画面：

| 图片 | 内容 |
|---|---|
| `01-system-overview.jpg` | 系统与 ROS 版本概览 |
| `02-package-list.jpg` | 26 个功能包分类列表 |
| `03-workspace-structure.jpg` | 工作空间目录与磁盘占用 |
| `04-rviz-robot-model.jpg` | RViz 中加载的机器人模型 |
| `05-gazebo-world.jpg` | Gazebo 仿真世界（yes_my_world） |
| `06-navigation-params.jpg` | 导航栈参数构成（TEB/DWA/AMCL） |
| `07-task-chain.jpg` | 送餐导览任务链与任务序列 |
