# PX4 + ROS2 + OpenCV 无人机视觉追踪：从零开始到实习

本章实现一架基于 PX4 飞控 + ROS2 + OpenCV 的四旋翼无人机的视觉目标追踪系统：在 Ubuntu 22.04.5 LTS 上完成从 ROS2 环境搭建、PX4 固件编译、机外通信桥接，到视觉目标检测、机外控制闭环的全流程，并给出从仿真到实习的真机与工程化要点。完整工程代码位于工作空间 `~/ws_px4/src` 下的 `px4_msgs`、`px4_ros_com` 与自建的 `vision_tracking` 功能包。

## 1. 环境配置与工程结构

### 1.1 运行环境

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 22.04.5 LTS |
| ROS 版本 | ROS2 Humble Hawksbill（LTS，支持至 2027） |
| 飞控固件 | PX4-Autopilot v1.14.x |
| 仿真环境 | Gazebo Classic 11（随 PX4 集成） |
| 通信桥 | Micro XRCE-DDS Agent + uXRCE-DDS 客户端 |
| 视觉库 | OpenCV 4.x（Python 3.10 绑定 cv2） |
| 构建工具 | colcon / catkin（ROS2 用 colcon） |
| ROS2 工作空间 | `~/ws_px4` |

> 说明：ROS2 Humble 与 Ubuntu 22.04 是官方长期支持配对；PX4 v1.14 默认使用 Gazebo Classic，与 Humble 的 Gazebo 版本兼容，后续高版本 PX4 迁移到新 Gazebo 时注意环境变化。

### 1.2 工程结构

`vision_tracking` 功能包与 PX4 桥接包共同组成追踪系统，目录结构如下：

```
~/ws_px4/src/
├── px4_msgs/                     # PX4 官方 ROS2 消息定义
│   └── msg/
│       ├── OffboardControlMode.msg   # 机外控制模式
│       ├── TrajectorySetpoint.msg    # 轨迹设定点
│       ├── VehicleCommand.msg        # 模式切换/解锁指令
│       ├── VehicleLocalPosition.msg  # 机体本地位置
│       └── VehicleAttitude.msg       # 姿态四元数
├── px4_ros_com/                  # PX4-ROS2 桥接示例（官方）
│   ├── launch/
│   └── test/                     # offboard 示例
└── vision_tracking/              # 自建视觉追踪功能包
    ├── launch/
    │   └── vision_tracking.launch.py   # 总启动文件
    ├── scripts/
    │   ├── color_tracker.py            # OpenCV 颜色追踪节点
    │   ├── image_pub.py                # 摄像头图像发布节点
    │   └── offboard_control.py         # 机外控制 + 追踪闭环
    ├── package.xml
    └── setup.py
```

## 2. 基础软件安装与编译

### 2.1 ROS2 Humble 安装

在干净的 Ubuntu 22.04.5 LTS 上，先配置软件源与密钥，再安装桌面版完整套件：

```bash
# 1) 配置 apt 软件源
sudo apt update && sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe

# 2) 添加 ROS2 官方仓库
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3) 安装 ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep python3-pip

# 4) 初始化 rosdep 与 bashrc
sudo rosdep init
rosdep update
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

验证安装：

```bash
ros2 --version          # 输出 ros2 <版本号>
ros2 run demo_nodes_cpp talker
```

能看到 `Hello World` 周期性输出即说明 ROS2 通信正常。

### 2.2 PX4 固件编译

PX4 提供一键环境脚本，但国内网络建议先配置代理或使用镜像。以官方仓库方式安装：

```bash
# 1) 克隆 PX4 固件（带子模块）
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot

# 2) 安装依赖（含 Gazebo Classic、仿真相关工具）
bash ./Tools/setup/ubuntu.sh --no-nuttx
# 或仅仿真环境：./Tools/setup/ubuntu_sim.sh

# 3) 更新子模块
git submodule update --init --recursive

# 4) 编译 SITL 仿真固件
make px4_sitl gazebo-classic
```

编译成功后会弹出 Gazebo 空世界窗口，终端显示 PX4 的 `INFO [simulator]` 与 `INFO [commander]` 启动日志，说明 SITL 已就绪：

```
INFO  [px4] Startup script returned successfully
INFO  [mavlink] mode: Normal, data rate: 4000000 B/s on udp port 14540
INFO  [commander] ready for takeoff
```

### 2.3 依赖与通信工具安装

```bash
# 通信桥 Micro XRCE-DDS Agent
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig

# 消息与桥接包
mkdir -p ~/ws_px4/src
cd ~/ws_px4/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```

## 3. 仿真环境与通信桥接

### 3.1 启动 uXRCE-DDS Agent

PX4 与 ROS2 之间通过 uXRCE-DDS（UXRCE 客户端/Agent）桥接，Agent 运行在主机侧，监听 UDP 端口接收 PX4 发来的数据：

```bash
MicroXRCEAgent udp4 -p 8888
```

正常启动输出：

```
[ INFO] [1700000000.000000000] Agent: ... UDP transport ready
```

### 3.2 启动 PX4 SITL 并连接 Agent

PX4 默认已启用 `uxrce_dds_client` 模块，启动仿真后自动连接本机 8888 端口：

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
```

此时 ROS2 端即可看到 PX4 发布的话题。新开终端验证：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_px4/install/setup.bash
ros2 topic list
```

其中应包含以下 PX4 状态话题（`/fmu/out/...` 表示 PX4 → ROS2 方向）：

```
/fmu/out/vehicle_local_position
/fmu/out/vehicle_attitude
/fmu/out/vehicle_odometry
/fmu/out/battery_status
/fmu/out/sensor_combined
```

### 3.3 编译并验证桥接包

```bash
cd ~/ws_px4
colcon build
source install/setup.bash
ros2 topic echo /fmu/out/vehicle_attitude --once
```

能打印出姿态四元数数据，即表示 PX4-ROS2 双向通信打通：

```
---
header:
  stamp:
    sec: 12345
    nanosec: 678901234
timestamp: 1700000000000000
q: [1.0, -0.001, 0.002, 0.001]
```

## 4. 视觉追踪：目标检测

### 4.1 摄像头图像获取

在仿真中，用 Gazebo 摄像头或从仿真话题读取图像。此处以 ROS2 图像话题为标准接口，`image_pub.py` 将图像帧发布到 `/camera/image_raw`：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_pub')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.timer_cb)  # ~30Hz
        # 仿真/本地摄像头：cv2.VideoCapture(0)
        self.cap = cv2.VideoCapture(0)

    def timer_cb(self):
        ret, frame = self.cap.read()
        if ret:
            self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImagePublisher())
    rclpy.shutdown()
```

### 4.2 HSV 颜色阈值分割

视觉追踪通常基于目标颜色（如红色气球）进行分割。OpenCV 默认使用 BGR，需转换到 HSV 空间以更鲁棒地应对光照变化：

```python
#!/usr/bin/env python3
import cv2
import numpy as np

def detect_red_target(frame):
    """返回目标质心 (cx, cy)、面积与包围盒；未检测到返回 None"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 红色在 HSV 色环两端，取两个区间合并
    lower1 = np.array([0, 120, 70])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 120, 70])
    upper2 = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    # 开闭运算去噪
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # 轮廓检测
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    # 取面积最大的轮廓作为目标
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 100:      # 过滤噪声
        return None
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy, cv2.boundingRect(c)
```

### 4.3 质心提取与误差计算

将目标质心与图像中心做差，得到追踪误差（像素域）：

```
error_x = cx - frame_width / 2     # 目标在图像中的横向偏差
error_y = cy - frame_height / 2    # 纵向偏差
area     = 目标包围盒面积          # 用于估算距离
```

误差越大，说明目标偏离画面中心越远；面积越大，说明无人机越接近目标。这些量将作为后续控制器的输入。

## 5. 无人机机外控制

### 5.1 机外控制原理

PX4 的 Offboard 模式允许外部程序直接下发设定点（位置/速度/姿态），是视觉追踪的常规控制通道。通过四个话题与 PX4 交互：

| 话题 | 消息类型 | 作用 |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | px4_msgs/OffboardControlMode | 声明当前控制类型（位置/速度/姿态） |
| `/fmu/in/trajectory_setpoint` | px4_msgs/TrajectorySetpoint | 下发目标位置/速度/偏航 |
| `/fmu/in/vehicle_command` | px4_msgs/VehicleCommand | 模式切换、解锁/上锁 |
| `/fmu/out/vehicle_local_position` | px4_msgs/VehicleLocalPosition | 读取机体当前位置（闭环反馈） |

**关键约束**：机外控制必须持续周期性下发设定点（典型 10~50 Hz），且必须在解锁前先切换到 Offboard 模式并保持，否则 PX4 会在超时后退出机外模式。

### 5.2 解锁与起飞流程

正确流程为：**先发送 Offboard 模式切换指令 → 持续下发设定点 → 解锁（Arm）**。核心代码：

```python
#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        # 10Hz 控制循环
        self.timer = self.create_timer(0.1, self.control_cb)
        self.t0 = time.time()
        self.armed = False

    def control_cb(self):
        # 1) 持续声明"速度控制"模式
        off = OffboardControlMode()
        off.timestamp = int(time.time() * 1e6)
        off.position = False
        off.velocity = True
        off.acceleration = False
        off.attitude = False
        off.body_rate = False
        off.thrust = False
        self.offboard_pub.publish(off)

        # 2) 下发速度设定点（悬停：vx=vy=vz=0）
        sp = TrajectorySetpoint()
        sp.timestamp = int(time.time() * 1e6)
        sp.position = [0.0, 0.0, -2.0]   # 目标高度 2m（NED，负为上升）
        sp.velocity = [0.0, 0.0, 0.0]
        sp.yaw = 0.0
        self.traj_pub.publish(sp)

        # 3) 起飞后约 2 秒切换 Offboard 并解锁
        if time.time() - self.t0 > 2.0 and not self.armed:
            self.mode_offboard()
            self.arm_disarm(True)
            self.armed = True

    def mode_offboard(self):
        cmd = VehicleCommand()
        cmd.timestamp = int(time.time() * 1e6)
        cmd.command = 176    # MAV_CMD_DO_SET_MODE
        cmd.param2 = 6       # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        self.cmd_pub.publish(cmd)

    def arm_disarm(self, arm):
        cmd = VehicleCommand()
        cmd.timestamp = int(time.time() * 1e6)
        cmd.command = 400    # MAV_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0 if arm else 0.0
        self.cmd_pub.publish(cmd)
```

> 注意：PX4 使用 NED（北东地）坐标系，z 轴向下，因此“起飞”即目标 z 为负值；偏航角 yaw 逆时针为正。

## 6. 视觉追踪闭环控制

### 6.1 追踪控制策略

将第 4 节的像素误差映射为第 5 节的速度指令，形成闭环：

| 误差 | 控制量 | 说明 |
| --- | --- | --- |
| `error_x`（目标偏左/右） | 机体横向速度 `vy` | 让目标回到画面水平中心 |
| `error_y`（目标偏上/下） | 机体前向速度 `vx` + 高度 `vz` | 保持目标高度中心 |
| 目标面积过大/过小 | 前向速度 `vx` 进退 | 保持期望距离（面积目标值） |

采用 P（比例）控制即可获得可用的追踪效果，进阶可加 I/D 项：

```python
def velocity_from_error(err_x, err_y, area, Kp_xy=0.15, area_target=8000, Kp_d=0.02):
    # 横向/纵向偏航跟随：x 轴（机体前向）速度由偏航误差驱动
    vy = -Kp_xy * err_x          # 目标在右 => 向右飞（y 正）
    # 高度：err_y 控制 vz（NED，z 向下）
    vz = -Kp_xy * err_y
    # 前后：面积误差控制前向速度
    vx = Kp_d * (area_target - area)
    return vx, vy, vz
```

### 6.2 完整追踪节点

`offboard_control.py` 订阅视觉结果（自定义 `TargetInfo.msg`，或直接订阅质心话题），计算误差并下发速度设定点：

```python
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VisionTracker(OffboardControl):
    def __init__(self):
        super().__init__()
        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.img_cb, 10)
        self.bridge = CvBridge()
        self.target = None

    def img_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        res = detect_red_target(frame)
        if res:
            cx, cy, rect = res
            h, w = frame.shape[:2]
            err_x, err_y = cx - w / 2, cy - h / 2
            area = rect[2] * rect[3]
            self.target = (err_x, err_y, area)

    def control_cb(self):
        if self.target:
            vx, vy, vz = velocity_from_error(*self.target)
            # 复用父类速度设定点下发逻辑
            self.publish_setpoint(vx, vy, vz)
```

### 6.3 启动文件

用 `launch.py` 一次性拉起视觉与控制器节点：

```python
# vision_tracking/launch/vision_tracking.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='vision_tracking', executable='image_pub',  name='image_pub'),
        Node(package='vision_tracking', executable='offboard_control', name='offboard_control'),
    ])
```

## 7. 仿真验证

### 7.1 启动完整系统

分三个终端依次启动：

```bash
# 终端1：通信桥
MicroXRCEAgent udp4 -p 8888

# 终端2：PX4 SITL（Gazebo 中放置红色目标物体）
cd ~/PX4-Autopilot && make px4_sitl gazebo-classic

# 终端3：视觉追踪
source /opt/ros/humble/setup.bash
source ~/ws_px4/install/setup.bash
ros2 launch vision_tracking vision_tracking.launch.py
```

### 7.2 观察追踪效果

- 查看图像与检测框：`rqt_image_view /camera/image_raw`
- 查看 PX4 位置反馈：`ros2 topic echo /fmu/out/vehicle_local_position`
- 观察无人机是否朝目标方向移动、悬停时是否稳定。

### 7.3 调试技巧

若追踪发散，按顺序检查：

1. 视觉是否稳定检出目标（打印 `target` 是否频繁为 None）；
2. 控制模式是否处于 Offboard（`ros2 topic echo /fmu/out/vehicle_status` 查看 `nav_state`）；
3. PID 增益是否过大（先降 `Kp` 到 0.05 量级再逐步增大）。

## 8. 从仿真到实习

### 8.1 真机迁移要点

| 项目 | 仿真 | 真机 |
| --- | --- | --- |
| 视觉计算 | 本机 | Jetson Nano / Raspberry Pi 4 等机载电脑 |
| 通信桥 | 本机运行 Agent | 机载电脑跑 Agent，飞控经串口/网口连接 |
| 飞行安全 | 无风险 | 必须加 GPS 返航、围栏、急停开关 |
| 硬件 | 无 | 需避障、桨叶护罩、备用电池 |

### 8.2 代码工程化（实习加分项）

- 使用 ROS2 **参数（Parameter）** 替代硬编码的 `Kp`、颜色阈值；
- 使用 **Launch 文件** 组织多节点与参数配置；
- 编写 **unit test** 验证颜色阈值与误差计算；
- 使用 Git 管理版本、写好 README 与参数说明。

### 8.3 面试与实习准备

视觉追踪是典型的“感知 + 控制”综合项目，实习面试可重点准备：

- 讲清楚**机外控制的时序约束**（先模式后解锁、持续下发）；
- 能现场画出**感知→控制→PX4 的 ROS2 话题/节点图**；
- 说明 OpenCV 颜色分割的**局限性与改进**（光照、动态阈值、换用深度学习检测）；
- 展示仿真 → 真机的**安全意识与降级策略**（丢目标时悬停）。

## 9. 常见问题

**Q1：`ros2 topic list` 看不到 `/fmu/out/...` 话题？**
多半是 MicroXRCEAgent 未启动或 PX4 未连接。先 `MicroXRCEAgent udp4 -p 8888`，再启动 PX4，确认 Agent 终端出现 `[Client connected]` 提示。

**Q2：无人机起飞后立刻失控/翻机？**
Offboard 模式未在解锁前完成切换，或设定点下发频率过低。确保先切 Offboard 再 Arm，且控制循环 ≥10Hz，并在解锁后持续下发。

**Q3：启动 PX4 编译报缺少依赖？**
未跑完 `./Tools/setup/ubuntu.sh`。该脚本会安装 Gazebo 与所有依赖；国内网络失败时改用国内镜像或手动逐个安装 Gazebo Classic。

**Q4：视觉检测框频繁闪烁/追踪抖动？**
颜色阈值过宽或噪声未滤除。增大最小轮廓面积阈值、加强形态学操作，或对质心做一阶低通滤波（`cx = 0.7*cx_old + 0.3*cx_new`）。

**Q5：`colcon build` 提示 `px4_msgs` 未找到？**
`px4_msgs` 必须与功能包在同一工作空间 `src` 下，且先编译一次。确认 `source install/setup.bash` 已执行。

**Q6：Gazebo 弹窗卡死或模型加载慢？**
先 `killall gzserver gzclient` 清掉残留实例，重新启动；并确保主机内存 ≥4GB、开启硬件加速。

**Q7：真机上视觉延迟高？**
OpenCV 默认在 CPU 跑，可改用 `cv2` 的 `dnn` 后处理或换 Jetson 的 CUDA 加速；同时把图像分辨率降到 640×480、检测帧率降到 20Hz。

---

> 本文档对应“PX4 + ROS2 + OpenCV 无人机视觉追踪”从环境搭建到实习准备的全流程，可作为教程章节或实习培训材料使用。
