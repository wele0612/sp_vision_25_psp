# `io::` API 文档

> 本文档由代码分析自动生成，涵盖 `io/` 目录下所有公开类、函数、枚举和结构体。
> `io::` 是 sp_vision 框架的**硬件抽象层（I/O Layer）**，负责与相机、下位机（C板/云台）、CAN 总线、串口、ROS2 等硬件/通信接口交互，为上层 `tasks::` 提供统一的数据输入输出。

---

## 目录

1. [相机抽象 (`camera.hpp`)](#相机抽象-camerahpp)
2. [下位机通信 (`cboard.hpp`)](#下位机通信-cboardhpp)
3. [指令结构体 (`command.hpp`)](#指令结构体-commandhpp)
4. [SocketCAN (`socketcan.hpp`)](#socketcan-socketcanhpp)
5. [达妙 IMU (`dm_imu/dm_imu.hpp`)](#达妙-imu-dm_imudm_imuhpp)
6. [云台串口通信 (`gimbal/gimbal.hpp`)](#云台串口通信-gimbalgimbalhpp)
7. [海康工业相机 (`hikrobot/hikrobot.hpp`)](#海康工业相机-hikrobothikrobothpp)
8. [迈德威视相机 (`mindvision/mindvision.hpp`)](#迈德威视相机-mindvisionmindvisionhpp)
9. [ROS2 发布节点 (`ros2/publish2nav.hpp`)](#ros2-发布节点-ros2publish2navhpp)
10. [ROS2 订阅节点 (`ros2/subscribe2nav.hpp`)](#ros2-订阅节点-ros2subscribe2navhpp)
11. [ROS2 高层封装 (`ros2/ros2.hpp`)](#ros2-高层封装-ros2ros2hpp)
12. [USB 摄像头 (`usbcamera/usbcamera.hpp`)](#usb-摄像头-usbcamerausbcamerahpp)

---

## 相机抽象 (`camera.hpp`)

**命名空间：** `io`

### `class CameraBase`（抽象基类）

所有相机实现的统一接口。

| 成员 | 签名 | 说明 |
|------|------|------|
| 析构函数 | `virtual ~CameraBase() = default` | — |
| `read` | `virtual void read(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp) = 0` | 读取一帧图像及其时间戳。 |

### `class Camera`

多态相机工厂/包装器。根据 YAML 配置中的 `camera_name` 自动实例化 `MindVision` 或 `HikRobot` 后端。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Camera(const std::string &config_path)` | 加载 YAML，读取 `camera_name`、`exposure_ms` 及相机专属参数（`gamma`、`gain`、`vid_pid`），构造对应后端。 |
| `read` | `void read(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp)` | 委托给内部 `camera_` 实例。 |

**私有成员：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `camera_` | `std::unique_ptr<CameraBase>` | 多态相机后端。 |

---

## 下位机通信 (`cboard.hpp`)

**命名空间：** `io`

### 枚举

| 枚举 | 值 | 说明 |
|------|-----|------|
| `Mode` | `idle`, `auto_aim`, `small_buff`, `big_buff`, `outpost` | C板工作模式。 |
| `ShootMode` | `left_shoot`, `right_shoot`, `both_shoot` | 射击模式（哨兵专用）。 |

字符串查找表：`MODES`、`SHOOT_MODES`。

### `class CBoard`

通过 **SocketCAN** 与嵌入式 C板（STM32F407）通信：接收 IMU 四元数、弹速、模式信息；发送控制指令帧。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `CBoard(const std::string &config_path)` | 读取 YAML 中的 CAN 接口名与 CAN ID，初始化 `SocketCAN`，等待前两个 IMU 数据点。 |
| `imu_at` | `Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp)` | 在 IMU 队列中进行球面线性插值（SLERP），获取指定时间戳的姿态四元数。 |
| `send` | `void send(Command command) const` | 将 `Command` 打包为 CAN 帧并发送。 |

**公开成员变量：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `bullet_speed` | `double` | 最新弹速（m/s）。 |
| `mode` | `Mode` | 当前工作模式。 |
| `shoot_mode` | `ShootMode` | 当前射击模式。 |
| `ft_angle` | `double` | 无人机专用飞行角度。 |

**私有结构体 `IMUData`：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `q` | `Eigen::Quaterniond` | 姿态四元数。 |
| `timestamp` | `std::chrono::steady_clock::time_point` | 时间戳。 |

**私有成员：** `tools::ThreadSafeQueue<IMUData> queue_`、`SocketCAN can_`、`IMUData data_ahead_, data_behind_`、各 CAN ID。

---

## 指令结构体 (`command.hpp`)

**命名空间：** `io`

### `struct Command`

视觉系统发给下位机的控制指令。

| 成员 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `control` | `bool` | — | 是否控制云台。 |
| `shoot` | `bool` | — | 是否开火。 |
| `yaw` | `double` | — | Yaw 指令值。 |
| `pitch` | `double` | — | Pitch 指令值。 |
| `horizon_distance` | `double` | `0` | 无人机专用水平距离。 |

---

## SocketCAN (`socketcan.hpp`)

**命名空间：** `io`

### `class SocketCAN`

Linux SocketCAN 封装。使用 `epoll` 异步接收 CAN 帧，并带有**自动重连守护线程**。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `SocketCAN(const std::string &interface, std::function<void(const can_frame &frame)> rx_handler)` | 打开 CAN socket、绑定接口、创建 epoll、启动读线程与守护线程。 |
| 析构函数 | `~SocketCAN()` | 发送退出信号、join 线程、关闭 socket。 |
| `write` | `void write(can_frame *frame) const` | 向 CAN 总线写入一帧。 |

**私有成员：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `interface_` | `std::string` | CAN 接口名（如 `can0`）。 |
| `socket_fd_`, `epoll_fd_` | `int` | 文件描述符。 |
| `quit_`, `ok_` | `bool` | 线程控制标志。 |
| `read_thread_`, `daemon_thread_` | `std::thread` | 工作线程。 |
| `rx_handler_` | `std::function<void(const can_frame&)>` | 用户提供的接收回调。 |

**私有方法：** `open()`、`try_open()`、`read()`、`close()`。

---

## 达妙 IMU (`dm_imu/dm_imu.hpp`)

**命名空间：** `io`

### `struct IMU_Receive_Frame`（`packed`）

达妙 IMU 的原始串口帧结构，包含加速度计、陀螺仪、欧拉角三个子帧，各带子帧头、从机 ID、寄存器地址、CRC 校验和帧尾。

### `typedef struct IMU_Data`

解析后的浮点 IMU 数据。

| 成员 | 类型 | 说明 |
|------|------|------|
| `accx`, `accy`, `accz` | `float` | 加速度。 |
| `gyrox`, `gyroy`, `gyroz` | `float` | 角速度。 |
| `roll`, `pitch`, `yaw` | `float` | 欧拉角。 |

### `class DM_IMU`

达妙 IMU 串口驱动。通过 UART 读取原始帧、解析数据、转四元数，并支持时间戳插值查询。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `DM_IMU()` | 打开 `/dev/ttyACM0`（921600 波特率），启动接收线程，等待前两个四元数样本。 |
| 析构函数 | `~DM_IMU()` | 发送停止信号、join 线程、关闭串口。 |
| `imu_at` | `Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp)` | SLERP 插值获取指定时刻的姿态。 |

**私有结构体 `IMUData`：** 同 `CBoard`（四元数 + 时间戳）。

**私有方法：** `init_serial()`、`get_imu_data_thread()`（阻塞读循环，校验 CRC16，欧拉角转四元数后入队）。

---

## 云台串口通信 (`gimbal/gimbal.hpp`)

**命名空间：** `io`

### `struct GimbalToVision`（`packed`）

云台/下位机 → 视觉系统的协议帧。

| 成员 | 类型 | 说明 |
|------|------|------|
| `head` | `uint8_t[2]` | 帧头 `{'S', 'P'}`。 |
| `mode` | `uint8_t` | `0=idle`, `1=auto_aim`, `2=small_buff`, `3=big_buff`。 |
| `q` | `float[4]` | 四元数 `(w, x, y, z)`。 |
| `yaw`, `yaw_vel`, `pitch`, `pitch_vel`, `bullet_speed` | `float` | 云台状态与弹速。 |
| `bullet_count` | `uint16_t` | 子弹计数。 |
| `crc16` | `uint16_t` | CRC16 校验。 |

### `struct VisionToGimbal`（`packed`）

视觉系统 → 云台/下位机的协议帧。

| 成员 | 类型 | 说明 |
|------|------|------|
| `head` | `uint8_t[2]` | 帧头 `{'S', 'P'}`。 |
| `mode` | `uint8_t` | `0=不控制`, `1=控制但不射击`, `2=控制并射击`。 |
| `yaw`, `yaw_vel`, `yaw_acc` | `float` | Yaw 位置、速度、加速度。 |
| `pitch`, `pitch_vel`, `pitch_acc` | `float` | Pitch 位置、速度、加速度。 |
| `crc16` | `uint16_t` | CRC16 校验。 |

### `enum class GimbalMode`

`IDLE`, `AUTO_AIM`, `SMALL_BUFF`, `BIG_BUFF`

### `struct GimbalState`

| 成员 | 类型 | 说明 |
|------|------|------|
| `yaw`, `yaw_vel`, `pitch`, `pitch_vel`, `bullet_speed` | `float` | — |
| `bullet_count` | `uint16_t` | — |

### `class Gimbal`

与云台/嵌入式系统的串口通信接口。接收状态与四元数，发送控制指令（带 CRC16）。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Gimbal(const std::string &config_path)` | 读取 YAML 中的 `com_port`，打开串口，启动读线程，等待第一个四元数。 |
| 析构函数 | `~Gimbal()` | 发送退出信号、join 线程、关闭串口。 |
| `mode` | `GimbalMode mode() const` | 线程安全读取当前模式。 |
| `state` | `GimbalState state() const` | 线程安全读取当前状态。 |
| `str` | `std::string str(GimbalMode mode) const` | 枚举转字符串。 |
| `q` | `Eigen::Quaterniond q(std::chrono::steady_clock::time_point t)` | SLERP 插值四元数队列获取指定时刻姿态。 |
| `send` | `void send(io::VisionToGimbal VisionToGimbal)` | 发送已填充的 `VisionToGimbal` 帧。 |
| `send` | `void send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel, float pitch_acc)` | 便捷重载：自动打包并发送。 |

**私有方法：** `read(uint8_t*, size_t)`、`read_thread()`（主接收循环：校验头、CRC16、更新状态/模式、四元数入队）、`reconnect()`（最多重连 10 次，间隔 1 秒）。

---

## 海康工业相机 (`hikrobot/hikrobot.hpp`)

**命名空间：** `io`

### `class HikRobot` : `public CameraBase`

海康机器人（Hikvision）工业 USB 相机驱动，基于 MVS SDK（`MvCameraControl.h`）。带有守护线程：捕获异常时自动重启，并支持 USB 设备复位。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `HikRobot(double exposure_ms, double gain, const std::string &vid_pid)` | 设置曝光/增益，解析 VID/PID，初始化 libusb，启动守护线程。 |
| 析构函数 | `~HikRobot()` | 通知守护线程退出并 join。 |
| `read` | `void read(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp) override` | 从线程安全队列弹出下一帧。 |

**私有结构体 `CameraData`：** `cv::Mat img` + `std::chrono::steady_clock::time_point timestamp`。

**私有方法：**

- `capture_start()`：枚举设备、创建设备句柄、设置曝光/增益/白平衡/帧率、开始取流、启动捕获线程（Bayer 转 RGB 后入队）。
- `capture_stop()`：停止取流、关闭设备、销毁句柄。
- `set_float_value(name, value)` / `set_enum_value(name, value)`：MVS SDK 参数设置。
- `reset_usb()`：通过 libusb 复位相机 USB 设备。

---

## 迈德威视相机 (`mindvision/mindvision.hpp`)

**命名空间：** `io`

### `class MindVision` : `public CameraBase`

迈德威视（MindVision）USB 相机驱动，基于 `CameraApi.h`。架构与 `HikRobot` 类似，同样具备守护线程自动恢复与 USB 复位能力。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `MindVision(double exposure_ms, double gamma, const std::string &vid_pid)` | 设置参数，解析 VID/PID，初始化 libusb，尝试首次打开，启动守护线程。 |
| 析构函数 | `~MindVision()` | 通知退出、join 线程、关闭相机。 |
| `read` | `void read(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp) override` | 从队列弹出下一帧。 |

**私有方法：**

- `open()`：初始化 SDK、枚举设备、打开相机、设置曝光/gamma/BGR 格式/连续模式、启动捕获线程（`CameraGetImageBuffer` 读取缓冲区、处理后入队）。
- `try_open()` / `close()`：异常安全包装与关闭。
- `reset_usb()`：libusb 设备复位。

---

## ROS2 发布节点 (`ros2/publish2nav.hpp`)

**命名空间：** `io`

### `class Publish2Nav` : `public rclcpp::Node`

ROS2 节点，将自瞄目标位置以逗号分隔字符串形式发布到 `auto_aim_target_pos` 话题，供导航模块使用。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Publish2Nav()` | 初始化节点名 `"auto_aim_target_pos_publisher"`，创建发布者。 |
| 析构函数 | `~Publish2Nav()` | 记录关闭日志。 |
| `start` | `void start()` | 调用 `rclcpp::spin()`（阻塞）。 |
| `send_data` | `void send_data(const Eigen::Vector4d &data)` | 将 `x,y,z,w` 格式化为逗号分隔字符串并发布。 |

**私有成员：** `rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_`

---

## ROS2 订阅节点 (`ros2/subscribe2nav.hpp`)

**命名空间：** `io`

### `class Subscribe2Nav` : `public rclcpp::Node`

ROS2 节点，订阅 `enemy_status` 与 `autoaim_target` 话题，将最新消息存入线程安全队列并提供访问接口。若 1500ms 无新消息则自动清空队列。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Subscribe2Nav()` | 初始化节点名 `"nav_subscriber"`，创建两个订阅。 |
| 析构函数 | `~Subscribe2Nav()` | 记录关闭日志。 |
| `start` | `void start()` | 调用 `rclcpp::spin()`（阻塞）。 |
| `subscribe_enemy_status` | `std::vector<int8_t> subscribe_enemy_status()` | 返回最新 `EnemyStatusMsg` 中的 `invincible_enemy_ids`；队列为空时返回空向量。 |
| `subscribe_autoaim_target` | `std::vector<int8_t> subscribe_autoaim_target()` | 返回最新 `AutoaimTargetMsg` 中的 `target_ids`；队列为空时返回空向量。 |

**私有回调：** `enemy_status_callback(...)`、`autoaim_target_callback(...)`。

**私有成员：** 消息计数器、不活动定时器、订阅者、`tools::ThreadSafeQueue<...>` 队列。

---

## ROS2 高层封装 (`ros2/ros2.hpp`)

**命名空间：** `io`

### `class ROS2`

ROS2 高层封装：初始化 `rclcpp`，持有 `Publish2Nav` 与 `Subscribe2Nav`，分别在独立 spin 线程中运行。同时提供通用发布者工厂模板。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `ROS2()` | 调用 `rclcpp::init()`，创建发布/订阅节点，启动 spin 线程。 |
| 析构函数 | `~ROS2()` | 关闭 `rclcpp`，join 线程。 |
| `publish` | `void publish(const Eigen::Vector4d &target_pos)` | 委托给 `publish2nav_->send_data()`。 |
| `subscribe_enemy_status` | `std::vector<int8_t> subscribe_enemy_status()` | 委托给 `subscribe2nav_`。 |
| `subscribe_autoaim_target` | `std::vector<int8_t> subscribe_autoaim_target()` | 委托给 `subscribe2nav_`。 |
| `create_publisher` | `template <typename T> std::shared_ptr<rclcpp::Publisher<T>> create_publisher(const std::string &node_name, const std::string &topic_name, size_t queue_size)` | 工厂：创建节点、发布者，并分离一个 spin 线程。 |

---

## USB 摄像头 (`usbcamera/usbcamera.hpp`)

**命名空间：** `io`

### `class USBCamera`

通用 V4L2 USB 摄像头驱动，基于 OpenCV `VideoCapture`。支持直接读取与队列读取两种接口，并带有自动重连守护线程。通过 `CAP_PROP_SHARPNESS` 区分左右相机。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `USBCamera(const std::string &open_name, const std::string &config_path)` | 加载 YAML（分辨率、曝光、FPS、gamma、gain），尝试打开设备，启动守护线程。 |
| 析构函数 | `~USBCamera()` | 发送退出信号、关闭相机、join 线程。 |
| `read` | `cv::Mat read()` | **直接读取**：加锁后从 `cap_` 读取；相机关闭时返回空 `cv::Mat`。 |
| `read` | `void read(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp)` | **队列读取**：从捕获线程填充的线程安全队列弹出帧。 |

**公开成员变量：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `device_name` | `std::string` | `sharpness==2` 时为 `"left"`，`sharpness==3` 时为 `"right"`。 |

**私有方法：**

- `open()`：打开 `/dev/<open_name_>`，设置 MJPEG 编码、FPS、自动曝光、gamma、gain、分辨率、曝光值；根据 sharpness 确定 `device_name`；启动捕获线程。
- `try_open()` / `close()`：异常安全包装与释放。

---

*文档结束。*
