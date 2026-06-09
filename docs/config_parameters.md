# 配置文件参数说明

本文档详细说明 `configs/` 目录下 YAML 配置文件中的所有可用参数。项目采用**单一配置文件架构**：每个机器人/场景对应一个 YAML 文件，启动时通过命令行传入，各子系统从中读取各自需要的参数。

> **单位约定**：所有物理量参数的单位均在配置文件中以注释形式标注，代码读取时会在必要时进行单位转换（例如角度从 degree 转换为 rad）。本文档中标注的"配置单位"指 YAML 中应填写的数值单位，"代码内部单位"指程序实际使用的单位。

---

## 目录

1. [通用/团队参数](#1-通用团队参数)
2. [神经网络检测参数](#2-神经网络检测参数)
3. [ROI 参数](#3-roi-参数)
4. [USB 相机参数](#4-usb-相机参数)
5. [工业相机参数](#5-工业相机参数)
6. [传统视觉检测参数](#6-传统视觉检测参数)
7. [Tracker（目标跟踪）参数](#7-tracker目标跟踪参数)
8. [Aimer（瞄准）参数](#8-aimer瞄准参数)
9. [Shooter（射击决策）参数](#9-shooter射击决策参数)
10. [Planner（MPC 轨迹规划）参数](#10-plannermpc-轨迹规划参数)
11. [相机标定参数](#11-相机标定参数)
12. [CBoard（CAN 通信）参数](#12-cboardcan-通信参数)
13. [Gimbal（串口通信）参数](#13-gimbal串口通信参数)
14. [Buff（能量机关）检测参数](#14-buff能量机关检测参数)
15. [Buff Aimer（能量机关瞄准）参数](#15-buff-aimer能量机关瞄准参数)
16. [Omniperception/Decider（全向感知）参数](#16-omniperceptiondecider全向感知参数)
17. [标定工具专用参数](#17-标定工具专用参数)

---

## 1. 通用/团队参数

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `enemy_color` | string | - | 敌方颜色， `"red"` 或 `"blue"`。Tracker 和 Decider 据此过滤目标 |

---

## 2. 神经网络检测参数

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `yolo_name` | string | - | 使用的 YOLO 版本，可选 `"yolov5"` / `"yolov8"` / `"yolo11"` |
| `classify_model` | string | - | 装甲板数字分类 ONNX 模型路径，如 `assets/tiny_resnet.onnx` |
| `yolo11_model_path` | string | - | YOLO11 OpenVINO IR 模型路径（`.xml`） |
| `yolov8_model_path` | string | - | YOLOv8 OpenVINO IR 模型路径（`.xml`） |
| `yolov5_model_path` | string | - | YOLOv5 OpenVINO IR 模型路径（`.xml`） |
| `device` | string | - | OpenVINO 推理设备，`"CPU"` 或 `"GPU"` |
| `min_confidence` | double | 0.0~1.0 | 装甲板识别置信度阈值，低于此值的检测结果会被过滤 |
| `use_traditional` | bool | - | 是否启用传统 CV 方法对 YOLO 输出进行二次矫正（仅 YOLOv5 支持） |

---

## 3. ROI 参数

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `roi.x` | int | pixel | ROI 区域左上角 x 坐标 |
| `roi.y` | int | pixel | ROI 区域左上角 y 坐标 |
| `roi.width` | int | pixel | ROI 区域宽度 |
| `roi.height` | int | pixel | ROI 区域高度 |
| `use_roi` | bool | - | 是否启用 ROI 裁剪进行推理。设为 `false` 则使用全图 |

> ROI 配置示例：
> ```yaml
> roi:
>   x: 420
>   y: 50
>   width: 600
>   height: 600
> use_roi: false
> ```

---

## 4. USB 相机参数

以下参数用于 `io::USBCamera` 类，驱动 USB 摄像头（如全向感知相机）。

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `image_width` | double | pixel | USB 相机采集图像宽度 |
| `image_height` | double | pixel | USB 相机采集图像高度 |
| `new_image_width` | double | pixel | 裁剪/缩放后的图像宽度（部分配置使用） |
| `new_image_height` | double | pixel | 裁剪/缩放后的图像高度（部分配置使用） |
| `fov_h` | double | degree | USB 相机水平视场角 |
| `fov_v` | double | degree | USB 相机垂直视场角 |
| `new_fov_h` | double | degree | 裁剪/缩放后的水平视场角（用于 Decider 计算） |
| `new_fov_v` | double | degree | 裁剪/缩放后的垂直视场角（用于 Decider 计算） |
| `usb_frame_rate` | double | FPS | USB 相机采集帧率 |
| `usb_exposure` | double | V4L2 单位 | USB 相机曝光值（OpenCV V4L2 范围约 1~80000） |
| `new_usb_exposure` | double | V4L2 单位 | 备用曝光值（部分配置使用） |
| `usb_gamma` | double | - | USB 相机 Gamma 校正值 |
| `usb_gain` | double | - | USB 相机增益值（部分相机范围为 0~96） |

---

## 5. 工业相机参数

以下参数用于 `io::Camera` 类，驱动工业相机（MindVision 或 Hikrobot）。

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `camera_name` | string | - | 工业相机驱动名称，`"mindvision"` 或 `"hikrobot"` |
| `exposure_ms` | double | ms | 曝光时间，单位毫秒 |
| `gain` | double | - | 模拟增益（Hikrobot）或数字增益（厂商特定单位） |
| `gamma` | double | - | Gamma 值（仅 MindVision 相机使用） |
| `vid_pid` | string | - | USB 设备的 Vendor ID 和 Product ID，如 `"2bdf:0001"`（海康）、`"f622:d13a"`（迈德威视） |

---

## 6. 传统视觉检测参数

以下参数用于 `auto_aim::Detector` 传统视觉检测模块，以及 YOLO 的后处理辅助。

| 参数名 | 类型 | 配置单位 | 代码内部单位 | 说明 |
|--------|------|----------|--------------|------|
| `threshold` | double | 0~255 | 0~255 | 二值化阈值，用于灯条提取 |
| `max_angle_error` | double | degree | rad | 灯条最大允许角度偏差（相对于竖直方向） |
| `min_lightbar_ratio` | double | - | - | 灯条最小宽高比（height/width） |
| `max_lightbar_ratio` | double | - | - | 灯条最大宽高比 |
| `min_lightbar_length` | double | pixel | pixel | 灯条最小长度（像素） |
| `min_armor_ratio` | double | - | - | 装甲板最小宽高比（width/height） |
| `max_armor_ratio` | double | - | - | 装甲板最大宽高比 |
| `max_side_ratio` | double | - | - | 左右灯条长度比的最大值 |
| `max_rectangular_error` | double | degree | rad | 装甲板偏离矩形的最大角度误差 |

---

## 7. Tracker（目标跟踪）参数

以下参数用于 `auto_aim::Tracker` 目标跟踪器。

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `min_detect_count` | int | frame | 连续检测到目标的帧数阈值，达到后从 "detecting" 进入 "tracking" 状态 |
| `max_temp_lost_count` | int | frame | 普通目标允许的最大临时丢失帧数，超过则转为 "lost" |
| `outpost_max_temp_lost_count` | int | frame | 前哨站（outpost）目标允许的最大临时丢失帧数，通常设置更大 |

---

## 8. Aimer（瞄准）参数

以下参数用于 `auto_aim::Aimer` 和 `auto_aim::Planner` 瞄准模块。

| 参数名 | 类型 | 配置单位 | 代码内部单位 | 说明 |
|--------|------|----------|--------------|------|
| `yaw_offset` | double | degree | rad | 静态 yaw 偏置，用于补偿系统性的水平瞄准偏差 |
| `pitch_offset` | double | degree | rad | 静态 pitch 偏置，用于补偿系统性的俯仰瞄准偏差 |
| `comming_angle` | double | degree | rad | 目标旋转时，开始跟踪"接近中"装甲板的角度阈值 |
| `leaving_angle` | double | degree | rad | 目标旋转时，停止跟踪"远离中"装甲板的角度阈值 |
| `left_yaw_offset` | double | degree | rad | 左发射模式下的 yaw 偏置（可选） |
| `right_yaw_offset` | double | degree | rad | 右发射模式下的 yaw 偏置（可选） |
| `decision_speed` | double | rad/s | rad/s | 判断目标是否为"高速旋转"的角速度阈值 |
| `high_speed_delay_time` | double | s | s | 高速旋转目标的发弹延时补偿时间 |
| `low_speed_delay_time` | double | s | s | 普通/低速目标的发弹延时补偿时间 |
| `min_spin_speed` | double | rad/s | rad/s | 最小判定为旋转的角速度（部分配置使用，如 UAV） |

---

## 9. Shooter（射击决策）参数

以下参数用于 `auto_aim::Shooter` 射击决策模块。

| 参数名 | 类型 | 配置单位 | 代码内部单位 | 说明 |
|--------|------|----------|--------------|------|
| `first_tolerance` | double | degree | rad | **近距离**目标的射击角度容差 |
| `second_tolerance` | double | degree | rad | **远距离**目标的射击角度容差 |
| `judge_distance` | double | m | m | 近距离/远距离判断距离阈值 |
| `auto_fire` | bool | - | - | 是否由自瞄系统控制射击（`true` 为自动，`false` 为手动） |

---

## 10. Planner（MPC 轨迹规划）参数

以下参数用于 `auto_aim::Planner` 基于 TinyMPC 的轨迹规划器。

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `fire_thresh` | double | rad | MPC 轨迹误差阈值，小于此值时允许开火 |
| `max_yaw_acc` | double | rad/s² | Yaw 轴最大加速度约束（MPC 控制量约束） |
| `Q_yaw` | vector<double> | - | Yaw 轴 MPC 状态代价权重矩阵对角元素 |
| `R_yaw` | vector<double> | - | Yaw 轴 MPC 控制代价权重矩阵对角元素 |
| `max_pitch_acc` | double | rad/s² | Pitch 轴最大加速度约束（MPC 控制量约束） |
| `Q_pitch` | vector<double> | - | Pitch 轴 MPC 状态代价权重矩阵对角元素 |
| `R_pitch` | vector<double> | - | Pitch 轴 MPC 控制代价权重矩阵对角元素 |

> Planner 同时复用 `yaw_offset`、`pitch_offset`、`decision_speed`、`high_speed_delay_time`、`low_speed_delay_time` 等 Aimer 参数。

---

## 11. 相机标定参数

以下参数用于 `auto_aim::Solver`、`auto_buff::Solver` 以及标定工具，是 PnP 解算和坐标变换的核心。

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `R_gimbal2imubody` | vector<double> (9) | - | 云台坐标系到 IMU 体坐标系的旋转矩阵（3×3，行主序） |
| `camera_matrix` | vector<double> (9) | pixel | 相机内参矩阵 K（3×3，行主序，如 `fx, 0, cx, 0, fy, cy, 0, 0, 1`） |
| `distort_coeffs` | vector<double> (5) | - | 畸变系数 `[k1, k2, p1, p2, k3]` |
| `R_camera2gimbal` | vector<double> (9) | - | 相机坐标系到云台坐标系的旋转矩阵（3×3，行主序） |
| `t_camera2gimbal` | vector<double> (3) | m | 相机坐标系到云台坐标系的平移向量（单位：米） |

---

## 12. CBoard（CAN 通信）参数

以下参数用于 `io::CBoard` 类，通过 CAN 总线与下位机通信。

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `quaternion_canid` | int (hex) | - | IMU 四元数数据的 CAN ID（如 `0x100`、`0x01`） |
| `bullet_speed_canid` | int (hex) | - | 弹速/模式数据的 CAN ID（如 `0x101`、`0x110`） |
| `send_canid` | int (hex) | - | 视觉系统向下位机发送指令的 CAN ID（如 `0xff`） |
| `can_interface` | string | - | Linux CAN 接口名称，如 `"can0"` |

> CBoard 接收的 `bullet_speed` 单位为 **m/s**，`ft_angle` 单位为 **rad**。

---

## 13. Gimbal（串口通信）参数

以下参数用于 `io::Gimbal` 类，通过串口与云台通信。

| 参数名 | 类型 | 配置单位 | 默认值 | 说明 |
|--------|------|----------|--------|------|
| `com_port` | string | - | - | 串口设备路径，如 `"/dev/gimbal"`、`"/dev/ttyUSB0"` |
| `baudrate` | uint32_t | bps | 9600 | 串口波特率（常用 921600） |
| `bytesize` | int | bit | 8 | 数据位（5/6/7/8） |
| `parity` | string | - | "none" | 校验位：`"none"`/`"odd"`/`"even"`/`"mark"`/`"space"` |
| `stopbits` | int | - | 1 | 停止位（1 或 2） |
| `flowcontrol` | string | - | "none" | 流控：`"none"`/`"software"`/`"hardware"` |
| `timeout_ms` | uint32_t | ms | - | 串口读取超时时间 |
| `yaw_kp` | double | - | 0 | Yaw 轴比例增益（注：当前代码中未读取使用） |
| `yaw_kd` | double | - | 0 | Yaw 轴微分增益（注：当前代码中未读取使用） |
| `pitch_kp` | double | - | 0 | Pitch 轴比例增益（注：当前代码中未读取使用） |
| `pitch_kd` | double | - | 0 | Pitch 轴微分增益（注：当前代码中未读取使用） |

---

## 14. Buff（能量机关）检测参数

以下参数用于 `auto_buff::Buff_Detector` 和 `auto_buff::YOLO11_BUFF` 能量机关检测模块。

### 14.1 YOLO 模型参数

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `model` | string | - | Buff 检测的 OpenVINO 模型路径，如 `assets/yolo11_buff_int8.xml` |

### 14.2 传统检测参数（`detect` 命名空间）

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `detect.contrast` | int | - | 图像对比度调整值 |
| `detect.brightness.blue` | int | - | 蓝色敌方时的亮度偏移 |
| `detect.brightness.red` | int | - | 红色敌方时的亮度偏移 |
| `detect.brightness_threshold.blue` | int | 0~255 | 蓝色敌方时的二值化阈值 |
| `detect.brightness_threshold.red` | int | 0~255 | 红色敌方时的二值化阈值 |
| `detect.morphology_size.blue` | int | pixel | 蓝色敌方的形态学核大小 |
| `detect.morphology_size.red` | int | pixel | 红色敌方的形态学核大小 |
| `detect.dilate_size` | int | pixel | 膨胀操作的核大小 |
| `detect.R_contours_min_area` | int | pixel² | R 中心轮廓最小面积 |
| `detect.R_contours_max_area` | int | pixel² | R 中心轮廓最大面积 |
| `detect.fanblades_head_contours_min_area` | int | pixel² | 扇叶头部轮廓最小面积 |
| `detect.fanblades_head_contours_max_area` | int | pixel² | 扇叶头部轮廓最大面积 |
| `detect.fanblades_body_contours_min_area` | int | pixel² | 扇叶主体轮廓最小面积 |
| `detect.fanblades_body_contours_max_area` | int | pixel² | 扇叶主体轮廓最大面积 |
| `detect.standard_fanblade_path` | string | - | 标准扇叶模板图片路径，用于模板匹配 |

---

## 15. Buff Aimer（能量机关瞄准）参数

以下参数用于 `auto_buff::Aimer` 能量机关瞄准模块。

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `fire_gap_time` | double | s | 连续射击之间的最小间隔时间 |
| `predict_time` | double | s | 目标运动预测时间偏移量 |
| `aim_time` | double | s | 瞄准动作分配的时间 |
| `wait_time` | double | s | Buff 激活命令之间的等待时间 |
| `command_fire_gap` | double | s | 开火命令之间的最小间隔时间 |

> Buff Aimer 同时复用 `yaw_offset` 和 `pitch_offset` 参数（角度单位 degree，内部转换为 rad）。

---

## 16. Omniperception/Decider（全向感知）参数

以下参数用于 `omniperception::Decider` 全向感知决策模块。

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `mode` | int/double | - | 优先级决策模式，当前支持 `1` 或 `2`，不同模式对应不同的打击优先级策略 |

> Decider 同时复用 `image_width`、`image_height`、`fov_h`、`fov_v`、`new_fov_h`、`new_fov_v`、`enemy_color` 等参数。

---

## 17. 标定工具专用参数

以下参数仅用于 `calibration/` 目录下的标定工具（`calibrate_camera`、`calibrate_handeye`、`calibrate_robotworld_handeye`）。

| 参数名 | 类型 | 配置单位 | 说明 |
|--------|------|----------|------|
| `pattern_cols` | int | - | 标定板图案列数（圆点数量） |
| `pattern_rows` | int | - | 标定板图案行数（圆点数量） |
| `center_distance_mm` | double | mm | 标定板上相邻圆心之间的距离 |

---

## 物理量单位速查表

| 物理量 | 配置单位 | 代码内部单位 | 转换方式 |
|--------|----------|--------------|----------|
| 角度（偏移、容差、FOV 等） | **degree** | rad | 除以 57.3（≈180/π） |
| 角速度（`decision_speed`、`min_spin_speed`） | **rad/s** | rad/s | 直接使用 |
| 距离（`judge_distance`、`t_camera2gimbal`） | **m** | m | 直接使用 |
| 时间（`delay_time`、`predict_time`、`fire_gap_time` 等） | **s** | s | 直接使用 |
| 弹速（`bullet_speed`，CAN 接收） | **m/s** | m/s | 直接使用 |
| 加速度（`max_yaw_acc`、`max_pitch_acc`） | **rad/s²** | rad/s² | 直接使用 |
| 相机内参（`camera_matrix`） | **pixel** | pixel | 直接使用 |
| 图像尺寸（`image_width`、`roi.width` 等） | **pixel** | pixel | 直接使用 |
| 标定板间距（`center_distance_mm`） | **mm** | mm | 直接使用 |
| 曝光时间（`exposure_ms`） | **ms** | ms | 直接使用 |
| USB 曝光（`usb_exposure`） | **V4L2 单位** | V4L2 单位 | 直接使用 |

---

## 配置文件示例结构

```yaml
# 1. 通用参数
enemy_color: "red"

# 2. 神经网络参数
yolo_name: yolov5
classify_model: assets/tiny_resnet.onnx
yolo11_model_path: assets/yolo11.xml
yolov8_model_path: assets/yolov8.xml
yolov5_model_path: assets/yolov5.xml
device: GPU
min_confidence: 0.8
use_traditional: true

# 3. ROI
roi:
  x: 420
  y: 50
  width: 600
  height: 600
use_roi: false

# 4. 工业相机参数
camera_name: "hikrobot"
exposure_ms: 2
gain: 16
vid_pid: "2bdf:0001"

# 5. 传统检测参数
threshold: 150
max_angle_error: 45        # degree
min_lightbar_ratio: 1.5
max_lightbar_ratio: 20
min_lightbar_length: 8     # pixel
min_armor_ratio: 1
max_armor_ratio: 5
max_side_ratio: 1.5
max_rectangular_error: 25  # degree

# 6. Tracker参数
min_detect_count: 5
max_temp_lost_count: 15
outpost_max_temp_lost_count: 75

# 7. Aimer参数
yaw_offset: -2             # degree
pitch_offset: 0            # degree
comming_angle: 60          # degree
leaving_angle: 20          # degree
decision_speed: 8          # rad/s
high_speed_delay_time: 0.015  # s
low_speed_delay_time: 0.015   # s

# 8. Shooter参数
first_tolerance: 3         # degree
second_tolerance: 2        # degree
judge_distance: 2          # m
auto_fire: true

# 9. Planner参数
fire_thresh: 0.003
max_yaw_acc: 50
Q_yaw: [9e6, 0]
R_yaw: [1]
max_pitch_acc: 100
Q_pitch: [9e6, 0]
R_pitch: [1]

# 10. 相机标定参数
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]
camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distort_coeffs: [k1, k2, p1, p2, k3]
R_camera2gimbal: [r11, r12, r13, r21, r22, r23, r31, r32, r33]
t_camera2gimbal: [tx, ty, tz]   # m

# 11. CBoard参数
quaternion_canid: 0x100
bullet_speed_canid: 0x101
send_canid: 0xff
can_interface: "can0"

# 12. Gimbal参数
com_port: "/dev/gimbal"
baudrate: 921600

# 13. Buff参数
model: "assets/yolo11_buff_int8.xml"

# 14. Buff Aimer参数
fire_gap_time: 0.520   # s
predict_time: 0.100    # s
```
