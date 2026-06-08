# 可执行文件说明文档

本文档描述本项目编译后生成的所有可执行文件的作用及用法。项目基于 CMake 构建，编译完成后所有可执行文件位于 `build/` 目录下。

> **运行环境要求**：Ubuntu 22.04、OpenCV、OpenVINO、Ceres、YAML-CPP、spdlog、fmt、Eigen3 等，详见项目 [readme.md](../readme.md)。

---

## 目录

- [1. 应用层主程序（src/）](#1-应用层主程序src)
- [2. 标定工具（calibration/）](#2-标定工具calibration)
- [3. 模块测试程序（tests/）](#3-模块测试程序tests)

---

## 1. 应用层主程序（src/）

应用层主程序对应不同兵种/场景的实际运行程序，负责从相机获取图像、根据电控信号切换功能、执行算法并向下位机发送控制指令。

| 可执行文件 | 作用 |
|-----------|------|
| `standard` | **标准兵种（步兵/英雄）自瞄主程序**。使用传统决策器（Aimer + Shooter），支持自瞄和打符模式切换，运行于 CBoard（STM32）通信协议。 |
| `mt_standard` | **标准兵种多线程自瞄+打符主程序**。将 YOLO 检测放到独立线程，减少主循环阻塞，支持自瞄和打符双模式。 |
| `standard_mpc` | **标准兵种 MPC 轨迹规划版主程序**。使用自瞄轨迹规划器（Planner）替代传统决策器，结合 Gimbal 通信协议，支持自瞄和打符。 |
| `auto_aim_debug_mpc` | **MPC 轨迹规划器实车调试程序**。在 `standard_mpc` 基础上增加可视化重投影、PlotJuggler 调试数据输出，用于验证规划器效果。 |
| `mt_auto_aim_debug` | **多线程自瞄调试程序**。使用 CBoard 协议，多线程检测 + 传统决策器，带可视化调试输出。 |
| `auto_buff_debug` | **打符调试程序**。使用 CBoard 协议，运行打符识别、解算、瞄准全流程，支持 PlotJuggler 曲线调试。 |
| `auto_buff_debug_mpc` | **打符 MPC 调试程序**。在打符流程中使用 MPC 轨迹规划器进行瞄准决策。 |
| `uav` | **无人机自瞄+打符主程序**。使用传统识别器（Detector）和 YOLO，支持自瞄、前哨站、打符等模式。 |
| `uav_debug` | **无人机自瞄调试程序**。在 `uav` 基础上增加可视化重投影和 PlotJuggler 调试输出。 |
| `sentry` | **哨兵自瞄主程序**。集成 ROS2 通信、全向感知（Omniperception）模块，使用 CBoard + 多相机（工业相机 + USB 相机）。 |
| `sentry_bp` | **哨兵自瞄简化版**。与 `sentry` 类似，但去掉了 USB 相机，仅用背面工业相机进行全向感知。 |
| `sentry_debug` | **哨兵调试程序**。在 `sentry` 基础上增加完整的可视化重投影、PlotJuggler 数据输出、状态显示。 |
| `sentry_multithread` | **哨兵多线程全向感知主程序**。集成 4 路 USB 相机 + Perceptron 模块，支持目标切换和丢失后的全向搜索。 |

### 通用用法

所有主程序均通过命令行传入 YAML 配置文件路径：

```bash
./build/<executable> <config-path>
```

例如：

```bash
./build/standard configs/standard3.yaml
./build/standard_mpc configs/standard4.yaml
./build/sentry configs/sentry.yaml
```

---

## 2. 标定工具（calibration/）

标定工具用于相机内参标定、手眼标定以及录制视频的切割。标定流程依赖于对称圆点标定板（默认 10 列 × 7 行，圆心距 40mm）。

| 可执行文件 | 作用 |
|-----------|------|
| `capture` | **标定数据采集程序**。连接工业相机和 CBoard，实时显示标定板识别结果，按 `s` 保存当前图像及对应 IMU 四元数，按 `q` 退出。 |
| `calibrate_camera` | **相机内参标定程序**。读取采集的图像序列，使用 OpenCV `calibrateCamera` 计算相机内参矩阵和畸变系数。 |
| `calibrate_handeye` | **手眼标定程序**。读取图像及对应四元数，计算相机相对于云台坐标系的旋转和平移 `R_camera2gimbal`、`t_camera2gimbal`。 |
| `calibrate_robotworld_handeye` | **RobotWorld 手眼标定程序**。在标定相机-云台关系的同时，计算标定板相对于世界坐标系的位置。 |
| `split_video` | **录制视频切割工具**。将 `.avi` + `.txt`（时间戳+四元数）的录制文件按指定帧范围切割，生成新的子视频。 |

---

### 2.1 `capture` — 标定数据采集

**功能**：
- 打开工业相机（海康/迈德威视），读取实时图像；
- 通过 CAN 总线获取 CBoard 的 IMU 绝对四元数；
- 实时检测对称圆点标定板，显示识别结果和 IMU 欧拉角；
- 按 `s` 保存当前原始图像和四元数到指定文件夹；
- 按 `q` 退出程序。

**用法**：

```bash
./build/capture <config-path> [-o=<output-folder>]
```

| 参数 | 默认值 | 说明 |
|-----|--------|------|
| `<config-path>` | `configs/calibration.yaml` | YAML 配置文件路径，包含相机参数、CAN ID 等 |
| `-o` | `assets/img_with_q` | 输出文件夹路径 |

**输出文件格式**：

```
<output-folder>/
  ├── 1.jpg       # 原始图像
  ├── 1.txt       # 对应四元数 (w x y z)
  ├── 2.jpg
  ├── 2.txt
  └── ...
```

> **注意**：四元数输出顺序为 `w x y z`。采集时应从不同角度和距离拍摄标定板，保证覆盖图像各个区域。

---

### 2.2 `calibrate_camera` — 相机内参标定

**功能**：
- 读取采集的图像序列（`1.jpg`、`2.jpg` ...）；
- 使用 `cv::findCirclesGrid` 检测对称圆点标定板；
- 调用 `cv::calibrateCamera` 计算相机内参矩阵 `camera_matrix` 和畸变系数 `distort_coeffs`；
- 计算重投影误差并输出 YAML 格式结果。

**用法**：

```bash
./build/calibrate_camera <input-folder> [-c=<config-path>]
```

| 参数 | 默认值 | 说明 |
|-----|--------|------|
| `<input-folder>` | `assets/img_with_q` | 输入图像文件夹 |
| `-c` | `configs/calibration.yaml` | YAML 配置文件，需包含 `pattern_cols`、`pattern_rows`、`center_distance_mm` |

**配置文件示例**（`configs/calibration.yaml`）：

```yaml
pattern_cols: 10        # 标定板列数
pattern_rows: 7         # 标定板行数
center_distance_mm: 40  # 圆心距（mm）
```

**输出示例**：

```yaml
# 重投影误差: 0.1234px
camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distort_coeffs: [k1, k2, p1, p2]
```

> 标定完成后，将 `camera_matrix` 和 `distort_coeffs` 填入 `configs/calibration.yaml` 或对应兵种的配置文件中。

---

### 2.3 `calibrate_handeye` — 手眼标定

**功能**：
- 读取图像及对应四元数文件；
- 通过 `R_gimbal2imubody` 将 IMU 四元数转换为云台相对于世界坐标系的姿态；
- 使用 `cv::solvePnP` 计算标定板相对于相机的位姿；
- 调用 `cv::calibrateHandEye` 求解相机到云台的旋转 `R_camera2gimbal` 和平移 `t_camera2gimbal`；
- 输出相机相对于理想安装位置的偏角，用于检验安装精度。

**用法**：

```bash
./build/calibrate_handeye <input-folder> [-c=<config-path>]
```

**前置要求**：
- `configs/calibration.yaml` 中必须已包含：
  - `camera_matrix`（内参标定结果）
  - `distort_coeffs`（畸变系数）
  - `R_gimbal2imubody`（云台到 IMU 的旋转矩阵，3×3 按行展开）

**输入文件格式**：

```
<input-folder>/
  ├── 1.jpg
  ├── 1.txt        # w x y z（四元数，IMU绝对系）
  ├── 2.jpg
  ├── 2.txt
  └── ...
```

**输出示例**：

```yaml
R_gimbal2imubody: [...]

# 相机同理想情况的偏角: yaw1.23 pitch-0.45 roll2.67 degree
R_camera2gimbal: [r11, r12, r13, r21, r22, r23, r31, r32, r33]
t_camera2gimbal: [tx, ty, tz]   # 单位：米
```

> **操作提示**：
> 1. 采集数据时，固定标定板位置，移动云台（改变 yaw/pitch）拍摄不同角度；
> 2. 程序会实时显示云台的欧拉角（yaw/pitch/roll），用于检验 `R_gimbal2imubody` 是否正确；
> 3. 如果 yaw/pitch/roll 变化不符合预期，说明 `R_gimbal2imubody` 的符号或顺序有误。

---

### 2.4 `calibrate_robotworld_handeye` — RobotWorld 手眼标定

**功能**：
- 与 `calibrate_handeye` 类似，但使用 `cv::calibrateRobotWorldHandEye`；
- 在求解相机-云台关系的同时，计算标定板相对于世界坐标系的位置和姿态；
- 适用于需要同时确定标定板在世界坐标系中位置的场景。

**用法**：

```bash
./build/calibrate_robotworld_handeye <input-folder> [-c=<config-path>]
```

**输出示例**：

```yaml
R_gimbal2imubody: [...]

# 相机同理想情况的偏角: yaw1.23 pitch-0.45 roll2.67 degree
# 标定板到世界坐标系原点的水平距离: 1.50 m
# 标定板同竖直摆放时的偏角: yaw0.00 pitch-1.23 roll0.50 degree
R_camera2gimbal: [...]
t_camera2gimbal: [...]
```

> 标定板坐标系定义：以标定板中心为原点，x 轴垂直于标定板平面向外，y/z 轴在标定板平面内。

---

### 2.5 `split_video` — 录制视频切割

**功能**：
- 将本项目录制器生成的 `.avi` 视频和对应的 `.txt` 四元数文件按指定帧范围切割；
- 生成新的 `.avi` 和 `.txt` 文件，保留原视频的编码格式和帧率。

**用法**：

```bash
./build/split_video <input-path> [-s=<start-index>] [-e=<end-index>] [-p=<output-path>]
```

| 参数 | 默认值 | 说明 |
|-----|--------|------|
| `<input-path>` | — | 输入文件路径（不含扩展名，程序会自动查找 `.avi` 和 `.txt`） |
| `-s` | — | 起始帧下标（从 0 开始） |
| `-e` | — | 结束帧下标（不包含） |
| `-p` | `records/Big/2024-05-14_11-6-26` | 输出文件路径（不含扩展名） |

**示例**：

```bash
# 将 demo.avi / demo.txt 的第 100~500 帧切割出来
./build/split_video assets/demo/demo -s=100 -e=500 -p=assets/demo/demo_cut
```

> 切割后的文件为 `demo_cut.avi` 和 `demo_cut.txt`，可直接用于 `auto_aim_test` 等离线测试程序。

---

## 3. 模块测试程序（tests/）

测试程序用于独立验证某个硬件模块或算法模块的功能，便于赛前排查问题和赛后复现问题。

| 可执行文件 | 作用 |
|-----------|------|
| `auto_aim_test` | **自瞄离线视频测试**。读取录制的 `.avi` + `.txt`，运行完整的自瞄识别-跟踪-决策流程，支持 PlotJuggler 调试输出和可视化重投影。 |
| `auto_buff_test` | **打符离线视频测试**。读取录制视频，运行打符识别-解算-瞄准全流程，支持可视化调试。 |
| `camera_test` | **工业相机测试**。打开工业相机，读取图像并显示帧率，用于验证相机驱动是否正常。 |
| `camera_thread_test` | **相机多线程 YOLO 测试**。测试多线程 YOLO 推理性能，使用线程池并发检测。 |
| `camera_detect_test` | **工业相机识别器测试**。连接工业相机，实时运行传统识别器或 YOLO，显示检测帧率。 |
| `cboard_test` | **C 板通信测试**。连接 CBoard，循环读取 IMU 四元数和弹速，用于验证通信协议。 |
| `detector_video_test` | **视频识别器测试**。对录制视频运行传统识别器或 YOLO，输出装甲板像素坐标到 PlotJuggler。 |
| `dm_test` | **达妙 IMU 测试**。连接达妙 IMU，循环读取并输出欧拉角。 |
| `fire_test` | **开火测试**。连接 Gimbal，周期性发送开火指令，测试摩擦轮和供弹机构。 |
| `gimbal_test` | **云台通信测试**。连接 Gimbal，周期性发送控制指令，读取并显示云台状态（yaw/pitch/速度/弹速/发弹数）。 |
| `gimbal_response_test` | **云台响应测试**。向 CBoard 发送三角波/阶跃/圆周运动指令，测试云台响应特性，用于电控调参。 |
| `handeye_test` | **手眼标定效果验证**。在图像上投影世界坐标系下的网格点，验证标定结果是否正确。 |
| `minimum_vision_system` | **最小视觉系统测试**。使用达妙 IMU + 工业相机 + 多线程检测，构成最小可运行自瞄系统。 |
| `multi_usbcamera_test` | **多 USB 摄像头测试**。同时打开多个 USB 摄像头（如 `video0`、`video2`）和工业相机，显示帧率。 |
| `planner_test` | **规划器实车测试**。连接 Gimbal，生成虚拟目标运动，测试 MPC 轨迹规划器的实时输出。 |
| `planner_test_offline` | **规划器离线测试**。不连接硬件，纯仿真运行 MPC 规划器，输出规划轨迹到 PlotJuggler。 |
| `publish_test` | **ROS 发布测试**。测试 ROS2 话题发布功能（需 ROS2 环境）。 |
| `subscribe_test` | **ROS 订阅测试**。测试 ROS2 话题订阅功能（需 ROS2 环境）。 |
| `topic_loop_test` | **ROS 话题循环测试**。测试自定义 ROS2 消息类型的发布与订阅循环（需 ROS2 环境）。 |
| `usbcamera_test` | **USB 摄像头测试**。打开指定 USB 摄像头，读取图像并显示帧率。 |
| `usbcamera_detect_test` | **USB 摄像头识别测试**。打开 USB 摄像头，实时运行 YOLO 检测，显示帧率。 |

### 通用用法

大多数测试程序支持 `-h` 或 `--help` 查看完整参数说明：

```bash
./build/<test_executable> -h
```

典型调用示例：

```bash
# 自瞄离线测试（从第 0 帧运行到结束，使用 demo.yaml 配置）
./build/auto_aim_test assets/demo/demo -c=configs/demo.yaml

# 工业相机测试（显示图像）
./build/camera_test configs/camera.yaml -d

# C 板测试
./build/cboard_test configs/standard.yaml

# 云台响应测试（yaw 轴三角波，幅度 8°，周期 0.2s）
./build/gimbal_response_test configs/sentry.yaml -a=8 -c=0.2 -m=triangle_wave -x=yaw

# USB 摄像头测试
./build/usbcamera_test configs/sentry.yaml -n=video0 -d

# 手眼标定验证
./build/handeye_test configs/handeye.yaml -d
```

---

## 附录：标定完整流程建议

1. **准备标定板**：打印或购买对称圆点标定板（10×7，圆心距 40mm），固定于平整墙面；
2. **采集数据**：
   ```bash
   ./build/capture configs/calibration.yaml -o=assets/img_with_q
   ```
   从不同角度、距离拍摄 15~30 张，确保标定板在图像各区域均有分布；
3. **相机内参标定**：
   ```bash
   ./build/calibrate_camera assets/img_with_q -c=configs/calibration.yaml
   ```
   将输出的 `camera_matrix` 和 `distort_coeffs` 填入 `configs/calibration.yaml`；
4. **手眼标定**：
   ```bash
   ./build/calibrate_handeye assets/img_with_q -c=configs/calibration.yaml
   ```
   将输出的 `R_camera2gimbal` 和 `t_camera2gimbal` 填入对应兵种的 YAML 配置文件；
5. **验证标定**：
   ```bash
   ./build/handeye_test configs/handeye.yaml -d
   ```
   观察网格点是否正确投影到图像平面上。
