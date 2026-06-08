# `auto_aim::` API 文档

> 本文档由代码分析自动生成，涵盖 `tasks/auto_aim/` 目录下所有公开类、函数、枚举和结构体。
> `auto_aim::` 是 sp_vision 框架的**自瞄功能层**，包含装甲板识别（传统/OpenVINO）、位姿解算、目标跟踪与状态估计、瞄准决策、开火决策、轨迹规划（MPC）等完整自瞄链路。

---

## 目录

1. [瞄准器 (`aimer.hpp`)](#瞄准器-aimerhpp)
2. [装甲板定义 (`armor.hpp`)](#装甲板定义-armorhpp)
3. [图案分类器 (`classifier.hpp`)](#图案分类器-classifierhpp)
4. [传统检测器 (`detector.hpp`)](#传统检测器-detectorhpp)
5. [多线程指令生成 (`multithread/commandgener.hpp`)](#多线程指令生成-multithreadcommandgenerhpp)
6. [多线程检测器 (`multithread/mt_detector.hpp`)](#多线程检测器-multithreadmt_detectorhpp)
7. [轨迹规划器 (`planner/planner.hpp`)](#轨迹规划器-plannerplannerhpp)
8. [开火决策 (`shooter.hpp`)](#开火决策-shooterhpp)
9. [位姿解算器 (`solver.hpp`)](#位姿解算器-solverhpp)
10. [目标状态估计 (`target.hpp`)](#目标状态估计-targethpp)
11. [目标跟踪 (`tracker.hpp`)](#目标跟踪-trackerhpp)
12. [投票器 (`voter.hpp`)](#投票器-voterhpp)
13. [YOLO 包装器 (`yolo.hpp`)](#yolo-包装器-yolohpp)
14. [YOLO11 (`yolos/yolo11.hpp`)](#yolo11-yolosyolo11hpp)
15. [YOLOv5 (`yolos/yolov5.hpp`)](#yolov5-yolosyolov5hpp)
16. [YOLOv8 (`yolos/yolov8.hpp`)](#yolov8-yolosyolov8hpp)
17. [TinyMPC 规划求解器 (`planner/tinympc/`)](#tinympc-规划求解器-plannertinympc)

---

## 瞄准器 (`aimer.hpp`)

**命名空间：** `auto_aim`

### `struct AimPoint`

| 成员 | 类型 | 说明 |
|------|------|------|
| `valid` | `bool` | 是否有效。 |
| `xyza` | `Eigen::Vector4d` | 3D 位置 + 角度。 |

### `class Aimer`

计算云台控制指令（yaw/pitch），包含弹道补偿与迭代飞行时间预测。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `explicit Aimer(const std::string &config_path)` | 从 YAML 加载 yaw/pitch 偏移量、进入/离开角度、延迟时间、决策速度。 |
| `aim` | `io::Command aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed, bool to_now = true)` | 主瞄准逻辑：选择最优装甲板，迭代预测弹丸飞行时间，计算补偿后的 yaw/pitch 指令。 |
| `aim` | `io::Command aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed, io::ShootMode shoot_mode, bool to_now = true)` | 重载：支持左/右射击模式的偏移量。 |

**公开成员变量：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `debug_aim_point` | `AimPoint` | 最后一次计算的瞄准点（调试用）。 |

---

## 装甲板定义 (`armor.hpp`)

**命名空间：** `auto_aim`

### 枚举

| 枚举 | 值 | 说明 |
|------|-----|------|
| `Color` | `red`, `blue`, `extinguish`, `purple` | 颜色。 |
| `ArmorType` | `big`, `small` | 装甲板大小。 |
| `ArmorName` | `one`, `two`, `three`, `four`, `five`, `sentry`, `outpost`, `base`, `not_armor` | 装甲板编号/名称。 |
| `ArmorPriority` | `first=1`, `second`, `third`, `forth`, `fifth` | 优先级。 |

字符串查找表：`COLORS`、`ARMOR_TYPES`、`ARMOR_NAMES`。

类 ID 到属性的映射：`armor_properties`：`std::vector<std::tuple<Color, ArmorName, ArmorType>>`。

### `struct Lightbar`

灯条（传统检测的中间结构）。

| 成员 | 类型 | 说明 |
|------|------|------|
| `id` | `std::size_t` | 编号。 |
| `color` | `Color` | 颜色。 |
| `center`, `top`, `bottom` | `cv::Point2f` | 几何点。 |
| `top2bottom` | `cv::Point2f` | — |
| `points` | `std::vector<cv::Point2f>` | 轮廓点。 |
| `angle`, `angle_error`, `length`, `width`, `ratio` | `float` | 几何特征。 |
| `rotated_rect` | `cv::RotatedRect` | 最小外接旋转矩形。 |

**构造函数：**

| 签名 | 说明 |
|------|------|
| `Lightbar(const cv::RotatedRect &rotated_rect, std::size_t id)` | 从旋转矩形构造。 |
| `Lightbar() = default` | 默认构造。 |

### `struct Armor`

装甲板结构体，同时承载传统检测与神经网络检测的结果。

| 成员 | 类型 | 说明 |
|------|------|------|
| `color` | `Color` | 颜色。 |
| `left`, `right` | `Lightbar` | 左右灯条（传统检测）。 |
| `center`, `center_norm` | `cv::Point2f` | 中心点（像素 / 归一化）。 |
| `points` | `std::vector<cv::Point2f>` | 四个角点。 |
| `ratio`, `side_ratio`, `rectangular_error` | `float` | 几何特征。 |
| `type` | `ArmorType` | 大小类型。 |
| `name` | `ArmorName` | 编号。 |
| `priority` | `ArmorPriority` | 优先级。 |
| `class_id` | `int` | 网络输出类别 ID。 |
| `box` | `cv::Rect` | 边界框。 |
| `pattern` | `std::string` | 图案/数字字符串。 |
| `confidence` | `float` | 置信度。 |
| `duplicated` | `bool` | 是否重复。 |
| `xyz_in_gimbal`, `xyz_in_world` | `Eigen::Vector3d` | 3D 位置（云台系 / 世界系）。 |
| `ypr_in_gimbal`, `ypr_in_world` | `Eigen::Vector3d` | YPR 姿态（云台系 / 世界系）。 |
| `ypd_in_world` | `Eigen::Vector3d` | 世界系下的 YPD（yaw, pitch, distance）。 |
| `yaw_raw` | `double` | 原始 yaw。 |

**构造函数：**

| 签名 | 说明 |
|------|------|
| `Armor(const Lightbar &left, const Lightbar &right)` | 传统检测：由左右灯条构造。 |
| `Armor(int class_id, float confidence, const cv::Rect &box, std::vector<cv::Point2f> armor_keypoints)` | NN 检测（无偏移）。 |
| `Armor(int class_id, float confidence, const cv::Rect &box, std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset)` | NN 检测（含 ROI 偏移）。 |
| `Armor(int color_id, int num_id, float confidence, const cv::Rect &box, std::vector<cv::Point2f> armor_keypoints)` | YOLOv5 风格。 |
| `Armor(int color_id, int num_id, float confidence, const cv::Rect &box, std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset)` | YOLOv5 风格（含 ROI 偏移）。 |

---

## 图案分类器 (`classifier.hpp`)

**命名空间：** `auto_aim`

### `class Classifier`

装甲板数字/图案分类器，同时支持 OpenCV DNN 与 OpenVINO 后端。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `explicit Classifier(const std::string &config_path)` | 从 YAML 加载 ONNX 模型路径，初始化 `cv::dnn::Net` 与 OpenVINO 编译模型。 |
| `classify` | `void classify(Armor &armor)` | 使用 OpenCV DNN 进行分类。 |
| `ovclassify` | `void ovclassify(Armor &armor)` | 使用 OpenVINO 进行分类（推荐）。 |

**私有成员：** `cv::dnn::Net net_`、`ov::Core core_`、`ov::CompiledModel compiled_model_`。

---

## 传统检测器 (`detector.hpp`)

**命名空间：** `auto_aim`

### `class Detector`

传统装甲板检测流水线：阈值分割 → 轮廓提取 → 灯条匹配 → 几何筛选 → 分类。同时可作为神经网络检测的角点精修后端。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Detector(const std::string &config_path, bool debug = true)` | 加载阈值参数，初始化分类器，创建保存目录。 |
| `detect` | `std::list<Armor> detect(const cv::Mat &bgr_img, int frame_count = -1)` | 对 BGR 图像执行完整检测流水线。 |
| `detect` | `bool detect(Armor &armor, const cv::Mat &bgr_img)` | 对单个装甲板使用传统方法精修角点（YOLOv5 调用）。 |

**友元类：** `friend class YOLOV8`

**私有方法：** `lightbar_points_corrector`（PCA 角点修正）、`check_geometry`、`check_name`、`check_type`、`get_color`、`get_pattern`、`get_type`、`get_center_norm`、`save`、`show_result`。

---

## 多线程指令生成 (`multithread/commandgener.hpp`)

**命名空间：** `auto_aim::multithread`

### `class CommandGener`

后台线程持续运行，使用最新目标数据生成并发送云台指令。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `CommandGener(auto_aim::Shooter &shooter, auto_aim::Aimer &aimer, io::CBoard &cboard, tools::Plotter &plotter, bool debug = false)` | 启动后台线程。 |
| 析构函数 | `~CommandGener()` | 发送停止信号并 join 线程。 |
| `push` | `void push(const std::list<auto_aim::Target> &targets, const std::chrono::steady_clock::time_point &t, double bullet_speed, const Eigen::Vector3d &gimbal_pos)` | 推送最新数据供指令生成使用。 |

**私有结构体 `Input`：** 保存 `targets_`、`t`、`bullet_speed`、`gimbal_pos`。

**私有方法：** `generate_command()`（约 500Hz 循环：调用 Aimer、Shooter，通过 CBoard 发送指令）。

---

## 多线程检测器 (`multithread/mt_detector.hpp`)

**命名空间：** `auto_aim::multithread`

### `class MultiThreadDetector`

异步 YOLO 检测器：使用 OpenVINO 编译模型，通过线程安全队列解耦推理的 push/pop，消除推理等待对主循环的阻塞。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `MultiThreadDetector(const std::string &config_path, bool debug = false)` | 加载模型，配置 OpenVINO 预处理（NHWC→NCHW、BGR→RGB、scale 255），编译为 `THROUGHPUT` 模式。 |
| `push` | `void push(cv::Mat img, std::chrono::steady_clock::time_point t)` | 预处理、启动异步推理、推入队列。 |
| `pop` | `std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop()` | 等待推理完成，执行 YOLO 后处理，返回装甲板列表 + 时间戳。 |
| `debug_pop` | `std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop()` | 同 `pop()`，额外返回原始图像。 |

**私有成员：** OpenVINO `core_`、`compiled_model_`、`device_`；`auto_aim::YOLO yolo_`；容量为 16 的 `tools::ThreadSafeQueue` 队列（存储图像/时间戳/推理请求元组）。

---

## 轨迹规划器 (`planner/planner.hpp`)

**命名空间：** `auto_aim`

### 常量与类型别名

| 名称 | 定义 | 说明 |
|------|------|------|
| `DT` | `constexpr double DT = 0.01` | 离散时间步长（10 ms）。 |
| `HALF_HORIZON` | `constexpr int HALF_HORIZON = 50` | 半预测时域。 |
| `HORIZON` | `constexpr int HORIZON = 100` | 总预测时域。 |
| `Trajectory` | `Eigen::Matrix<double, 4, HORIZON>` | 轨迹矩阵：`[yaw, yaw_vel, pitch, pitch_vel]` × 时域长度。 |

### `struct Plan`

| 成员 | 类型 | 说明 |
|------|------|------|
| `control` | `bool` | 是否控制云台。 |
| `fire` | `bool` | 是否开火。 |
| `target_yaw`, `target_pitch` | `float` | 目标 yaw/pitch。 |
| `yaw`, `yaw_vel`, `yaw_acc` | `float` | Yaw 位置/速度/加速度指令。 |
| `pitch`, `pitch_vel`, `pitch_acc` | `float` | Pitch 位置/速度/加速度指令。 |

### `class Planner`

基于 TinyMPC 的模型预测控制（MPC）轨迹规划器，分别为 yaw 和 pitch 通道求解最优轨迹，实现提前减速策略与开火决策。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Planner(const std::string &config_path)` | 加载偏移量/开火阈值/延迟参数，分别初始化 yaw 和 pitch 的 TinyMPC 求解器。 |
| `plan` | `Plan plan(Target target, double bullet_speed)` | 使用 MPC 规划轨迹，求解 yaw 与 pitch，返回控制/开火标志。 |
| `plan` | `Plan plan(std::optional<Target> target, double bullet_speed)` | 包装：处理空目标，并根据弹速应用不同的延迟时间。 |

**公开成员变量：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `debug_xyza` | `Eigen::Vector4d` | 调试用瞄准位置。 |

**私有方法：** `setup_yaw_solver()`、`setup_pitch_solver()`、`aim(const Target&, double)` → 返回 `{yaw, pitch}`、`get_trajectory()` → 计算预测时域内的参考轨迹。

---

## 开火决策 (`shooter.hpp`)

**命名空间：** `auto_aim`

### `class Shooter`

根据指令稳定性与云台对准容差判断是否开火。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Shooter(const std::string &config_path)` | 加载 `first_tolerance_`、`second_tolerance_`、`judge_distance_`、`auto_fire_`。 |
| `shoot` | `bool shoot(const io::Command &command, const auto_aim::Aimer &aimer, const std::list<auto_aim::Target> &targets, const Eigen::Vector3d &gimbal_pos)` | 若指令稳定且云台在容差范围内则返回 `true`。 |

**私有成员：** `last_command_`、`judge_distance_`、`first_tolerance_`、`second_tolerance_`、`auto_fire_`。

---

## 位姿解算器 (`solver.hpp`)

**命名空间：** `auto_aim`

### `class Solver`

装甲板 3D 位姿解算：PnP（IPPE）、坐标变换（相机→云台→世界）、yaw 优化（通过重投影误差最小化）、世界坐标→像素投影。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `explicit Solver(const std::string &config_path)` | 从 YAML 加载相机内参、畸变系数、R/t 变换矩阵。 |
| `R_gimbal2world` | `Eigen::Matrix3d R_gimbal2world() const` | 返回当前世界旋转矩阵。 |
| `set_R_gimbal2world` | `void set_R_gimbal2world(const Eigen::Quaterniond &q)` | 根据 IMU 四元数更新世界旋转。 |
| `solve` | `void solve(Armor &armor) const` | 执行 `solvePnP`（IPPE），计算装甲板在云台系和世界系下的 `xyz`/`ypr`，调用 `optimize_yaw`。 |
| `reproject_armor` | `std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d &xyz_in_world, double yaw, ArmorType type, ArmorName name) const` | 将世界坐标下的装甲板角点重投影到图像像素。 |
| `oupost_reprojection_error` | `double oupost_reprojection_error(Armor armor, const double &pitch)` | 计算前哨站装甲板在给定 pitch 下的重投影误差。 |
| `world2pixel` | `std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> &worldPoints)` | 将世界坐标 3D 点投影到像素坐标。 |

**私有方法：** `optimize_yaw`（140° 网格搜索）、`armor_reprojection_error`、`SJTU_cost`。

---

## 目标状态估计 (`target.hpp`)

**命名空间：** `auto_aim`

### `class Target`

基于扩展卡尔曼滤波（EKF）的目标状态估计器。跟踪敌方机器人的旋转中心（x, y, z, yaw, 角速度, 半径等）并预测各装甲板位置。

| 成员 | 签名 | 说明 |
|------|------|------|
| 默认构造 | `Target() = default` | — |
| 参数构造 | `Target(const Armor &armor, std::chrono::steady_clock::time_point t, double radius, int armor_num, Eigen::VectorXd P0_dig)` | 由首次检测到的装甲板初始化 EKF。 |
| 简化构造 | `Target(double x, double vyaw, double radius, double h)` | 简化构造（用于仿真/测试）。 |
| `predict` | `void predict(std::chrono::steady_clock::time_point t)` | 预测到指定时间戳。 |
| `predict` | `void predict(double dt)` | 按时间差 `dt` 预测（使用分段白噪声模型）。 |
| `update` | `void update(const Armor &armor)` | 以装甲板测量值（ypda：yaw, pitch, distance, angle）进行 EKF 更新。 |
| `ekf_x` | `Eigen::VectorXd ekf_x() const` | 返回当前 EKF 状态向量。 |
| `ekf` | `const tools::ExtendedKalmanFilter &ekf() const` | 返回 EKF 引用。 |
| `armor_xyza_list` | `std::vector<Eigen::Vector4d> armor_xyza_list() const` | 根据状态计算所有装甲板的 xyz+angle 位置。 |
| `diverged` | `bool diverged() const` | 检查半径参数是否在物理合理范围内。 |
| `convergened` | `bool convergened()` | 连续多次更新未发散则返回 `true`。 |
| `checkinit` | `bool checkinit()` | 别名，同 `isinit`。 |

**公开成员变量：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `name` | `ArmorName` | 目标名称。 |
| `armor_type` | `ArmorType` | 装甲板类型。 |
| `priority` | `ArmorPriority` | 优先级。 |
| `jumped` | `bool` | 是否发生装甲板跳变。 |
| `last_id` | `int` | 上一次更新的装甲板 ID。 |
| `isinit` | `bool` | 是否已初始化。 |

**私有方法：** `update_ypda`、`h_armor_xyz`、`h_jacobian`。

---

## 目标跟踪 (`tracker.hpp`)

**命名空间：** `auto_aim`

### `class Tracker`

基于状态机的目标跟踪器：管理检测、暂时丢失、目标切换等逻辑。支持与 `omniperception` 多相机目标交接。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Tracker(const std::string &config_path, Solver &solver)` | 加载敌方颜色、检测/丢失阈值，初始状态为 `"lost"`。 |
| `state` | `std::string state() const` | 返回当前状态字符串。 |
| `track` | `std::list<Target> track(std::list<Armor> &armors, std::chrono::steady_clock::time_point t, bool use_enemy_color = true)` | 单相机跟踪主流程。 |
| `track` | `std::tuple<omniperception::DetectionResult, std::list<Target>> track(const std::vector<omniperception::DetectionResult> &detection_queue, std::list<Armor> &armors, std::chrono::steady_clock::time_point t, bool use_enemy_color = true)` | 多相机版本：结合 omniperception 进行目标切换。 |

**私有方法：** `state_machine(bool found)`、`set_target(...)`、`update_target(...)`。

---

## 投票器 (`voter.hpp`)

**命名空间：** `auto_aim`

### `class Voter`

装甲板属性（颜色、名称、类型）的简单投票计数器。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Voter()` | 初始化所有组合的计数为零。 |
| `vote` | `void vote(const Color color, const ArmorName name, const ArmorType type)` | 对指定组合投一票。 |
| `count` | `std::size_t count(const Color color, const ArmorName name, const ArmorType type)` | 返回指定组合的票数。 |

---

## YOLO 包装器 (`yolo.hpp`)

**命名空间：** `auto_aim`

### `class YOLOBase`（抽象基类）

| 成员 | 签名 | 说明 |
|------|------|------|
| `detect` | `virtual std::list<Armor> detect(const cv::Mat &img, int frame_count) = 0` | 在图像上执行检测。 |
| `postprocess` | `virtual std::list<Armor> postprocess(double scale, cv::Mat &output, const cv::Mat &bgr_img, int frame_count) = 0` | 网络输出后处理。 |

### `class YOLO`

多态包装器：根据 YAML 配置中的 `yolo_name` 自动实例化 `YOLOV8`、`YOLO11` 或 `YOLOV5` 后端。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `YOLO(const std::string &config_path, bool debug = true)` | 读取 `yolo_name` 并创建对应后端。 |

**公开接口：** 全部委托给内部 `std::unique_ptr<YOLOBase> yolo_`。

---

## YOLO11 (`yolos/yolo11.hpp`)

**命名空间：** `auto_aim`

### `class YOLO11` : `public YOLOBase`

基于 YOLO11 的装甲板检测器（OpenVINO，38 类，关键点输出）。支持 ROI 裁剪与可选的传统检测器精修。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `YOLO11(const std::string &config_path, bool debug)` | 加载模型、ROI 设置、OpenVINO 预处理。 |
| `detect` | `std::list<Armor> detect(const cv::Mat &bgr_img, int frame_count) override` | 执行检测。 |
| `postprocess` | `std::list<Armor> postprocess(double scale, cv::Mat &output, const cv::Mat &bgr_img, int frame_count) override` | 网络输出后处理：解析关键点、NMS、生成 `Armor` 列表。 |

**关键私有成员：** `class_num_ = 38`、`nms_threshold_ = 0.3`、`score_threshold_ = 0.7`、`use_roi_`、`detector_`（传统检测器指针）。

**私有方法：** `check_name`、`check_type`、`get_center_norm`、`parse`、`save`、`draw_detections`、`sort_keypoints`。

---

## YOLOv5 (`yolos/yolov5.hpp`)

**命名空间：** `auto_aim`

### `class YOLOV5` : `public YOLOBase`

基于 YOLOv5 的检测器（OpenVINO，13 类，颜色+数字分类，关键点输出）。支持传统角点精修。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `YOLOV5(const std::string &config_path, bool debug)` | 加载模型路径、ROI、`use_traditional_` 标志。 |
| `detect` | `std::list<Armor> detect(const cv::Mat &bgr_img, int frame_count) override` | 执行检测。 |
| `postprocess` | `std::list<Armor> postprocess(double scale, cv::Mat &output, const cv::Mat &bgr_img, int frame_count) override` | 后处理，含 `sigmoid`。 |

**关键私有成员：** `class_num_ = 13`、`use_traditional_`、`detector_`（`Detector` 友元）。

---

## YOLOv8 (`yolos/yolov8.hpp`)

**命名空间：** `auto_aim`

### `class YOLOV8` : `public YOLOBase`

基于 YOLOv8 的检测器（OpenVINO，2 类，关键点输出）。使用独立的 `Classifier` 进行装甲板数字分类，`Detector` 进行类型/名称校验。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `YOLOV8(const std::string &config_path, bool debug)` | 加载模型（输入尺寸 416×416）、ROI，初始化分类器与检测器。 |
| `detect` | `std::list<Armor> detect(const cv::Mat &bgr_img, int frame_count) override` | 执行检测。 |
| `postprocess` | `std::list<Armor> postprocess(double scale, cv::Mat &output, const cv::Mat &bgr_img, int frame_count) override` | 后处理。 |

**关键私有成员：** `classifier_`、`detector_`、`class_num_ = 2`、输入尺寸 416×416。

---

## TinyMPC 规划求解器 (`planner/tinympc/`)

> 该子模块为 `extern "C"` C API，由 `Planner` 内部调用，不直接暴露于 `auto_aim::` 命名空间。

### `types.hpp`

| 类型/结构 | 说明 |
|-----------|------|
| `tinytype`（`double`） | 求解器数值类型。 |
| `tinyMatrix`（`Eigen::Dynamic`） | 动态矩阵。 |
| `tinyVector`（`Eigen::Dynamic×1`） | 动态向量。 |
| `TinySolution` | `iter`、`solved`、`x`（`nx×N`）、`u`（`nu×N-1`）。 |
| `TinyCache` | `rho`、`Kinf`、`Pinf`、`Quu_inv`、`AmBKt`、`APf`、`BPf`、`C1`、`C2` 及灵敏度矩阵。 |
| `TinySettings` | 容差、最大迭代、约束开关、自适应 rho 设置。 |
| `TinyWorkspace` | 完整状态/输入矩阵、松弛/对偶变量、边界、残差、状态位。 |
| `TinySolver` | 持有指向 Solution、Settings、Cache、Workspace 的指针。 |

### `tiny_api.hpp`

| 函数 | 说明 |
|------|------|
| `tiny_setup(...)` | 分配并初始化求解器，预计算 Cache。 |
| `tiny_set_bound_constraints(...)` | 设置 box 约束。 |
| `tiny_set_cone_constraints(...)` | 设置二阶锥约束。 |
| `tiny_set_linear_constraints(...)` | 设置一般线性约束。 |
| `tiny_precompute_and_set_cache(...)` | Riccati 递归求解 `Kinf`/`Pinf`。 |
| `compute_sensitivity_matrices(...)` | 自适应 rho 的导数计算。 |
| `tiny_update_matrices_with_derivatives(...)` | 泰勒展开更新矩阵。 |
| `tiny_solve(...)` | 包装 `solve()`。 |
| `tiny_update_settings(...)` / `tiny_set_default_settings(...)` | 更新/重置求解器设置。 |
| `tiny_set_x0(...)` / `tiny_set_x_ref(...)` / `tiny_set_u_ref(...)` | 设置初始状态与参考轨迹。 |

### `admm.hpp`

| 函数 | 说明 |
|------|------|
| `solve(TinySolver *solver)` | 主 ADMM 求解循环：后向递推、前向 rollout、更新松弛/对偶/线性成本、终止判断、每 5 次迭代自适应 rho。 |
| `backward_pass_grad(...)` | Riccati 后向递推。 |
| `forward_pass(...)` | LQR 前向 rollout。 |
| `update_slack(...)` | 投影到边界/锥/线性约束。 |
| `update_dual(...)` | 增广拉格朗日乘子更新。 |
| `update_linear_cost(...)` | 更新 q/r/p（含参考轨迹与对偶/松弛项）。 |
| `termination_condition(...)` | 检查原/对偶残差。 |
| `project_soc(...)` | 二阶锥投影。 |
| `project_hyperplane(...)` | 超平面投影。 |

### `rho_benchmark.hpp`

| 结构/函数 | 说明 |
|-----------|------|
| `RhoAdapter` | 预分配残差计算与格式化矩阵。 |
| `RhoBenchmarkResult` | 计时、rho 前后值、残差/范数。 |
| `predict_rho(...)` | 根据归一化残差比预测新 rho。 |
| `update_matrices_with_derivatives(...)` | 将 delta_rho 应用到 Cache 矩阵。 |
| `benchmark_rho_adaptation(...)` | 完整自适应 rho 基准测试调用。 |

### `codegen.hpp`

| 函数 | 说明 |
|------|------|
| `tiny_codegen(...)` | 从已配置的求解器生成 C++ 数据文件。 |
| `tiny_codegen_with_sensitivity(...)` | 包含灵敏度矩阵的代码生成。 |

### `error.hpp`

- `ERROR_MSG(exit_code, format, ...)`：致命错误日志宏。

### `tiny_api_constants.hpp`

- 默认常量：`TINY_DEFAULT_ABS_PRI_TOL`（1e-3）、`TINY_DEFAULT_ABS_DUA_TOL`（1e-3）、`TINY_DEFAULT_MAX_ITER`（1000）及约束开关默认值。

---

*文档结束。*
