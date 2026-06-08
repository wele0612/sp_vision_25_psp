# `tools::` API 文档

> 本文档由代码分析自动生成，涵盖 `tools/` 目录下所有公开类、函数和结构体。
> `tools::` 是 sp_vision 框架的**工具层**，提供日志、数学工具、图像工具、卡尔曼滤波、线程安全容器、弹道解算等通用能力，供 `io::` 和 `tasks::` 各模块复用。

---

## 目录

1. [CRC 校验 (`crc.hpp`)](#crc-校验-crchpp)
2. [退出检测 (`exiter.hpp`)](#退出检测-exiterhpp)
3. [扩展卡尔曼滤波 (`extended_kalman_filter.hpp`)](#扩展卡尔曼滤波-extended_kalman_filterhpp)
4. [图像工具 (`img_tools.hpp`)](#图像工具-img_toolshpp)
5. [日志记录器 (`logger.hpp`)](#日志记录器-loggerhpp)
6. [数学工具 (`math_tools.hpp`)](#数学工具-math_toolshpp)
7. [PID 控制器 (`pid.hpp`)](#pid-控制器-pidhpp)
8. [曲线图绘制 (`plotter.hpp`)](#曲线图绘制-plotterhpp)
9. [RANSAC 正弦拟合 (`ransac_sine_fitter.hpp`)](#ransac-正弦拟合-ransac_sine_fitterhpp)
10. [视频录制器 (`recorder.hpp`)](#视频录制器-recorderhpp)
11. [线程池与有序队列 (`thread_pool.hpp`)](#线程池与有序队列-thread_poolhpp)
12. [线程安全队列 (`thread_safe_queue.hpp`)](#线程安全队列-thread_safe_queuehpp)
13. [弹道解算 (`trajectory.hpp`)](#弹道解算-trajectoryhpp)
14. [YAML 解析器 (`yaml.hpp`)](#yaml-解析器-yamlhpp)

---

## CRC 校验 (`crc.hpp`)

**命名空间：** `tools`

提供基于查表法的 CRC-8 / CRC-16 计算与校验。无类，纯自由函数。

| 函数 | 返回类型 | 参数 | 说明 |
|------|----------|------|------|
| `get_crc8` | `uint8_t` | `const uint8_t *data`, `uint16_t len` | 计算 `len` 字节数据的 CRC-8（`len` **不包含** CRC 字节本身）。 |
| `check_crc8` | `bool` | `const uint8_t *data`, `uint16_t len` | 校验 CRC-8（`len` **包含** CRC 字节）。 |
| `get_crc16` | `uint16_t` | `const uint8_t *data`, `uint32_t len` | 计算 `len` 字节数据的 CRC-16（`len` **不包含** CRC 字节）。 |
| `check_crc16` | `bool` | `const uint8_t *data`, `uint32_t len` | 校验 CRC-16（`len` **包含** CRC 字节）。 |

**实现细节：** 内部预计算 `CRC8_TABLE[256]`（初值 `0xff`）和 `CRC16_TABLE[256]`（初值 `0xffff`）。

---

## 退出检测 (`exiter.hpp`)

**命名空间：** `tools`

### `class Exiter`

单例风格的 SIGINT 信号处理器。安装 `SIGINT`（Ctrl-C）处理器后，可通过 `exit()` 标志查询是否收到退出信号。若构造超过一次会抛出 `std::runtime_error`。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Exiter()` | 安装 SIGINT 处理器。 |
| `exit` | `bool exit()` | 收到 `SIGINT` 后返回 `true`。 |

---

## 扩展卡尔曼滤波 (`extended_kalman_filter.hpp`)

**命名空间：** `tools`

### `class ExtendedKalmanFilter`

支持线性与非线性 predict/update 的扩展卡尔曼滤波器，内置 NIS/NEES 卡方一致性检验（带滑动窗口失败率统计）。

| 成员 | 签名 | 说明 |
|------|------|------|
| 默认构造 | `ExtendedKalmanFilter() = default` | — |
| 参数构造 | `ExtendedKalmanFilter(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0, std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add = [](a,b){return a+b;})` | 指定初始状态、协方差与状态加法算子。 |
| `predict` | `Eigen::VectorXd predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q)` | 线性预测：`x = F·x`，`P = F·P·Fᵀ + Q`。 |
| `predict` | `Eigen::VectorXd predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f)` | 非线性预测：先执行 `f(x)`，再线性传播协方差。 |
| `update` | `Eigen::VectorXd update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract = [](a,b){return a-b;})` | 线性更新，可自定义观测残差算子。 |
| `update` | `Eigen::VectorXd update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R, std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h, std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract = [](a,b){return a-b;})` | 非线性更新：使用观测函数 `h(x)`。 |

**重要公开成员变量：**

| 变量 | 类型 | 说明 |
|------|------|------|
| `x` | `Eigen::VectorXd` | 状态向量。 |
| `P` | `Eigen::MatrixXd` | 协方差矩阵。 |
| `data` | `std::map<std::string, double>` | 调试用指标：`residual_yaw`、`residual_pitch`、`residual_distance`、`residual_angle`、`nis`、`nees`、`nis_fail`、`nees_fail`、`recent_nis_failures`。 |
| `recent_nis_failures` | `std::deque<int>` | NIS 检验结果的滑动窗口（0/1）。 |
| `window_size` | `size_t` | 滑动窗口大小（默认 100）。 |
| `last_nis` | `double` | 最近一次 NIS 值。 |

---

## 图像工具 (`img_tools.hpp`)

**命名空间：** `tools`

OpenCV 辅助绘制函数，无类。

| 函数 | 返回类型 | 参数 | 说明 |
|------|----------|------|------|
| `draw_point` | `void` | `cv::Mat &img`, `const cv::Point &point`, `const cv::Scalar &color = {0,0,255}`, `int radius = 3` | 绘制实心圆点。 |
| `draw_points` | `void` | `cv::Mat &img`, `const std::vector<cv::Point> &points`, `const cv::Scalar &color = {0,0,255}`, `int thickness = 2` | 绘制多边形轮廓。 |
| `draw_points` | `void` | `cv::Mat &img`, `const std::vector<cv::Point2f> &points`, `const cv::Scalar &color = {0,0,255}`, `int thickness = 2` | `cv::Point2f` 重载，内部转 `cv::Point` 后绘制。 |
| `draw_text` | `void` | `cv::Mat &img`, `const std::string &text`, `const cv::Point &point`, `const cv::Scalar &color = {0,255,255}`, `double font_scale = 1.0`, `int thickness = 2` | 使用 `FONT_HERSHEY_SIMPLEX` 绘制文字。 |

---

## 日志记录器 (`logger.hpp`)

**命名空间：** `tools`

单例日志访问器，基于 `spdlog`，无类。

| 函数 | 返回类型 | 参数 | 说明 |
|------|----------|------|------|
| `logger` | `std::shared_ptr<spdlog::logger>` | — | 懒初始化双输出日志器：`logs/` 目录下的文件 + 彩色终端，等级均为 `debug`，`info` 时自动 flush。 |

---

## 数学工具 (`math_tools.hpp`)

**命名空间：** `tools`

纯数学/几何自由函数与一个模板函数。

| 函数 | 返回类型 | 参数 | 说明 |
|------|----------|------|------|
| `limit_rad` | `double` | `double angle` | 将角度归约到 `(-π, π]`。 |
| `eulers` | `Eigen::Vector3d` | `Eigen::Quaterniond q`, `int axis0`, `int axis1`, `int axis2`, `bool extrinsic = false` | 四元数 → 欧拉角，支持任意旋转轴顺序（`0=x, 1=y, 2=z`）。 |
| `eulers` | `Eigen::Vector3d` | `Eigen::Matrix3d R`, `int axis0`, `int axis1`, `int axis2`, `bool extrinsic = false` | 旋转矩阵 → 欧拉角（委托给四元数版本）。 |
| `rotation_matrix` | `Eigen::Matrix3d` | `const Eigen::Vector3d &ypr` | YPR 向量 → ZYX 旋转矩阵。 |
| `xyz2ypd` | `Eigen::Vector3d` | `const Eigen::Vector3d &xyz` | 笛卡尔坐标 `(x,y,z)` → 球坐标 `(yaw, pitch, distance)`。 |
| `xyz2ypd_jacobian` | `Eigen::MatrixXd` | `const Eigen::Vector3d &xyz` | `xyz2ypd` 对 `xyz` 的雅可比矩阵。 |
| `ypd2xyz` | `Eigen::Vector3d` | `const Eigen::Vector3d &ypd` | 球坐标 `(yaw, pitch, distance)` → 笛卡尔坐标 `(x,y,z)`。 |
| `ypd2xyz_jacobian` | `Eigen::MatrixXd` | `const Eigen::Vector3d &ypd` | `ypd2xyz` 对 `ypd` 的雅可比矩阵。 |
| `delta_time` | `double` | `const std::chrono::steady_clock::time_point &a`, `const std::chrono::steady_clock::time_point &b` | 计算 `a - b` 的秒数。 |
| `get_abs_angle` | `double` | `const Eigen::Vector2d &vec1`, `const Eigen::Vector2d &vec2` | 两二维向量的绝对夹角（`0 ~ π`）。 |
| `square` | `T` | `T const &a` | **模板函数**，返回 `a * a`。 |
| `limit_min_max` | `double` | `double input`, `double min`, `double max` | 将 `input` 钳制到 `[min, max]`。 |

---

## PID 控制器 (`pid.hpp`)

**命名空间：** `tools`

### `class PID`

离散 PID 控制器，支持角度误差环绕（通过 `limit_rad`）。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `PID(float dt, float kp, float ki, float kd, float max_out, float max_iout, bool angular = false)` | `dt`：采样周期；`angular`：为 `true` 时对误差使用 `limit_rad` 环绕。 |
| `calc` | `float calc(float set, float fdb)` | 计算 PID 输出；总输出钳制在 `±max_out_`，积分项钳制在 `±max_iout_`。 |

**调试用公开成员变量：**

| 变量 | 类型 | 说明 |
|------|------|------|
| `pout` | `float` | 最新比例项输出。 |
| `iout` | `float` | 最新积分项输出。 |
| `dout` | `float` | 最新微分项输出。 |

---

## 曲线图绘制 (`plotter.hpp`)

**命名空间：** `tools`

### `class Plotter`

UDP 发送器，将 `nlohmann::json` 序列化后发送到指定远程主机:端口，用于配合 PlotJuggler 等软件实时绘图。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Plotter(std::string host = "127.0.0.1", uint16_t port = 9870)` | 创建 UDP socket。 |
| 析构函数 | `~Plotter()` | 关闭 UDP socket。 |
| `plot` | `void plot(const nlohmann::json &json)` | 序列化 JSON 为字符串并通过 UDP 发送（线程安全，内部加锁）。 |

---

## RANSAC 正弦拟合 (`ransac_sine_fitter.hpp`)

**命名空间：** `tools`

### `class RansacSineFitter`

基于 RANSAC 的正弦曲线 `A·sin(ωt+φ)+C` 拟合器：随机采样角频率 `ω`，再用线性最小二乘求解幅值/相位/直流偏移。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `RansacSineFitter(int max_iterations, double threshold, double min_omega, double max_omega)` | 最大迭代次数、内点阈值、角频率搜索范围。 |
| `add_data` | `void add_data(double t, double v)` | 追加样本 `(t, v)`；若时间间隔 > 5s 则清空缓冲区。 |
| `fit` | `void fit()` | 执行 RANSAC 迭代，最优结果存入 `best_result_`；缓冲区上限 150 个样本。 |
| `sine_function` | `double sine_function(double t, double A, double omega, double phi, double C)` | 计算正弦模型值。 |

**结构体 `Result`：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `A` | `double` | 幅值。 |
| `omega` | `double` | 角频率。 |
| `phi` | `double` | 相位。 |
| `C` | `double` | 直流偏移。 |
| `inliers` | `int` | 最优模型的内点数量。 |

**公开成员变量：**

| 变量 | 类型 | 说明 |
|------|------|------|
| `best_result_` | `Result` | 最近一次 `fit()` 后的最优结果。 |

---

## 视频录制器 (`recorder.hpp`)

**命名空间：** `tools`

### `class Recorder`

异步视频 + IMU 四元数录制器。通过 `cv::VideoWriter` 写入视频，同时写入时间戳与四元数文本文件。后台消费者线程由 `ThreadSafeQueue` 驱动。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Recorder(double fps = 30)` | — |
| 析构函数 | `~Recorder()` | 发送停止信号、推入空帧以解除消费者阻塞、等待线程结束、释放写入器。 |
| `record` | `void record(const cv::Mat &img, const Eigen::Quaterniond &q, const std::chrono::steady_clock::time_point &timestamp)` | 按 `fps_` 节流后推入队列；首次收到有效图像时懒初始化写入器。 |

**私有结构体 `FrameData`：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `img` | `cv::Mat` | 图像帧。 |
| `q` | `Eigen::Quaterniond` | IMU 四元数。 |
| `timestamp` | `std::chrono::steady_clock::time_point` | 时间戳。 |

---

## 线程池与有序队列 (`thread_pool.hpp`)

**命名空间：** `tools`

### `struct Frame`

多线程推理/处理中传递的帧数据结构。

| 成员 | 类型 | 说明 |
|------|------|------|
| `id` | `int` | 帧序号。 |
| `img` | `cv::Mat` | 图像。 |
| `t` | `std::chrono::steady_clock::time_point` | 时间戳。 |
| `q` | `Eigen::Quaterniond` | IMU 四元数。 |
| `armors` | `std::list<auto_aim::Armor>` | 检测到的装甲板列表。 |

### 工厂函数

| 函数 | 返回类型 | 参数 | 说明 |
|------|----------|------|------|
| `create_yolo11s` | `std::vector<auto_aim::YOLO>` | `const std::string &config_path`, `int numebr`, `bool debug` | 批量创建 `numebr` 个 `auto_aim::YOLO` 实例。 |
| `create_yolov8s` | `std::vector<auto_aim::YOLO>` | 同上 | 同上（实现相同）。 |

### `class OrderedQueue`

线程安全的**按序**队列：允许乱序入队，但消费者严格按 `id` 从 `1` 开始顺序出队。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `OrderedQueue()` | — |
| 析构函数 | `~OrderedQueue()` | 清空队列与缓冲区并记录日志。 |
| `enqueue` | `void enqueue(const tools::Frame &item)` | 插入帧；若 `item.id == current_id_` 则推入主队列并排空缓存的后继帧。 |
| `dequeue` | `tools::Frame dequeue()` | 阻塞式出队（等待 `cond_var_`）。 |
| `try_dequeue` | `bool try_dequeue(tools::Frame &item)` | 非阻塞出队；空队列时返回 `false`。 |
| `get_size` | `size_t get_size()` | 返回 `main_queue_.size() + buffer_.size()`。 |

### `class ThreadPool`

经典固定大小线程池。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `ThreadPool(size_t num_threads)` | 创建工作线程。 |
| 析构函数 | `~ThreadPool()` | 设置停止标志、清空任务、唤醒所有线程、等待 join。 |
| `enqueue` | `void enqueue(F &&f)` | 将可调用对象加入任务队列；若已停止则抛出异常。 |

---

## 线程安全队列 (`thread_safe_queue.hpp`)

**命名空间：** `tools`

### `template <typename T, bool PopWhenFull = false> class ThreadSafeQueue`

有界线程安全 FIFO 队列。当 `PopWhenFull == true` 时，满队列会**弹出最旧元素**再压入新元素；否则调用 `full_handler_` 并丢弃新值。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `ThreadSafeQueue(size_t max_size, std::function<void(void)> full_handler = []{})` | — |
| `push` | `void push(const T &value)` | 压入值；队列行为由模板参数 `PopWhenFull` 控制。 |
| `pop` | `void pop(T &value)` | 阻塞式弹出（等待 `not_empty_condition_`）。 |
| `pop` | `T pop()` | 阻塞式弹出，按值返回。 |
| `front` | `T front()` | 阻塞式查看队首。 |
| `back` | `void back(T &value)` | 非阻塞查看队尾（空队列时记录错误日志）。 |
| `empty` | `bool empty()` | 是否为空。 |
| `clear` | `void clear()` | 弹出所有元素并唤醒等待者。 |

---

## 弹道解算 (`trajectory.hpp`)

**命名空间：** `tools`

### `struct Trajectory`

不考虑空气阻力，求解击中水平距离 `d`、高度 `h` 的目标所需的枪管俯仰角。选取飞行时间更短的解。

| 成员 | 签名 | 说明 |
|------|------|------|
| 构造函数 | `Trajectory(const double v0, const double d, const double h)` | `v0`：弹丸初速（m/s）。 |

**公开成员变量：**

| 变量 | 类型 | 说明 |
|------|------|------|
| `unsolvable` | `bool` | 判别式 < 0 时置为 `true`（目标超出射程）。 |
| `fly_time` | `double` | 飞行时间（秒），可解时有效。 |
| `pitch` | `double` | 所需俯仰角（弧度，正值为上抬），可解时有效。 |

---

## YAML 解析器 (`yaml.hpp`)

**命名空间：** `tools`

`yaml-cpp` 的薄封装，解析失败时记录致命日志并终止程序。

| 函数 | 返回类型 | 参数 | 说明 |
|------|----------|------|------|
| `load` | `YAML::Node` | `const std::string &path` | 加载 YAML 文件；`BadFile` 或 `ParserException` 时记录错误并 `exit(1)`。 |
| `read` | `T` | `const YAML::Node &yaml`, `const std::string &key` | 读取 `yaml[key].as<T>()`；键缺失时记录错误并 `exit(1)`。 |

---

*文档结束。*
