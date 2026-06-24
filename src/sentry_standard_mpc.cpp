#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>
#include <sentry_msg/msg/sentry_msg.hpp>

#include "io/camera.hpp"
#include "io/dm_imu/dm_imu.hpp"
#include "io/ros2/ros2.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  io::ROS2 ros2;


  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Detector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  // auto_buff::Buff_Detector buff_detector(config_path);
  // auto_buff::Solver buff_solver(config_path);
  // auto_buff::SmallTarget buff_small_target;
  // auto_buff::BigTarget buff_big_target;
  // auto_buff::Aimer buff_aimer(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  std::atomic<bool> quit = false;

  std::atomic<io::GimbalMode> mode{io::GimbalMode::IDLE};
  auto last_mode{io::GimbalMode::IDLE};

  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;
    int plan_frame_count = 0;
    auto plan_last_time = std::chrono::steady_clock::now();

    float forward_vel = 0;
    float leftward_vel = 0;

    sentry_msg::msg::SentryMsg data;

    while (!quit) {
      if (!target_queue.empty() && mode == io::GimbalMode::AUTO_AIM) {
        auto target = target_queue.front();
        auto gs = gimbal.state();
        auto plan = planner.plan(target, gs.bullet_speed);

        auto expected_enemy_color = gs.is_enemy_red ? auto_aim::Color::red : auto_aim::Color::blue;
        if (tracker.enemy_color() != expected_enemy_color) {
          tracker.set_enemy_color(expected_enemy_color);
          tools::logger()->info(
            "[plan_thread] Correct enemy color to {} (from gimbal state)",
            auto_aim::COLORS[expected_enemy_color]);
        }

        // read twist from ros2
        auto twist = ros2.subscribe_twist();
        if (twist.size() == 2) {
          forward_vel = twist[0];
          leftward_vel = twist[1];
        } else {
          forward_vel = 0;
          leftward_vel = 0;
        }

        data.self_hp = gs.self_HP;
        data.match_started = gs.match_started;
        ros2.publish(data);

        gimbal.send(
          plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
          plan.pitch_acc, forward_vel, leftward_vel, 0);

        std::this_thread::sleep_for(4ms);
      } else {
        auto gs = gimbal.state();
        // read twist from ros2
        auto twist = ros2.subscribe_twist();
        if (twist.size() == 2) {
          forward_vel = twist[0];
          leftward_vel = twist[1];
        } else {
          forward_vel = 0;
          leftward_vel = 0;
        }

        data.self_hp = gs.self_HP;
        data.match_started = gs.match_started;
        ros2.publish(data);

        gimbal.send(
          false, false, 0, 0, 0, 0, 0, 0, forward_vel, leftward_vel, 0);

        std::this_thread::sleep_for(4ms);
      }

      plan_frame_count++;
      if (plan_frame_count % 200 == 0) {
        auto now = std::chrono::steady_clock::now();
        auto dt = tools::delta_time(now, plan_last_time);
        auto fps = 200.0 / dt;
        tools::logger()->info("[plan_thread] FPS: {:.2f}", fps);
        plan_last_time = now;
      }
    }
  });

  int main_frame_count = 0;
  auto main_last_time = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    mode = gimbal.mode();

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", gimbal.str(mode));
      last_mode = mode.load();
    }

    camera.read(img, t);
    auto q = gimbal.q(t);
    auto gs = gimbal.state();
    //recorder.record(img, q, t);
    solver.set_R_gimbal2world(q);

    /// 自瞄
    if (mode.load() == io::GimbalMode::AUTO_AIM) {
      auto armors = yolo.detect(img);
      // auto armors = detector.detect(img, main_frame_count);
      auto targets = tracker.track(armors, t);
      if (!targets.empty())
        target_queue.push(targets.front());
      else
        target_queue.push(std::nullopt);
    // }

    /// 打符
    // else if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) {
    //   buff_solver.set_R_gimbal2world(q);

    //   auto power_runes = buff_detector.detect(img);

    //   buff_solver.solve(power_runes);

    //   auto_aim::Plan buff_plan;
    //   if (mode.load() == io::GimbalMode::SMALL_BUFF) {
    //     buff_small_target.get_target(power_runes, t);
    //     auto target_copy = buff_small_target;
    //     buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
    //   } else if (mode.load() == io::GimbalMode::BIG_BUFF) {
    //     buff_big_target.get_target(power_runes, t);
    //     auto target_copy = buff_big_target;
    //     buff_plan = buff_aimer.mpc_aim(target_copy, t, gs, true);
    //   }
    //   gimbal.send(
    //     buff_plan.control, buff_plan.fire, buff_plan.yaw, buff_plan.yaw_vel, buff_plan.yaw_acc,
    //     buff_plan.pitch, buff_plan.pitch_vel, buff_plan.pitch_acc);

    } 

    main_frame_count++;
    if (main_frame_count % 200 == 0) {
      auto now = std::chrono::steady_clock::now();
      auto dt = tools::delta_time(now, main_last_time);
      auto fps = 200.0 / dt;
      tools::logger()->info("[main_thread] FPS: {:.2f}", fps);
      main_last_time = now;
    }
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  return 0;
}