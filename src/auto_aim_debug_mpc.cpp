#include <fmt/core.h>

#include <atomic>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Detector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Planner planner(config_path);

  tools::ThreadSafeQueue<std::optional<auto_aim::Target>, true> target_queue(1);
  target_queue.push(std::nullopt);

  std::atomic<bool> quit = false;
  auto plan_thread = std::thread([&]() {
    auto t0 = std::chrono::steady_clock::now();
    uint16_t last_bullet_count = 0;
    int plan_frame_count = 0;
    auto plan_last_time = std::chrono::steady_clock::now();

    while (!quit) {
      auto target = target_queue.front();
      auto gs = gimbal.state();

      // 如果云台状态中的敌方颜色与当前跟踪器使用的不一致，进行校正
      auto expected_enemy_color = gs.is_enemy_red ? auto_aim::Color::red : auto_aim::Color::blue;
      if (tracker.enemy_color() != expected_enemy_color) {
        tracker.set_enemy_color(expected_enemy_color);
        tools::logger()->info(
          "[plan_thread] Correct enemy color to {} (from gimbal state)",
          auto_aim::COLORS[expected_enemy_color]);
      }

      auto plan = planner.plan(target, gs.bullet_speed);

      gimbal.send(
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch, plan.pitch_vel,
        plan.pitch_acc);

      auto fired = gs.bullet_count > last_bullet_count;
      last_bullet_count = gs.bullet_count;

      nlohmann::json data;
      data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

      data["gimbal_yaw"] = gs.yaw;
      data["gimbal_yaw_vel"] = gs.yaw_vel;
      data["gimbal_pitch"] = gs.pitch;
      data["gimbal_pitch_vel"] = gs.pitch_vel;

      data["target_yaw"] = plan.target_yaw;
      data["target_pitch"] = plan.target_pitch;

      data["plan_yaw"] = plan.yaw;
      data["plan_yaw_vel"] = plan.yaw_vel;
      data["plan_yaw_acc"] = plan.yaw_acc;

      data["plan_pitch"] = plan.pitch;
      data["plan_pitch_vel"] = plan.pitch_vel;
      data["plan_pitch_acc"] = plan.pitch_acc;

      data["fire"] = plan.fire ? 1 : 0;
      data["fired"] = fired ? 1 : 0;

      data["ammo_speed"] = gs.bullet_speed;

      if (target.has_value()) {
        data["target_z"] = target->ekf_x()[4];   //z
        data["target_vz"] = target->ekf_x()[5];  //vz
      }

      if (target.has_value()) {
        data["w"] = target->ekf_x()[7];
      } else {
        data["w"] = 0.0;
      }

      plotter.plot(data);

      plan_frame_count++;
      if (plan_frame_count % 100 == 0) {
        auto now = std::chrono::steady_clock::now();
        auto dt = tools::delta_time(now, plan_last_time);
        auto fps = 100.0 / dt;
        tools::logger()->info("[plan_thread] FPS: {:.2f}", fps);
        plan_last_time = now;
      }

      std::this_thread::sleep_for(3ms);
    }
  });

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  int main_frame_count = 0;
  auto main_last_time = std::chrono::steady_clock::now();

  auto ms = [](auto a, auto b) -> double {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };

  while (!exiter.exit()) {
    auto loop_start = std::chrono::steady_clock::now();

    auto t0 = std::chrono::steady_clock::now();
    camera.read(img, t);
    auto t1 = std::chrono::steady_clock::now();

    auto q = gimbal.q(t);
    auto t2 = std::chrono::steady_clock::now();

    solver.set_R_gimbal2world(q);
    auto t3 = std::chrono::steady_clock::now();

    auto armors = yolo.detect(img);
    // auto armors = detector.detect(img, main_frame_count);
    auto t4 = std::chrono::steady_clock::now();

    auto targets = tracker.track(armors, t);
    auto t5 = std::chrono::steady_clock::now();

    if (!targets.empty())
      target_queue.push(targets.front());
    else
      target_queue.push(std::nullopt);
    auto t6 = std::chrono::steady_clock::now();

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, image_points, {0, 255, 0});
      }

      Eigen::Vector4d aim_xyza = planner.debug_xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      tools::draw_points(img, image_points, {0, 0, 255});
    }
    auto t7 = std::chrono::steady_clock::now();

    // cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    // cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    auto t8 = std::chrono::steady_clock::now();
    if (key == 'q') break;

    
    main_frame_count++;
    if (main_frame_count % 50 == 0) {
      tools::logger()->info(
      "[Timing] read:{:.2f} q:{:.2f} setR:{:.2f} detect:{:.2f} track:{:.2f} push:{:.2f} viz:{:.2f} show:{:.2f} total:{:.2f} ms",
      ms(t0, t1), ms(t1, t2), ms(t2, t3), ms(t3, t4), ms(t4, t5),
      ms(t5, t6), ms(t6, t7), ms(t7, t8), ms(loop_start, t8));


      auto now = std::chrono::steady_clock::now();
      auto dt = tools::delta_time(now, main_last_time);
      auto fps = 50.0 / dt;
      tools::logger()->info("[main_thread] FPS: {:.2f}", fps);
      main_last_time = now;
    }
  }

  quit = true;
  if (plan_thread.joinable()) plan_thread.join();
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

  return 0;
}