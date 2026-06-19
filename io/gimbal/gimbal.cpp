#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  // Optional serial parameters with defaults
  uint32_t baudrate = 9600;
  if (yaml["baudrate"]) baudrate = yaml["baudrate"].as<uint32_t>();
  tools::logger()->info("[Gimbal] Using Baudrate {}", baudrate);

  serial::bytesize_t bytesize = serial::eightbits;
  if (yaml["bytesize"]) {
    int bs = yaml["bytesize"].as<int>();
    switch (bs) {
      case 5:
        bytesize = serial::fivebits;
        break;
      case 6:
        bytesize = serial::sixbits;
        break;
      case 7:
        bytesize = serial::sevenbits;
        break;
      case 8:
        bytesize = serial::eightbits;
        break;
      default:
        tools::logger()->warn("[Gimbal] Invalid bytesize: {}, using default 8", bs);
        break;
    }
  }

  serial::parity_t parity = serial::parity_none;
  if (yaml["parity"]) {
    std::string p = yaml["parity"].as<std::string>();
    if (p == "none")
      parity = serial::parity_none;
    else if (p == "odd")
      parity = serial::parity_odd;
    else if (p == "even")
      parity = serial::parity_even;
    else if (p == "mark")
      parity = serial::parity_mark;
    else if (p == "space")
      parity = serial::parity_space;
    else
      tools::logger()->warn("[Gimbal] Invalid parity: {}, using default none", p);
  }

  serial::stopbits_t stopbits = serial::stopbits_one;
  if (yaml["stopbits"]) {
    int sb = yaml["stopbits"].as<int>();
    switch (sb) {
      case 1:
        stopbits = serial::stopbits_one;
        break;
      case 2:
        stopbits = serial::stopbits_two;
        break;
      default:
        tools::logger()->warn("[Gimbal] Invalid stopbits: {}, using default 1", sb);
        break;
    }
  }

  serial::flowcontrol_t flowcontrol = serial::flowcontrol_none;
  if (yaml["flowcontrol"]) {
    std::string fc = yaml["flowcontrol"].as<std::string>();
    if (fc == "none")
      flowcontrol = serial::flowcontrol_none;
    else if (fc == "software")
      flowcontrol = serial::flowcontrol_software;
    else if (fc == "hardware")
      flowcontrol = serial::flowcontrol_hardware;
    else
      tools::logger()->warn("[Gimbal] Invalid flowcontrol: {}, using default none", fc);
  }

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(baudrate);
    serial_.setBytesize(bytesize);
    serial_.setParity(parity);
    serial_.setStopbits(stopbits);
    serial_.setFlowcontrol(flowcontrol);

    if (yaml["timeout_ms"]) {
      uint32_t timeout_ms = yaml["timeout_ms"].as<uint32_t>();
      serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
      serial_.setTimeout(timeout);
    }

    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  // thread_ = std::thread(&Gimbal::read_thread, this);
  thread_ = std::thread(&Gimbal::read_thread_fsm_safe, this);
  // Safer UART process logic

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc, float forward_vel, float leftward_vel, uint8_t spintop_level)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.forward_vel = forward_vel;
  tx_data_.leftward_vel = leftward_vel;
  tx_data_.spintop_level = spintop_level;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread_fsm_safe()
{
  // Behaves exactly the same as read_thread. Except:

  // COM use a safer FSM model to process.
  // It will process COM messages byte-by-byte.

  // It has 3 states:
  // Init: IDLE
  // if in IDLE receive 'S': -> RECEIVED_S

  // if in RECEIVED_S receive 'P' -> RECEIVED_SP
  // else -> IDLE (If not P but S, go to RECEIVED_S)

  // if in RECEIVED_SP received N bytes (n dependes on packet length):
  // Do CRC. If CRC success, process message. Go to IDLE.

  tools::logger()->info("[Gimbal] read_thread_fsm_safe started.");
  int error_count = 0;

  enum class State { IDLE, RECEIVED_S, RECEIVED_SP };
  State state = State::IDLE;
  std::chrono::steady_clock::time_point rx_timestamp;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      state = State::IDLE;
      continue;
    }

    if (state == State::IDLE) {
      uint8_t byte = 0;
      if (!read(&byte, 1)) {
        error_count++;
        continue;
      }
      if (byte == 'S') {
        rx_data_.head[0] = byte;
        state = State::RECEIVED_S;
      }
    } else if (state == State::RECEIVED_S) {
      uint8_t byte = 0;
      if (!read(&byte, 1)) {
        error_count++;
        state = State::IDLE;
        continue;
      }
      if (byte == 'P') {
        rx_data_.head[1] = byte;
        rx_timestamp = std::chrono::steady_clock::now();
        state = State::RECEIVED_SP;
      } else if (byte == 'S') {
        rx_data_.head[0] = byte;
        // stay in RECEIVED_S
      } else {
        state = State::IDLE;
      }
    } else if (state == State::RECEIVED_SP) {
      size_t body_size = sizeof(GimbalToVision) - sizeof(rx_data_.head);
      if (!read(
            reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head), body_size)) {
        error_count++;
        state = State::IDLE;
        continue;
      }

      if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
        tools::logger()->debug("[Gimbal] CRC16 check failed.");
        state = State::IDLE;
        continue;
      // }else{
      //   tools::logger()->debug("[Gimbal] CRC16 check success.");
      }

      error_count = 0;
      Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
      queue_.push({q, rx_timestamp});

      std::lock_guard<std::mutex> lock(mutex_);

      state_.yaw = rx_data_.yaw;
      state_.yaw_vel = rx_data_.yaw_vel;
      state_.pitch = rx_data_.pitch;
      state_.pitch_vel = rx_data_.pitch_vel;
      state_.bullet_speed = rx_data_.bullet_speed;
      state_.bullet_count = rx_data_.bullet_count;
      state_.is_enemy_red = (rx_data_.aim_color == 1);

      switch (rx_data_.mode) {
        case 0:
          mode_ = GimbalMode::IDLE;
          break;
        case 1:
          mode_ = GimbalMode::AUTO_AIM;
          break;
        case 2:
          mode_ = GimbalMode::SMALL_BUFF;
          break;
        case 3:
          mode_ = GimbalMode::BIG_BUFF;
          break;
        default:
          mode_ = GimbalMode::IDLE;
          tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
          break;
      }

      state = State::IDLE;
    }
  }

  tools::logger()->info("[Gimbal] read_thread_fsm_safe stopped.");
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }else{
      tools::logger()->info("[Gimbal] CRC16 check PASSED.");
    }

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;
    state_.is_enemy_red = (rx_data_.aim_color == 1);

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io