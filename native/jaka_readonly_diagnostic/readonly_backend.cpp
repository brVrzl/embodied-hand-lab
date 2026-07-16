#include "readonly_backend.hpp"

#include <JAKAZuRobot.h>
#include <jkerr.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace jaka_readonly {
namespace {
using Clock = std::chrono::steady_clock;

template <typename Function>
int measured(Batch& batch, Call call, Function&& function) {
  const auto start = Clock::now();
  const int code = function();
  const auto end = Clock::now();
  auto& observation = batch.calls[static_cast<std::size_t>(call)];
  observation.attempted = true;
  observation.code = code;
  observation.duration_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
  return code;
}

std::array<double, 6> pose_values(const CartesianPose& pose) {
  return {pose.tran.x, pose.tran.y, pose.tran.z, pose.rpy.rx, pose.rpy.ry, pose.rpy.rz};
}

class VendorBackend final : public Backend {
 public:
  ~VendorBackend() override {
    if (connected_) client_.login_out();
  }
  const char* name() const noexcept override { return "vendor_jaka_sdk_v2.2.7"; }
  void query_sdk_version(State& state, Batch& batch) override {
    std::array<char, 256> version{};
    if (measured(batch, Call::SdkVersion, [&] { return client_.get_sdk_version(version.data()); }) == ERR_SUCC)
      state.sdk_version = version.data();
  }
  int connect(const std::string& robot_ip, Batch& batch) override {
    const int code = measured(batch, Call::Login, [&] { return client_.login_in(robot_ip.c_str()); });
    connected_ = code == ERR_SUCC;
    return code;
  }
  void read_static(State& state, Batch& batch) override {
    int tool_id = -1;
    if (measured(batch, Call::ToolId, [&] { return client_.get_tool_id(&tool_id); }) == ERR_SUCC) {
      state.tool_id_available = true; state.tool_id = tool_id;
      CartesianPose tool{};
      if (measured(batch, Call::ToolData, [&] { return client_.get_tool_data(tool_id, &tool); }) == ERR_SUCC) {
        state.tool_data_available = true; state.tool_mm_rpy_rad = pose_values(tool);
      }
    }
    int user_id = -1;
    if (measured(batch, Call::UserFrameId, [&] { return client_.get_user_frame_id(&user_id); }) == ERR_SUCC) {
      state.user_frame_id_available = true; state.user_frame_id = user_id;
      CartesianPose user{};
      if (measured(batch, Call::UserFrameData, [&] { return client_.get_user_frame_data(user_id, &user); }) == ERR_SUCC) {
        state.user_frame_data_available = true; state.user_frame_mm_rpy_rad = pose_values(user);
      }
    }
    ProgramInfo info{};
    if (measured(batch, Call::ProgramInfo, [&] { return client_.get_program_info(&info); }) == ERR_SUCC) {
      state.program_info_available = true; state.program_motion_line = info.motion_line;
    }
  }
  bool read_fast(State& state, Batch& batch) override {
    JointValue joints{};
    const int joint_code = measured(batch, Call::ActualJointPosition,
                                    [&] { return client_.get_actual_joint_position(&joints); });
    if (joint_code == ERR_SUCC) {
      state.joint_position_available = true;
      std::copy(std::begin(joints.jVal), std::end(joints.jVal), state.joint_position_rad.begin());
    }
    CartesianPose tcp{};
    const int tcp_code = measured(batch, Call::ActualTcpPosition,
                                  [&] { return client_.get_actual_tcp_position(&tcp); });
    if (tcp_code == ERR_SUCC) { state.tcp_available = true; state.tcp_mm_rpy_rad = pose_values(tcp); }
    return joint_code == ERR_SUCC && tcp_code == ERR_SUCC;
  }
  void read_slow(State& state, Batch& batch) override {
    RobotStatus_simple simple{};
    if (measured(batch, Call::RobotStatusSimple, [&] { return client_.get_robot_status_simple(&simple); }) == ERR_SUCC) {
      state.status_available = true; state.fault_code = simple.errcode; state.fault_message = simple.errmsg;
      state.powered = simple.powered_on != 0; state.enabled = simple.enabled != 0;
    }
    RobotState robot_state{};
    if (measured(batch, Call::RobotState, [&] { return client_.get_robot_state(&robot_state); }) == ERR_SUCC) {
      state.emergency_stop_available = true; state.emergency_stop = robot_state.estoped != 0;
      state.powered = robot_state.poweredOn != 0; state.enabled = robot_state.servoEnabled != 0;
    }
    BOOL value = FALSE;
    if (measured(batch, Call::ServoState, [&] { return client_.is_in_servomove(&value); }) == ERR_SUCC) {
      state.servo_state_available = true; state.servo_move_active = value != 0;
    }
    if (measured(batch, Call::EmergencyStop, [&] { return client_.is_in_estop(&value); }) == ERR_SUCC) {
      state.emergency_stop_available = true; state.emergency_stop = value != 0;
    }
    if (measured(batch, Call::CollisionState, [&] { return client_.is_in_collision(&value); }) == ERR_SUCC) {
      state.collision_available = true; state.collision = value != 0;
    }
    ProgramState program_state{};
    if (measured(batch, Call::ProgramState, [&] { return client_.get_program_state(&program_state); }) == ERR_SUCC) {
      state.program_state_available = true; state.program_state = static_cast<int>(program_state);
    }
    RobotStatus combined{};
    if (measured(batch, Call::RobotStatusCombined, [&] { return client_.get_robot_status(&combined); }) == ERR_SUCC) {
      state.socket_connected_available = true; state.socket_connected = combined.is_socket_connect != 0;
      state.joint_velocity_available = true;
      for (std::size_t i = 0; i < 6; ++i) state.joint_velocity_rad_s[i] = combined.robot_monitor_data.jointMonitorData[i].instVel;
    }
  }
  int disconnect(Batch& batch) noexcept override {
    if (!connected_) return ERR_SUCC;
    const int code = measured(batch, Call::Logout, [&] { return client_.login_out(); });
    connected_ = false;
    return code;
  }
  bool connected() const noexcept override { return connected_; }
 private:
  JAKAZuRobot client_;
  bool connected_ = false;
};

class FakeBackend final : public Backend {
 public:
  explicit FakeBackend(FakeOptions options) : options_(options) {}
  ~FakeBackend() override = default;
  const char* name() const noexcept override { return "fake_lifecycle_only"; }
  void query_sdk_version(State& state, Batch& batch) override {
    call(batch, Call::SdkVersion); state.sdk_version = "fake-not-hardware";
  }
  int connect(const std::string&, Batch& batch) override {
    const int code = options_.fail_login ? -3 : call(batch, Call::Login);
    batch.calls[static_cast<std::size_t>(Call::Login)].code = code;
    connected_ = code == ERR_SUCC; return code;
  }
  void read_static(State& state, Batch& batch) override {
    call(batch, Call::ToolId); state.tool_id_available = true; state.tool_id = 0;
    call(batch, Call::UserFrameId); state.user_frame_id_available = true; state.user_frame_id = 0;
    call(batch, Call::ProgramInfo); state.program_info_available = true; state.program_motion_line = 0;
  }
  bool read_fast(State& state, Batch& batch) override {
    ++fast_reads_;
    int code = call(batch, Call::ActualJointPosition);
    if (options_.disconnect_after_fast_reads && fast_reads_ >= options_.disconnect_after_fast_reads) code = -61;
    batch.calls[static_cast<std::size_t>(Call::ActualJointPosition)].code = code;
    const int tcp_code = call(batch, Call::ActualTcpPosition);
    if (code == ERR_SUCC && tcp_code == ERR_SUCC) {
      state.joint_position_available = true; state.tcp_available = true; return true;
    }
    return false;
  }
  void read_slow(State& state, Batch& batch) override {
    call(batch, Call::RobotStatusSimple); state.status_available = true; state.powered = true; state.enabled = true;
    call(batch, Call::RobotState); state.emergency_stop_available = true;
    call(batch, Call::ServoState); state.servo_state_available = true;
    call(batch, Call::EmergencyStop); call(batch, Call::CollisionState); state.collision_available = true;
    call(batch, Call::ProgramState); state.program_state_available = true; state.program_state = 0;
    call(batch, Call::RobotStatusCombined); state.joint_velocity_available = true;
  }
  int disconnect(Batch& batch) noexcept override {
    if (!connected_) return ERR_SUCC;
    int code = call(batch, Call::Logout);
    if (options_.fail_logout) { code = -3; batch.calls[static_cast<std::size_t>(Call::Logout)].code = code; }
    connected_ = false; return code;
  }
  bool connected() const noexcept override { return connected_; }
 private:
  int call(Batch& batch, Call kind) noexcept {
    const auto start = Clock::now();
    if (options_.call_delay_ns) std::this_thread::sleep_for(std::chrono::nanoseconds(options_.call_delay_ns));
    const auto end = Clock::now();
    auto& observation = batch.calls[static_cast<std::size_t>(kind)];
    observation.attempted = true; observation.duration_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count());
    ++calls_;
    observation.code = options_.timeout_every && calls_ % options_.timeout_every == 0 ? -61 : ERR_SUCC;
    return observation.code;
  }
  FakeOptions options_;
  std::uint64_t calls_ = 0, fast_reads_ = 0;
  bool connected_ = false;
};
}  // namespace

const std::array<const char*, kCallCount> kCallNames{
    "sdk_version", "login_in_combined_connection", "actual_joint_position", "actual_tcp_position",
    "robot_status_simple", "robot_state", "servo_state", "emergency_stop", "collision_state",
    "robot_status_combined_deprecated", "tool_id", "tool_data", "user_frame_id", "user_frame_data",
    "program_state", "program_info", "login_out"};

std::unique_ptr<Backend> make_vendor_backend() { return std::make_unique<VendorBackend>(); }
std::unique_ptr<Backend> make_fake_backend(const FakeOptions& options) { return std::make_unique<FakeBackend>(options); }
}  // namespace jaka_readonly
