#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace jaka_readonly {

enum class Call : std::size_t {
  SdkVersion,
  Login,
  ActualJointPosition,
  ActualTcpPosition,
  RobotStatusSimple,
  RobotState,
  ServoState,
  EmergencyStop,
  CollisionState,
  RobotStatusCombined,
  ToolId,
  ToolData,
  UserFrameId,
  UserFrameData,
  ProgramState,
  ProgramInfo,
  Logout,
  Count,
};

constexpr std::size_t kCallCount = static_cast<std::size_t>(Call::Count);
extern const std::array<const char*, kCallCount> kCallNames;

struct Observation {
  bool attempted = false;
  int code = 0;
  std::uint64_t duration_ns = 0;
};

struct Batch {
  std::array<Observation, kCallCount> calls{};
};

struct State {
  std::string sdk_version;
  bool joint_position_available = false;
  std::array<double, 6> joint_position_rad{};
  bool joint_velocity_available = false;
  std::array<double, 6> joint_velocity_rad_s{};
  bool tcp_available = false;
  std::array<double, 6> tcp_mm_rpy_rad{};
  bool status_available = false;
  int fault_code = 0;
  std::string fault_message;
  bool powered = false;
  bool enabled = false;
  bool emergency_stop_available = false;
  bool emergency_stop = false;
  bool collision_available = false;
  bool collision = false;
  bool servo_state_available = false;
  bool servo_move_active = false;
  bool socket_connected_available = false;
  bool socket_connected = false;
  bool tool_id_available = false;
  int tool_id = -1;
  bool tool_data_available = false;
  std::array<double, 6> tool_mm_rpy_rad{};
  bool user_frame_id_available = false;
  int user_frame_id = -1;
  bool user_frame_data_available = false;
  std::array<double, 6> user_frame_mm_rpy_rad{};
  bool program_state_available = false;
  int program_state = -1;
  bool program_info_available = false;
  int program_motion_line = -1;
};

struct FakeOptions {
  std::uint64_t call_delay_ns = 0;
  std::uint64_t timeout_every = 0;
  std::uint64_t disconnect_after_fast_reads = 0;
  bool fail_login = false;
  bool fail_logout = false;
};

class Backend {
 public:
  virtual ~Backend() = default;
  virtual const char* name() const noexcept = 0;
  virtual void query_sdk_version(State& state, Batch& batch) = 0;
  virtual int connect(const std::string& robot_ip, Batch& batch) = 0;
  virtual void read_static(State& state, Batch& batch) = 0;
  virtual bool read_fast(State& state, Batch& batch) = 0;
  virtual void read_slow(State& state, Batch& batch) = 0;
  virtual int disconnect(Batch& batch) noexcept = 0;
  virtual bool connected() const noexcept = 0;
};

std::unique_ptr<Backend> make_vendor_backend();
std::unique_ptr<Backend> make_fake_backend(const FakeOptions& options);

}  // namespace jaka_readonly
