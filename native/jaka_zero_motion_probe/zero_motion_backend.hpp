#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace jaka_zero {

constexpr std::size_t kJointCount = 6;
using Joints = std::array<double, kJointCount>;

struct TimedResult {
  int code = 0;
  std::uint64_t duration_ns = 0;
};

struct PreflightState {
  std::string sdk_version;
  int fault_code = 0;
  std::string fault_message;
  bool powered = false;
  bool enabled = false;
  bool emergency_stop = false;
  bool collision = false;
  bool servo_move_active = false;
  int tool_id = -1;
  int user_frame_id = -1;
  Joints captured_joint_rad{};
};

struct EdgObservation {
  Joints joint_position_rad{};
  Joints joint_velocity_rad_s{};
};

struct FakeOptions {
  bool nonfinite_target = false;
  bool servo_active = false;
  bool entry_failure = false;
  bool servo_enable_failure = false;
  bool servo_disable_failure = false;
  bool exit_failure = false;
  bool logout_failure = false;
  std::uint64_t read_failure_cycle = 0;
  std::uint64_t command_failure_cycle = 0;
  std::uint64_t read_delay_ns = 0;
  std::uint64_t command_delay_ns = 0;
  double observed_joint_delta_rad = 0.0;
};

class Backend {
 public:
  virtual ~Backend() = default;
  virtual const char* name() const noexcept = 0;
  virtual TimedResult initialize_and_login(const std::string& robot_ip) = 0;
  virtual TimedResult preflight(PreflightState& state) = 0;
  virtual TimedResult precommand_check(PreflightState& state) = 0;
  virtual TimedResult enter_edg(const std::string& local_state_ip) = 0;
  virtual TimedResult read_edg(EdgObservation& observation) = 0;
#ifndef GATE3B_ENTRY_EXIT_ONLY
  virtual TimedResult enable_servo_move() = 0;
  virtual TimedResult command_invariant(const Joints& target) = 0;
  virtual TimedResult disable_servo_move() noexcept = 0;
  virtual bool servo_move_active() const noexcept = 0;
#endif
  virtual TimedResult exit_edg() noexcept = 0;
  virtual TimedResult logout() noexcept = 0;
  virtual bool logged_in() const noexcept = 0;
  virtual bool edg_active() const noexcept = 0;
  virtual std::string lifecycle_trace() const = 0;
};

std::unique_ptr<Backend> make_vendor_backend();
std::unique_ptr<Backend> make_fake_backend(const FakeOptions& options);

bool finite_joints(const Joints& joints) noexcept;
double maximum_absolute_delta(const Joints& left, const Joints& right) noexcept;

}  // namespace jaka_zero
