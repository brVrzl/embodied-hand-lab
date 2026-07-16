#include "zero_motion_backend.hpp"

#include <JAKAZuRobot.h>
#include <jkerr.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <thread>
#include <vector>

namespace jaka_zero {
namespace {
using Clock = std::chrono::steady_clock;

template <typename Function>
TimedResult timed(Function&& function) noexcept {
  const auto start = Clock::now();
  const int code = function();
  return {code, static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count())};
}

class VendorBackend final : public Backend {
 public:
  VendorBackend() { trace_.reserve(8); }
  ~VendorBackend() override {
#ifndef GATE3B_ENTRY_EXIT_ONLY
    if (servo_move_active_) client_.servo_move_enable(FALSE);
#endif
    if (edg_active_) client_.edg_init(FALSE, local_state_ip_.c_str());
    if (logged_in_) client_.login_out();
  }
  const char* name() const noexcept override { return "vendor_jaka_sdk_v2.2.7"; }
  TimedResult initialize_and_login(const std::string& robot_ip) override {
    trace_.push_back("login");
    auto result = timed([&] { return client_.login_in(robot_ip.c_str()); });
    logged_in_ = result.code == ERR_SUCC;
    return result;
  }
  TimedResult preflight(PreflightState& state) override {
    trace_.push_back("preflight");
    return collect_preflight(state);
  }
  TimedResult precommand_check(PreflightState& state) override {
    trace_.push_back("precommand_check");
    return collect_preflight(state);
  }
 private:
  TimedResult collect_preflight(PreflightState& state) {
    const auto start = Clock::now();
    std::array<char, 256> version{};
    int code = client_.get_sdk_version(version.data());
    if (code == ERR_SUCC) state.sdk_version = version.data();
    RobotStatus_simple status{};
    if (code == ERR_SUCC) code = client_.get_robot_status_simple(&status);
    if (code == ERR_SUCC) {
      state.fault_code = status.errcode; state.fault_message = status.errmsg;
      state.powered = status.powered_on != 0; state.enabled = status.enabled != 0;
    }
    RobotState robot_state{};
    if (code == ERR_SUCC) code = client_.get_robot_state(&robot_state);
    if (code == ERR_SUCC) state.emergency_stop = robot_state.estoped != 0;
    BOOL value = FALSE;
    if (code == ERR_SUCC) code = client_.is_in_estop(&value);
    if (code == ERR_SUCC) state.emergency_stop = state.emergency_stop || value != 0;
    if (code == ERR_SUCC) code = client_.is_in_collision(&value);
    if (code == ERR_SUCC) state.collision = value != 0;
    if (code == ERR_SUCC) code = client_.is_in_servomove(&value);
    if (code == ERR_SUCC) state.servo_move_active = value != 0;
    if (code == ERR_SUCC) code = client_.get_tool_id(&state.tool_id);
    if (code == ERR_SUCC) code = client_.get_user_frame_id(&state.user_frame_id);
    JointValue joints{};
    if (code == ERR_SUCC) code = client_.get_actual_joint_position(&joints);
    if (code == ERR_SUCC) std::copy(std::begin(joints.jVal), std::end(joints.jVal), state.captured_joint_rad.begin());
    return {code, static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count())};
  }
 public:
  TimedResult enter_edg(const std::string& local_state_ip) override {
    trace_.push_back("enter_edg"); local_state_ip_ = local_state_ip;
    auto result = timed([&] { return client_.edg_init(TRUE, local_state_ip.c_str()); });
    edg_active_ = result.code == ERR_SUCC; return result;
  }
  TimedResult read_edg(EdgObservation& observation) override {
    EDGState state{};
    auto result = timed([&] { return client_.edg_get_stat(&state); });
    if (result.code == ERR_SUCC) {
      std::copy(std::begin(state.jointVal.jVal), std::end(state.jointVal.jVal), observation.joint_position_rad.begin());
      std::copy(std::begin(state.jointVel.jVel), std::end(state.jointVel.jVel), observation.joint_velocity_rad_s.begin());
    }
    return result;
  }
#ifndef GATE3B_ENTRY_EXIT_ONLY
  TimedResult enable_servo_move() override {
    trace_.push_back("enable_servo_move");
    const auto start = Clock::now();
    int code = client_.servo_move_enable(TRUE);
    if (code == ERR_SUCC) {
      servo_move_active_ = true;
      BOOL active = FALSE;
      code = client_.is_in_servomove(&active);
      if (code == ERR_SUCC && active == FALSE) code = -1;
    }
    return {code, static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count())};
  }
  TimedResult command_invariant(const Joints& target) override {
    JointValue joints{};
    std::copy(target.begin(), target.end(), std::begin(joints.jVal));
    return timed([&] { return client_.edg_servo_j(&joints, ABS, 1); });
  }
  TimedResult disable_servo_move() noexcept override {
    if (!servo_move_active_) return {};
    trace_.push_back("disable_servo_move");
    auto result = timed([&] { return client_.servo_move_enable(FALSE); });
    servo_move_active_ = false;
    return result;
  }
  bool servo_move_active() const noexcept override { return servo_move_active_; }
#endif
  TimedResult exit_edg() noexcept override {
    if (!edg_active_) return {};
    trace_.push_back("exit_edg"); auto result = timed([&] { return client_.edg_init(FALSE, local_state_ip_.c_str()); });
    edg_active_ = false; return result;
  }
  TimedResult logout() noexcept override {
    if (!logged_in_) return {};
    trace_.push_back("logout"); auto result = timed([&] { return client_.login_out(); });
    logged_in_ = false; return result;
  }
  bool logged_in() const noexcept override { return logged_in_; }
  bool edg_active() const noexcept override { return edg_active_; }
  std::string lifecycle_trace() const override {
    std::ostringstream output;
    for (std::size_t i = 0; i < trace_.size(); ++i) { if (i) output << ','; output << trace_[i]; }
    return output.str();
  }
 private:
  JAKAZuRobot client_;
  std::string local_state_ip_;
  std::vector<std::string> trace_;
  bool logged_in_ = false, edg_active_ = false;
#ifndef GATE3B_ENTRY_EXIT_ONLY
  bool servo_move_active_ = false;
#endif
};

class FakeBackend final : public Backend {
 public:
  explicit FakeBackend(FakeOptions options) : options_(options) { trace_.reserve(8); }
  ~FakeBackend() override = default;
  const char* name() const noexcept override { return "fake_lifecycle_only"; }
  TimedResult initialize_and_login(const std::string&) override { trace_.push_back("login"); logged_in_ = true; return {}; }
  TimedResult preflight(PreflightState& state) override {
    trace_.push_back("preflight"); state.sdk_version = "fake-not-hardware"; state.powered = true; state.enabled = true;
    state.servo_move_active = options_.servo_active; state.tool_id = 0; state.user_frame_id = 0;
    if (options_.nonfinite_target) state.captured_joint_rad[2] = std::numeric_limits<double>::quiet_NaN();
    return {};
  }
  TimedResult precommand_check(PreflightState& state) override {
    trace_.push_back("precommand_check"); state.sdk_version = "fake-not-hardware"; state.powered = true; state.enabled = true;
    state.servo_move_active = options_.servo_active; state.tool_id = 0; state.user_frame_id = 0;
    if (options_.nonfinite_target) state.captured_joint_rad[2] = std::numeric_limits<double>::quiet_NaN();
    return {};
  }
  TimedResult enter_edg(const std::string&) override {
    trace_.push_back("enter_edg"); if (options_.entry_failure) return {-57, 0}; edg_active_ = true; return {};
  }
  TimedResult read_edg(EdgObservation& observation) override {
    ++read_cycle_; delay(options_.read_delay_ns);
    if (options_.read_failure_cycle && read_cycle_ >= options_.read_failure_cycle) return {-61, options_.read_delay_ns};
    observation.joint_position_rad = target_;
    observation.joint_position_rad[0] += options_.observed_joint_delta_rad;
    return {0, options_.read_delay_ns};
  }
#ifndef GATE3B_ENTRY_EXIT_ONLY
  TimedResult enable_servo_move() override {
    trace_.push_back("enable_servo_move");
    if (options_.servo_enable_failure) return {-3, 0};
    servo_move_active_ = true;
    return {};
  }
  TimedResult command_invariant(const Joints& target) override {
    ++command_cycle_; delay(options_.command_delay_ns);
    if (options_.command_failure_cycle && command_cycle_ >= options_.command_failure_cycle) return {-3, options_.command_delay_ns};
    target_ = target; return {0, options_.command_delay_ns};
  }
  TimedResult disable_servo_move() noexcept override {
    if (!servo_move_active_) return {};
    trace_.push_back("disable_servo_move");
    servo_move_active_ = false;
    return {options_.servo_disable_failure ? -3 : 0, 0};
  }
  bool servo_move_active() const noexcept override { return servo_move_active_; }
#endif
  TimedResult exit_edg() noexcept override {
    if (!edg_active_) return {};
    trace_.push_back("exit_edg"); edg_active_ = false; return {options_.exit_failure ? -57 : 0, 0};
  }
  TimedResult logout() noexcept override {
    if (!logged_in_) return {};
    trace_.push_back("logout"); logged_in_ = false; return {options_.logout_failure ? -3 : 0, 0};
  }
  bool logged_in() const noexcept override { return logged_in_; }
  bool edg_active() const noexcept override { return edg_active_; }
  std::string lifecycle_trace() const override {
    std::ostringstream output;
    for (std::size_t i = 0; i < trace_.size(); ++i) { if (i) output << ','; output << trace_[i]; }
    return output.str();
  }
 private:
  static void delay(std::uint64_t ns) noexcept { if (ns) std::this_thread::sleep_for(std::chrono::nanoseconds(ns)); }
  FakeOptions options_;
  Joints target_{};
  std::vector<std::string> trace_;
  std::uint64_t read_cycle_ = 0, command_cycle_ = 0;
  bool logged_in_ = false, edg_active_ = false;
#ifndef GATE3B_ENTRY_EXIT_ONLY
  bool servo_move_active_ = false;
#endif
};
}  // namespace

std::unique_ptr<Backend> make_vendor_backend() { return std::make_unique<VendorBackend>(); }
std::unique_ptr<Backend> make_fake_backend(const FakeOptions& options) { return std::make_unique<FakeBackend>(options); }

bool finite_joints(const Joints& joints) noexcept {
  return std::all_of(joints.begin(), joints.end(), [](double value) { return std::isfinite(value); });
}

double maximum_absolute_delta(const Joints& left, const Joints& right) noexcept {
  double result = 0.0;
  for (std::size_t i = 0; i < left.size(); ++i) result = std::max(result, std::abs(left[i] - right[i]));
  return result;
}
}  // namespace jaka_zero
