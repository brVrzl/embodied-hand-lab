#include <JAKAZuRobot.h>
#include <jkerr.h>

#include "joint_servo_resampler.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <fcntl.h>
#include <glob.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sched.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {
using jaka_servo::JointServoResampler;
using jaka_servo::ResampledServoPoint;
using jaka_servo::kJointLower;
using jaka_servo::kJointUpper;
using jaka_servo::validate_manufacturer_joint_position_limits;

constexpr std::uint32_t kTargetMagic = 0x4A544754;
constexpr std::uint32_t kStatusMagic = 0x4A535441;
constexpr std::uint16_t kWireVersion = 1;
constexpr std::uint64_t kPeriodNs = 8'000'000;
constexpr std::size_t kMaximumSamples = 250'000;
constexpr const char* kHardwareAck = "I_ACKNOWLEDGE_JAKA_HARDWARE_RISK";
constexpr const char* kShadowAck = "I_ACKNOWLEDGE_JAKA_COMMAND_SHADOW_NO_EDG";
constexpr const char* kBoundedTeleopAck = "I_ACKNOWLEDGE_BOUNDED_TELEDEX_JAKA_MOTION";
constexpr const char* kQuestShadowAck = "I_AUTHORIZE_P2_QUEST_JAKA_COMMAND_SHADOW";
constexpr const char* kQuestMotionAck = "I_AUTHORIZE_P4_LIVE_QUEST_JAKA_TELEOPERATION";
constexpr const char* kE1ZeroMotionAck = "I_AUTHORIZE_E1_ZERO_MOTION_EDG_RESAMPLER";
constexpr double kProbeMaximumVelocityRadS = 0.005;
constexpr double kProbeMaximumAccelerationRadS2 = 0.02;

enum class Mode {
  DryRun, StateRead, ZeroMotion, MinimalMotion,
  CommandShadowDryRun, CommandShadow, BoundedTeleopDryRun, BoundedTeleop,
  JointShadowDryRun, JointShadow, JointTeleopDryRun, JointTeleop,
  JointZeroMotionDryRun, JointZeroMotion
};
enum class State : std::uint16_t {
  Disconnected, Connecting, Connected, Armed, EdgReady, Holding, Running,
  ControlledStop, Fault, Shutdown
};
enum class TargetKind : std::uint16_t { Heartbeat, HoldCurrent, JointPosition, CartesianPose, Stop };
constexpr std::uint32_t kTargetAllowMotion = 1u << 0;
constexpr std::uint32_t kFrameStartupTcpRelative = 2;
constexpr std::uint32_t kStatusConnected = 1u << 0;
constexpr std::uint32_t kStatusEdgActive = 1u << 1;
constexpr std::uint32_t kStatusHolding = 1u << 2;
constexpr std::uint32_t kStatusHasTarget = 1u << 3;
constexpr std::uint32_t kStatusAccepted = 1u << 4;
constexpr std::uint32_t kStatusRejected = 1u << 5;
constexpr std::uint32_t kStatusTargetWarning = 1u << 6;
constexpr std::uint32_t kStatusOutputAccelerationHold = 1u << 7;
constexpr std::uint32_t kStatusOutputAccelerationRecovered = 1u << 8;
constexpr std::uint32_t kStatusControlledBraking = 1u << 9;
constexpr std::uint32_t kStatusStoppedReady = 1u << 10;
constexpr std::uint32_t kStatusMeasuredStateRefresh = 1u << 11;

#pragma pack(push, 1)
struct TargetPacket {
  std::uint32_t magic;
  std::uint16_t version;
  std::uint16_t kind;
  std::uint32_t flags;
  std::uint32_t frame_id;
  std::uint64_t sequence;
  std::uint64_t source_capture_ns;
  std::uint64_t local_receive_ns;
  std::uint64_t processing_ns;
  std::uint64_t dispatch_ns;
  double payload[8];
  std::uint32_t crc32;
};
struct StatusPacket {
  std::uint32_t magic;
  std::uint16_t version;
  std::uint16_t state;
  std::uint32_t flags;
  std::uint64_t last_sequence;
  std::uint64_t loop_sequence;
  std::uint64_t worker_monotonic_ns;
  std::uint64_t command_monotonic_ns;
  std::uint64_t observation_monotonic_ns;
  double joint_position_rad[6];
  std::int32_t error_code;
  std::uint32_t crc32;
};
#pragma pack(pop)
static_assert(sizeof(TargetPacket) == 124);
static_assert(sizeof(StatusPacket) == 108);

std::atomic<bool> g_stop{false};
void signal_handler(int) { g_stop.store(true, std::memory_order_relaxed); }

std::uint64_t now_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}

std::uint64_t wall_now_ns() {
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1'000'000'000ULL + static_cast<std::uint64_t>(ts.tv_nsec);
}

std::uint32_t crc32(const void* data, std::size_t size) {
  std::uint32_t crc = 0xFFFFFFFFu;
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

void require_sdk(errno_t code, const char* operation) {
  if (code != ERR_SUCC) throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(code));
}

double shortest_joint_difference(double target, double measured) {
  return std::remainder(target - measured, 2.0 * M_PI);
}

struct Options {
  Mode mode = Mode::DryRun;
  std::string robot_ip;
  std::string edg_state_ip = "0.0.0.0";
  std::string target_socket = "/tmp/jaka_servo_target.sock";
  std::string status_socket;
  std::string metrics_file;
  std::string emitted_points_file;
  std::string cycle_telemetry_file;
  std::string acknowledgement;
  double duration_s = 5.0;
  int expected_tool_id = 0;
  int expected_user_frame_id = 0;
  int probe_joint = 0;
  double probe_delta_rad = 0.001;
  double probe_motion_s = 2.0;
  bool hardware = false;
  std::uint64_t warning_ns = 40'000'000;
  std::uint64_t hold_ns = 100'000'000;
  std::uint64_t stop_ns = 500'000'000;
  std::uint64_t fatal_ns = 2'000'000'000;
  std::uint32_t max_consecutive_overruns = 50;
  std::uint64_t fake_connect_delay_ns = 0;
  std::uint64_t fake_edg_delay_ns = 0;
  std::uint64_t fake_start_delay_once_ns = 0;
  std::uint64_t fake_read_delay_ns = 0;
  std::uint64_t fake_write_delay_ns = 0;
  std::uint64_t fake_fail_after = 0;
  std::array<double, 6> fake_initial_joints_rad{};
  std::array<double, 6> fake_post_edg_joint_offset_rad{};
  std::array<double, 3> workspace_min_mm{};
  std::array<double, 3> workspace_max_mm{};
  bool workspace_min_set = false;
  bool workspace_max_set = false;
  double relative_translation_limit_m = 0.015;
  double relative_rotation_limit_rad = 0.06981317007977318;
  double joint_velocity_limit_rad_s = 0.03;
  double joint_acceleration_limit_rad_s2 = 0.15;
  double joint_jerk_limit_rad_s3 = 1.5;
  double joint_soft_margin_rad = 0.08726646259971647;
  double maximum_ik_step_rad = 0.10;
  double maximum_jacobian_condition = 200.0;
  double excessive_tracking_error_abort_rad = 0.35;
  std::uint32_t excessive_tracking_error_consecutive_cycles = 2;
  double startup_alignment_tolerance_rad = 0.001;
  double maximum_output_joint_velocity_rad_s = M_PI;
  std::array<double, 6> maximum_output_joint_velocity_rad_s_per_joint{
      M_PI, M_PI, M_PI, M_PI, M_PI, M_PI};
  bool maximum_output_joint_velocity_scalar_set = false;
  bool maximum_output_joint_velocity_per_joint_set = false;
  double diagnostic_joint_acceleration_boundary_rad_s2 = 4.0 * M_PI;
  bool abort_on_diagnostic_acceleration_boundary = false;
  double maximum_output_joint_acceleration_rad_s2 = 4.0 * M_PI;
  double output_joint_jerk_limit_rad_s3 = 20.0 * M_PI;
  bool recover_output_acceleration_transition = false;
  std::uint64_t output_acceleration_hold_degraded_ns = 250'000'000;
  std::uint64_t output_acceleration_hold_hard_stop_ns = 2'000'000'000;
  std::uint32_t maximum_consecutive_output_acceleration_hold_cycles = 250;
  std::uint32_t startup_timing_grace_cycles = 25;
  bool monitor_controller_health_each_cycle = false;
};

bool is_shadow_mode(Mode mode) {
  return mode == Mode::CommandShadowDryRun || mode == Mode::CommandShadow;
}

bool is_bounded_mode(Mode mode) {
  return mode == Mode::BoundedTeleopDryRun || mode == Mode::BoundedTeleop;
}

bool is_joint_shadow_mode(Mode mode) {
  return mode == Mode::JointShadowDryRun || mode == Mode::JointShadow;
}

bool is_joint_teleop_mode(Mode mode) {
  return mode == Mode::JointTeleopDryRun || mode == Mode::JointTeleop;
}

bool is_joint_zero_motion_mode(Mode mode) {
  return mode == Mode::JointZeroMotionDryRun || mode == Mode::JointZeroMotion;
}

bool is_joint_mode(Mode mode) {
  return is_joint_shadow_mode(mode) || is_joint_teleop_mode(mode) ||
         is_joint_zero_motion_mode(mode);
}

bool is_stream_mode(Mode mode) {
  return is_shadow_mode(mode) || is_bounded_mode(mode) || is_joint_mode(mode);
}

bool uses_fake_backend(Mode mode) {
  return mode == Mode::DryRun || mode == Mode::CommandShadowDryRun ||
         mode == Mode::BoundedTeleopDryRun || mode == Mode::JointShadowDryRun ||
         mode == Mode::JointTeleopDryRun || mode == Mode::JointZeroMotionDryRun;
}

bool is_connected_mode(Mode mode) {
  return !uses_fake_backend(mode);
}

std::array<double, 3> parse_xyz(const std::string& value) {
  std::array<double, 3> result{};
  std::size_t start = 0;
  for (std::size_t i = 0; i < result.size(); ++i) {
    const auto end = value.find(',', start);
    if ((i < 2 && end == std::string::npos) || (i == 2 && end != std::string::npos))
      throw std::runtime_error("workspace bounds must be x,y,z");
    result[i] = std::stod(value.substr(start, end - start));
    start = end + 1;
  }
  return result;
}

std::array<double, 6> parse_six(const std::string& value, const char* label) {
  std::array<double, 6> result{};
  std::size_t start = 0;
  for (std::size_t i = 0; i < result.size(); ++i) {
    const auto end = value.find(',', start);
    if ((i < result.size() - 1 && end == std::string::npos) ||
        (i == result.size() - 1 && end != std::string::npos))
      throw std::runtime_error(std::string(label) + " must contain six comma-separated values");
    result[i] = std::stod(value.substr(start, end - start));
    if (!std::isfinite(result[i])) throw std::runtime_error(std::string(label) + " must be finite");
    start = end + 1;
  }
  return result;
}

std::string value_after(int& index, int argc, char** argv) {
  if (++index >= argc) throw std::runtime_error(std::string("missing value after ") + argv[index - 1]);
  return argv[index];
}

Options parse_options(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--mode") {
      const auto v = value_after(i, argc, argv);
      if (v == "dry-run") o.mode = Mode::DryRun;
      else if (v == "state-read") o.mode = Mode::StateRead;
      else if (v == "zero-motion") o.mode = Mode::ZeroMotion;
      else if (v == "minimal-motion") o.mode = Mode::MinimalMotion;
      else if (v == "command-shadow-dry-run") o.mode = Mode::CommandShadowDryRun;
      else if (v == "command-shadow") o.mode = Mode::CommandShadow;
      else if (v == "bounded-teleop-dry-run") o.mode = Mode::BoundedTeleopDryRun;
      else if (v == "bounded-teleop") o.mode = Mode::BoundedTeleop;
      else if (v == "joint-shadow-dry-run") o.mode = Mode::JointShadowDryRun;
      else if (v == "joint-shadow") o.mode = Mode::JointShadow;
      else if (v == "joint-teleop-dry-run") o.mode = Mode::JointTeleopDryRun;
      else if (v == "joint-teleop") o.mode = Mode::JointTeleop;
      else if (v == "joint-zero-motion-dry-run") o.mode = Mode::JointZeroMotionDryRun;
      else if (v == "joint-zero-motion") o.mode = Mode::JointZeroMotion;
      else throw std::runtime_error("invalid --mode");
    } else if (a == "--robot-ip") o.robot_ip = value_after(i, argc, argv);
    else if (a == "--edg-state-ip") o.edg_state_ip = value_after(i, argc, argv);
    else if (a == "--target-socket") o.target_socket = value_after(i, argc, argv);
    else if (a == "--status-socket") o.status_socket = value_after(i, argc, argv);
    else if (a == "--metrics-file") o.metrics_file = value_after(i, argc, argv);
    else if (a == "--emitted-points-file") o.emitted_points_file = value_after(i, argc, argv);
    else if (a == "--cycle-telemetry-file") o.cycle_telemetry_file = value_after(i, argc, argv);
    else if (a == "--duration-s") o.duration_s = std::stod(value_after(i, argc, argv));
    else if (a == "--expected-tool-id") o.expected_tool_id = std::stoi(value_after(i, argc, argv));
    else if (a == "--expected-user-frame-id") o.expected_user_frame_id = std::stoi(value_after(i, argc, argv));
    else if (a == "--probe-joint") o.probe_joint = std::stoi(value_after(i, argc, argv));
    else if (a == "--probe-delta-rad") o.probe_delta_rad = std::stod(value_after(i, argc, argv));
    else if (a == "--probe-motion-s") o.probe_motion_s = std::stod(value_after(i, argc, argv));
    else if (a == "--hardware") o.hardware = true;
    else if (a == "--acknowledgement") o.acknowledgement = value_after(i, argc, argv);
    else if (a == "--warning-ms") o.warning_ns = static_cast<std::uint64_t>(std::stod(value_after(i, argc, argv)) * 1e6);
    else if (a == "--hold-ms") o.hold_ns = static_cast<std::uint64_t>(std::stod(value_after(i, argc, argv)) * 1e6);
    else if (a == "--controlled-stop-ms") o.stop_ns = static_cast<std::uint64_t>(std::stod(value_after(i, argc, argv)) * 1e6);
    else if (a == "--fatal-timeout-ms") o.fatal_ns = static_cast<std::uint64_t>(std::stod(value_after(i, argc, argv)) * 1e6);
    else if (a == "--fake-connect-delay-us") o.fake_connect_delay_ns = std::stoull(value_after(i, argc, argv)) * 1000;
    else if (a == "--fake-edg-delay-us") o.fake_edg_delay_ns = std::stoull(value_after(i, argc, argv)) * 1000;
    else if (a == "--fake-start-delay-once-us") o.fake_start_delay_once_ns = std::stoull(value_after(i, argc, argv)) * 1000;
    else if (a == "--fake-read-delay-us") o.fake_read_delay_ns = std::stoull(value_after(i, argc, argv)) * 1000;
    else if (a == "--fake-write-delay-us") o.fake_write_delay_ns = std::stoull(value_after(i, argc, argv)) * 1000;
    else if (a == "--fake-fail-after") o.fake_fail_after = std::stoull(value_after(i, argc, argv));
    else if (a == "--fake-initial-joints-rad") o.fake_initial_joints_rad = parse_six(value_after(i, argc, argv), "fake initial joints");
    else if (a == "--fake-post-edg-joint-offset-rad") o.fake_post_edg_joint_offset_rad = parse_six(value_after(i, argc, argv), "fake post-EDG joint offset");
    else if (a == "--workspace-min-mm") { o.workspace_min_mm = parse_xyz(value_after(i, argc, argv)); o.workspace_min_set = true; }
    else if (a == "--workspace-max-mm") { o.workspace_max_mm = parse_xyz(value_after(i, argc, argv)); o.workspace_max_set = true; }
    else if (a == "--relative-translation-limit-m") o.relative_translation_limit_m = std::stod(value_after(i, argc, argv));
    else if (a == "--relative-rotation-limit-rad") o.relative_rotation_limit_rad = std::stod(value_after(i, argc, argv));
    else if (a == "--joint-velocity-limit-rad-s") o.joint_velocity_limit_rad_s = std::stod(value_after(i, argc, argv));
    else if (a == "--joint-acceleration-limit-rad-s2") o.joint_acceleration_limit_rad_s2 = std::stod(value_after(i, argc, argv));
    else if (a == "--joint-jerk-limit-rad-s3") o.joint_jerk_limit_rad_s3 = std::stod(value_after(i, argc, argv));
    else if (a == "--joint-soft-margin-rad") o.joint_soft_margin_rad = std::stod(value_after(i, argc, argv));
    else if (a == "--maximum-ik-step-rad") o.maximum_ik_step_rad = std::stod(value_after(i, argc, argv));
    else if (a == "--maximum-jacobian-condition") o.maximum_jacobian_condition = std::stod(value_after(i, argc, argv));
    else if (a == "--excessive-tracking-error-abort-rad") o.excessive_tracking_error_abort_rad = std::stod(value_after(i, argc, argv));
    else if (a == "--excessive-tracking-error-consecutive-cycles") o.excessive_tracking_error_consecutive_cycles = static_cast<std::uint32_t>(std::stoul(value_after(i, argc, argv)));
    else if (a == "--startup-alignment-tolerance-rad") o.startup_alignment_tolerance_rad = std::stod(value_after(i, argc, argv));
    else if (a == "--maximum-output-joint-velocity-rad-s") {
      o.maximum_output_joint_velocity_rad_s = std::stod(value_after(i, argc, argv));
      o.maximum_output_joint_velocity_scalar_set = true;
    }
    else if (a == "--maximum-output-joint-velocity-rad-s-per-joint") {
      o.maximum_output_joint_velocity_rad_s_per_joint =
          parse_six(value_after(i, argc, argv),
                    "per-joint output velocity boundaries");
      o.maximum_output_joint_velocity_per_joint_set = true;
    }
    else if (a == "--diagnostic-joint-acceleration-boundary-rad-s2") o.diagnostic_joint_acceleration_boundary_rad_s2 = std::stod(value_after(i, argc, argv));
    else if (a == "--abort-on-diagnostic-acceleration-boundary") o.abort_on_diagnostic_acceleration_boundary = true;
    else if (a == "--maximum-output-joint-acceleration-rad-s2") o.maximum_output_joint_acceleration_rad_s2 = std::stod(value_after(i, argc, argv));
    else if (a == "--output-joint-jerk-limit-rad-s3") o.output_joint_jerk_limit_rad_s3 = std::stod(value_after(i, argc, argv));
    else if (a == "--recover-output-acceleration-transition") o.recover_output_acceleration_transition = true;
    else if (a == "--output-acceleration-hold-degraded-ms") o.output_acceleration_hold_degraded_ns = static_cast<std::uint64_t>(std::stod(value_after(i, argc, argv)) * 1e6);
    else if (a == "--output-acceleration-hold-hard-stop-ms") o.output_acceleration_hold_hard_stop_ns = static_cast<std::uint64_t>(std::stod(value_after(i, argc, argv)) * 1e6);
    else if (a == "--maximum-consecutive-output-acceleration-hold-cycles") o.maximum_consecutive_output_acceleration_hold_cycles = static_cast<std::uint32_t>(std::stoul(value_after(i, argc, argv)));
    else if (a == "--startup-timing-grace-cycles") o.startup_timing_grace_cycles = static_cast<std::uint32_t>(std::stoul(value_after(i, argc, argv)));
    else if (a == "--monitor-controller-health-each-cycle") o.monitor_controller_health_each_cycle = true;
    else if (a == "--help") {
      std::cout << "jaka_servo_worker --mode dry-run|state-read|zero-motion|minimal-motion|command-shadow-dry-run|command-shadow|bounded-teleop-dry-run|bounded-teleop|joint-shadow-dry-run|joint-shadow|joint-teleop-dry-run|joint-teleop|joint-zero-motion-dry-run|joint-zero-motion [options]\n";
      std::cout << "  --maximum-output-joint-velocity-rad-s VALUE (legacy scalar)\n";
      std::cout << "  --maximum-output-joint-velocity-rad-s-per-joint J1,J2,J3,J4,J5,J6\n";
      std::cout << "  --output-joint-jerk-limit-rad-s3 VALUE (project-selected transition shaper)\n";
      std::cout << "  --recover-output-acceleration-transition\n";
      std::cout << "  --diagnostic-joint-acceleration-boundary-rad-s2 VALUE (shared recoverable boundary)\n";
      std::cout << "  --maximum-output-joint-acceleration-rad-s2 VALUE (native final hard boundary)\n";
      std::exit(0);
    } else throw std::runtime_error("unknown option: " + a);
  }
  if (!(o.duration_s > 0.0 && o.duration_s <= 2000.0)) throw std::runtime_error("duration must be in (0, 2000] s");
  if (!(o.warning_ns < o.hold_ns && o.hold_ns < o.stop_ns && o.stop_ns < o.fatal_ns))
    throw std::runtime_error("stale thresholds must be strictly increasing");
  if (is_connected_mode(o.mode)) {
    const char* expected_ack = is_joint_zero_motion_mode(o.mode) ? kE1ZeroMotionAck :
                               is_joint_shadow_mode(o.mode) ? kQuestShadowAck :
                               is_joint_teleop_mode(o.mode) ? kQuestMotionAck :
                               is_shadow_mode(o.mode) ? kShadowAck :
                               is_bounded_mode(o.mode) ? kBoundedTeleopAck : kHardwareAck;
    if (!o.hardware || o.acknowledgement != expected_ack || o.robot_ip.empty())
      throw std::runtime_error("connected mode requires --hardware, --robot-ip, and its exact acknowledgement");
  }
  if (is_bounded_mode(o.mode) || is_shadow_mode(o.mode)) {
    if (!(o.relative_translation_limit_m > 0.0 && o.relative_translation_limit_m <= 0.0200001))
      throw std::runtime_error("relative translation limit must be in (0, 0.020] m");
    if (!(o.relative_rotation_limit_rad > 0.0 && o.relative_rotation_limit_rad <= 0.08726646259971647 + 1e-12))
      throw std::runtime_error("relative rotation limit must be in (0, 5 deg]");
    if (!(o.joint_velocity_limit_rad_s > 0.0 && o.joint_velocity_limit_rad_s <= 0.10 &&
          o.joint_acceleration_limit_rad_s2 > 0.0 && o.joint_acceleration_limit_rad_s2 <= 0.50 &&
          o.joint_jerk_limit_rad_s3 > 0.0 && o.joint_jerk_limit_rad_s3 <= 2.0 &&
          o.joint_soft_margin_rad >= 0.08726646259971647 && o.maximum_ik_step_rad > 0.0 &&
          o.maximum_ik_step_rad <= 0.10 && o.maximum_jacobian_condition >= 10.0 &&
          o.maximum_jacobian_condition <= 500.0))
      throw std::runtime_error("bounded joint limits are outside commissioning envelope");
    if (is_bounded_mode(o.mode) && o.duration_s > 10.0)
      throw std::runtime_error("bounded TeleDex session may not exceed 10 seconds");
  }
  if (is_joint_mode(o.mode)) {
    if (o.maximum_output_joint_velocity_scalar_set &&
        o.maximum_output_joint_velocity_per_joint_set)
      throw std::runtime_error(
          "scalar and per-joint output velocity boundaries are mutually exclusive");
    if (!o.maximum_output_joint_velocity_per_joint_set)
      o.maximum_output_joint_velocity_rad_s_per_joint.fill(
          o.maximum_output_joint_velocity_rad_s);
    else
      o.maximum_output_joint_velocity_rad_s = *std::max_element(
          o.maximum_output_joint_velocity_rad_s_per_joint.begin(),
          o.maximum_output_joint_velocity_rad_s_per_joint.end());
    const bool valid_per_joint_output_velocity = std::all_of(
        o.maximum_output_joint_velocity_rad_s_per_joint.begin(),
        o.maximum_output_joint_velocity_rad_s_per_joint.end(),
        [](double value) {
          return std::isfinite(value) && value > 0.0 &&
                 value <= M_PI + 1e-12;
        });
    if (!(std::isfinite(o.excessive_tracking_error_abort_rad) &&
          o.excessive_tracking_error_abort_rad >= 0.25 &&
          o.excessive_tracking_error_abort_rad <= 1.0 &&
          o.excessive_tracking_error_consecutive_cycles >= 1 &&
          o.excessive_tracking_error_consecutive_cycles <= 10 &&
          std::isfinite(o.startup_alignment_tolerance_rad) &&
          o.startup_alignment_tolerance_rad > 0.0 &&
          o.startup_alignment_tolerance_rad <= 0.01 &&
          std::isfinite(o.maximum_output_joint_velocity_rad_s) &&
          o.maximum_output_joint_velocity_rad_s > 0.0 &&
          o.maximum_output_joint_velocity_rad_s <= M_PI + 1e-12 &&
          valid_per_joint_output_velocity &&
          std::isfinite(o.diagnostic_joint_acceleration_boundary_rad_s2) &&
          o.diagnostic_joint_acceleration_boundary_rad_s2 > 0.0 &&
          std::isfinite(o.maximum_output_joint_acceleration_rad_s2) &&
          o.maximum_output_joint_acceleration_rad_s2 > 0.0 &&
          std::isfinite(o.output_joint_jerk_limit_rad_s3) &&
          o.output_joint_jerk_limit_rad_s3 > 0.0 &&
          o.maximum_output_joint_acceleration_rad_s2 + 1e-12 >=
              o.diagnostic_joint_acceleration_boundary_rad_s2 &&
          o.output_acceleration_hold_degraded_ns > 0 &&
          o.output_acceleration_hold_hard_stop_ns >
              o.output_acceleration_hold_degraded_ns &&
          o.maximum_consecutive_output_acceleration_hold_cycles >= 2 &&
          o.maximum_consecutive_output_acceleration_hold_cycles <= 10'000 &&
          o.startup_timing_grace_cycles >= 1 &&
          o.startup_timing_grace_cycles <= 1'000 &&
          !(o.recover_output_acceleration_transition &&
            o.abort_on_diagnostic_acceleration_boundary)))
      throw std::runtime_error("joint adapter fault-containment settings are invalid");
    if (!o.emitted_points_file.empty() && !uses_fake_backend(o.mode))
      throw std::runtime_error("emitted point recording is fake-backend/offline only");
    if (!o.cycle_telemetry_file.empty() && o.duration_s > 65.0)
      throw std::runtime_error("cycle telemetry is bounded to sessions of at most 65 seconds");
  }
  if (o.mode == Mode::MinimalMotion) {
    if (o.probe_joint < 0 || o.probe_joint >= 6 || std::abs(o.probe_delta_rad) > 0.002 || std::abs(o.probe_delta_rad) < 1e-9)
      throw std::runtime_error("minimal probe requires joint 0..5 and |delta| in (0, 0.002] rad");
    if (o.probe_motion_s < 1.0) throw std::runtime_error("minimal probe motion duration must be at least 1 s");
    const double peak_velocity = 1.875 * std::abs(o.probe_delta_rad) / o.probe_motion_s;
    const double peak_acceleration = 5.774 * std::abs(o.probe_delta_rad) / (o.probe_motion_s * o.probe_motion_s);
    if (peak_velocity > kProbeMaximumVelocityRadS || peak_acceleration > kProbeMaximumAccelerationRadS2)
      throw std::runtime_error("minimal probe exceeds conservative analytic velocity/acceleration bounds");
    if (!o.workspace_min_set || !o.workspace_max_set || !std::equal(o.workspace_min_mm.begin(), o.workspace_min_mm.end(), o.workspace_max_mm.begin(),
                                        [](double low, double high) { return std::isfinite(low) && std::isfinite(high) && low < high; }))
      throw std::runtime_error("minimal probe requires valid explicit --workspace-min-mm and --workspace-max-mm bounds");
  }
  return o;
}

struct CartesianState {
  std::array<double, 3> position_mm{};
  std::array<double, 3> rpy_rad{};
};

struct ControllerHealth {
  std::uint64_t monotonic_ns = 0;
  std::uint64_t wall_ns = 0;
  int status_sdk_return_code = 0;
  int estop_sdk_return_code = 0;
  int collision_sdk_return_code = 0;
  int controller_error_code = 0;
  std::string controller_error_message;
  bool powered_on = true;
  bool enabled = true;
  bool emergency_stop = false;
  bool collision = false;
  std::string monitor_failure;
};

void require_healthy_controller(const ControllerHealth& health) {
  if (!health.monitor_failure.empty())
    throw std::runtime_error("controller health monitor failed: " + health.monitor_failure);
  if (health.status_sdk_return_code != ERR_SUCC ||
      health.estop_sdk_return_code != ERR_SUCC ||
      health.collision_sdk_return_code != ERR_SUCC)
    throw std::runtime_error(
        "controller health SDK query failed: status=" +
        std::to_string(health.status_sdk_return_code) + " estop=" +
        std::to_string(health.estop_sdk_return_code) + " collision=" +
        std::to_string(health.collision_sdk_return_code));
  if (health.controller_error_code != 0)
    throw std::runtime_error(
        "controller reported error code " +
        std::to_string(health.controller_error_code) + ": " +
        health.controller_error_message);
  if (health.collision)
    throw std::runtime_error("controller collision alarm asserted");
  if (health.emergency_stop)
    throw std::runtime_error("controller emergency/protective stop asserted");
  if (!health.powered_on || !health.enabled)
    throw std::runtime_error("controller power or servo-enable state became invalid");
}

using Matrix3 = std::array<std::array<double, 3>, 3>;

Matrix3 rpy_to_matrix(const std::array<double, 3>& rpy) {
  const double cr = std::cos(rpy[0]), sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]), sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]), sy = std::sin(rpy[2]);
  return {{{cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr},
           {sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr},
           {-sp, cp*sr, cp*cr}}};
}

std::array<double, 3> matrix_to_rpy(const Matrix3& r) {
  const double pitch = std::asin(std::clamp(-r[2][0], -1.0, 1.0));
  const double cp = std::cos(pitch);
  if (std::abs(cp) > 1e-8)
    return {std::atan2(r[2][1], r[2][2]), pitch, std::atan2(r[1][0], r[0][0])};
  return {std::atan2(-r[1][2], r[1][1]), pitch, 0.0};
}

Matrix3 multiply_matrix(const Matrix3& a, const Matrix3& b) {
  Matrix3 result{};
  for (std::size_t row = 0; row < 3; ++row)
    for (std::size_t column = 0; column < 3; ++column)
      for (std::size_t inner = 0; inner < 3; ++inner)
        result[row][column] += a[row][inner] * b[inner][column];
  return result;
}

Matrix3 quaternion_to_matrix(const double* payload) {
  double x = payload[3], y = payload[4], z = payload[5], w = payload[6];
  const double norm = std::sqrt(x*x + y*y + z*z + w*w);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3 || norm < 1e-12)
    throw std::runtime_error("relative target quaternion is not unit length");
  x /= norm; y /= norm; z /= norm; w /= norm;
  return {{{1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)},
           {2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)},
           {2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)}}};
}

double quaternion_angle(const double* payload) {
  double w = payload[6];
  const double norm = std::sqrt(payload[3]*payload[3] + payload[4]*payload[4] +
                                payload[5]*payload[5] + w*w);
  if (norm < 1e-12) throw std::runtime_error("relative target quaternion norm is zero");
  w = std::clamp(std::abs(w / norm), 0.0, 1.0);
  return 2.0 * std::acos(w);
}

CartesianState relative_target_from_packet(const TargetPacket& packet,
                                           const CartesianState& startup,
                                           const Options& options) {
  if (packet.kind != static_cast<std::uint16_t>(TargetKind::CartesianPose) ||
      packet.frame_id != kFrameStartupTcpRelative)
    throw std::runtime_error("bounded TeleDex target must be startup-TCP-relative Cartesian pose");
  double norm_sq = 0.0;
  CartesianState target = startup;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (std::abs(packet.payload[axis]) > options.relative_translation_limit_m + 1e-12)
      throw std::runtime_error("relative Cartesian translation exceeds configured envelope");
    norm_sq += packet.payload[axis] * packet.payload[axis];
    target.position_mm[axis] += packet.payload[axis] * 1000.0;
  }
  if (std::sqrt(norm_sq) > std::sqrt(3.0) * options.relative_translation_limit_m + 1e-12)
    throw std::runtime_error("relative Cartesian translation norm is invalid");
  if (quaternion_angle(packet.payload) > options.relative_rotation_limit_rad + 1e-12)
    throw std::runtime_error("relative Cartesian orientation exceeds configured envelope");
  const Matrix3 rotation = multiply_matrix(quaternion_to_matrix(packet.payload),
                                           rpy_to_matrix(startup.rpy_rad));
  target.rpy_rad = matrix_to_rpy(rotation);
  return target;
}

void validate_joint_solution(const std::array<double, 6>& solution,
                             const std::array<double, 6>& reference,
                             const Options& options) {
  double maximum_step = 0.0;
  for (std::size_t joint = 0; joint < 6; ++joint) {
    if (!std::isfinite(solution[joint]) ||
        solution[joint] < kJointLower[joint] + options.joint_soft_margin_rad ||
        solution[joint] > kJointUpper[joint] - options.joint_soft_margin_rad)
      throw std::runtime_error("IK solution violates joint soft-limit envelope");
    maximum_step = std::max(maximum_step, std::abs(solution[joint] - reference[joint]));
  }
  if (maximum_step > options.maximum_ik_step_rad)
    throw std::runtime_error("IK solution violates branch-continuity step envelope");
}

struct OutputMotionSample {
  std::array<double, 6> velocity{}, acceleration{}, jerk{};
  std::uint64_t command_ns = 0;
};

// Classification thresholds only; they do not change any command boundary.
// A destination gap above numerical noise plus no selected step above numerical
// noise is a true stall. Any larger safe selected step is limited progress.
constexpr double kOutputHoldDestinationGapRad = 1e-6;
constexpr double kOutputHoldMinimumProgressRad = 1e-9;

struct OutputTransitionProgress {
  double destination_gap_rad = 0.0;
  double selected_progress_rad = 0.0;
  bool no_progress_hold = false;
};

OutputTransitionProgress classify_output_transition(
    const std::array<double, 6>& prior,
    const std::array<double, 6>& selected,
    const std::array<double, 6>& destination) {
  OutputTransitionProgress result{};
  for (std::size_t joint = 0; joint < prior.size(); ++joint) {
    result.destination_gap_rad = std::max(
        result.destination_gap_rad,
        std::abs(destination[joint] - prior[joint]));
    result.selected_progress_rad = std::max(
        result.selected_progress_rad,
        std::abs(selected[joint] - prior[joint]));
  }
  result.no_progress_hold =
      result.destination_gap_rad > kOutputHoldDestinationGapRad &&
      result.selected_progress_rad <= kOutputHoldMinimumProgressRad;
  return result;
}

class OutputMotionDiagnostics {
 public:
  explicit OutputMotionDiagnostics(const Options& options) : options_(options) {}

  void initialize(const std::array<double, 6>& position, std::uint64_t command_ns) {
    validate_manufacturer_joint_position_limits(position);
    if (command_ns == 0) throw std::runtime_error("output diagnostic initialization time is invalid");
    previous_position_ = position;
    previous_velocity_.fill(0.0);
    previous_acceleration_.fill(0.0);
    previous_command_ns_ = command_ns;
    initialized_ = true;
  }

  OutputMotionSample check_candidate(const ResampledServoPoint& point,
                                     std::uint64_t command_ns) {
    OutputMotionSample sample = measure(point, command_ns);
    const bool velocity_crossing = velocity_boundary_crossing(sample);
    // A raw latest-segment slope can cross the final velocity boundary even
    // though a bounded transition point from the last emitted state is safe.
    // Let the recoverable transition path evaluate that point; check_final()
    // still enforces the final native boundary before SDK dispatch.
    if (velocity_crossing && !options_.recover_output_acceleration_transition)
      require_velocity_boundary(point, sample);
    if (velocity_crossing && options_.recover_output_acceleration_transition)
      return sample;
    const bool recoverable_crossing = record_recoverable_crossings(sample);
    if (recoverable_crossing && options_.recover_output_acceleration_transition)
      return sample;
    if (recoverable_crossing && options_.abort_on_diagnostic_acceleration_boundary)
      throw std::runtime_error(
          "diagnostic output acceleration boundary crossed before SDK call: J" +
          std::to_string(first_recoverable_violating_joint_ + 1) +
          " acceleration=" +
          std::to_string(sample.acceleration[first_recoverable_violating_joint_]) +
          " rad/s2 limit=" +
          std::to_string(options_.diagnostic_joint_acceleration_boundary_rad_s2));
    if (options_.recover_output_acceleration_transition ||
        options_.abort_on_diagnostic_acceleration_boundary)
      require_hard_acceleration_boundary(point, sample);
    return sample;
  }

  OutputMotionSample check_final(const ResampledServoPoint& point,
                                 std::uint64_t command_ns) {
    OutputMotionSample sample = measure(point, command_ns);
    last_checked_jerk_ = sample.jerk;
    require_velocity_boundary(point, sample);
    require_hard_acceleration_boundary(point, sample);
    require_jerk_boundary(point, sample);
    return sample;
  }

  bool recoverable_acceleration_crossing(
      const OutputMotionSample& sample) const {
    return std::any_of(
        sample.acceleration.begin(), sample.acceleration.end(),
        [this](double value) {
          return std::abs(value) >
              options_.diagnostic_joint_acceleration_boundary_rad_s2 + 1e-12;
        });
  }

  bool velocity_boundary_crossing(const OutputMotionSample& sample) const {
    for (std::size_t joint = 0; joint < sample.velocity.size(); ++joint) {
      const double limit = options_.maximum_output_joint_velocity_per_joint_set
          ? options_.maximum_output_joint_velocity_rad_s_per_joint[joint]
          : options_.maximum_output_joint_velocity_rad_s;
      if (std::abs(sample.velocity[joint]) > limit + 1e-12) return true;
    }
    return false;
  }

  ResampledServoPoint transition_limited_point(
      const ResampledServoPoint& proposed,
      const OutputMotionSample& proposed_motion) const {
    if (!initialized_)
      throw std::runtime_error("output transition limiter is not initialized");
    if (proposed_motion.command_ns <= previous_command_ns_)
      throw std::runtime_error("output transition limiter timestamp is invalid");
    const double dt_s =
        static_cast<double>(proposed_motion.command_ns - previous_command_ns_) / 1e9;
    const double selected_boundary =
        std::min(options_.diagnostic_joint_acceleration_boundary_rad_s2,
                 options_.maximum_output_joint_acceleration_rad_s2);
    // Leave deterministic floating-point headroom for reconstructing velocity
    // from the selected position on the same command timestamp.
    return jaka_servo::transition_limited_point(
        proposed, previous_position_, previous_velocity_,
        previous_acceleration_, dt_s, selected_boundary,
        options_.output_joint_jerk_limit_rad_s3);
  }

  const std::array<double, 6>& previous_position() const {
    return previous_position_;
  }
  const std::array<double, 6>& previous_velocity() const {
    return previous_velocity_;
  }
  const std::array<double, 6>& previous_acceleration() const {
    return previous_acceleration_;
  }
  const std::array<double, 6>& last_checked_jerk() const {
    return last_checked_jerk_;
  }
  bool stationary(double velocity_tolerance = 1e-4,
                  double acceleration_tolerance = 1e-3) const {
    return std::all_of(previous_velocity_.begin(), previous_velocity_.end(),
                       [=](double value) {
                         return std::abs(value) <= velocity_tolerance;
                       }) &&
           std::all_of(previous_acceleration_.begin(),
                       previous_acceleration_.end(), [=](double value) {
                         return std::abs(value) <= acceleration_tolerance;
                       });
  }
  std::size_t first_recoverable_violating_joint() const {
    return first_recoverable_violating_joint_;
  }

 private:
  OutputMotionSample measure(const ResampledServoPoint& point,
                             std::uint64_t command_ns) const {
    validate_manufacturer_joint_position_limits(point.position);
    if (command_ns == 0) throw std::runtime_error("output command timestamp is invalid");
    OutputMotionSample sample{};
    sample.command_ns = command_ns;
    if (!initialized_) return sample;
    if (command_ns <= previous_command_ns_)
      throw std::runtime_error("output command timestamps are not strictly monotonic");
    const double dt_s = static_cast<double>(command_ns - previous_command_ns_) / 1e9;
    for (std::size_t joint = 0; joint < point.position.size(); ++joint) {
      sample.velocity[joint] = (point.position[joint] - previous_position_[joint]) / dt_s;
      sample.acceleration[joint] = (sample.velocity[joint] - previous_velocity_[joint]) / dt_s;
      sample.jerk[joint] = (sample.acceleration[joint] - previous_acceleration_[joint]) / dt_s;
      if (!std::isfinite(sample.velocity[joint]) || !std::isfinite(sample.acceleration[joint]) ||
          !std::isfinite(sample.jerk[joint]))
        throw std::runtime_error("non-finite controller-visible output diagnostic");
    }
    return sample;
  }

  void require_velocity_boundary(const ResampledServoPoint& point,
                                 const OutputMotionSample& sample) {
    for (std::size_t joint = 0; joint < point.position.size(); ++joint) {
      if (std::abs(sample.velocity[joint]) >
          options_.maximum_output_joint_velocity_rad_s_per_joint[joint] +
              1e-12) {
        ++speed_boundary_rejections_[joint];
        throw std::runtime_error(
            "internal output-feasibility contract violation before SDK call: "
            "native output velocity hard boundary crossed before SDK call: J" +
            std::to_string(joint + 1) + " velocity=" + std::to_string(sample.velocity[joint]) +
            " rad/s limit=" +
            std::to_string(
                options_.maximum_output_joint_velocity_rad_s_per_joint[joint]) +
            " from_sequence=" + std::to_string(point.from_sequence) +
            " to_sequence=" + std::to_string(point.to_sequence) +
            " alpha=" + std::to_string(point.alpha));
      }
    }
  }

  bool record_recoverable_crossings(const OutputMotionSample& sample) {
    bool crossing = false;
    for (std::size_t joint = 0; joint < sample.acceleration.size(); ++joint) {
      if (std::abs(sample.acceleration[joint]) >
          options_.diagnostic_joint_acceleration_boundary_rad_s2 + 1e-12) {
        ++acceleration_boundary_rejections_[joint];
        if (!crossing) first_recoverable_violating_joint_ = joint;
        crossing = true;
      }
    }
    return crossing;
  }

  void require_hard_acceleration_boundary(
      const ResampledServoPoint& point, const OutputMotionSample& sample) {
    for (std::size_t joint = 0; joint < sample.acceleration.size(); ++joint) {
      if (std::abs(sample.acceleration[joint]) >
          options_.maximum_output_joint_acceleration_rad_s2 + 1e-12) {
        ++hard_acceleration_boundary_rejections_[joint];
        throw std::runtime_error(
            "internal output-feasibility contract violation before SDK call: "
            "native output acceleration hard boundary crossed before SDK call: J" +
            std::to_string(joint + 1) + " acceleration=" +
            std::to_string(sample.acceleration[joint]) + " rad/s2 limit=" +
            std::to_string(options_.maximum_output_joint_acceleration_rad_s2) +
            " from_sequence=" + std::to_string(point.from_sequence) +
            " to_sequence=" + std::to_string(point.to_sequence) +
            " alpha=" + std::to_string(point.alpha));
      }
    }
  }

  void require_jerk_boundary(const ResampledServoPoint& point,
                             const OutputMotionSample& sample) {
    for (std::size_t joint = 0; joint < sample.jerk.size(); ++joint) {
      if (!jaka_servo::output_jerk_within_hard_boundary(
              sample.jerk[joint], options_.output_joint_jerk_limit_rad_s3))
        throw std::runtime_error(
            "internal output-feasibility contract violation before SDK call: "
            "native output jerk hard boundary crossed before SDK call: J" +
            std::to_string(joint + 1) + " jerk=" +
            std::to_string(sample.jerk[joint]) + " rad/s3 limit=" +
            std::to_string(options_.output_joint_jerk_limit_rad_s3) +
            " from_sequence=" + std::to_string(point.from_sequence) +
            " to_sequence=" + std::to_string(point.to_sequence) +
            " alpha=" + std::to_string(point.alpha));
    }
  }

 public:
  void commit(const ResampledServoPoint& point, const OutputMotionSample& sample) {
    if (initialized_) {
      for (std::size_t joint = 0; joint < point.position.size(); ++joint) {
        maximum_delta_[joint] = std::max(maximum_delta_[joint], std::abs(point.position[joint] - previous_position_[joint]));
        maximum_velocity_[joint] = std::max(maximum_velocity_[joint], std::abs(sample.velocity[joint]));
        maximum_acceleration_[joint] = std::max(maximum_acceleration_[joint], std::abs(sample.acceleration[joint]));
        maximum_jerk_[joint] = std::max(maximum_jerk_[joint], std::abs(sample.jerk[joint]));
        if (std::abs(sample.acceleration[joint]) > options_.diagnostic_joint_acceleration_boundary_rad_s2)
          ++acceleration_boundary_crossings_[joint];
      }
    }
    previous_position_ = point.position;
    previous_velocity_ = sample.velocity;
    previous_acceleration_ = sample.acceleration;
    previous_command_ns_ = sample.command_ns;
    initialized_ = true;
  }

  const std::array<double, 6>& maximum_delta() const { return maximum_delta_; }
  const std::array<double, 6>& maximum_velocity() const { return maximum_velocity_; }
  const std::array<double, 6>& maximum_acceleration() const { return maximum_acceleration_; }
  const std::array<double, 6>& maximum_jerk() const { return maximum_jerk_; }
  const std::array<std::uint64_t, 6>& acceleration_boundary_crossings() const { return acceleration_boundary_crossings_; }
  const std::array<std::uint64_t, 6>& speed_boundary_rejections() const { return speed_boundary_rejections_; }
  const std::array<std::uint64_t, 6>& acceleration_boundary_rejections() const { return acceleration_boundary_rejections_; }
  const std::array<std::uint64_t, 6>& hard_acceleration_boundary_rejections() const { return hard_acceleration_boundary_rejections_; }

 private:
  const Options& options_;
  std::array<double, 6> previous_position_{}, previous_velocity_{}, previous_acceleration_{};
  std::array<double, 6> last_checked_jerk_{};
  std::array<double, 6> maximum_delta_{}, maximum_velocity_{}, maximum_acceleration_{}, maximum_jerk_{};
  std::array<std::uint64_t, 6> acceleration_boundary_crossings_{};
  std::array<std::uint64_t, 6> acceleration_boundary_rejections_{};
  std::array<std::uint64_t, 6> hard_acceleration_boundary_rejections_{};
  std::array<std::uint64_t, 6> speed_boundary_rejections_{};
  std::uint64_t previous_command_ns_ = 0;
  std::size_t first_recoverable_violating_joint_ = 0;
  bool initialized_ = false;
};

struct OutputHoldUpdate {
  bool started = false;
  bool degraded = false;
  std::uint64_t duration_ns = 0;
  std::uint32_t consecutive_cycles = 0;
};

class OutputNoProgressHoldTracker {
 public:
  explicit OutputNoProgressHoldTracker(const Options& options)
      : options_(options) {}

  OutputHoldUpdate hold(std::uint64_t command_ns,
                        std::uint64_t destination_sequence) {
    OutputHoldUpdate update{};
    if (!active_) {
      active_ = true;
      start_ns_ = command_ns;
      start_sequence_ = destination_sequence;
      current_consecutive_cycles_ = 0;
      degraded_reported_ = false;
      ++hold_count_;
      update.started = true;
    }
    last_ns_ = command_ns;
    ++current_consecutive_cycles_;
    maximum_consecutive_cycles_ = std::max(
        maximum_consecutive_cycles_, current_consecutive_cycles_);
    update.duration_ns = command_ns - start_ns_;
    update.consecutive_cycles = current_consecutive_cycles_;
    if (!degraded_reported_ &&
        update.duration_ns >= options_.output_acceleration_hold_degraded_ns) {
      degraded_reported_ = true;
      update.degraded = true;
      ++degraded_count_;
    }
    if (update.duration_ns >=
            options_.output_acceleration_hold_hard_stop_ns ||
        current_consecutive_cycles_ >
            options_.maximum_consecutive_output_acceleration_hold_cycles) {
      throw std::runtime_error(
          "sustained no-progress output hold exceeded escalation policy: "
          "duration_ns=" + std::to_string(update.duration_ns) +
          " consecutive_cycles=" +
          std::to_string(current_consecutive_cycles_) +
          " start_sequence=" + std::to_string(start_sequence_) +
          " latest_sequence=" + std::to_string(destination_sequence));
    }
    return update;
  }

  std::uint64_t recover(std::uint64_t command_ns,
                        std::uint64_t destination_sequence) {
    if (!active_) return 0;
    const std::uint64_t duration_ns = command_ns - start_ns_;
    total_hold_duration_ns_ += duration_ns;
    longest_hold_duration_ns_ =
        std::max(longest_hold_duration_ns_, duration_ns);
    recovery_sequence_ = destination_sequence;
    ++recovery_count_;
    active_ = false;
    current_consecutive_cycles_ = 0;
    return duration_ns;
  }

  void finalize(std::uint64_t end_ns) {
    if (!active_) return;
    const std::uint64_t duration_ns =
        end_ns >= start_ns_ ? end_ns - start_ns_ : 0;
    total_hold_duration_ns_ += duration_ns;
    longest_hold_duration_ns_ =
        std::max(longest_hold_duration_ns_, duration_ns);
    active_ = false;
  }

  bool active() const { return active_; }
  std::uint64_t start_ns() const { return start_ns_; }
  std::uint64_t hold_count() const { return hold_count_; }
  std::uint64_t recovery_count() const { return recovery_count_; }
  std::uint64_t degraded_count() const { return degraded_count_; }
  std::uint64_t total_hold_duration_ns() const {
    return total_hold_duration_ns_;
  }
  std::uint64_t longest_hold_duration_ns() const {
    return longest_hold_duration_ns_;
  }
  std::uint32_t current_consecutive_cycles() const {
    return current_consecutive_cycles_;
  }
  std::uint32_t maximum_consecutive_cycles() const {
    return maximum_consecutive_cycles_;
  }
  std::uint64_t recovery_sequence() const { return recovery_sequence_; }

 private:
  const Options& options_;
  bool active_ = false;
  bool degraded_reported_ = false;
  std::uint64_t start_ns_ = 0;
  std::uint64_t last_ns_ = 0;
  std::uint64_t start_sequence_ = 0;
  std::uint64_t recovery_sequence_ = 0;
  std::uint64_t hold_count_ = 0;
  std::uint64_t recovery_count_ = 0;
  std::uint64_t degraded_count_ = 0;
  std::uint64_t total_hold_duration_ns_ = 0;
  std::uint64_t longest_hold_duration_ns_ = 0;
  std::uint32_t current_consecutive_cycles_ = 0;
  std::uint32_t maximum_consecutive_cycles_ = 0;
};

struct RecordedServoPoint {
  ResampledServoPoint point;
  OutputMotionSample motion;
};

enum class PlacementEventReason : std::uint8_t {
  WorkerStart,
  FirstTimingWarning,
  CpuMigration,
  TerminalTimingFault,
  WorkerShutdown,
};

const char* placement_event_reason_name(PlacementEventReason reason) {
  switch (reason) {
    case PlacementEventReason::WorkerStart: return "worker_start";
    case PlacementEventReason::FirstTimingWarning: return "first_timing_warning";
    case PlacementEventReason::CpuMigration: return "cpu_migration";
    case PlacementEventReason::TerminalTimingFault: return "terminal_timing_fault";
    case PlacementEventReason::WorkerShutdown: return "worker_shutdown";
  }
  return "unknown";
}

std::int64_t read_locked_memory_kb() noexcept {
  try {
    std::ifstream status("/proc/self/status");
    std::string key;
    while (status >> key) {
      if (key == "VmLck:") {
        std::int64_t value = -1;
        status >> value;
        return value;
      }
      std::string rest;
      std::getline(status, rest);
    }
  } catch (...) {
    return -1;
  }
  return -1;
}

struct PlacementEvent {
  PlacementEventReason reason = PlacementEventReason::WorkerStart;
  std::uint64_t monotonic_ns = 0;
  pid_t process_id = 0;
  pid_t thread_id = 0;
  int cpu = -1;
  int previous_cpu = -1;
  std::uint64_t migration_count = 0;
  std::uint64_t time_since_last_migration_ns = 0;
  int scheduler_policy = -1;
  int scheduler_priority = -1;
  int nice_value = 0;
  cpu_set_t affinity{};
  bool affinity_available = false;
  std::int64_t locked_memory_kb = -1;
};

class PlacementEvidence {
 public:
  PlacementEvidence() : process_id_(getpid()), thread_id_(static_cast<pid_t>(syscall(SYS_gettid))) {}

  void record(PlacementEventReason reason, std::uint64_t monotonic_ns,
              int cpu, int previous_cpu, bool include_full_state,
              bool include_memory_state) noexcept {
    if (count_ >= events_.size()) {
      ++dropped_events_;
      return;
    }
    auto& event = events_[count_++];
    event.reason = reason;
    event.monotonic_ns = monotonic_ns;
    event.process_id = process_id_;
    event.thread_id = thread_id_;
    event.cpu = cpu;
    event.previous_cpu = previous_cpu;
    event.migration_count = migration_count_;
    event.time_since_last_migration_ns =
        last_migration_ns_ == 0 || monotonic_ns < last_migration_ns_
            ? 0 : monotonic_ns - last_migration_ns_;
    if (include_full_state) {
      event.scheduler_policy = sched_getscheduler(thread_id_);
      sched_param parameter{};
      if (sched_getparam(thread_id_, &parameter) == 0)
        event.scheduler_priority = parameter.sched_priority;
      errno = 0;
      const int nice_value = getpriority(PRIO_PROCESS, thread_id_);
      if (errno == 0) event.nice_value = nice_value;
      CPU_ZERO(&event.affinity);
      event.affinity_available =
          sched_getaffinity(thread_id_, sizeof(event.affinity), &event.affinity) == 0;
      cached_scheduler_policy_ = event.scheduler_policy;
      cached_scheduler_priority_ = event.scheduler_priority;
      cached_nice_value_ = event.nice_value;
      cached_affinity_ = event.affinity;
      cached_affinity_available_ = event.affinity_available;
      cached_full_state_available_ = true;
    } else if (cached_full_state_available_) {
      event.scheduler_policy = cached_scheduler_policy_;
      event.scheduler_priority = cached_scheduler_priority_;
      event.nice_value = cached_nice_value_;
      event.affinity = cached_affinity_;
      event.affinity_available = cached_affinity_available_;
    }
    if (include_memory_state) event.locked_memory_kb = read_locked_memory_kb();
  }

  void observe_cpu(std::uint64_t monotonic_ns, int current_cpu) noexcept {
    if (previous_cpu_ < 0) {
      previous_cpu_ = current_cpu;
      last_migration_ns_ = monotonic_ns;
      return;
    }
    if (current_cpu == previous_cpu_) return;
    const int prior = previous_cpu_;
    previous_cpu_ = current_cpu;
    ++migration_count_;
    record(PlacementEventReason::CpuMigration, monotonic_ns, current_cpu,
           prior, false, false);
    last_migration_ns_ = monotonic_ns;
  }

  const auto& events() const { return events_; }
  std::size_t count() const { return count_; }
  std::uint64_t migration_count() const { return migration_count_; }
  std::uint64_t dropped_events() const { return dropped_events_; }
  int previous_cpu() const { return previous_cpu_; }
  std::uint64_t last_migration_ns() const { return last_migration_ns_; }

 private:
  static constexpr std::size_t kMaximumPlacementEvents = 256;
  std::array<PlacementEvent, kMaximumPlacementEvents> events_{};
  pid_t process_id_ = 0;
  pid_t thread_id_ = 0;
  std::size_t count_ = 0;
  std::uint64_t migration_count_ = 0;
  std::uint64_t dropped_events_ = 0;
  int previous_cpu_ = -1;
  std::uint64_t last_migration_ns_ = 0;
  int cached_scheduler_policy_ = -1;
  int cached_scheduler_priority_ = -1;
  int cached_nice_value_ = 0;
  cpu_set_t cached_affinity_{};
  bool cached_affinity_available_ = false;
  bool cached_full_state_available_ = false;
};

enum class SystemSnapshotTrigger : std::uint8_t {
  FirstTimingWarning = 1,
  TerminalTimingFault = 2,
};

const char* system_snapshot_trigger_name(SystemSnapshotTrigger trigger) {
  switch (trigger) {
    case SystemSnapshotTrigger::FirstTimingWarning: return "first_timing_warning";
    case SystemSnapshotTrigger::TerminalTimingFault: return "terminal_timing_fault";
  }
  return "unknown";
}

struct SystemSnapshotRequest {
  SystemSnapshotTrigger trigger = SystemSnapshotTrigger::FirstTimingWarning;
  std::uint64_t requested_monotonic_ns = 0;
  int trigger_cpu = -1;
};

struct SystemSnapshot {
  SystemSnapshotRequest request{};
  std::uint64_t collected_start_monotonic_ns = 0;
  std::uint64_t collected_end_monotonic_ns = 0;
  pid_t observer_thread_id = 0;
  int observer_cpu = -1;
  int observer_scheduler_policy = -1;
  int observer_scheduler_priority = -1;
  int observer_nice_value = 0;
  std::string proc_stat;
  std::string proc_loadavg;
  std::string proc_interrupts;
  std::string proc_softirqs;
  std::string pressure_cpu;
  std::string pressure_io;
  std::string pressure_memory;
  std::string cpu_frequency;
  std::string cpu_governor;
  std::string thermal_state;
  std::string errors;
};

std::string read_bounded_text(const std::string& path, std::size_t limit,
                              std::string& errors) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    if (!errors.empty()) errors += ";";
    errors += "open:" + path;
    return {};
  }
  std::string value(limit, '\0');
  input.read(value.data(), static_cast<std::streamsize>(limit));
  value.resize(static_cast<std::size_t>(input.gcount()));
  return value;
}

std::string read_thermal_state(std::string& errors) {
  glob_t matches{};
  const int result = glob("/sys/class/thermal/thermal_zone*/temp", 0, nullptr, &matches);
  if (result != 0 && result != GLOB_NOMATCH) {
    errors += errors.empty() ? "glob:thermal" : ";glob:thermal";
    globfree(&matches);
    return {};
  }
  std::ostringstream out;
  const std::size_t count = std::min<std::size_t>(matches.gl_pathc, 32);
  for (std::size_t i = 0; i < count; ++i) {
    out << matches.gl_pathv[i] << '='
        << read_bounded_text(matches.gl_pathv[i], 128, errors);
  }
  globfree(&matches);
  return out.str();
}

class BoundarySystemObserver {
 public:
  explicit BoundarySystemObserver(bool enabled) : enabled_(enabled) {
    if (!enabled_) return;
    if (pipe(pipe_fds_) != 0) {
      enabled_ = false;
      startup_error_ = "pipe_failed:" + std::to_string(errno);
      return;
    }
    const int flags = fcntl(pipe_fds_[1], F_GETFL, 0);
    if (flags < 0 || fcntl(pipe_fds_[1], F_SETFL, flags | O_NONBLOCK) != 0) {
      startup_error_ = "pipe_nonblocking_failed:" + std::to_string(errno);
      close(pipe_fds_[0]);
      close(pipe_fds_[1]);
      pipe_fds_[0] = pipe_fds_[1] = -1;
      enabled_ = false;
      return;
    }
    observer_ = std::thread([this] { run(); });
  }

  ~BoundarySystemObserver() { stop(); }

  void request(SystemSnapshotTrigger trigger, std::uint64_t monotonic_ns,
               int cpu) noexcept {
    if (!enabled_ || pipe_fds_[1] < 0) return;
    const SystemSnapshotRequest request{trigger, monotonic_ns, cpu};
    const ssize_t written = write(pipe_fds_[1], &request, sizeof(request));
    if (written != static_cast<ssize_t>(sizeof(request)))
      dropped_requests_.fetch_add(1, std::memory_order_relaxed);
  }

  void stop() noexcept {
    if (pipe_fds_[1] >= 0) {
      close(pipe_fds_[1]);
      pipe_fds_[1] = -1;
    }
    if (observer_.joinable()) observer_.join();
    if (pipe_fds_[0] >= 0) {
      close(pipe_fds_[0]);
      pipe_fds_[0] = -1;
    }
  }

  const auto& snapshots() const { return snapshots_; }
  std::uint64_t dropped_requests() const {
    return dropped_requests_.load(std::memory_order_relaxed);
  }
  const std::string& startup_error() const { return startup_error_; }
  const std::string& runtime_error() const { return runtime_error_; }

 private:
  void run() noexcept {
    try {
      const pid_t thread_id = static_cast<pid_t>(syscall(SYS_gettid));
      setpriority(PRIO_PROCESS, thread_id, 10);
      while (true) {
        SystemSnapshotRequest request{};
        const ssize_t received = read(pipe_fds_[0], &request, sizeof(request));
        if (received == 0) break;
        if (received < 0) {
          if (errno == EINTR) continue;
          break;
        }
        if (received != static_cast<ssize_t>(sizeof(request))) continue;
        if (snapshots_.size() >= 8) {
          dropped_requests_.fetch_add(1, std::memory_order_relaxed);
          continue;
        }
        SystemSnapshot snapshot{};
        snapshot.request = request;
        snapshot.collected_start_monotonic_ns = now_ns();
        snapshot.observer_thread_id = thread_id;
        snapshot.observer_cpu = sched_getcpu();
        snapshot.observer_scheduler_policy = sched_getscheduler(thread_id);
        sched_param parameter{};
        if (sched_getparam(thread_id, &parameter) == 0)
          snapshot.observer_scheduler_priority = parameter.sched_priority;
        errno = 0;
        const int nice_value = getpriority(PRIO_PROCESS, thread_id);
        if (errno == 0) snapshot.observer_nice_value = nice_value;
        snapshot.proc_stat = read_bounded_text("/proc/stat", 48 * 1024, snapshot.errors);
        snapshot.proc_loadavg = read_bounded_text("/proc/loadavg", 1024, snapshot.errors);
        snapshot.proc_interrupts = read_bounded_text("/proc/interrupts", 48 * 1024, snapshot.errors);
        snapshot.proc_softirqs = read_bounded_text("/proc/softirqs", 48 * 1024, snapshot.errors);
        snapshot.pressure_cpu = read_bounded_text("/proc/pressure/cpu", 4096, snapshot.errors);
        snapshot.pressure_io = read_bounded_text("/proc/pressure/io", 4096, snapshot.errors);
        snapshot.pressure_memory = read_bounded_text("/proc/pressure/memory", 4096, snapshot.errors);
        if (request.trigger_cpu >= 0) {
          const std::string cpu_path = "/sys/devices/system/cpu/cpu" +
              std::to_string(request.trigger_cpu) + "/cpufreq/";
          snapshot.cpu_frequency = read_bounded_text(
              cpu_path + "scaling_cur_freq", 1024, snapshot.errors);
          snapshot.cpu_governor = read_bounded_text(
              cpu_path + "scaling_governor", 1024, snapshot.errors);
        }
        snapshot.thermal_state = read_thermal_state(snapshot.errors);
        snapshot.collected_end_monotonic_ns = now_ns();
        snapshots_.push_back(std::move(snapshot));
      }
    } catch (const std::exception& error) {
      try {
        runtime_error_ = error.what();
      } catch (...) {}
    } catch (...) {
      try { runtime_error_ = "unknown_observer_failure"; } catch (...) {}
    }
  }

  bool enabled_ = false;
  int pipe_fds_[2]{-1, -1};
  std::thread observer_;
  std::vector<SystemSnapshot> snapshots_;
  std::atomic<std::uint64_t> dropped_requests_{0};
  std::string startup_error_;
  std::string runtime_error_;
};

struct CycleTelemetry {
  std::uint64_t monotonic_ns = 0;
  std::uint64_t wall_ns = 0;
  std::uint64_t cycle_start_ns = 0;
  std::uint64_t scheduled_deadline_ns = 0;
  std::uint64_t cycle_end_ns = 0;
  std::uint64_t wake_lateness_ns = 0;
  std::uint64_t completion_lateness_ns = 0;
  std::uint64_t command_start_ns = 0;
  std::uint64_t command_end_ns = 0;
  std::uint64_t command_duration_ns = 0;
  int cpu = -1;
  std::uint64_t cpu_migration_count = 0;
  std::uint64_t last_sequence = 0;
  std::uint64_t heartbeat_age_ns = 0;
  std::string event = "normal_output";
  std::array<double, 6> emitted{}, destination{}, measured{}, tracking{};
  std::array<double, 6> prior_emitted{}, proposed_emitted{}, continuity_error{};
  OutputMotionSample prior_motion{}, proposed_motion{};
  ResampledServoPoint point{};
  OutputMotionSample motion{};
  ControllerHealth health{};
  std::size_t violating_joint = 0;
  std::uint64_t hold_start_ns = 0;
  std::uint64_t hold_duration_ns = 0;
  std::uint32_t consecutive_hold_cycles = 0;
  std::uint64_t recovery_sequence = 0;
  double destination_gap_rad = 0.0;
  double selected_progress_rad = 0.0;
  bool transition_limited = false;
  bool no_progress_hold = false;
  bool hold_degraded = false;
};

void write_six_json(std::ostream& out, const std::array<double, 6>& values) {
  out << '[';
  for (std::size_t joint = 0; joint < values.size(); ++joint)
    out << (joint ? "," : "") << values[joint];
  out << ']';
}

void write_cycle_telemetry(const Options& options,
                           const std::vector<CycleTelemetry>& rows) {
  if (options.cycle_telemetry_file.empty()) return;
  std::ofstream out(options.cycle_telemetry_file);
  if (!out) throw std::runtime_error("cannot open cycle telemetry file");
  out << std::setprecision(17);
  for (const auto& row : rows) {
    out << "{\"host_monotonic_ns\":" << row.monotonic_ns
        << ",\"local_wall_clock_unix_ns\":" << row.wall_ns
        << ",\"cycle_start_ns\":" << row.cycle_start_ns
        << ",\"scheduled_deadline_ns\":" << row.scheduled_deadline_ns
        << ",\"cycle_end_ns\":" << row.cycle_end_ns
        << ",\"wake_lateness_ns\":" << row.wake_lateness_ns
        << ",\"completion_lateness_ns\":" << row.completion_lateness_ns
        << ",\"command_start_ns\":" << row.command_start_ns
        << ",\"command_end_ns\":" << row.command_end_ns
        << ",\"command_duration_ns\":" << row.command_duration_ns
        << ",\"cpu\":" << row.cpu
        << ",\"cpu_migration_count\":" << row.cpu_migration_count
        << ",\"output_event\":\"" << row.event << "\""
        << ",\"controller_health_monotonic_ns\":" << row.health.monotonic_ns
        << ",\"controller_health_age_ns\":"
        << (row.monotonic_ns >= row.health.monotonic_ns ? row.monotonic_ns - row.health.monotonic_ns : 0)
        << ",\"last_accepted_sequence\":" << row.last_sequence
        << ",\"generator_heartbeat_age_ns\":" << row.heartbeat_age_ns
        << ",\"controller_error_code\":" << row.health.controller_error_code
        << ",\"controller_collision\":" << (row.health.collision ? "true" : "false")
        << ",\"controller_emergency_stop\":" << (row.health.emergency_stop ? "true" : "false")
        << ",\"controller_powered_on\":" << (row.health.powered_on ? "true" : "false")
        << ",\"controller_enabled\":" << (row.health.enabled ? "true" : "false")
        << ",\"controller_status_sdk_return_code\":" << row.health.status_sdk_return_code
        << ",\"collision_sdk_return_code\":" << row.health.collision_sdk_return_code
        << ",\"estop_sdk_return_code\":" << row.health.estop_sdk_return_code
        << ",\"resampler_from_sequence\":" << row.point.from_sequence
        << ",\"resampler_to_sequence\":" << row.point.to_sequence
        << ",\"resampler_alpha\":" << row.point.alpha
        << ",\"violating_joint_one_based\":"
        << (row.violating_joint + 1)
        << ",\"recoverable_output_acceleration_boundary_rad_s2\":"
        << options.diagnostic_joint_acceleration_boundary_rad_s2
        << ",\"native_output_acceleration_hard_boundary_rad_s2\":"
        << options.maximum_output_joint_acceleration_rad_s2
        << ",\"output_joint_jerk_hard_boundary_rad_s3\":"
        << options.output_joint_jerk_limit_rad_s3
        << ",\"output_joint_jerk_tolerance_absolute_rad_s3\":"
        << jaka_servo::kOutputJerkHardBoundaryToleranceAbsoluteRadS3
        << ",\"output_joint_jerk_tolerance_relative\":"
        << jaka_servo::kOutputJerkHardBoundaryToleranceRelative
        << ",\"output_joint_jerk_hard_boundary_with_tolerance_rad_s3\":"
        << jaka_servo::output_jerk_hard_boundary_with_tolerance(
               options.output_joint_jerk_limit_rad_s3)
        << ",\"output_acceleration_hold_start_ns\":" << row.hold_start_ns
        << ",\"output_acceleration_hold_duration_ns\":"
        << row.hold_duration_ns
        << ",\"consecutive_output_acceleration_hold_cycles\":"
        << row.consecutive_hold_cycles
        << ",\"output_acceleration_hold_degraded\":"
        << (row.hold_degraded ? "true" : "false")
        << ",\"output_acceleration_recovery_sequence\":"
        << row.recovery_sequence
        << ",\"transition_limited\":"
        << (row.transition_limited ? "true" : "false")
        << ",\"destination_gap_rad\":" << row.destination_gap_rad
        << ",\"selected_progress_rad\":" << row.selected_progress_rad
        << ",\"no_progress_hold\":"
        << (row.no_progress_hold ? "true" : "false")
        << ",\"prior_emitted_command_rad\":";
    write_six_json(out, row.prior_emitted);
    out << ",\"prior_emitted_velocity_rad_s\":";
    write_six_json(out, row.prior_motion.velocity);
    out << ",\"prior_emitted_acceleration_rad_s2\":";
    write_six_json(out, row.prior_motion.acceleration);
    out << ",\"proposed_emitted_command_rad\":";
    write_six_json(out, row.proposed_emitted);
    out << ",\"proposed_emitted_velocity_rad_s\":";
    write_six_json(out, row.proposed_motion.velocity);
    out << ",\"proposed_emitted_acceleration_rad_s2\":";
    write_six_json(out, row.proposed_motion.acceleration);
    out << ",\"proposed_minus_selected_continuity_error_rad\":";
    write_six_json(out, row.continuity_error);
    out << ",\"emitted_command_rad\":";
    write_six_json(out, row.emitted);
    out << ",\"active_segment_destination_rad\":";
    write_six_json(out, row.destination);
    out << ",\"measured_joint_rad\":";
    write_six_json(out, row.measured);
    out << ",\"emitted_minus_measured_tracking_difference_rad\":";
    write_six_json(out, row.tracking);
    out << ",\"active_segment_target_velocity_rad_s\":";
    write_six_json(out, row.point.segment_velocity_rad_s);
    out << ",\"emitted_velocity_rad_s\":";
    write_six_json(out, row.motion.velocity);
    out << ",\"emitted_acceleration_rad_s2\":";
    write_six_json(out, row.motion.acceleration);
    out << ",\"emitted_jerk_rad_s3\":";
    write_six_json(out, row.motion.jerk);
    out << ",\"raw_output_jerk_rad_s3\":";
    write_six_json(out, row.motion.jerk);
    out << "}\n";
  }
}

void write_emitted_points(const Options& options, const std::vector<RecordedServoPoint>& points) {
  if (options.emitted_points_file.empty()) return;
  std::ofstream out(options.emitted_points_file);
  if (!out) throw std::runtime_error("cannot open emitted points file");
  out << std::setprecision(17);
  for (const auto& row : points) {
    out << "{\"servo_time_ns\":" << row.point.servo_time_ns
        << ",\"command_ns\":" << row.motion.command_ns
        << ",\"from_sequence\":" << row.point.from_sequence
        << ",\"to_sequence\":" << row.point.to_sequence
        << ",\"from_accepted_ns\":" << row.point.from_accepted_ns
        << ",\"to_accepted_ns\":" << row.point.to_accepted_ns
        << ",\"alpha\":" << row.point.alpha
        << ",\"endpoint\":" << (row.point.endpoint ? "true" : "false")
        << ",\"joint_position_rad\":[";
    for (std::size_t joint = 0; joint < 6; ++joint) out << (joint ? "," : "") << row.point.position[joint];
    out << "],\"joint_velocity_rad_s\":[";
    for (std::size_t joint = 0; joint < 6; ++joint) out << (joint ? "," : "") << row.motion.velocity[joint];
    out << "],\"joint_acceleration_rad_s2\":[";
    for (std::size_t joint = 0; joint < 6; ++joint) out << (joint ? "," : "") << row.motion.acceleration[joint];
    out << "]}\n";
  }
}

double rotation_distance(const std::array<double, 3>& left_rpy,
                         const std::array<double, 3>& right_rpy) {
  const Matrix3 left = rpy_to_matrix(left_rpy), right = rpy_to_matrix(right_rpy);
  Matrix3 relative{};
  for (std::size_t row = 0; row < 3; ++row)
    for (std::size_t column = 0; column < 3; ++column)
      for (std::size_t inner = 0; inner < 3; ++inner)
        relative[row][column] += left[row][inner] * right[column][inner];
  const double cosine = std::clamp((relative[0][0] + relative[1][1] + relative[2][2] - 1.0) / 2.0,
                                   -1.0, 1.0);
  return std::acos(cosine);
}

class Backend {
 public:
  virtual ~Backend() = default;
  virtual void connect() = 0;
  virtual void verify(int tool, int user) = 0;
  virtual void enter_edg() = 0;
  virtual void validate_probe(const std::array<double, 6>& initial, const std::array<double, 6>& target) = 0;
  virtual void read(std::array<double, 6>& joints) = 0;
  virtual void read_tcp(CartesianState& pose) = 0;
  virtual ControllerHealth read_controller_health() = 0;
  virtual void solve_ik(const CartesianState& target, const std::array<double, 6>& reference,
                        std::array<double, 6>& solution) = 0;
  virtual double validate_kinematics(const CartesianState& target,
                                     const std::array<double, 6>& solution) = 0;
  virtual void command(const std::array<double, 6>& joints) = 0;
  virtual bool edg_active() const noexcept = 0;
  virtual void cleanup() noexcept = 0;
  virtual int cleanup_error_code() const noexcept = 0;
};

class FakeBackend final : public Backend {
 public:
  explicit FakeBackend(const Options& o) : options_(o), joints_(o.fake_initial_joints_rad) {}
  ~FakeBackend() override { cleanup(); }
  void connect() override { delay(options_.fake_connect_delay_ns); connected_ = true; }
  void verify(int, int) override { if (!connected_) throw std::runtime_error("fake disconnected"); }
  void enter_edg() override {
    if (!connected_) throw std::runtime_error("fake disconnected");
    delay(options_.fake_edg_delay_ns);
    for (std::size_t joint = 0; joint < joints_.size(); ++joint)
      joints_[joint] += options_.fake_post_edg_joint_offset_rad[joint];
    edg_ = true;
  }
  void validate_probe(const std::array<double, 6>&, const std::array<double, 6>&) override {}
  void read(std::array<double, 6>& joints) override { delay(options_.fake_read_delay_ns); fail(); joints = joints_; }
  void read_tcp(CartesianState& pose) override { pose = {}; }
  ControllerHealth read_controller_health() override {
    ControllerHealth health{};
    health.monotonic_ns = now_ns();
    health.wall_ns = wall_now_ns();
    return health;
  }
  void solve_ik(const CartesianState& target, const std::array<double, 6>&,
                std::array<double, 6>& solution) override {
    fail();
    for (std::size_t axis = 0; axis < 3; ++axis) {
      solution[axis] = target.position_mm[axis] / 1000.0;
      solution[axis + 3] = target.rpy_rad[axis];
    }
  }
  double validate_kinematics(const CartesianState&, const std::array<double, 6>&) override {
    return 1.0;
  }
  void command(const std::array<double, 6>& joints) override { delay(options_.fake_write_delay_ns); fail(); if (!edg_) throw std::runtime_error("fake EDG inactive"); joints_ = joints; }
  bool edg_active() const noexcept override { return edg_; }
  void cleanup() noexcept override { edg_ = false; connected_ = false; }
  int cleanup_error_code() const noexcept override { return 0; }
 private:
  void fail() { if (options_.fake_fail_after && ++calls_ >= options_.fake_fail_after) throw std::runtime_error("injected fake SDK failure"); }
  static void delay(std::uint64_t ns) { if (ns) { timespec t{static_cast<time_t>(ns / 1'000'000'000), static_cast<long>(ns % 1'000'000'000)}; nanosleep(&t, nullptr); } }
  const Options& options_;
  std::array<double, 6> joints_{};
  std::uint64_t calls_ = 0;
  bool connected_ = false;
  bool edg_ = false;
};

class RealBackend final : public Backend {
 public:
  explicit RealBackend(const Options& o) : options_(o) {}
  ~RealBackend() override { cleanup(); }
  void connect() override {
    require_sdk(robot_.login_in(options_.robot_ip.c_str()), "login_in");
    connected_ = true;
  }
  void verify(int expected_tool, int expected_user) override {
    RobotStatus_simple status{};
    require_sdk(robot_.get_robot_status_simple(&status), "get_robot_status_simple");
    if (status.errcode != 0 || !status.powered_on || !status.enabled) throw std::runtime_error("robot must be fault-free, powered, and servo-enabled");
    BOOL estop = FALSE, collision = FALSE;
    require_sdk(robot_.is_in_estop(&estop), "is_in_estop");
    require_sdk(robot_.is_in_collision(&collision), "is_in_collision");
    if (estop || collision) throw std::runtime_error("robot is in E-stop or collision state");
    int tool = -1, user = -1;
    require_sdk(robot_.get_tool_id(&tool), "get_tool_id");
    require_sdk(robot_.get_user_frame_id(&user), "get_user_frame_id");
    if (tool != expected_tool || user != expected_user) throw std::runtime_error("active tool/user frame does not match explicit expected IDs");
  }
  void enter_edg() override {
    require_sdk(robot_.edg_init(TRUE, options_.edg_state_ip.c_str()), "edg_init(true)");
    edg_ = true;
    require_sdk(robot_.servo_move_enable(TRUE), "servo_move_enable(true)");
    servo_ = true;
    BOOL active = FALSE;
    require_sdk(robot_.is_in_servomove(&active), "is_in_servomove");
    if (!active) throw std::runtime_error("SDK did not enter servo mode");
  }
  void validate_probe(const std::array<double, 6>& initial, const std::array<double, 6>& target) override {
    JointValue q0{}, q1{};
    for (std::size_t i = 0; i < 6; ++i) { q0.jVal[i] = initial[i]; q1.jVal[i] = target[i]; }
    CartesianPose tool{}, user{}, p0{}, p1{};
    require_sdk(robot_.get_tool_data(options_.expected_tool_id, &tool), "get_tool_data");
    require_sdk(robot_.get_user_frame_data(options_.expected_user_frame_id, &user), "get_user_frame_data");
    require_sdk(robot_.kine_forward(&q0, &p0, &tool, &user), "kine_forward(initial)");
    require_sdk(robot_.kine_forward(&q1, &p1, &tool, &user), "kine_forward(probe)");
    const std::array<double, 3> a{p0.tran.x, p0.tran.y, p0.tran.z}, b{p1.tran.x, p1.tran.y, p1.tran.z};
    double displacement_sq = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
      if (!std::isfinite(a[i]) || !std::isfinite(b[i]) || a[i] < options_.workspace_min_mm[i] || a[i] > options_.workspace_max_mm[i] ||
          b[i] < options_.workspace_min_mm[i] || b[i] > options_.workspace_max_mm[i])
        throw std::runtime_error("minimal probe endpoint violates explicit Cartesian workspace");
      const double d = b[i] - a[i]; displacement_sq += d * d;
    }
    if (std::sqrt(displacement_sq) > 5.0) throw std::runtime_error("minimal probe FK displacement exceeds 5 mm");
  }
  void read(std::array<double, 6>& joints) override {
    JointValue value{};
    if (edg_) {
      EDGState state{};
      require_sdk(robot_.edg_get_stat(&state), "edg_get_stat");
      value = state.jointVal;
    } else require_sdk(robot_.get_actual_joint_position(&value), "get_actual_joint_position");
    for (std::size_t i = 0; i < joints.size(); ++i) {
      if (!std::isfinite(value.jVal[i])) throw std::runtime_error("non-finite joint state");
      joints[i] = value.jVal[i];
    }
  }
  void read_tcp(CartesianState& pose) override {
    CartesianPose value{};
    require_sdk(robot_.get_actual_tcp_position(&value), "get_actual_tcp_position");
    pose.position_mm = {value.tran.x, value.tran.y, value.tran.z};
    pose.rpy_rad = {value.rpy.rx, value.rpy.ry, value.rpy.rz};
    if (!std::all_of(pose.position_mm.begin(), pose.position_mm.end(), [](double v) { return std::isfinite(v); }) ||
        !std::all_of(pose.rpy_rad.begin(), pose.rpy_rad.end(), [](double v) { return std::isfinite(v); }))
      throw std::runtime_error("non-finite TCP state");
  }
  ControllerHealth read_controller_health() override {
    ControllerHealth health{};
    health.monotonic_ns = now_ns();
    health.wall_ns = wall_now_ns();
    RobotStatus_simple status{};
    health.status_sdk_return_code = robot_.get_robot_status_simple(&status);
    health.controller_error_code = status.errcode;
    health.controller_error_message = status.errmsg;
    health.powered_on = status.powered_on != 0;
    health.enabled = status.enabled != 0;
    // Keep the healthy-path query to one supported lightweight SDK call.
    // Once it reports a fault/power transition, classify collision/E-stop;
    // timing no longer matters because no subsequent ServoJ point is allowed.
    if (health.status_sdk_return_code != ERR_SUCC || status.errcode != 0 ||
        !health.powered_on || !health.enabled) {
      BOOL estop = FALSE, collision = FALSE;
      health.estop_sdk_return_code = robot_.is_in_estop(&estop);
      health.collision_sdk_return_code = robot_.is_in_collision(&collision);
      health.emergency_stop = estop != FALSE;
      health.collision = collision != FALSE;
    }
    return health;
  }
  void solve_ik(const CartesianState& target, const std::array<double, 6>& reference,
                std::array<double, 6>& solution) override {
    JointValue ref{}, result{};
    CartesianPose pose{};
    for (std::size_t joint = 0; joint < 6; ++joint) ref.jVal[joint] = reference[joint];
    pose.tran.x = target.position_mm[0]; pose.tran.y = target.position_mm[1]; pose.tran.z = target.position_mm[2];
    pose.rpy.rx = target.rpy_rad[0]; pose.rpy.ry = target.rpy_rad[1]; pose.rpy.rz = target.rpy_rad[2];
    require_sdk(robot_.kine_inverse(&ref, &pose, &result), "kine_inverse");
    for (std::size_t joint = 0; joint < 6; ++joint) solution[joint] = result.jVal[joint];
  }
  double validate_kinematics(const CartesianState& target,
                             const std::array<double, 6>& solution) override {
    const CartesianState actual = forward_state(solution);
    double translation_sq = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double difference = actual.position_mm[axis] - target.position_mm[axis];
      translation_sq += difference * difference;
    }
    if (std::sqrt(translation_sq) > 1.0 || rotation_distance(actual.rpy_rad, target.rpy_rad) > 0.03490658503988659)
      throw std::runtime_error("IK forward residual exceeds 1 mm / 2 deg envelope");

    constexpr double epsilon = 1e-4;
    std::array<std::array<double, 6>, 6> jacobian{};
    const Matrix3 base_rotation = rpy_to_matrix(actual.rpy_rad);
    for (std::size_t joint = 0; joint < 6; ++joint) {
      auto perturbed = solution;
      perturbed[joint] += epsilon;
      const CartesianState plus = forward_state(perturbed);
      for (std::size_t axis = 0; axis < 3; ++axis)
        jacobian[axis][joint] = (plus.position_mm[axis] - actual.position_mm[axis]) / 1000.0 / epsilon;
      const Matrix3 plus_rotation = rpy_to_matrix(plus.rpy_rad);
      Matrix3 relative{};
      for (std::size_t row = 0; row < 3; ++row)
        for (std::size_t column = 0; column < 3; ++column)
          for (std::size_t inner = 0; inner < 3; ++inner)
            relative[row][column] += plus_rotation[row][inner] * base_rotation[column][inner];
      jacobian[3][joint] = (relative[2][1] - relative[1][2]) / (2.0 * epsilon);
      jacobian[4][joint] = (relative[0][2] - relative[2][0]) / (2.0 * epsilon);
      jacobian[5][joint] = (relative[1][0] - relative[0][1]) / (2.0 * epsilon);
    }
    std::array<std::array<double, 12>, 6> augmented{};
    double norm = 0.0;
    for (std::size_t row = 0; row < 6; ++row) {
      double row_sum = 0.0;
      for (std::size_t column = 0; column < 6; ++column) {
        augmented[row][column] = jacobian[row][column];
        row_sum += std::abs(jacobian[row][column]);
      }
      augmented[row][row + 6] = 1.0;
      norm = std::max(norm, row_sum);
    }
    for (std::size_t pivot = 0; pivot < 6; ++pivot) {
      std::size_t best = pivot;
      for (std::size_t row = pivot + 1; row < 6; ++row)
        if (std::abs(augmented[row][pivot]) > std::abs(augmented[best][pivot])) best = row;
      if (std::abs(augmented[best][pivot]) < 1e-9)
        throw std::runtime_error("numerical Jacobian is singular");
      std::swap(augmented[pivot], augmented[best]);
      const double divisor = augmented[pivot][pivot];
      for (double& value : augmented[pivot]) value /= divisor;
      for (std::size_t row = 0; row < 6; ++row) {
        if (row == pivot) continue;
        const double factor = augmented[row][pivot];
        for (std::size_t column = 0; column < 12; ++column)
          augmented[row][column] -= factor * augmented[pivot][column];
      }
    }
    double inverse_norm = 0.0;
    for (std::size_t row = 0; row < 6; ++row) {
      double row_sum = 0.0;
      for (std::size_t column = 6; column < 12; ++column)
        row_sum += std::abs(augmented[row][column]);
      inverse_norm = std::max(inverse_norm, row_sum);
    }
    const double condition = norm * inverse_norm;
    if (!std::isfinite(condition) || condition > options_.maximum_jacobian_condition)
      throw std::runtime_error("numerical Jacobian condition exceeds configured limit");
    return condition;
  }
  void command(const std::array<double, 6>& joints) override {
    JointValue value{};
    for (std::size_t i = 0; i < joints.size(); ++i) value.jVal[i] = joints[i];
    require_sdk(robot_.edg_servo_j(&value, ABS, 1), "edg_servo_j");
  }
  bool edg_active() const noexcept override { return edg_; }
  void cleanup() noexcept override {
    if (servo_) { record_cleanup_error(robot_.servo_move_enable(FALSE)); servo_ = false; }
    if (edg_) { record_cleanup_error(robot_.edg_init(FALSE, options_.edg_state_ip.c_str())); edg_ = false; }
    if (connected_) { record_cleanup_error(robot_.login_out()); connected_ = false; }
  }
  int cleanup_error_code() const noexcept override { return cleanup_error_code_; }
 private:
  CartesianState forward_state(const std::array<double, 6>& joints) {
    JointValue value{};
    CartesianPose pose{};
    for (std::size_t joint = 0; joint < 6; ++joint) value.jVal[joint] = joints[joint];
    require_sdk(robot_.kine_forward(&value, &pose), "kine_forward(teleop_validation)");
    CartesianState result{};
    result.position_mm = {pose.tran.x, pose.tran.y, pose.tran.z};
    result.rpy_rad = {pose.rpy.rx, pose.rpy.ry, pose.rpy.rz};
    return result;
  }
  void record_cleanup_error(errno_t code) noexcept {
    if (code != ERR_SUCC && cleanup_error_code_ == 0) cleanup_error_code_ = code;
  }
  const Options& options_;
  JAKAZuRobot robot_;
  bool connected_ = false, edg_ = false, servo_ = false;
  int cleanup_error_code_ = 0;
};

class TargetSocket {
 public:
  explicit TargetSocket(const std::string& path) : path_(path) {
    fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd_ < 0) throw std::runtime_error("target socket creation failed");
    unlink(path_.c_str());
    sockaddr_un address{}; address.sun_family = AF_UNIX;
    if (path_.size() >= sizeof(address.sun_path)) throw std::runtime_error("target socket path too long");
    std::strncpy(address.sun_path, path_.c_str(), sizeof(address.sun_path) - 1);
    if (bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) throw std::runtime_error("target socket bind failed");
  }
  ~TargetSocket() { shutdown(); }
  void shutdown() noexcept {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
    unlink(path_.c_str());
  }
  bool drain_newest(TargetPacket& newest, std::uint64_t last_sequence, bool has_previous,
                    std::uint64_t now, std::uint64_t& rejected, bool& invalid_command,
                    bool& transport_failure) noexcept {
    bool found = false;
    TargetPacket candidate{};
    while (true) {
      const auto received = recv(fd_, &candidate, sizeof(candidate), 0);
      if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (received < 0) { transport_failure = true; break; }
      if (received != static_cast<ssize_t>(sizeof(candidate))) { ++rejected; invalid_command = true; continue; }
      const bool header = candidate.magic == kTargetMagic && candidate.version == kWireVersion;
      const bool checksum = crc32(&candidate, sizeof(candidate) - sizeof(candidate.crc32)) == candidate.crc32;
      const bool kind = candidate.kind <= static_cast<std::uint16_t>(TargetKind::Stop);
      const std::uint64_t stage_before_dispatch = candidate.processing_ns ? candidate.processing_ns : candidate.local_receive_ns;
      const bool timestamp = candidate.local_receive_ns > 0 &&
                             (candidate.processing_ns == 0 || candidate.processing_ns >= candidate.local_receive_ns) &&
                             candidate.dispatch_ns >= stage_before_dispatch &&
                             candidate.dispatch_ns <= now + 5'000'000;
      const bool finite = std::all_of(std::begin(candidate.payload), std::end(candidate.payload), [](double v) { return std::isfinite(v); });
      if (!header || !checksum || !kind || !timestamp || !finite) {
        ++rejected; invalid_command = true; continue;
      }
      if ((has_previous && candidate.sequence <= last_sequence) ||
          (found && candidate.sequence <= newest.sequence)) { ++rejected; continue; }
      newest = candidate; found = true;
    }
    return found;
  }
 private:
  int fd_ = -1;
  std::string path_;
};

class StatusSender {
 public:
  explicit StatusSender(const std::string& path) : path_(path) {
    if (!path.empty()) fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
  }
  ~StatusSender() { if (fd_ >= 0) close(fd_); }
  void send_status(StatusPacket& packet) noexcept {
    if (fd_ < 0) return;
    packet.crc32 = crc32(&packet, sizeof(packet) - sizeof(packet.crc32));
    sockaddr_un address{}; address.sun_family = AF_UNIX;
    if (path_.size() >= sizeof(address.sun_path)) return;
    std::strncpy(address.sun_path, path_.c_str(), sizeof(address.sun_path) - 1);
    sendto(fd_, &packet, sizeof(packet), MSG_DONTWAIT, reinterpret_cast<sockaddr*>(&address), sizeof(address));
  }
 private:
  int fd_ = -1;
  std::string path_;
};

class JerkBoundedJointTracker {
 public:
  explicit JerkBoundedJointTracker(const Options& options) : options_(options) {}
  void reset(const std::array<double, 6>& position) {
    position_ = position;
    velocity_.fill(0.0);
    acceleration_.fill(0.0);
    initialized_ = true;
  }
  const std::array<double, 6>& update(const std::array<double, 6>& target) {
    if (!initialized_) throw std::runtime_error("joint tracker is not initialized");
    constexpr double dt = static_cast<double>(kPeriodNs) / 1e9;
    const double frequency = std::min(4.0, options_.joint_acceleration_limit_rad_s2 /
                                           options_.joint_velocity_limit_rad_s);
    for (std::size_t joint = 0; joint < 6; ++joint) {
      const double error = target[joint] - position_[joint];
      const double desired_velocity = std::clamp(frequency * error,
          -options_.joint_velocity_limit_rad_s, options_.joint_velocity_limit_rad_s);
      const double desired_acceleration = std::clamp((desired_velocity - velocity_[joint]) / dt,
          -options_.joint_acceleration_limit_rad_s2, options_.joint_acceleration_limit_rad_s2);
      const double acceleration_step = std::clamp(desired_acceleration - acceleration_[joint],
          -options_.joint_jerk_limit_rad_s3 * dt, options_.joint_jerk_limit_rad_s3 * dt);
      const double previous_acceleration = acceleration_[joint];
      acceleration_[joint] = std::clamp(acceleration_[joint] + acceleration_step,
          -options_.joint_acceleration_limit_rad_s2, options_.joint_acceleration_limit_rad_s2);
      velocity_[joint] = std::clamp(velocity_[joint] + acceleration_[joint] * dt,
          -options_.joint_velocity_limit_rad_s, options_.joint_velocity_limit_rad_s);
      position_[joint] += velocity_[joint] * dt;
      if (position_[joint] < kJointLower[joint] + options_.joint_soft_margin_rad ||
          position_[joint] > kJointUpper[joint] - options_.joint_soft_margin_rad)
        throw std::runtime_error("joint trajectory crossed soft-limit envelope");
      maximum_velocity_ = std::max(maximum_velocity_, std::abs(velocity_[joint]));
      maximum_acceleration_ = std::max(maximum_acceleration_, std::abs(acceleration_[joint]));
      maximum_jerk_ = std::max(maximum_jerk_, std::abs(acceleration_[joint] - previous_acceleration) / dt);
    }
    return position_;
  }
  const std::array<double, 6>& position() const { return position_; }
  const std::array<double, 6>& velocity() const { return velocity_; }
  const std::array<double, 6>& acceleration() const { return acceleration_; }
  bool stationary(double velocity_tolerance = 1e-4, double acceleration_tolerance = 1e-3) const {
    return std::all_of(velocity_.begin(), velocity_.end(), [=](double value) {
             return std::abs(value) <= velocity_tolerance;
           }) &&
           std::all_of(acceleration_.begin(), acceleration_.end(), [=](double value) {
             return std::abs(value) <= acceleration_tolerance;
           });
  }
  double maximum_velocity() const { return maximum_velocity_; }
  double maximum_acceleration() const { return maximum_acceleration_; }
  double maximum_jerk() const { return maximum_jerk_; }
 private:
  const Options& options_;
  std::array<double, 6> position_{}, velocity_{}, acceleration_{};
  bool initialized_ = false;
  double maximum_velocity_ = 0.0, maximum_acceleration_ = 0.0, maximum_jerk_ = 0.0;
};

struct TeleopMetrics {
  std::uint64_t producer_heartbeat_packets = 0;
  std::uint64_t ik_calls = 0;
  std::uint64_t ik_duration_total_ns = 0;
  std::uint64_t ik_duration_max_ns = 0;
  std::uint64_t kinematics_validation_calls = 0;
  std::uint64_t kinematics_validation_total_ns = 0;
  std::uint64_t kinematics_validation_max_ns = 0;
  std::uint64_t tracking_warning_cycles = 0;
  std::uint64_t tracking_hard_crossings = 0;
  double maximum_ik_joint_step_rad = 0.0;
  double maximum_tracking_difference_rad = 0.0;
  double maximum_jacobian_condition = 0.0;
  std::array<double, 6> maximum_tracking_difference_rad_per_joint{};
  std::array<std::uint64_t, 6> maximum_tracking_difference_monotonic_ns_per_joint{};
  std::array<std::uint64_t, 6> maximum_tracking_difference_sequence_per_joint{};
  std::array<double, 6> maximum_observed_joint_delta_rad_per_joint{};
  std::array<double, 6> pre_edg_measured_joint_position_rad{};
  std::array<double, 6> post_edg_q_hold_rad{};
  std::array<double, 6> pre_to_post_edg_difference_rad{};
  std::array<double, 6> zero_motion_fixed_destination_rad{};
  std::array<double, 6> zero_motion_first_command_rad{};
  std::array<double, 6> zero_motion_last_command_rad{};
  std::uint64_t zero_motion_command_count = 0;
  std::uint64_t zero_motion_command_mismatch_count = 0;
  bool zero_motion_q_hold_initialized = false;
  CartesianState startup_tcp{};
  std::array<double, 6> last_ik_target{};
  ControllerHealth last_controller_health{};
  std::uint64_t controller_health_samples = 0;
  std::uint64_t controller_alarm_events = 0;
  bool joint_specific_servo_alarm_code_available = false;
};

struct Samples {
  std::array<std::uint64_t, kMaximumSamples> periods{}, wakes{}, reads{}, writes{}, sdk{}, target_ages{}, accepted_target_ages{}, command_ages{};
  std::size_t count = 0;
  std::uint64_t missed = 0, maximum_consecutive = 0;
  std::uint64_t timing_warnings = 0, hard_timing_misses = 0, schedule_realignments = 0;
  std::string terminal_timing_fault_phase;
  std::uint64_t terminal_timing_fault_monotonic_ns = 0;
  std::uint64_t terminal_actual_cycle_period_ns = 0;
  std::uint64_t terminal_wake_lateness_ns = 0;
  std::uint64_t terminal_completion_lateness_ns = 0;
  std::uint64_t terminal_consecutive_warning_count = 0;
  int terminal_cpu = -1;
};

double percentile(std::vector<std::uint64_t>& values, double p) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double x = p * static_cast<double>(values.size() - 1);
  const auto lo = static_cast<std::size_t>(x), hi = std::min(lo + 1, values.size() - 1);
  return static_cast<double>(values[lo]) + (x - static_cast<double>(lo)) * static_cast<double>(values[hi] - values[lo]);
}

void metric_json(std::ostream& out, const char* name, const std::array<std::uint64_t, kMaximumSamples>& data, std::size_t count, bool comma) {
  std::vector<std::uint64_t> v(data.begin(), data.begin() + static_cast<std::ptrdiff_t>(count));
  long double sum = 0; for (auto x : v) sum += x;
  const double mean = v.empty() ? 0.0 : static_cast<double>(sum / v.size());
  long double sq = 0; for (auto x : v) { const long double d = x - mean; sq += d * d; }
  auto sorted = v;
  out << "    \"" << name << "\":{\"count\":" << count << ",\"mean_ns\":" << mean
      << ",\"median_ns\":" << percentile(sorted, .5) << ",\"stddev_ns\":" << (v.empty() ? 0.0 : std::sqrt(static_cast<double>(sq / v.size())))
      << ",\"min_ns\":" << (v.empty() ? 0 : *std::min_element(v.begin(), v.end())) << ",\"max_ns\":" << (v.empty() ? 0 : *std::max_element(v.begin(), v.end()))
      << ",\"p95_ns\":" << percentile(sorted, .95) << ",\"p99_ns\":" << percentile(sorted, .99);
  if (count >= 1000) out << ",\"p999_ns\":" << percentile(sorted, .999); else out << ",\"p999_ns\":null";
  out << "}" << (comma ? "," : "") << "\n";
}

double cpu_seconds(const rusage& r) { return r.ru_utime.tv_sec + r.ru_utime.tv_usec / 1e6 + r.ru_stime.tv_sec + r.ru_stime.tv_usec / 1e6; }

std::string json_escape(const std::string& value) {
  std::string result;
  result.reserve(value.size());
  for (const char c : value) {
    if (c == '\\' || c == '"') result.push_back('\\');
    if (c == '\n') { result += "\\n"; continue; }
    if (c == '\r') { result += "\\r"; continue; }
    result.push_back(c);
  }
  return result;
}

const char* scheduler_policy_name(int policy) {
  switch (policy) {
    case SCHED_OTHER: return "SCHED_OTHER";
    case SCHED_FIFO: return "SCHED_FIFO";
    case SCHED_RR: return "SCHED_RR";
#ifdef SCHED_BATCH
    case SCHED_BATCH: return "SCHED_BATCH";
#endif
#ifdef SCHED_IDLE
    case SCHED_IDLE: return "SCHED_IDLE";
#endif
    default: return "unknown";
  }
}

std::string affinity_mask_string(const PlacementEvent& event) {
  if (!event.affinity_available) return {};
  std::ostringstream out;
  bool first = true;
  for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
    if (!CPU_ISSET(cpu, &event.affinity)) continue;
    if (!first) out << ',';
    out << cpu;
    first = false;
  }
  return out.str();
}

const char* mode_name(Mode mode) {
  switch (mode) {
    case Mode::DryRun: return "native_no_robot";
    case Mode::StateRead: return "connected_state_read";
    case Mode::ZeroMotion: return "zero_motion_edg";
    case Mode::MinimalMotion: return "minimal_motion";
    case Mode::CommandShadowDryRun: return "command_shadow_fake_no_edg";
    case Mode::CommandShadow: return "command_shadow_connected_no_edg";
    case Mode::BoundedTeleopDryRun: return "bounded_teleop_fake";
    case Mode::BoundedTeleop: return "bounded_teleop_connected";
    case Mode::JointShadowDryRun: return "quest_joint_shadow_fake_no_edg";
    case Mode::JointShadow: return "quest_joint_shadow_connected_no_edg";
    case Mode::JointTeleopDryRun: return "quest_joint_teleop_fake";
    case Mode::JointTeleop: return "quest_joint_teleop_connected";
    case Mode::JointZeroMotionDryRun: return "joint_zero_motion_resampler_fake";
    case Mode::JointZeroMotion: return "joint_zero_motion_resampler_connected";
  }
  return "unknown";
}

std::string stop_classification(const std::string& outcome, int error_code) {
  if (outcome == "operator_stop_command")
    return "normal_clutch_release";
  if (outcome.find("sustained no-progress output hold") !=
      std::string::npos)
    return "native_output_no_progress_hard_fault";
  if (outcome.find("native output velocity hard boundary") !=
      std::string::npos)
    return "native_output_velocity_hard_fault";
  if (outcome.find("native output jerk hard boundary") !=
      std::string::npos)
    return "native_output_jerk_hard_fault";
  if (outcome.find("native output acceleration hard boundary") !=
          std::string::npos ||
      outcome.find("diagnostic output acceleration boundary") !=
          std::string::npos)
    return "native_output_acceleration_hard_fault";
  if (outcome.find("controller ") != std::string::npos ||
      outcome.find("robot is in E-stop or collision") != std::string::npos ||
      outcome.find("powered, and servo-enabled") != std::string::npos)
    return "controller_alarm";
  if (outcome.find("edg_servo_j failed") != std::string::npos ||
      outcome.find("injected fake SDK failure") != std::string::npos)
    return "SDK_command_failure";
  if (outcome == "target_transport_failure" ||
      outcome == "invalid_command")
    return "IPC_failure";
  if (outcome.find("target_timeout") != std::string::npos ||
      outcome == "command_stream_timeout")
    return "producer_liveness_loss";
  if (outcome.find("timing_miss") != std::string::npos ||
      outcome == "control_loop_overrun")
    return "hard_timing_fault";
  if (outcome.find("tracking") != std::string::npos)
    return "tracking_hard_fault";
  if (outcome.find("non-finite") != std::string::npos)
    return "non_finite_output_hard_fault";
  return error_code == 0 ? "normal_completion" : "worker_exit";
}

void write_metrics(const Options& o, const Samples& s, std::uint64_t accepted, std::uint64_t rejected,
                   std::uint64_t warning_cycles,
                   double elapsed_s, double cpu_s, double maximum_command_delta_rad,
                   double maximum_observed_delta_rad, int error_code, int cleanup_error_code,
                   const std::string& outcome, const TeleopMetrics& teleop,
                   const JerkBoundedJointTracker& tracker,
                   const std::array<double, 6>& initial_joint_position_rad,
                   const JointServoResampler& resampler,
                   const OutputMotionDiagnostics& output_diagnostics,
                   const OutputNoProgressHoldTracker& output_hold,
                   std::uint64_t transition_limited_progress_points,
                   const std::array<double, 6>& final_accepted_target_rad,
                   const PlacementEvidence& placement,
                   const BoundarySystemObserver& system_observer) {
  std::ofstream file;
  std::ostream* output = &std::cout;
  if (!o.metrics_file.empty()) { file.open(o.metrics_file); if (!file) throw std::runtime_error("cannot open metrics file"); output = &file; }
  auto& out = *output;
  out << std::setprecision(12) << "{\n  \"schema_version\":\"jaka_worker_metrics.v3\",\n"
      << "  \"mode\":\"" << mode_name(o.mode) << "\",\n"
      << "  \"outcome\":\"" << outcome << "\",\n"
      << "  \"stop_classification\":\""
      << stop_classification(outcome, error_code) << "\",\n"
      << "  \"requested_period_ns\":" << kPeriodNs << ",\n"
      << "  \"elapsed_s\":" << elapsed_s << ",\n  \"worker_cpu_s\":" << cpu_s << ",\n  \"worker_cpu_percent\":" << (elapsed_s > 0 ? cpu_s / elapsed_s * 100.0 : 0.0) << ",\n"
      << "  \"loop_rate_hz\":" << (elapsed_s > 0 ? s.count / elapsed_s : 0.0) << ",\n"
      << "  \"accepted_target_rate_hz\":" << (elapsed_s > 0 ? accepted / elapsed_s : 0.0) << ",\n"
      << "  \"maximum_intentional_command_delta_rad\":" << maximum_command_delta_rad << ",\n"
      << "  \"maximum_observed_joint_delta_rad\":" << maximum_observed_delta_rad << ",\n"
      << "  \"accepted_targets\":" << accepted << ",\n  \"rejected_targets\":" << rejected << ",\n"
      << "  \"producer_heartbeat_packets\":" << teleop.producer_heartbeat_packets << ",\n"
      << "  \"target_age_warning_cycles\":" << warning_cycles << ",\n"
      << "  \"error_code\":" << error_code << ",\n  \"cleanup_error_code\":" << cleanup_error_code << ",\n"
      << "  \"ik_calls\":" << teleop.ik_calls << ",\n"
      << "  \"ik_duration_mean_ns\":" << (teleop.ik_calls ? teleop.ik_duration_total_ns / teleop.ik_calls : 0) << ",\n"
      << "  \"ik_duration_max_ns\":" << teleop.ik_duration_max_ns << ",\n"
      << "  \"kinematics_validation_calls\":" << teleop.kinematics_validation_calls << ",\n"
      << "  \"kinematics_validation_mean_ns\":" << (teleop.kinematics_validation_calls ? teleop.kinematics_validation_total_ns / teleop.kinematics_validation_calls : 0) << ",\n"
      << "  \"kinematics_validation_max_ns\":" << teleop.kinematics_validation_max_ns << ",\n"
      << "  \"maximum_jacobian_condition\":" << teleop.maximum_jacobian_condition << ",\n"
      << "  \"maximum_ik_joint_step_rad\":" << teleop.maximum_ik_joint_step_rad << ",\n"
      << "  \"maximum_tracking_difference_rad\":" << teleop.maximum_tracking_difference_rad << ",\n"
      << "  \"tracking_warning_cycles\":" << teleop.tracking_warning_cycles << ",\n"
      << "  \"tracking_hard_crossings\":" << teleop.tracking_hard_crossings << ",\n"
      << "  \"controller_health_samples\":" << teleop.controller_health_samples << ",\n"
      << "  \"controller_alarm_events\":" << teleop.controller_alarm_events << ",\n"
      << "  \"controller_error_code\":" << teleop.last_controller_health.controller_error_code << ",\n"
      << "  \"controller_error_message\":\"" << json_escape(teleop.last_controller_health.controller_error_message) << "\",\n"
      << "  \"controller_health_monitor_failure\":\"" << json_escape(teleop.last_controller_health.monitor_failure) << "\",\n"
      << "  \"controller_collision\":" << (teleop.last_controller_health.collision ? "true" : "false") << ",\n"
      << "  \"controller_emergency_stop\":" << (teleop.last_controller_health.emergency_stop ? "true" : "false") << ",\n"
      << "  \"controller_powered_on\":" << (teleop.last_controller_health.powered_on ? "true" : "false") << ",\n"
      << "  \"controller_enabled\":" << (teleop.last_controller_health.enabled ? "true" : "false") << ",\n"
      << "  \"controller_status_sdk_return_code\":" << teleop.last_controller_health.status_sdk_return_code << ",\n"
      << "  \"collision_status_sdk_return_code\":" << teleop.last_controller_health.collision_sdk_return_code << ",\n"
      << "  \"estop_status_sdk_return_code\":" << teleop.last_controller_health.estop_sdk_return_code << ",\n"
      << "  \"controller_event_monotonic_ns\":" << teleop.last_controller_health.monotonic_ns << ",\n"
      << "  \"controller_event_wall_clock_unix_ns\":" << teleop.last_controller_health.wall_ns << ",\n"
      << "  \"joint_specific_servo_alarm_code_available\":" << (teleop.joint_specific_servo_alarm_code_available ? "true" : "false") << ",\n"
      << "  \"controller_alarm_history_available\":false,\n"
      << "  \"initial_joint_position_rad\":[" << initial_joint_position_rad[0] << ',' << initial_joint_position_rad[1] << ',' << initial_joint_position_rad[2] << ',' << initial_joint_position_rad[3] << ',' << initial_joint_position_rad[4] << ',' << initial_joint_position_rad[5] << "],\n"
      << "  \"edg_step_num\":1,\n"
      << "  \"resampler_timestamp_domain\":\"AcceptedArmTarget.generated_monotonic_ns/CLOCK_MONOTONIC\",\n"
      << "  \"resampler_emitted_points\":" << resampler.emitted_points() << ",\n"
      << "  \"resampler_repeated_points\":" << resampler.repeated_points() << ",\n"
      << "  \"resampler_destination_switches\":" << resampler.destination_switches() << ",\n"
      << "  \"resampler_preemptions\":" << resampler.preemptions() << ",\n"
      << "  \"resampler_endpoint_points\":" << resampler.endpoint_points() << ",\n"
      << "  \"resampler_transition_limited_points\":"
      << resampler.transition_limited_points() << ",\n"
      << "  \"transition_limited_progress_points\":"
      << transition_limited_progress_points << ",\n"
      << "  \"output_hold_destination_gap_rad\":"
      << kOutputHoldDestinationGapRad << ",\n"
      << "  \"output_hold_minimum_progress_rad\":"
      << kOutputHoldMinimumProgressRad << ",\n"
      << "  \"resampler_maximum_segment_duration_ns\":" << resampler.maximum_segment_duration_ns() << ",\n"
      << "  \"resampler_maximum_endpoint_latency_ns\":" << resampler.maximum_endpoint_latency_ns() << ",\n"
      << "  \"resampler_active_segment\":" << (resampler.active() ? "true" : "false") << ",\n"
      << "  \"output_joint_velocity_boundary_rad_s\":" << o.maximum_output_joint_velocity_rad_s << ",\n"
      << "  \"output_joint_velocity_boundary_provenance\":\"project-selected normal operating boundary, bounded by official ServoJ 180 deg/s ceiling\",\n"
      << "  \"official_servoj_joint_speed_ceiling_rad_s\":" << M_PI << ",\n"
      << "  \"official_servoj_joint_speed_ceiling_provenance\":\"JAKA ServoJ documentation\",\n"
      << "  \"startup_timing_grace_cycles\":" << o.startup_timing_grace_cycles << ",\n"
      << "  \"completion_warning_lateness_ns\":" << kPeriodNs << ",\n"
      << "  \"completion_hard_lateness_ns\":12000000,\n"
      << "  \"timing_policy_provenance\":\"project-selected bounded 8 ms policy; not a JAKA acceleration/latency maximum\",\n"
      << "  \"recoverable_output_joint_acceleration_boundary_rad_s2\":" << o.diagnostic_joint_acceleration_boundary_rad_s2 << ",\n"
      << "  \"diagnostic_joint_acceleration_boundary_rad_s2\":" << o.diagnostic_joint_acceleration_boundary_rad_s2 << ",\n"
      << "  \"native_output_joint_acceleration_hard_boundary_rad_s2\":" << o.maximum_output_joint_acceleration_rad_s2 << ",\n"
      << "  \"output_joint_acceleration_boundary_provenance\":\"project-selected; no Mini2 ServoJ acceleration maximum was found in official documentation or installed SDK readback\",\n"
      << "  \"output_joint_jerk_hard_boundary_rad_s3\":" << o.output_joint_jerk_limit_rad_s3 << ",\n"
      << "  \"output_joint_jerk_tolerance_absolute_rad_s3\":"
      << jaka_servo::kOutputJerkHardBoundaryToleranceAbsoluteRadS3 << ",\n"
      << "  \"output_joint_jerk_tolerance_relative\":"
      << jaka_servo::kOutputJerkHardBoundaryToleranceRelative << ",\n"
      << "  \"output_joint_jerk_hard_boundary_with_tolerance_rad_s3\":"
      << jaka_servo::output_jerk_hard_boundary_with_tolerance(
             o.output_joint_jerk_limit_rad_s3) << ",\n"
      << "  \"output_joint_jerk_tolerance_provenance\":\"numeric comparison envelope: observed 1.3e-5 rad/s3 finite-difference discrepancy at 62.831853 rad/s3; nominal limit unchanged\",\n"
      << "  \"recover_output_acceleration_transition\":" << (o.recover_output_acceleration_transition ? "true" : "false") << ",\n"
      << "  \"output_acceleration_hold_degraded_ns\":" << o.output_acceleration_hold_degraded_ns << ",\n"
      << "  \"output_acceleration_hold_hard_stop_ns\":" << o.output_acceleration_hold_hard_stop_ns << ",\n"
      << "  \"maximum_consecutive_output_acceleration_hold_cycles\":" << o.maximum_consecutive_output_acceleration_hold_cycles << ",\n"
      << "  \"true_output_hold_count\":" << output_hold.hold_count() << ",\n"
      << "  \"recovered_from_true_output_hold_count\":" << output_hold.recovery_count() << ",\n"
      << "  \"recoverable_output_acceleration_hold_count\":" << output_hold.hold_count() << ",\n"
      << "  \"recovered_from_output_acceleration_hold_count\":" << output_hold.recovery_count() << ",\n"
      << "  \"output_acceleration_hold_degraded_count\":" << output_hold.degraded_count() << ",\n"
      << "  \"output_acceleration_hold_total_duration_ns\":" << output_hold.total_hold_duration_ns() << ",\n"
      << "  \"output_acceleration_hold_longest_duration_ns\":" << output_hold.longest_hold_duration_ns() << ",\n"
      << "  \"output_acceleration_hold_maximum_consecutive_cycles\":" << output_hold.maximum_consecutive_cycles() << ",\n"
      << "  \"output_acceleration_hold_last_recovery_sequence\":" << output_hold.recovery_sequence() << ",\n"
      << "  \"maximum_joint_velocity_rad_s\":" << tracker.maximum_velocity() << ",\n"
      << "  \"maximum_joint_acceleration_rad_s2\":" << tracker.maximum_acceleration() << ",\n"
      << "  \"maximum_joint_jerk_rad_s3\":" << tracker.maximum_jerk() << ",\n"
      << "  \"final_joint_velocity_max_rad_s\":" << *std::max_element(tracker.velocity().begin(), tracker.velocity().end(), [](double a, double b) { return std::abs(a) < std::abs(b); }) << ",\n"
      << "  \"final_joint_acceleration_max_rad_s2\":" << *std::max_element(tracker.acceleration().begin(), tracker.acceleration().end(), [](double a, double b) { return std::abs(a) < std::abs(b); }) << ",\n"
      << "  \"startup_tcp_mm_rpy_rad\":[" << teleop.startup_tcp.position_mm[0] << ',' << teleop.startup_tcp.position_mm[1] << ',' << teleop.startup_tcp.position_mm[2] << ',' << teleop.startup_tcp.rpy_rad[0] << ',' << teleop.startup_tcp.rpy_rad[1] << ',' << teleop.startup_tcp.rpy_rad[2] << "],\n"
      << "  \"last_ik_target_rad\":[" << teleop.last_ik_target[0] << ',' << teleop.last_ik_target[1] << ',' << teleop.last_ik_target[2] << ',' << teleop.last_ik_target[3] << ',' << teleop.last_ik_target[4] << ',' << teleop.last_ik_target[5] << "],\n"
      << "  \"missed_deadlines\":" << s.missed << ",\n  \"max_consecutive_missed_deadlines\":" << s.maximum_consecutive << ",\n"
      << "  \"timing_warning_events\":" << s.timing_warnings << ",\n"
      << "  \"hard_timing_misses\":" << s.hard_timing_misses << ",\n"
      << "  \"schedule_realignments\":" << s.schedule_realignments << ",\n"
      << "  \"terminal_timing_fault\":";
  if (s.terminal_timing_fault_phase.empty()) {
    out << "null,\n";
  } else {
    out << "{\"phase\":\"" << s.terminal_timing_fault_phase
        << "\",\"monotonic_ns\":" << s.terminal_timing_fault_monotonic_ns
        << ",\"actual_cycle_period_ns\":" << s.terminal_actual_cycle_period_ns
        << ",\"wake_lateness_ns\":" << s.terminal_wake_lateness_ns
        << ",\"completion_lateness_ns\":" << s.terminal_completion_lateness_ns
        << ",\"consecutive_warning_count\":" << s.terminal_consecutive_warning_count
        << ",\"cpu\":" << s.terminal_cpu << "},\n";
  }
  out << "  \"worker_placement\":{\"migration_count\":"
      << placement.migration_count()
      << ",\"dropped_events\":" << placement.dropped_events()
      << ",\"events\":[";
  for (std::size_t index = 0; index < placement.count(); ++index) {
    const auto& event = placement.events()[index];
    out << (index ? "," : "")
        << "{\"reason\":\"" << placement_event_reason_name(event.reason)
        << "\",\"monotonic_ns\":" << event.monotonic_ns
        << ",\"process_id\":" << event.process_id
        << ",\"thread_id\":" << event.thread_id
        << ",\"cpu\":" << event.cpu
        << ",\"previous_cpu\":" << event.previous_cpu
        << ",\"migration_count\":" << event.migration_count
        << ",\"time_since_last_migration_ns\":"
        << event.time_since_last_migration_ns
        << ",\"migration_near_boundary\":"
        << ((event.reason == PlacementEventReason::FirstTimingWarning ||
             event.reason == PlacementEventReason::TerminalTimingFault) &&
            event.migration_count > 0 &&
            event.time_since_last_migration_ns <= 50'000'000
                ? "true" : "false")
        << ",\"scheduler_policy\":\""
        << scheduler_policy_name(event.scheduler_policy)
        << "\",\"scheduler_policy_value\":" << event.scheduler_policy
        << ",\"scheduler_priority\":" << event.scheduler_priority
        << ",\"nice_value\":" << event.nice_value
        << ",\"affinity_mask\":\""
        << affinity_mask_string(event)
        << "\",\"locked_memory_kb\":" << event.locked_memory_kb << '}';
  }
  out << "]},\n";
  out << "  \"system_boundary_observer\":{\"startup_error\":\""
      << json_escape(system_observer.startup_error())
      << "\",\"runtime_error\":\""
      << json_escape(system_observer.runtime_error())
      << "\",\"dropped_requests\":" << system_observer.dropped_requests()
      << ",\"snapshots\":[";
  for (std::size_t index = 0; index < system_observer.snapshots().size(); ++index) {
    const auto& snapshot = system_observer.snapshots()[index];
    out << (index ? "," : "")
        << "{\"trigger\":\""
        << system_snapshot_trigger_name(snapshot.request.trigger)
        << "\",\"requested_monotonic_ns\":"
        << snapshot.request.requested_monotonic_ns
        << ",\"trigger_cpu\":" << snapshot.request.trigger_cpu
        << ",\"collected_start_monotonic_ns\":"
        << snapshot.collected_start_monotonic_ns
        << ",\"collected_end_monotonic_ns\":"
        << snapshot.collected_end_monotonic_ns
        << ",\"observer_thread_id\":" << snapshot.observer_thread_id
        << ",\"observer_cpu\":" << snapshot.observer_cpu
        << ",\"observer_scheduler_policy\":\""
        << scheduler_policy_name(snapshot.observer_scheduler_policy)
        << "\",\"observer_scheduler_priority\":"
        << snapshot.observer_scheduler_priority
        << ",\"observer_nice_value\":" << snapshot.observer_nice_value
        << ",\"proc_stat\":\"" << json_escape(snapshot.proc_stat)
        << "\",\"proc_loadavg\":\"" << json_escape(snapshot.proc_loadavg)
        << "\",\"proc_interrupts\":\"" << json_escape(snapshot.proc_interrupts)
        << "\",\"proc_softirqs\":\"" << json_escape(snapshot.proc_softirqs)
        << "\",\"pressure_cpu\":\"" << json_escape(snapshot.pressure_cpu)
        << "\",\"pressure_io\":\"" << json_escape(snapshot.pressure_io)
        << "\",\"pressure_memory\":\"" << json_escape(snapshot.pressure_memory)
        << "\",\"cpu_frequency\":\"" << json_escape(snapshot.cpu_frequency)
        << "\",\"cpu_governor\":\"" << json_escape(snapshot.cpu_governor)
        << "\",\"thermal_state\":\"" << json_escape(snapshot.thermal_state)
        << "\",\"errors\":\"" << json_escape(snapshot.errors) << "\"}";
  }
  out << "]},\n";
  out << "  \"statistics\":{\n";
  metric_json(out, "actual_cycle_period", s.periods, s.count, true);
  metric_json(out, "wake_lateness", s.wakes, s.count, true);
  metric_json(out, "sdk_call_duration", s.sdk, s.count, true);
  metric_json(out, "state_read_duration", s.reads, s.count, true);
  metric_json(out, "command_write_duration", s.writes, s.count, true);
  metric_json(out, "transport_age", s.target_ages, s.count, true);
  metric_json(out, "producer_heartbeat_age", s.target_ages, s.count, true);
  metric_json(out, "accepted_target_age", s.accepted_target_ages, s.count, true);
  metric_json(out, "command_age", s.command_ages, s.count, false);
  out << "  },\n";
  auto write_six = [&out](const char* name, const auto& values, bool comma) {
    out << "  \"" << name << "\":[";
    for (std::size_t joint = 0; joint < 6; ++joint) out << (joint ? "," : "") << values[joint];
    out << "]" << (comma ? "," : "") << "\n";
  };
  write_six("output_joint_velocity_boundary_rad_s_per_joint",
            o.maximum_output_joint_velocity_rad_s_per_joint, true);
  write_six("output_maximum_adjacent_delta_rad", output_diagnostics.maximum_delta(), true);
  write_six("output_maximum_velocity_rad_s", output_diagnostics.maximum_velocity(), true);
  write_six("output_maximum_acceleration_rad_s2", output_diagnostics.maximum_acceleration(), true);
  write_six("output_maximum_jerk_rad_s3", output_diagnostics.maximum_jerk(), true);
  write_six("last_output_check_raw_jerk_rad_s3",
            output_diagnostics.last_checked_jerk(), true);
  write_six("output_speed_boundary_rejections", output_diagnostics.speed_boundary_rejections(), true);
  write_six("output_acceleration_boundary_crossings", output_diagnostics.acceleration_boundary_crossings(), true);
  write_six("output_acceleration_boundary_rejections", output_diagnostics.acceleration_boundary_rejections(), true);
  write_six("output_acceleration_hard_boundary_rejections", output_diagnostics.hard_acceleration_boundary_rejections(), true);
  std::array<double, 6> endpoint_error{};
  if (resampler.emitted_points() > 0)
    for (std::size_t joint = 0; joint < 6; ++joint)
      endpoint_error[joint] = resampler.emitted()[joint] - final_accepted_target_rad[joint];
  write_six("final_resampler_endpoint_error_rad", endpoint_error, true);
  write_six("maximum_tracking_difference_rad_per_joint", teleop.maximum_tracking_difference_rad_per_joint, true);
  write_six("maximum_tracking_difference_monotonic_ns_per_joint", teleop.maximum_tracking_difference_monotonic_ns_per_joint, true);
  write_six("maximum_tracking_difference_sequence_per_joint", teleop.maximum_tracking_difference_sequence_per_joint, true);
  write_six("maximum_observed_joint_delta_rad_per_joint", teleop.maximum_observed_joint_delta_rad_per_joint, true);
  // Explicit E1 name; retain maximum_observed_joint_delta_rad_per_joint above
  // as a backwards-compatible alias for existing metrics readers.
  write_six("maximum_measured_displacement_from_q_hold_rad_per_joint",
            teleop.maximum_observed_joint_delta_rad_per_joint, true);
  write_six("pre_edg_measured_joint_position_rad", teleop.pre_edg_measured_joint_position_rad, true);
  write_six("post_edg_authoritative_q_hold_rad", teleop.post_edg_q_hold_rad, true);
  write_six("pre_to_post_edg_difference_rad", teleop.pre_to_post_edg_difference_rad, true);
  write_six("zero_motion_fixed_destination_rad", teleop.zero_motion_fixed_destination_rad, true);
  write_six("zero_motion_first_command_rad", teleop.zero_motion_first_command_rad, true);
  write_six("zero_motion_last_command_rad", teleop.zero_motion_last_command_rad, true);
  const auto maximum_of = [](const auto& values) {
    return *std::max_element(values.begin(), values.end());
  };
  out << "  \"output_maximum_adjacent_delta_rad_global\":" << maximum_of(output_diagnostics.maximum_delta()) << ",\n"
      << "  \"output_maximum_velocity_rad_s_global\":" << maximum_of(output_diagnostics.maximum_velocity()) << ",\n"
      << "  \"output_maximum_acceleration_rad_s2_global\":" << maximum_of(output_diagnostics.maximum_acceleration()) << ",\n"
      << "  \"zero_motion_command_count\":" << teleop.zero_motion_command_count << ",\n"
      << "  \"zero_motion_command_mismatch_count\":" << teleop.zero_motion_command_mismatch_count << ",\n"
      << "  \"zero_motion_q_hold_initialized\":" << (teleop.zero_motion_q_hold_initialized ? "true" : "false") << "\n";
  out << "}\n";
}

double smoothstep5(double x) { x = std::clamp(x, 0.0, 1.0); return x*x*x*(10.0 + x*(-15.0 + 6.0*x)); }

int run(const Options& o) {
  std::signal(SIGINT, signal_handler); std::signal(SIGTERM, signal_handler); std::signal(SIGHUP, signal_handler);
  auto backend = uses_fake_backend(o.mode) ? std::unique_ptr<Backend>(new FakeBackend(o)) : std::unique_ptr<Backend>(new RealBackend(o));
  TargetSocket target_socket(o.target_socket); StatusSender status_sender(o.status_socket);
  PlacementEvidence placement;
  const auto placement_start_ns = now_ns();
  const int placement_start_cpu = sched_getcpu();
  placement.observe_cpu(placement_start_ns, placement_start_cpu);
  placement.record(PlacementEventReason::WorkerStart, placement_start_ns,
                   placement_start_cpu, -1, true, true);
  BoundarySystemObserver system_observer(!o.metrics_file.empty());
  auto samples_storage = std::make_unique<Samples>();
  Samples& samples = *samples_storage;
  JerkBoundedJointTracker tracker(o);
  JointServoResampler joint_resampler;
  OutputMotionDiagnostics output_diagnostics(o);
  OutputNoProgressHoldTracker output_hold(o);
  std::vector<RecordedServoPoint> recorded_servo_points;
  if (!o.emitted_points_file.empty()) recorded_servo_points.reserve(4096);
  std::vector<CycleTelemetry> cycle_telemetry;
  if (!o.cycle_telemetry_file.empty())
    cycle_telemetry.reserve(static_cast<std::size_t>(o.duration_s * 125.0) + 16);
  TeleopMetrics teleop{};
  TargetPacket latest{}; bool ever_received = false;
  std::uint64_t accepted = 0, rejected = 0, last_sequence = 0, last_dispatch = 0, last_target_dispatch = 0, consecutive_overruns = 0;
  std::uint64_t consecutive_timing_warnings = 0, consecutive_completion_misses = 0;
  bool first_timing_warning_recorded = false;
  std::uint32_t startup_timing_cycles = 0;
  bool startup_timing_grace_active = is_joint_teleop_mode(o.mode);
  std::uint64_t warning_cycles = 0;
  std::uint64_t transition_limited_progress_points = 0;
  std::uint64_t status_accepted = 0, status_rejected = 0;
  bool output_acceleration_hold_status_pending = false;
  bool output_acceleration_recovery_status_pending = false;
  std::array<double, 6> initial{}, observed{}, target{}, ik_target{};
  std::array<double, 6> tracking_reference{}, command_reference{};
  bool has_ik_target = false;
  bool first_external_joint_target_received = false;
  bool pause_requested = false;
  bool pause_stopped_ready = false;
  bool stop_requested = false;
  std::uint64_t stop_request_ns = 0;
  std::string stop_reason;
  std::uint64_t consecutive_tracking_crossings = 0;
  std::array<double, 6> previous_absolute_tracking_difference{};
  std::array<std::uint32_t, 6> consecutive_increasing_tracking_cycles{};
  double maximum_command_delta_rad = 0.0, maximum_observed_delta_rad = 0.0;
  State state = State::Connecting; std::int32_t error_code = 0;
  const char* outcome = "completed";
  std::string fault_outcome;
  rusage usage_start{}, usage_end{}; getrusage(RUSAGE_SELF, &usage_start);
  auto start = now_ns(); auto previous = start; auto deadline = start;
  try {
    backend->connect(); state = State::Connected;
    backend->verify(o.expected_tool_id, o.expected_user_frame_id); state = State::Armed;
    if (o.monitor_controller_health_each_cycle) {
      teleop.last_controller_health = backend->read_controller_health();
      ++teleop.controller_health_samples;
      require_healthy_controller(teleop.last_controller_health);
    }
    backend->read(initial); observed = initial; target = initial;
    tracking_reference = command_reference = initial;
    teleop.pre_edg_measured_joint_position_rad = initial;
    if (!std::all_of(initial.begin(), initial.end(), [](double v) { return std::isfinite(v) && std::abs(v) <= 2.0 * M_PI; })) throw std::runtime_error("initial joint state failed radians/finiteness check");
    if (o.mode == Mode::StateRead || is_shadow_mode(o.mode) ||
        is_bounded_mode(o.mode) || is_joint_shadow_mode(o.mode) ||
        is_joint_teleop_mode(o.mode)) {
      backend->read_tcp(teleop.startup_tcp);
    }
    if (is_shadow_mode(o.mode) || is_bounded_mode(o.mode)) {
      validate_joint_solution(initial, initial, o);
    } else if (is_joint_mode(o.mode)) {
      validate_manufacturer_joint_position_limits(initial);
    }
    if (o.mode == Mode::MinimalMotion) {
      auto endpoint = initial; endpoint[static_cast<std::size_t>(o.probe_joint)] += o.probe_delta_rad;
      backend->validate_probe(initial, endpoint);
    }
    if (is_joint_zero_motion_mode(o.mode) || is_joint_teleop_mode(o.mode)) {
      backend->enter_edg();
      state = State::EdgReady;
      backend->read(observed);
      validate_manufacturer_joint_position_limits(observed);
      const auto handoff_ns = now_ns();
      const std::array<double, 6> q_hold = observed;
      tracking_reference = command_reference = q_hold;
      ik_target = q_hold;
      teleop.last_ik_target = q_hold;
      teleop.post_edg_q_hold_rad = q_hold;
      for (std::size_t joint = 0; joint < q_hold.size(); ++joint)
        teleop.pre_to_post_edg_difference_rad[joint] = q_hold[joint] - initial[joint];
      joint_resampler.initialize(q_hold, handoff_ns);
      // Sequence zero is an internal transport hold, leaving the external
      // AcceptedArmTarget sequence space untouched.  The worker emits this
      // exact post-EDG state while waiting for a fresh, aligned target.
      joint_resampler.hold(q_hold, handoff_ns, 0);
      output_diagnostics.initialize(q_hold, handoff_ns);
      has_ik_target = true;
      if (is_joint_zero_motion_mode(o.mode)) {
        teleop.zero_motion_fixed_destination_rad = q_hold;
        teleop.zero_motion_q_hold_initialized = true;
        state = State::Running;
      } else {
        state = State::Holding;
      }
    }
    if (o.mode != Mode::StateRead && !is_stream_mode(o.mode)) { backend->enter_edg(); state = State::EdgReady; backend->read(observed);
      double delta = 0; for (std::size_t i = 0; i < 6; ++i) delta = std::max(delta, std::abs(observed[i] - initial[i]));
      if (delta > 1e-4) throw std::runtime_error("near-zero initial command delta check failed");
    }
    if (!is_joint_zero_motion_mode(o.mode)) state = State::Holding;
    // Connection, verification, and initial state reads are setup work, not an
    // 8 ms command-stream cycle. Start deadline monitoring only after setup so
    // normal SDK/network startup latency cannot cause a false first-cycle abort.
    start = now_ns();
    previous = start;
    deadline = start;
    bool fake_start_delay_injected = false;
    while (!g_stop.load(std::memory_order_relaxed) && samples.count < kMaximumSamples) {
      deadline += kPeriodNs;
      timespec wake{static_cast<time_t>(deadline / 1'000'000'000), static_cast<long>(deadline % 1'000'000'000)};
      while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr) == EINTR && !g_stop.load()) {}
      if (!fake_start_delay_injected && o.fake_start_delay_once_ns > 0) {
        timespec delay{
            static_cast<time_t>(o.fake_start_delay_once_ns / 1'000'000'000),
            static_cast<long>(o.fake_start_delay_once_ns % 1'000'000'000)};
        while (nanosleep(&delay, &delay) == -1 && errno == EINTR && !g_stop.load()) {}
        fake_start_delay_injected = true;
      }
      const auto cycle_start = now_ns();
      const int cycle_cpu = sched_getcpu();
      placement.observe_cpu(cycle_start, cycle_cpu);
      if (startup_timing_grace_active) {
        if (ever_received || startup_timing_cycles >= o.startup_timing_grace_cycles)
          startup_timing_grace_active = false;
        else
          ++startup_timing_cycles;
      }
      if (cycle_start - start >= static_cast<std::uint64_t>(o.duration_s * 1e9)) {
        if (is_joint_zero_motion_mode(o.mode) && backend->edg_active()) {
          state = State::ControlledStop;
          outcome = "zero_motion_duration_complete";
          break;
        } else if (is_joint_teleop_mode(o.mode) && backend->edg_active()) {
          state = State::ControlledStop;
          outcome = "maximum_session_duration";
          break;
        } else if (is_bounded_mode(o.mode) && backend->edg_active()) {
          if (!stop_requested) {
            stop_requested = true;
            stop_request_ns = cycle_start;
            stop_reason = "maximum_session_duration";
            state = State::Holding;
          }
        } else break;
      }
      const std::uint64_t start_period = cycle_start - previous;
      const std::uint64_t wake_lateness = cycle_start > deadline ? cycle_start - deadline : 0;
      bool schedule_realign = false;
      bool timing_rearmed_after_edg = false;
      if (is_stream_mode(o.mode)) {
        // A single sub-period scheduler delay is recoverable by re-aligning the
        // absolute deadline. Fault only after a full 8 ms wake is missed, a
        // complete 16 ms start interval is exceeded, or warnings repeat.
        const bool startup_grace = startup_timing_grace_active && !ever_received;
        if (start_period > 16'000'000 ||
            (wake_lateness >= kPeriodNs && !startup_grace)) {
          const std::size_t row = samples.count++;
          samples.periods[row] = start_period;
          samples.wakes[row] = wake_lateness;
          ++samples.hard_timing_misses;
          samples.terminal_timing_fault_phase = "cycle_start";
          samples.terminal_timing_fault_monotonic_ns = cycle_start;
          samples.terminal_actual_cycle_period_ns = start_period;
          samples.terminal_wake_lateness_ns = wake_lateness;
          samples.terminal_consecutive_warning_count =
              consecutive_timing_warnings + 1;
          samples.terminal_cpu = cycle_cpu;
          placement.record(PlacementEventReason::TerminalTimingFault,
                           cycle_start, cycle_cpu, placement.previous_cpu(),
                           true, false);
          system_observer.request(SystemSnapshotTrigger::TerminalTimingFault,
                                  cycle_start, cycle_cpu);
          state = State::Fault;
          outcome = "hard_start_timing_miss";
          break;
        }
        const bool warning = start_period > 8'800'000 || wake_lateness > 2'000'000;
        if (warning) {
          ++samples.timing_warnings;
          ++consecutive_timing_warnings;
          schedule_realign = true;
          if (!first_timing_warning_recorded) {
            first_timing_warning_recorded = true;
            placement.record(PlacementEventReason::FirstTimingWarning,
                             cycle_start, cycle_cpu,
                             placement.previous_cpu(), false, false);
            system_observer.request(SystemSnapshotTrigger::FirstTimingWarning,
                                    cycle_start, cycle_cpu);
          }
        } else consecutive_timing_warnings = 0;
        if (consecutive_timing_warnings >= 2 && !startup_grace) {
          const std::size_t row = samples.count++;
          samples.periods[row] = start_period;
          samples.wakes[row] = wake_lateness;
          ++samples.hard_timing_misses;
          samples.terminal_timing_fault_phase = "cycle_start";
          samples.terminal_timing_fault_monotonic_ns = cycle_start;
          samples.terminal_actual_cycle_period_ns = start_period;
          samples.terminal_wake_lateness_ns = wake_lateness;
          samples.terminal_consecutive_warning_count =
              consecutive_timing_warnings;
          samples.terminal_cpu = cycle_cpu;
          placement.record(PlacementEventReason::TerminalTimingFault,
                           cycle_start, cycle_cpu, placement.previous_cpu(),
                           true, false);
          system_observer.request(SystemSnapshotTrigger::TerminalTimingFault,
                                  cycle_start, cycle_cpu);
          state = State::Fault;
          outcome = "consecutive_start_timing_misses";
          break;
        }
      }
      TargetPacket packet{};
      bool invalid_command = false, transport_failure = false;
      if (!is_joint_zero_motion_mode(o.mode) &&
          target_socket.drain_newest(packet, last_sequence, ever_received, cycle_start, rejected,
                                     invalid_command, transport_failure)) {
        latest = packet; last_sequence = packet.sequence; last_dispatch = packet.dispatch_ns; ever_received = true; ++accepted;
        const bool packet_stop = packet.kind == static_cast<std::uint16_t>(TargetKind::Stop);
        const bool packet_heartbeat = packet.kind == static_cast<std::uint16_t>(TargetKind::Heartbeat);
        const bool packet_hold_current =
            packet.kind == static_cast<std::uint16_t>(TargetKind::HoldCurrent);
        if (packet_heartbeat) ++teleop.producer_heartbeat_packets;
        else if (!packet_stop && !packet_hold_current)
          last_target_dispatch = packet.dispatch_ns;
        if (packet_stop && is_joint_teleop_mode(o.mode) && backend->edg_active()) {
          state = State::ControlledStop;
          outcome = "operator_stop_command";
        } else if (packet_stop && is_bounded_mode(o.mode) && backend->edg_active()) {
          if (!stop_requested) {
            stop_requested = true;
            stop_request_ns = cycle_start;
            stop_reason = "operator_stop_command";
          }
          state = State::Holding;
        } else {
          if (stop_requested && !packet_stop)
            throw std::runtime_error("new target received after controlled stop request");
          state = packet_stop ? State::ControlledStop
                              : ((packet_heartbeat || packet_hold_current)
                                     ? State::Holding
                                     : State::Running);
        }
        if (packet_hold_current) {
          if (!is_joint_teleop_mode(o.mode) || packet.flags != 0 ||
              packet.frame_id != 0 || !backend->edg_active() ||
              !has_ik_target)
            throw std::runtime_error("recoverable hold-current contract mismatch");
          const std::array<double, 6> hold_position =
              output_diagnostics.previous_position();
          joint_resampler.hold(
              hold_position, packet.processing_ns, packet.sequence);
          ik_target = hold_position;
          pause_requested = true;
          pause_stopped_ready = false;
          state = State::Holding;
        } else if ((is_shadow_mode(o.mode) || is_bounded_mode(o.mode)) &&
            state != State::ControlledStop && !packet_stop && !packet_heartbeat) {
          const std::uint32_t expected_flags = is_bounded_mode(o.mode) ? kTargetAllowMotion : 0u;
          if (packet.flags != expected_flags)
            throw std::runtime_error("target motion flag does not match native execution mode");
          const CartesianState cartesian = relative_target_from_packet(packet, teleop.startup_tcp, o);
          const std::array<double, 6>& reference = has_ik_target ? ik_target : observed;
          std::array<double, 6> solution{};
          const auto ik_start = now_ns();
          backend->solve_ik(cartesian, reference, solution);
          const auto ik_duration = now_ns() - ik_start;
          ++teleop.ik_calls;
          teleop.ik_duration_total_ns += ik_duration;
          teleop.ik_duration_max_ns = std::max(teleop.ik_duration_max_ns, ik_duration);
          validate_joint_solution(solution, reference, o);
          const auto validation_start = now_ns();
          const double condition = backend->validate_kinematics(cartesian, solution);
          const auto validation_duration = now_ns() - validation_start;
          ++teleop.kinematics_validation_calls;
          teleop.kinematics_validation_total_ns += validation_duration;
          teleop.kinematics_validation_max_ns = std::max(
              teleop.kinematics_validation_max_ns, validation_duration);
          teleop.maximum_jacobian_condition = std::max(
              teleop.maximum_jacobian_condition, condition);
          if (now_ns() > deadline + 12'000'000) {
            ++samples.hard_timing_misses;
            throw std::runtime_error("target generation exceeded hard completion boundary before command");
          }
          for (std::size_t joint = 0; joint < 6; ++joint)
            teleop.maximum_ik_joint_step_rad = std::max(
                teleop.maximum_ik_joint_step_rad, std::abs(solution[joint] - reference[joint]));
          ik_target = solution;
          teleop.last_ik_target = solution;
          has_ik_target = true;
          if (is_bounded_mode(o.mode) && !backend->edg_active()) {
            backend->enter_edg(); state = State::EdgReady; backend->read(observed);
            double delta = 0.0;
            for (std::size_t joint = 0; joint < 6; ++joint)
              delta = std::max(delta, std::abs(observed[joint] - initial[joint]));
            if (delta > 1e-4) throw std::runtime_error("bounded startup EDG observation delta exceeded");
            tracker.reset(observed);
            state = State::Running;
          }
        } else if (is_joint_mode(o.mode) && state != State::ControlledStop &&
                   !packet_stop && !packet_heartbeat) {
          const std::uint32_t expected_flags = is_joint_teleop_mode(o.mode) ? kTargetAllowMotion : 0u;
          if (packet.flags != expected_flags)
            throw std::runtime_error("joint target motion flag does not match native execution mode");
          if (packet.kind != static_cast<std::uint16_t>(TargetKind::JointPosition) || packet.frame_id != 0)
            throw std::runtime_error("Quest JAKA adapter accepts only absolute J1..J6 joint targets");
          std::array<double, 6> solution{};
          std::copy_n(packet.payload, solution.size(), solution.begin());
          validate_manufacturer_joint_position_limits(solution);
          if (is_joint_teleop_mode(o.mode) && !first_external_joint_target_received) {
            double startup_delta = 0.0;
            for (std::size_t joint = 0; joint < solution.size(); ++joint)
              startup_delta = std::max(
                  startup_delta, std::abs(solution[joint] - command_reference[joint]));
            if (startup_delta > o.startup_alignment_tolerance_rad)
              throw std::runtime_error("first Quest joint target is not aligned with measured startup pose");
            first_external_joint_target_received = true;
          }
          ik_target = solution;
          teleop.last_ik_target = solution;
          has_ik_target = true;
          pause_requested = false;
          pause_stopped_ready = false;
          if (is_joint_teleop_mode(o.mode) && !backend->edg_active()) {
            backend->enter_edg();
            state = State::EdgReady;
            backend->read(observed);
            double startup_delta = 0.0;
            for (std::size_t joint = 0; joint < solution.size(); ++joint)
              startup_delta = std::max(startup_delta, std::abs(solution[joint] - observed[joint]));
            if (startup_delta > o.startup_alignment_tolerance_rad)
              throw std::runtime_error("first Quest joint target lost alignment while entering EDG");
            // EDG activation is a one-time explicitly gated setup operation,
            // not part of an 8 ms repeat-latest command cycle. Re-arm timing
            // only after activation and the second startup-alignment check.
            previous = now_ns();
            deadline = previous;
            joint_resampler.initialize(observed, previous);
            joint_resampler.accept(solution, packet.processing_ns, packet.sequence);
            schedule_realign = false;
            consecutive_timing_warnings = 0;
            consecutive_completion_misses = 0;
            timing_rearmed_after_edg = true;
            state = State::Running;
          } else if (is_joint_teleop_mode(o.mode)) {
            joint_resampler.accept(solution, packet.processing_ns, packet.sequence);
          }
        } else if (packet_heartbeat && is_joint_teleop_mode(o.mode)) {
          const std::uint32_t expected_flags = kTargetAllowMotion;
          if (packet.flags != expected_flags || packet.frame_id != 0)
            throw std::runtime_error("producer heartbeat contract mismatch");
          // Liveness advances independently of target validity.  Deliberately
          // do not call resampler.accept(): the last safe endpoint remains the
          // exact 8 ms command while the shared generator evaluates retreat.
          state = State::Holding;
        }
      }
      if (transport_failure) { state = State::Fault; outcome = "target_transport_failure"; break; }
      if (invalid_command) { state = State::ControlledStop; outcome = "invalid_command"; break; }
      if (state == State::ControlledStop) { outcome = "operator_stop_command"; break; }
      const auto age = ever_received ? cycle_start - std::min(cycle_start, last_dispatch) : 0;
      if (ever_received && age >= o.warning_ns) ++warning_cycles;
      if (is_joint_teleop_mode(o.mode) && ever_received && age >= o.hold_ns) {
        state = State::ControlledStop;
        outcome = "command_stream_timeout";
        break;
      }
      if (!stop_requested && ever_received && age >= o.fatal_ns) { state = State::Fault; outcome = "fatal_target_timeout"; break; }
      if (!stop_requested && ever_received && age >= o.stop_ns) {
        if (is_bounded_mode(o.mode) && backend->edg_active()) {
          stop_requested = true;
          stop_request_ns = cycle_start;
          stop_reason = "controlled_stop_target_timeout";
          state = State::Holding;
        } else {
          state = State::ControlledStop; outcome = "controlled_stop_target_timeout"; break;
        }
      }
      if (!is_joint_zero_motion_mode(o.mode) && (!ever_received || age >= o.hold_ns))
        state = State::Holding;
      const auto read_start = now_ns(); backend->read(observed); const auto read_end = now_ns();
      ControllerHealth cycle_health = teleop.last_controller_health;
      if (o.monitor_controller_health_each_cycle && (samples.count % 2) == 0) {
        cycle_health = backend->read_controller_health();
        teleop.last_controller_health = cycle_health;
        ++teleop.controller_health_samples;
        const bool alarm = cycle_health.status_sdk_return_code != ERR_SUCC ||
            cycle_health.estop_sdk_return_code != ERR_SUCC ||
            cycle_health.collision_sdk_return_code != ERR_SUCC ||
            cycle_health.controller_error_code != 0 || cycle_health.collision ||
            cycle_health.emergency_stop || !cycle_health.powered_on || !cycle_health.enabled;
        if (alarm) ++teleop.controller_alarm_events;
        require_healthy_controller(cycle_health);
      }
      for (std::size_t joint = 0; joint < 6; ++joint) {
        const double displacement = std::abs(observed[joint] - tracking_reference[joint]);
        teleop.maximum_observed_joint_delta_rad_per_joint[joint] = std::max(
            teleop.maximum_observed_joint_delta_rad_per_joint[joint], displacement);
        maximum_observed_delta_rad = std::max(maximum_observed_delta_rad, displacement);
      }
      std::uint64_t write_duration = 0, command_time = 0, command_start = 0;
      if (is_bounded_mode(o.mode) && backend->edg_active()) {
        const std::array<double, 6>& desired =
            (has_ik_target && age < o.hold_ns && !stop_requested) ? ik_target : tracker.position();
        target = tracker.update(desired);
        bool hard_crossing = false;
        for (std::size_t joint = 0; joint < 6; ++joint) {
          const double difference = std::abs(shortest_joint_difference(target[joint], observed[joint]));
          if (difference > teleop.maximum_tracking_difference_rad_per_joint[joint]) {
            teleop.maximum_tracking_difference_rad_per_joint[joint] = difference;
            teleop.maximum_tracking_difference_monotonic_ns_per_joint[joint] = cycle_start;
            teleop.maximum_tracking_difference_sequence_per_joint[joint] = last_sequence;
          }
          teleop.maximum_tracking_difference_rad = std::max(teleop.maximum_tracking_difference_rad, difference);
          if (difference >= 0.003490658503988659) ++teleop.tracking_warning_cycles;
          const double hard = std::max(0.01308996938995747,
              2.5 * std::abs(tracker.velocity()[joint]) * 0.150);
          hard_crossing = hard_crossing || difference > hard;
        }
        if (hard_crossing) {
          ++teleop.tracking_hard_crossings;
          ++consecutive_tracking_crossings;
        } else consecutive_tracking_crossings = 0;
        if (consecutive_tracking_crossings >= 2)
          throw std::runtime_error("persistent dynamic joint tracking boundary crossed");
        for (std::size_t joint = 0; joint < 6; ++joint)
          maximum_command_delta_rad = std::max(maximum_command_delta_rad, std::abs(target[joint] - initial[joint]));
        const auto write_start = now_ns(); backend->command(target); command_time = now_ns(); write_duration = command_time - write_start;
      } else if ((is_joint_teleop_mode(o.mode) || is_joint_zero_motion_mode(o.mode)) &&
                 backend->edg_active() && has_ik_target) {
        // Normal trajectory evaluation stays on the exact 8 ms deadline grid.
        // A recovered late wake evaluates once at current time and re-arms;
        // expired historical ticks are never emitted in a catch-up burst.
        const std::uint64_t servo_evaluation_time = timing_rearmed_after_edg
            ? previous : (schedule_realign ? cycle_start : deadline);
        const ResampledServoPoint proposed_servo_point =
            joint_resampler.evaluate(servo_evaluation_time);
        ResampledServoPoint servo_point = proposed_servo_point;
        command_start = now_ns();
        const auto prior_emitted = output_diagnostics.previous_position();
        OutputMotionSample prior_motion{};
        prior_motion.velocity = output_diagnostics.previous_velocity();
        prior_motion.acceleration = output_diagnostics.previous_acceleration();
        const OutputMotionSample proposed_motion =
            output_diagnostics.check_candidate(
                proposed_servo_point, command_start);
        const bool transition_limited =
            o.recover_output_acceleration_transition &&
            (output_diagnostics.velocity_boundary_crossing(proposed_motion) ||
             output_diagnostics.recoverable_acceleration_crossing(
                 proposed_motion) ||
             std::any_of(proposed_motion.jerk.begin(), proposed_motion.jerk.end(),
                         [&o](double value) {
                           return std::abs(value) >
                               o.output_joint_jerk_limit_rad_s3 + 1e-12;
                         }));
        OutputHoldUpdate hold_update{};
        OutputMotionSample motion_sample = proposed_motion;
        OutputTransitionProgress transition_progress{};
        bool no_progress_hold = false;
        bool recovered_from_output_hold = false;
        std::uint64_t recovered_hold_duration_ns = 0;
        if (transition_limited) {
          servo_point = output_diagnostics.transition_limited_point(
              proposed_servo_point, proposed_motion);
          motion_sample =
              output_diagnostics.check_final(servo_point, command_start);
          transition_progress = classify_output_transition(
              prior_emitted, servo_point.position, ik_target);
          no_progress_hold = transition_progress.no_progress_hold;
        }
        if (no_progress_hold) {
          hold_update = output_hold.hold(
              command_start, proposed_servo_point.to_sequence);
          output_acceleration_hold_status_pending = true;
          state = State::Holding;
        } else {
          if (transition_limited) ++transition_limited_progress_points;
          if (output_hold.active()) recovered_from_output_hold = true;
        }
        target = servo_point.position;
        bool hard_crossing = false;
        for (std::size_t joint = 0; joint < target.size(); ++joint) {
          const double difference = std::abs(shortest_joint_difference(target[joint], observed[joint]));
          if (difference > teleop.maximum_tracking_difference_rad_per_joint[joint]) {
            teleop.maximum_tracking_difference_rad_per_joint[joint] = difference;
            teleop.maximum_tracking_difference_monotonic_ns_per_joint[joint] = cycle_start;
            teleop.maximum_tracking_difference_sequence_per_joint[joint] = last_sequence;
          }
          teleop.maximum_tracking_difference_rad = std::max(
              teleop.maximum_tracking_difference_rad, difference);
          hard_crossing = hard_crossing || difference > o.excessive_tracking_error_abort_rad;
          if (joint >= 3) {
            if (difference > previous_absolute_tracking_difference[joint] + 1e-6)
              ++consecutive_increasing_tracking_cycles[joint];
            else
              consecutive_increasing_tracking_cycles[joint] = 0;
            if (difference >= 0.5 * o.excessive_tracking_error_abort_rad)
              ++teleop.tracking_warning_cycles;
            if (difference >= 0.5 * o.excessive_tracking_error_abort_rad &&
                consecutive_increasing_tracking_cycles[joint] >= 3)
              throw std::runtime_error(
                  "sustained increasing wrist tracking lag before SDK call: J" +
                  std::to_string(joint + 1));
          }
          previous_absolute_tracking_difference[joint] = difference;
          maximum_command_delta_rad = std::max(
              maximum_command_delta_rad, std::abs(target[joint] - command_reference[joint]));
        }
        if (hard_crossing) {
          ++teleop.tracking_hard_crossings;
          ++consecutive_tracking_crossings;
        } else {
          consecutive_tracking_crossings = 0;
        }
        if (consecutive_tracking_crossings >= o.excessive_tracking_error_consecutive_cycles)
          throw std::runtime_error("clearly excessive measured joint tracking error");
        backend->command(target);
        command_time = now_ns();
        write_duration = command_time - command_start;
        output_diagnostics.commit(servo_point, motion_sample);
        if (transition_limited) {
          joint_resampler.commit_transition_limited(servo_point);
        } else {
          joint_resampler.commit(servo_point, command_time);
        }
        if (recovered_from_output_hold) {
          recovered_hold_duration_ns = output_hold.recover(
              command_start, servo_point.to_sequence);
          output_acceleration_recovery_status_pending = true;
          state = State::Running;
        }
        if (pause_requested) {
          bool at_hold_position = true;
          for (std::size_t joint = 0; joint < target.size(); ++joint)
            at_hold_position = at_hold_position &&
                std::abs(target[joint] - ik_target[joint]) <= 1e-4;
          pause_stopped_ready =
              at_hold_position && output_diagnostics.stationary();
          state = State::Holding;
        }
        if (!o.cycle_telemetry_file.empty()) {
          CycleTelemetry row{};
          row.monotonic_ns = command_time;
          row.wall_ns = wall_now_ns();
          row.cycle_start_ns = cycle_start;
          row.scheduled_deadline_ns = deadline;
          row.wake_lateness_ns = wake_lateness;
          row.last_sequence = last_sequence;
          row.heartbeat_age_ns = age;
          row.event = no_progress_hold
              ? "output_no_progress_hold"
              : (recovered_from_output_hold
                    ? "recovered_from_output_no_progress_hold"
                    : (transition_limited
                          ? "transition_limited_output"
                          : "normal_output"));
          row.emitted = target;
          row.destination = ik_target;
          row.measured = observed;
          row.prior_emitted = prior_emitted;
          row.proposed_emitted = proposed_servo_point.position;
          row.prior_motion = prior_motion;
          row.proposed_motion = proposed_motion;
          for (std::size_t joint = 0; joint < target.size(); ++joint)
            row.continuity_error[joint] =
                proposed_servo_point.position[joint] - target[joint];
          for (std::size_t joint = 0; joint < target.size(); ++joint)
            row.tracking[joint] = shortest_joint_difference(target[joint], observed[joint]);
          row.point = servo_point;
          row.motion = motion_sample;
          row.health = cycle_health;
          row.violating_joint =
              output_diagnostics.first_recoverable_violating_joint();
          row.hold_start_ns = output_hold.start_ns();
          row.hold_duration_ns = no_progress_hold
              ? hold_update.duration_ns : recovered_hold_duration_ns;
          row.consecutive_hold_cycles =
              no_progress_hold ? hold_update.consecutive_cycles : 0;
          row.recovery_sequence = recovered_from_output_hold
              ? servo_point.to_sequence : 0;
          row.destination_gap_rad = transition_progress.destination_gap_rad;
          row.selected_progress_rad = transition_progress.selected_progress_rad;
          row.transition_limited = transition_limited;
          row.no_progress_hold = no_progress_hold;
          row.hold_degraded = hold_update.degraded;
          row.command_start_ns = command_start;
          row.command_end_ns = command_time;
          row.command_duration_ns = write_duration;
          row.cpu = cycle_cpu;
          row.cpu_migration_count = placement.migration_count();
          cycle_telemetry.push_back(row);
        }
        if (is_joint_zero_motion_mode(o.mode)) {
          if (teleop.zero_motion_command_count == 0)
            teleop.zero_motion_first_command_rad = target;
          teleop.zero_motion_last_command_rad = target;
          ++teleop.zero_motion_command_count;
          bool mismatch = false;
          for (std::size_t joint = 0; joint < target.size(); ++joint)
            mismatch = mismatch || target[joint] != tracking_reference[joint];
          teleop.zero_motion_command_mismatch_count += mismatch ? 1 : 0;
        }
        if (!o.emitted_points_file.empty())
          recorded_servo_points.push_back(RecordedServoPoint{servo_point, motion_sample});
      } else if (o.mode != Mode::StateRead && !is_stream_mode(o.mode)) {
        target = initial;
        if (o.mode == Mode::MinimalMotion) {
          const double t = (cycle_start - start) / 1e9, segment = o.probe_motion_s;
          double scale = 0.0;
          if (t < segment) scale = smoothstep5(t / segment);
          else if (t < 2.0 * segment) scale = 1.0 - smoothstep5((t - segment) / segment);
          target[static_cast<std::size_t>(o.probe_joint)] += scale * o.probe_delta_rad;
        }
        for (std::size_t joint = 0; joint < 6; ++joint)
          maximum_command_delta_rad = std::max(maximum_command_delta_rad, std::abs(target[joint] - initial[joint]));
        const auto write_start = now_ns(); backend->command(target); command_time = now_ns(); write_duration = command_time - write_start;
      }
      const auto cycle_end = now_ns();
      const int completion_cpu = sched_getcpu();
      placement.observe_cpu(cycle_end, completion_cpu);
      bool exit_after_cycle = false;
      if (stop_requested && is_bounded_mode(o.mode) && backend->edg_active()) {
        if (tracker.stationary()) {
          state = State::ControlledStop;
          outcome = stop_reason.c_str();
          exit_after_cycle = true;
        } else if (cycle_end - stop_request_ns > 1'000'000'000) {
          state = State::Fault;
          outcome = "controlled_stop_trajectory_timeout";
          error_code = 1;
          exit_after_cycle = true;
        }
      }
      const std::size_t i = samples.count++;
      if (!cycle_telemetry.empty()) {
        auto& row = cycle_telemetry.back();
        row.cycle_end_ns = cycle_end;
        row.completion_lateness_ns =
            cycle_end > deadline ? cycle_end - deadline : 0;
      }
      samples.periods[i] = start_period; samples.wakes[i] = wake_lateness;
      if (!timing_rearmed_after_edg) previous = cycle_start;
      samples.reads[i] = read_end - read_start; samples.writes[i] = write_duration;
      samples.sdk[i] = samples.reads[i] + samples.writes[i];
      samples.target_ages[i] = ever_received ? age : 0;
      samples.accepted_target_ages[i] = last_target_dispatch
          ? cycle_start - std::min(cycle_start, last_target_dispatch) : 0;
      samples.command_ages[i] = command_time && last_dispatch ? command_time - std::min(command_time, last_dispatch) : 0;
      if (cycle_end > deadline + kPeriodNs) {
        ++samples.missed;
        if (is_stream_mode(o.mode)) {
          ++samples.timing_warnings;
          ++consecutive_completion_misses;
          schedule_realign = true;
          if (!first_timing_warning_recorded) {
            first_timing_warning_recorded = true;
            placement.record(PlacementEventReason::FirstTimingWarning,
                             cycle_end, completion_cpu,
                             placement.previous_cpu(), false, false);
            system_observer.request(SystemSnapshotTrigger::FirstTimingWarning,
                                    cycle_end, completion_cpu);
          }
          samples.maximum_consecutive = std::max(samples.maximum_consecutive, consecutive_completion_misses);
          const bool startup_grace = startup_timing_grace_active && !ever_received;
          if (cycle_end > deadline + 12'000'000 ||
              (!startup_grace && consecutive_completion_misses >= 2)) {
            ++samples.hard_timing_misses;
            samples.terminal_timing_fault_phase = "cycle_completion";
            samples.terminal_timing_fault_monotonic_ns = cycle_end;
            samples.terminal_actual_cycle_period_ns = start_period;
            samples.terminal_wake_lateness_ns = wake_lateness;
            samples.terminal_completion_lateness_ns = cycle_end - deadline;
            samples.terminal_consecutive_warning_count =
                consecutive_completion_misses;
            samples.terminal_cpu = completion_cpu;
            placement.record(PlacementEventReason::TerminalTimingFault,
                             cycle_end, samples.terminal_cpu,
                             placement.previous_cpu(), true, false);
            system_observer.request(SystemSnapshotTrigger::TerminalTimingFault,
                                    cycle_end, samples.terminal_cpu);
            state = State::Fault;
            outcome = "hard_completion_timing_miss";
            break;
          }
        } else {
          ++consecutive_overruns;
          samples.maximum_consecutive = std::max(samples.maximum_consecutive, consecutive_overruns);
        }
      } else {
        consecutive_overruns = 0;
        consecutive_completion_misses = 0;
      }
      if (!is_stream_mode(o.mode) &&
          consecutive_overruns >= o.max_consecutive_overruns) {
        state = State::Fault; outcome = "control_loop_overrun"; break;
      }
      if (schedule_realign) {
        deadline = cycle_start;
        ++samples.schedule_realignments;
      }
      if ((i % 13) == 0 || output_acceleration_hold_status_pending ||
          output_acceleration_recovery_status_pending) {
        std::uint32_t flags = kStatusConnected;
        if (backend->edg_active()) flags |= kStatusEdgActive;
        if (state == State::Holding) flags |= kStatusHolding;
        if (ever_received) flags |= kStatusHasTarget;
        if (accepted != status_accepted) flags |= kStatusAccepted;
        if (rejected != status_rejected) flags |= kStatusRejected;
        if (ever_received && age >= o.warning_ns) flags |= kStatusTargetWarning;
        if (output_hold.active() ||
            output_acceleration_hold_status_pending)
          flags |= kStatusOutputAccelerationHold;
        if (output_acceleration_recovery_status_pending)
          flags |= kStatusOutputAccelerationRecovered;
        if (pause_requested && !pause_stopped_ready)
          flags |= kStatusControlledBraking;
        if (pause_requested && pause_stopped_ready)
          flags |= kStatusStoppedReady | kStatusMeasuredStateRefresh;
        StatusPacket status{kStatusMagic, kWireVersion, static_cast<std::uint16_t>(state), flags, last_sequence, i,
                            cycle_end, command_time, read_end, {}, error_code, 0};
        std::copy(observed.begin(), observed.end(), status.joint_position_rad); status_sender.send_status(status);
        status_accepted = accepted; status_rejected = rejected;
        output_acceleration_hold_status_pending = false;
        output_acceleration_recovery_status_pending = false;
      }
      if (exit_after_cycle) break;
    }
    if (state == State::Fault) {
      if (error_code == 0) error_code = 1;
    } else {
      state = State::ControlledStop;
    }
  } catch (const std::exception& e) {
    state = State::Fault; error_code = 1;
    fault_outcome = std::string("fault: ") + e.what();
    outcome = fault_outcome.c_str();
  }
  // Make terminal state visible to the non-blocking producer before SDK
  // cleanup and non-critical telemetry serialization.  This prevents a bound
  // but undrained datagram endpoint from turning the authoritative native
  // fault into a prolonged secondary transport symptom.
  target_socket.shutdown();
  output_hold.finalize(now_ns());
  backend->cleanup();
  const int cleanup_error_code = backend->cleanup_error_code();
  if (cleanup_error_code != 0 && error_code == 0) {
    error_code = cleanup_error_code;
    outcome = "cleanup_failure";
  }
  state = State::Shutdown;
  StatusPacket final_status{kStatusMagic, kWireVersion, static_cast<std::uint16_t>(state),
                            ever_received ? kStatusHasTarget : 0u, last_sequence, samples.count,
                            now_ns(), 0, now_ns(), {}, error_code, 0};
  std::copy(observed.begin(), observed.end(), final_status.joint_position_rad);
  status_sender.send_status(final_status);
  system_observer.stop();
  const auto placement_shutdown_ns = now_ns();
  placement.record(PlacementEventReason::WorkerShutdown,
                   placement_shutdown_ns, sched_getcpu(),
                   placement.previous_cpu(), true, true);
  getrusage(RUSAGE_SELF, &usage_end); const auto end = now_ns();
  // Persist the authoritative terminal object and bounded scheduling context
  // before large optional JSONL artifacts.
  write_metrics(o, samples, accepted, rejected, warning_cycles, (end - start) / 1e9,
                cpu_seconds(usage_end) - cpu_seconds(usage_start), maximum_command_delta_rad,
                maximum_observed_delta_rad, error_code, cleanup_error_code, outcome, teleop, tracker,
                initial, joint_resampler, output_diagnostics,
                output_hold, transition_limited_progress_points, ik_target,
                placement, system_observer);
  write_emitted_points(o, recorded_servo_points);
  write_cycle_telemetry(o, cycle_telemetry);
  return error_code == 0 ? 0 : 2;
}
}  // namespace

int main(int argc, char** argv) {
  try { return run(parse_options(argc, argv)); }
  catch (const std::exception& e) { std::cerr << "configuration error: " << e.what() << '\n'; return 64; }
}
