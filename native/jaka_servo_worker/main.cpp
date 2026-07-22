#include <JAKAZuRobot.h>
#include <jkerr.h>

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
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {
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
constexpr double kProbeMaximumVelocityRadS = 0.005;
constexpr double kProbeMaximumAccelerationRadS2 = 0.02;

enum class Mode {
  DryRun, StateRead, ZeroMotion, MinimalMotion,
  CommandShadowDryRun, CommandShadow, BoundedTeleopDryRun, BoundedTeleop,
  JointShadowDryRun, JointShadow, JointTeleopDryRun, JointTeleop
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

struct Options {
  Mode mode = Mode::DryRun;
  std::string robot_ip;
  std::string edg_state_ip = "0.0.0.0";
  std::string target_socket = "/tmp/jaka_servo_target.sock";
  std::string status_socket;
  std::string metrics_file;
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
  std::uint64_t fake_read_delay_ns = 0;
  std::uint64_t fake_write_delay_ns = 0;
  std::uint64_t fake_fail_after = 0;
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

bool is_joint_mode(Mode mode) {
  return is_joint_shadow_mode(mode) || is_joint_teleop_mode(mode);
}

bool is_stream_mode(Mode mode) {
  return is_shadow_mode(mode) || is_bounded_mode(mode) || is_joint_mode(mode);
}

bool uses_fake_backend(Mode mode) {
  return mode == Mode::DryRun || mode == Mode::CommandShadowDryRun ||
         mode == Mode::BoundedTeleopDryRun || mode == Mode::JointShadowDryRun ||
         mode == Mode::JointTeleopDryRun;
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
      else throw std::runtime_error("invalid --mode");
    } else if (a == "--robot-ip") o.robot_ip = value_after(i, argc, argv);
    else if (a == "--edg-state-ip") o.edg_state_ip = value_after(i, argc, argv);
    else if (a == "--target-socket") o.target_socket = value_after(i, argc, argv);
    else if (a == "--status-socket") o.status_socket = value_after(i, argc, argv);
    else if (a == "--metrics-file") o.metrics_file = value_after(i, argc, argv);
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
    else if (a == "--fake-read-delay-us") o.fake_read_delay_ns = std::stoull(value_after(i, argc, argv)) * 1000;
    else if (a == "--fake-write-delay-us") o.fake_write_delay_ns = std::stoull(value_after(i, argc, argv)) * 1000;
    else if (a == "--fake-fail-after") o.fake_fail_after = std::stoull(value_after(i, argc, argv));
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
    else if (a == "--help") {
      std::cout << "jaka_servo_worker --mode dry-run|state-read|zero-motion|minimal-motion|command-shadow-dry-run|command-shadow|bounded-teleop-dry-run|bounded-teleop|joint-shadow-dry-run|joint-shadow|joint-teleop-dry-run|joint-teleop [options]\n";
      std::exit(0);
    } else throw std::runtime_error("unknown option: " + a);
  }
  if (!(o.duration_s > 0.0 && o.duration_s <= 2000.0)) throw std::runtime_error("duration must be in (0, 2000] s");
  if (!(o.warning_ns < o.hold_ns && o.hold_ns < o.stop_ns && o.stop_ns < o.fatal_ns))
    throw std::runtime_error("stale thresholds must be strictly increasing");
  if (is_connected_mode(o.mode)) {
    const char* expected_ack = is_joint_shadow_mode(o.mode) ? kQuestShadowAck :
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
    if (!(std::isfinite(o.excessive_tracking_error_abort_rad) &&
          o.excessive_tracking_error_abort_rad >= 0.25 &&
          o.excessive_tracking_error_abort_rad <= 1.0 &&
          o.excessive_tracking_error_consecutive_cycles >= 1 &&
          o.excessive_tracking_error_consecutive_cycles <= 10 &&
          std::isfinite(o.startup_alignment_tolerance_rad) &&
          o.startup_alignment_tolerance_rad > 0.0 &&
          o.startup_alignment_tolerance_rad <= 0.01))
      throw std::runtime_error("joint adapter fault-containment settings are invalid");
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

constexpr std::array<double, 6> kJointLower{-6.28, -2.09, -2.27, -6.28, -2.09, -6.28};
constexpr std::array<double, 6> kJointUpper{6.28, 2.09, 2.27, 6.28, 2.09, 6.28};

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

void validate_manufacturer_joint_position_limits(const std::array<double, 6>& target) {
  for (std::size_t joint = 0; joint < target.size(); ++joint) {
    if (!std::isfinite(target[joint]) || target[joint] < kJointLower[joint] ||
        target[joint] > kJointUpper[joint])
      throw std::runtime_error("joint target violates JAKA manufacturer position limits");
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
  explicit FakeBackend(const Options& o) : options_(o) {}
  ~FakeBackend() override { cleanup(); }
  void connect() override { delay(options_.fake_connect_delay_ns); connected_ = true; }
  void verify(int, int) override { if (!connected_) throw std::runtime_error("fake disconnected"); }
  void enter_edg() override {
    if (!connected_) throw std::runtime_error("fake disconnected");
    delay(options_.fake_edg_delay_ns);
    edg_ = true;
  }
  void validate_probe(const std::array<double, 6>&, const std::array<double, 6>&) override {}
  void read(std::array<double, 6>& joints) override { delay(options_.fake_read_delay_ns); fail(); joints = joints_; }
  void read_tcp(CartesianState& pose) override { pose = {}; }
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
  ~TargetSocket() { if (fd_ >= 0) close(fd_); unlink(path_.c_str()); }
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
  CartesianState startup_tcp{};
  std::array<double, 6> last_ik_target{};
};

struct Samples {
  std::array<std::uint64_t, kMaximumSamples> periods{}, wakes{}, reads{}, writes{}, sdk{}, target_ages{}, command_ages{};
  std::size_t count = 0;
  std::uint64_t missed = 0, maximum_consecutive = 0;
  std::uint64_t timing_warnings = 0, hard_timing_misses = 0, schedule_realignments = 0;
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
  }
  return "unknown";
}

void write_metrics(const Options& o, const Samples& s, std::uint64_t accepted, std::uint64_t rejected,
                   std::uint64_t warning_cycles,
                   double elapsed_s, double cpu_s, double maximum_command_delta_rad,
                   double maximum_observed_delta_rad, int error_code, int cleanup_error_code,
                   const std::string& outcome, const TeleopMetrics& teleop,
                   const JerkBoundedJointTracker& tracker,
                   const std::array<double, 6>& initial_joint_position_rad) {
  std::ofstream file;
  std::ostream* output = &std::cout;
  if (!o.metrics_file.empty()) { file.open(o.metrics_file); if (!file) throw std::runtime_error("cannot open metrics file"); output = &file; }
  auto& out = *output;
  out << std::setprecision(12) << "{\n  \"schema_version\":\"jaka_worker_metrics.v1\",\n"
      << "  \"mode\":\"" << mode_name(o.mode) << "\",\n"
      << "  \"outcome\":\"" << outcome << "\",\n  \"requested_period_ns\":" << kPeriodNs << ",\n"
      << "  \"elapsed_s\":" << elapsed_s << ",\n  \"worker_cpu_s\":" << cpu_s << ",\n  \"worker_cpu_percent\":" << (elapsed_s > 0 ? cpu_s / elapsed_s * 100.0 : 0.0) << ",\n"
      << "  \"loop_rate_hz\":" << (elapsed_s > 0 ? s.count / elapsed_s : 0.0) << ",\n"
      << "  \"accepted_target_rate_hz\":" << (elapsed_s > 0 ? accepted / elapsed_s : 0.0) << ",\n"
      << "  \"maximum_intentional_command_delta_rad\":" << maximum_command_delta_rad << ",\n"
      << "  \"maximum_observed_joint_delta_rad\":" << maximum_observed_delta_rad << ",\n"
      << "  \"accepted_targets\":" << accepted << ",\n  \"rejected_targets\":" << rejected << ",\n"
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
      << "  \"initial_joint_position_rad\":[" << initial_joint_position_rad[0] << ',' << initial_joint_position_rad[1] << ',' << initial_joint_position_rad[2] << ',' << initial_joint_position_rad[3] << ',' << initial_joint_position_rad[4] << ',' << initial_joint_position_rad[5] << "],\n"
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
      << "  \"schedule_realignments\":" << s.schedule_realignments << ",\n  \"statistics\":{\n";
  metric_json(out, "actual_cycle_period", s.periods, s.count, true);
  metric_json(out, "wake_lateness", s.wakes, s.count, true);
  metric_json(out, "sdk_call_duration", s.sdk, s.count, true);
  metric_json(out, "state_read_duration", s.reads, s.count, true);
  metric_json(out, "command_write_duration", s.writes, s.count, true);
  metric_json(out, "transport_age", s.target_ages, s.count, true);
  metric_json(out, "command_age", s.command_ages, s.count, false);
  out << "  }\n}\n";
}

double smoothstep5(double x) { x = std::clamp(x, 0.0, 1.0); return x*x*x*(10.0 + x*(-15.0 + 6.0*x)); }

int run(const Options& o) {
  std::signal(SIGINT, signal_handler); std::signal(SIGTERM, signal_handler); std::signal(SIGHUP, signal_handler);
  auto backend = uses_fake_backend(o.mode) ? std::unique_ptr<Backend>(new FakeBackend(o)) : std::unique_ptr<Backend>(new RealBackend(o));
  TargetSocket target_socket(o.target_socket); StatusSender status_sender(o.status_socket);
  auto samples_storage = std::make_unique<Samples>();
  Samples& samples = *samples_storage;
  JerkBoundedJointTracker tracker(o);
  TeleopMetrics teleop{};
  TargetPacket latest{}; bool ever_received = false;
  std::uint64_t accepted = 0, rejected = 0, last_sequence = 0, last_dispatch = 0, consecutive_overruns = 0;
  std::uint64_t consecutive_timing_warnings = 0, consecutive_completion_misses = 0;
  std::uint64_t warning_cycles = 0;
  std::uint64_t status_accepted = 0, status_rejected = 0;
  std::array<double, 6> initial{}, observed{}, target{}, ik_target{};
  bool has_ik_target = false;
  bool stop_requested = false;
  std::uint64_t stop_request_ns = 0;
  std::string stop_reason;
  std::uint64_t consecutive_tracking_crossings = 0;
  double maximum_command_delta_rad = 0.0, maximum_observed_delta_rad = 0.0;
  State state = State::Connecting; std::int32_t error_code = 0;
  const char* outcome = "completed";
  std::string fault_outcome;
  rusage usage_start{}, usage_end{}; getrusage(RUSAGE_SELF, &usage_start);
  auto start = now_ns(); auto previous = start; auto deadline = start;
  try {
    backend->connect(); state = State::Connected;
    backend->verify(o.expected_tool_id, o.expected_user_frame_id); state = State::Armed;
    backend->read(initial); observed = initial; target = initial;
    if (!std::all_of(initial.begin(), initial.end(), [](double v) { return std::isfinite(v) && std::abs(v) <= 2.0 * M_PI; })) throw std::runtime_error("initial joint state failed radians/finiteness check");
    if (o.mode == Mode::StateRead || is_shadow_mode(o.mode) ||
        is_bounded_mode(o.mode) || is_joint_mode(o.mode)) {
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
    if (o.mode != Mode::StateRead && !is_stream_mode(o.mode)) { backend->enter_edg(); state = State::EdgReady; backend->read(observed);
      double delta = 0; for (std::size_t i = 0; i < 6; ++i) delta = std::max(delta, std::abs(observed[i] - initial[i]));
      if (delta > 1e-4) throw std::runtime_error("near-zero initial command delta check failed");
    }
    state = State::Holding;
    // Connection, verification, and initial state reads are setup work, not an
    // 8 ms command-stream cycle. Start deadline monitoring only after setup so
    // normal SDK/network startup latency cannot cause a false first-cycle abort.
    start = now_ns();
    previous = start;
    deadline = start;
    while (!g_stop.load(std::memory_order_relaxed) && samples.count < kMaximumSamples) {
      deadline += kPeriodNs;
      timespec wake{static_cast<time_t>(deadline / 1'000'000'000), static_cast<long>(deadline % 1'000'000'000)};
      while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr) == EINTR && !g_stop.load()) {}
      const auto cycle_start = now_ns();
      if (cycle_start - start >= static_cast<std::uint64_t>(o.duration_s * 1e9)) {
        if (is_joint_teleop_mode(o.mode) && backend->edg_active()) {
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
        if (start_period > 12'000'000 || wake_lateness >= 8'000'000) {
          const std::size_t row = samples.count++;
          samples.periods[row] = start_period;
          samples.wakes[row] = wake_lateness;
          ++samples.hard_timing_misses;
          state = State::Fault;
          outcome = "hard_start_timing_miss";
          break;
        }
        const bool warning = start_period > 8'800'000 || wake_lateness > 2'000'000;
        if (warning) {
          ++samples.timing_warnings;
          ++consecutive_timing_warnings;
          schedule_realign = true;
        } else consecutive_timing_warnings = 0;
        if (consecutive_timing_warnings >= 2) {
          const std::size_t row = samples.count++;
          samples.periods[row] = start_period;
          samples.wakes[row] = wake_lateness;
          ++samples.hard_timing_misses;
          state = State::Fault;
          outcome = "consecutive_start_timing_misses";
          break;
        }
      }
      TargetPacket packet{};
      bool invalid_command = false, transport_failure = false;
      if (target_socket.drain_newest(packet, last_sequence, ever_received, cycle_start, rejected,
                                     invalid_command, transport_failure)) {
        latest = packet; last_sequence = packet.sequence; last_dispatch = packet.dispatch_ns; ever_received = true; ++accepted;
        const bool packet_stop = packet.kind == static_cast<std::uint16_t>(TargetKind::Stop);
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
          state = packet_stop ? State::ControlledStop : State::Running;
        }
        if ((is_shadow_mode(o.mode) || is_bounded_mode(o.mode)) &&
            state != State::ControlledStop && !packet_stop) {
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
        } else if (is_joint_mode(o.mode) && state != State::ControlledStop && !packet_stop) {
          const std::uint32_t expected_flags = is_joint_teleop_mode(o.mode) ? kTargetAllowMotion : 0u;
          if (packet.flags != expected_flags)
            throw std::runtime_error("joint target motion flag does not match native execution mode");
          if (packet.kind != static_cast<std::uint16_t>(TargetKind::JointPosition) || packet.frame_id != 0)
            throw std::runtime_error("Quest JAKA adapter accepts only absolute J1..J6 joint targets");
          std::array<double, 6> solution{};
          std::copy_n(packet.payload, solution.size(), solution.begin());
          validate_manufacturer_joint_position_limits(solution);
          if (!has_ik_target && is_joint_teleop_mode(o.mode)) {
            double startup_delta = 0.0;
            for (std::size_t joint = 0; joint < solution.size(); ++joint)
              startup_delta = std::max(startup_delta, std::abs(solution[joint] - observed[joint]));
            if (startup_delta > o.startup_alignment_tolerance_rad)
              throw std::runtime_error("first Quest joint target is not aligned with measured startup pose");
          }
          ik_target = solution;
          teleop.last_ik_target = solution;
          has_ik_target = true;
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
            consecutive_timing_warnings = 0;
            consecutive_completion_misses = 0;
            timing_rearmed_after_edg = true;
            state = State::Running;
          }
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
      if (!ever_received || age >= o.hold_ns) state = State::Holding;
      const auto read_start = now_ns(); backend->read(observed); const auto read_end = now_ns();
      for (std::size_t joint = 0; joint < 6; ++joint)
        maximum_observed_delta_rad = std::max(maximum_observed_delta_rad, std::abs(observed[joint] - initial[joint]));
      std::uint64_t write_duration = 0, command_time = 0;
      if (is_bounded_mode(o.mode) && backend->edg_active()) {
        const std::array<double, 6>& desired =
            (has_ik_target && age < o.hold_ns && !stop_requested) ? ik_target : tracker.position();
        target = tracker.update(desired);
        bool hard_crossing = false;
        for (std::size_t joint = 0; joint < 6; ++joint) {
          const double difference = std::abs(target[joint] - observed[joint]);
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
      } else if (is_joint_teleop_mode(o.mode) && backend->edg_active() && has_ik_target) {
        target = ik_target;
        bool hard_crossing = false;
        for (std::size_t joint = 0; joint < target.size(); ++joint) {
          const double difference = std::abs(target[joint] - observed[joint]);
          teleop.maximum_tracking_difference_rad = std::max(
              teleop.maximum_tracking_difference_rad, difference);
          hard_crossing = hard_crossing || difference > o.excessive_tracking_error_abort_rad;
          maximum_command_delta_rad = std::max(
              maximum_command_delta_rad, std::abs(target[joint] - initial[joint]));
        }
        if (hard_crossing) {
          ++teleop.tracking_hard_crossings;
          ++consecutive_tracking_crossings;
        } else {
          consecutive_tracking_crossings = 0;
        }
        if (consecutive_tracking_crossings >= o.excessive_tracking_error_consecutive_cycles)
          throw std::runtime_error("clearly excessive measured joint tracking error");
        const auto write_start = now_ns();
        backend->command(target);
        command_time = now_ns();
        write_duration = command_time - write_start;
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
      samples.periods[i] = start_period; samples.wakes[i] = wake_lateness;
      if (!timing_rearmed_after_edg) previous = cycle_start;
      samples.reads[i] = read_end - read_start; samples.writes[i] = write_duration;
      samples.sdk[i] = samples.reads[i] + samples.writes[i];
      samples.target_ages[i] = ever_received ? age : 0;
      samples.command_ages[i] = command_time && last_dispatch ? command_time - std::min(command_time, last_dispatch) : 0;
      if (cycle_end > deadline + kPeriodNs) {
        ++samples.missed;
        if (is_stream_mode(o.mode)) {
          ++samples.timing_warnings;
          ++consecutive_completion_misses;
          schedule_realign = true;
          samples.maximum_consecutive = std::max(samples.maximum_consecutive, consecutive_completion_misses);
          if (cycle_end > deadline + 12'000'000 || consecutive_completion_misses >= 2) {
            ++samples.hard_timing_misses;
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
      if ((i % 13) == 0) {
        std::uint32_t flags = kStatusConnected;
        if (backend->edg_active()) flags |= kStatusEdgActive;
        if (state == State::Holding) flags |= kStatusHolding;
        if (ever_received) flags |= kStatusHasTarget;
        if (accepted != status_accepted) flags |= kStatusAccepted;
        if (rejected != status_rejected) flags |= kStatusRejected;
        if (ever_received && age >= o.warning_ns) flags |= kStatusTargetWarning;
        StatusPacket status{kStatusMagic, kWireVersion, static_cast<std::uint16_t>(state), flags, last_sequence, i,
                            cycle_end, command_time, read_end, {}, error_code, 0};
        std::copy(observed.begin(), observed.end(), status.joint_position_rad); status_sender.send_status(status);
        status_accepted = accepted; status_rejected = rejected;
      }
      if (exit_after_cycle) break;
    }
    state = State::ControlledStop;
  } catch (const std::exception& e) {
    state = State::Fault; error_code = 1;
    fault_outcome = std::string("fault: ") + e.what();
    outcome = fault_outcome.c_str();
  }
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
  getrusage(RUSAGE_SELF, &usage_end); const auto end = now_ns();
  write_metrics(o, samples, accepted, rejected, warning_cycles, (end - start) / 1e9,
                cpu_seconds(usage_end) - cpu_seconds(usage_start), maximum_command_delta_rad,
                maximum_observed_delta_rad, error_code, cleanup_error_code, outcome, teleop, tracker,
                initial);
  return error_code == 0 ? 0 : 2;
}
}  // namespace

int main(int argc, char** argv) {
  try { return run(parse_options(argc, argv)); }
  catch (const std::exception& e) { std::cerr << "configuration error: " << e.what() << '\n'; return 64; }
}
