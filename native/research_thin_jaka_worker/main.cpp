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
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "jaka_sdk_translation.hpp"
#include "teleop_command_abi/abi_v1.hpp"
#include "teleop_shaping/engagement_joint_shaper.hpp"
#include "teleop_shaping/joint_shaper.hpp"
#include "teleop_shaping/thin_jaka_transport_adapter.hpp"

namespace {

using teleop_command_abi::AcceptedJointTargetV1;
using teleop_command_abi::EngagementState;
using teleop_command_abi::JointDynamicLimitsV1;
using teleop_command_abi::MeasuredJointStateV1;
using teleop_command_abi::OutputMode;
using teleop_command_abi::ShapedJointCommandV1;
using teleop_command_abi::StopReason;
using teleop_command_abi::TargetValidity;
using teleop_shaping::OperationCode;
using teleop_shaping::EngagementJointShaperV1;
using teleop_shaping::EngagementShaperState;
using teleop_shaping::ThinJakaCode;
using teleop_shaping::ThinJakaState;
using teleop_shaping::ThinJakaTransportAdapter;

constexpr std::uint32_t kTargetMagic = 0x4A544754U;
constexpr std::uint32_t kStatusMagic = 0x4A535441U;
constexpr std::uint16_t kWireVersion = 1U;
constexpr std::int64_t kPeriodNs = 8'000'000;
constexpr char kGateAck[] = "I_AUTHORIZE_ONE_BOUNDED_RESEARCH_THIN_ADAPTER_JAKA_GATE";
constexpr std::array<double, 6> kJointLower{-6.28, -2.09, -2.27, -6.28, -2.09, -6.28};
constexpr std::array<double, 6> kJointUpper{6.28, 2.09, 2.27, 6.28, 2.09, 6.28};
constexpr double kJointMargin = 0.08726646259971647;

enum class TargetKind : std::uint16_t {
  kHeartbeat = 0,
  kHoldCurrent = 1,
  kJointPosition = 2,
  kCartesianPose = 3,
  kStop = 4,
};

constexpr std::uint32_t kAllowMotion = 1U;
constexpr std::uint32_t kStatusConnected = 1U << 0;
constexpr std::uint32_t kStatusEdgActive = 1U << 1;
constexpr std::uint32_t kStatusHolding = 1U << 2;
constexpr std::uint32_t kStatusHasTarget = 1U << 3;
constexpr std::uint32_t kStatusAccepted = 1U << 4;
constexpr std::uint32_t kStatusRejected = 1U << 5;
constexpr std::uint32_t kStatusTargetWarning = 1U << 6;
constexpr std::uint32_t kStatusControlledBraking = 1U << 9;
constexpr std::uint32_t kStatusStoppedReady = 1U << 10;
constexpr std::uint32_t kStatusMeasuredRefresh = 1U << 11;

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
static_assert(sizeof(TargetPacket) == 124U);
static_assert(sizeof(StatusPacket) == 108U);

std::atomic<bool> g_stop{false};
void SignalHandler(int) { g_stop.store(true, std::memory_order_relaxed); }

std::int64_t NowNs() noexcept {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec;
}

std::uint32_t Crc32(const void* data, std::size_t size) noexcept {
  std::uint32_t crc = 0xFFFFFFFFU;
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1U) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
  }
  return ~crc;
}

void SleepUntil(std::int64_t monotonic_ns) noexcept {
  timespec ts{static_cast<time_t>(monotonic_ns / 1'000'000'000LL),
              static_cast<long>(monotonic_ns % 1'000'000'000LL)};
  while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {}
}

struct Options {
  std::string robot_ip;
  std::string edg_state_ip;
  std::string target_socket;
  std::string status_socket;
  std::string metrics_file;
  std::string telemetry_file;
  std::string acknowledgement;
  double duration_s{30.0};
  int expected_tool_id{0};
  int expected_user_frame_id{0};
  double expected_payload_mass_kg{0.8};
  std::array<double, 3> expected_com_mm{9.289, 12.427, 36.961};
  std::array<double, 6> maximum_velocity{0.35, 0.35, 0.35, 0.50, 0.50, 0.50};
  double maximum_acceleration{2.0};
  double maximum_jerk{20.0};
  double tracking_abort_rad{0.20};
  bool hardware{false};
  bool preflight_only{false};
};

std::string ValueAfter(int& index, int argc, char** argv) {
  if (++index >= argc) throw std::runtime_error("missing option value");
  return argv[index];
}

template <std::size_t N>
std::array<double, N> ParseArray(const std::string& text) {
  std::array<double, N> result{};
  std::size_t start = 0;
  for (std::size_t i = 0; i < N; ++i) {
    const std::size_t end = text.find(',', start);
    if ((i + 1U < N && end == std::string::npos) ||
        (i + 1U == N && end != std::string::npos)) {
      throw std::runtime_error("comma-separated value count mismatch");
    }
    result[i] = std::stod(text.substr(start, end - start));
    if (!std::isfinite(result[i])) throw std::runtime_error("non-finite option");
    start = end + 1U;
  }
  return result;
}

Options ParseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--robot-ip") options.robot_ip = ValueAfter(i, argc, argv);
    else if (arg == "--edg-state-ip") options.edg_state_ip = ValueAfter(i, argc, argv);
    else if (arg == "--target-socket") options.target_socket = ValueAfter(i, argc, argv);
    else if (arg == "--status-socket") options.status_socket = ValueAfter(i, argc, argv);
    else if (arg == "--metrics-file") options.metrics_file = ValueAfter(i, argc, argv);
    else if (arg == "--cycle-telemetry-file") options.telemetry_file = ValueAfter(i, argc, argv);
    else if (arg == "--acknowledgement") options.acknowledgement = ValueAfter(i, argc, argv);
    else if (arg == "--duration-s") options.duration_s = std::stod(ValueAfter(i, argc, argv));
    else if (arg == "--expected-tool-id") options.expected_tool_id = std::stoi(ValueAfter(i, argc, argv));
    else if (arg == "--expected-user-frame-id") options.expected_user_frame_id = std::stoi(ValueAfter(i, argc, argv));
    else if (arg == "--expected-payload-mass-kg") options.expected_payload_mass_kg = std::stod(ValueAfter(i, argc, argv));
    else if (arg == "--expected-payload-com-mm") options.expected_com_mm = ParseArray<3>(ValueAfter(i, argc, argv));
    else if (arg == "--maximum-output-joint-velocity-rad-s-per-joint") options.maximum_velocity = ParseArray<6>(ValueAfter(i, argc, argv));
    else if (arg == "--maximum-output-joint-acceleration-rad-s2") options.maximum_acceleration = std::stod(ValueAfter(i, argc, argv));
    else if (arg == "--output-joint-jerk-limit-rad-s3") options.maximum_jerk = std::stod(ValueAfter(i, argc, argv));
    else if (arg == "--excessive-tracking-error-abort-rad") options.tracking_abort_rad = std::stod(ValueAfter(i, argc, argv));
    else if (arg == "--hardware") options.hardware = true;
    else if (arg == "--preflight-only") options.preflight_only = true;
    else if (arg == "--help") {
      std::cout << "research_thin_jaka_worker --hardware --robot-ip IP --edg-state-ip IP "
                   "--target-socket PATH --status-socket PATH --metrics-file PATH "
                   "--cycle-telemetry-file PATH --duration-s SEC --acknowledgement TOKEN\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }
  if (!options.hardware || options.acknowledgement != kGateAck ||
      options.robot_ip.empty() || options.edg_state_ip.empty()) {
    throw std::runtime_error("physical thin-adapter worker requires the exact bounded gate acknowledgement");
  }
  if (!(options.duration_s > 0.0 && options.duration_s <= 30.0) ||
      (!options.preflight_only &&
       (options.target_socket.empty() || options.status_socket.empty() ||
        options.telemetry_file.empty())) || options.metrics_file.empty() ||
      !(options.maximum_acceleration > 0.0 && options.maximum_acceleration <= 4.0) ||
      !(options.maximum_jerk > 0.0 && options.maximum_jerk <= 40.0) ||
      !(options.tracking_abort_rad > 0.0 && options.tracking_abort_rad <= 0.35) ||
      !std::all_of(options.maximum_velocity.begin(), options.maximum_velocity.end(),
                   [](double value) { return value > 0.0 && value <= 0.75; })) {
    throw std::runtime_error("bounded worker configuration is outside the research gate envelope");
  }
  return options;
}

int RunPreflight(const Options& options) {
  research_thin_jaka::JakaSdkTranslation sdk(options.robot_ip,
                                             options.edg_state_ip);
  teleop_shaping::ThinJakaConfig config{};
  ThinJakaTransportAdapter adapter(sdk.FunctionTable(), config);
  research_thin_jaka::ReadOnlyControllerSnapshot snapshot{};
  bool verified = false;
  if (adapter.Connect(NowNs()) == ThinJakaCode::kOk) {
    verified = sdk.ReadAndVerifyPreflight(
        options.expected_tool_id, options.expected_user_frame_id,
        options.expected_payload_mass_kg, options.expected_com_mm,
        0.01, 0.1, &snapshot);
    (void)adapter.Cleanup(NowNs());
  }
  std::ofstream out(options.metrics_file, std::ios::trunc);
  if (!out) throw std::runtime_error("cannot write preflight report");
  out << std::setprecision(17)
      << "{\n  \"schema_version\":\"research_thin_jaka_preflight.v1\",\n"
      << "  \"verified\":" << (verified ? "true" : "false") << ",\n"
      << "  \"payload_mass_kg\":" << snapshot.payload_mass_kg << ",\n"
      << "  \"payload_com_mm\":[" << snapshot.payload_com_mm[0] << ','
      << snapshot.payload_com_mm[1] << ',' << snapshot.payload_com_mm[2] << "],\n"
      << "  \"installation_rpy_rad\":[" << snapshot.installation_rpy_rad[0] << ','
      << snapshot.installation_rpy_rad[1] << ',' << snapshot.installation_rpy_rad[2] << "],\n"
      << "  \"active_tcp_mm_rpy_rad\":[" << snapshot.active_tcp_mm_rpy_rad[0] << ','
      << snapshot.active_tcp_mm_rpy_rad[1] << ',' << snapshot.active_tcp_mm_rpy_rad[2] << ','
      << snapshot.active_tcp_mm_rpy_rad[3] << ',' << snapshot.active_tcp_mm_rpy_rad[4] << ','
      << snapshot.active_tcp_mm_rpy_rad[5] << "],\n"
      << "  \"tool_id\":" << snapshot.tool_id << ",\n"
      << "  \"user_frame_id\":" << snapshot.user_frame_id << ",\n"
      << "  \"collision_level\":" << snapshot.collision_level << ",\n"
      << "  \"controller_error_code\":" << snapshot.controller_error_code << ",\n"
      << "  \"powered_on\":" << (snapshot.powered_on ? "true" : "false") << ",\n"
      << "  \"enabled\":" << (snapshot.enabled ? "true" : "false") << ",\n"
      << "  \"estop\":" << (snapshot.estop ? "true" : "false") << ",\n"
      << "  \"collision\":" << (snapshot.collision ? "true" : "false") << ",\n"
      << "  \"servo_move\":" << (snapshot.servo_move ? "true" : "false") << ",\n"
      << "  \"last_sdk_return\":" << sdk.last_sdk_return_code() << ",\n"
      << "  \"last_sdk_operation\":\"" << sdk.last_operation() << "\",\n"
      << "  \"writes_performed\":false,\n"
      << "  \"edg_or_servo_enabled\":false\n}\n";
  return verified ? 0 : 2;
}

class TargetSocket final {
 public:
  explicit TargetSocket(const std::string& path) : path_(path) {
    fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd_ < 0) throw std::runtime_error("target socket creation failed");
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (path.size() >= sizeof(address.sun_path)) throw std::runtime_error("target socket path too long");
    std::strncpy(address.sun_path, path.c_str(), sizeof(address.sun_path) - 1U);
    unlink(path.c_str());
    if (bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      throw std::runtime_error("target socket bind failed");
    }
  }
  ~TargetSocket() { if (fd_ >= 0) close(fd_); unlink(path_.c_str()); }
  bool ReceiveLatest(TargetPacket* packet, std::uint64_t* superseded) noexcept {
    bool received = false;
    TargetPacket candidate{};
    for (;;) {
      const ssize_t count = recv(fd_, &candidate, sizeof(candidate), 0);
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (count != static_cast<ssize_t>(sizeof(candidate))) break;
      if (received) ++*superseded;
      *packet = candidate;
      received = true;
    }
    return received;
  }
 private:
  int fd_{-1};
  std::string path_;
};

class StatusSender final {
 public:
  explicit StatusSender(std::string path) : path_(std::move(path)) {
    fd_ = socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd_ < 0) throw std::runtime_error("status socket creation failed");
  }
  ~StatusSender() { if (fd_ >= 0) close(fd_); }
  void Send(StatusPacket packet) noexcept {
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (path_.size() >= sizeof(address.sun_path)) return;
    std::strncpy(address.sun_path, path_.c_str(), sizeof(address.sun_path) - 1U);
    packet.crc32 = Crc32(&packet, sizeof(packet) - sizeof(packet.crc32));
    (void)sendto(fd_, &packet, sizeof(packet), MSG_DONTWAIT,
                 reinterpret_cast<sockaddr*>(&address), sizeof(address));
  }
 private:
  int fd_{-1};
  std::string path_;
};

struct TelemetryRow {
  std::int64_t tick_ns{};
  std::int64_t wake_lateness_ns{};
  std::int64_t send_duration_ns{};
  std::int64_t command_age_ns{};
  std::int64_t feedback_call_start_ns{};
  std::int64_t feedback_call_end_ns{};
  std::int64_t feedback_sample_ns{};
  std::int64_t feedback_validation_ns{};
  std::int64_t feedback_call_duration_ns{};
  std::int64_t feedback_sample_age_ns{};
  std::uint64_t input_sequence{};
  std::uint64_t output_sequence{};
  std::uint64_t epoch{};
  ThinJakaState transport_state{};
  OutputMode output_mode{OutputMode::kInactive};
  std::array<double, 6> shaped_q{};
  std::array<double, 6> shaped_dq{};
  std::array<double, 6> shaped_ddq{};
  std::array<double, 6> measured_q{};
  std::array<double, 6> measured_dq{};
  int sdk_return{};
};

bool ValidPacket(const TargetPacket& packet, std::uint64_t previous_sequence,
                 std::int64_t now_ns) noexcept {
  if (packet.magic != kTargetMagic || packet.version != kWireVersion ||
      packet.kind > static_cast<std::uint16_t>(TargetKind::kStop) ||
      packet.sequence <= previous_sequence || packet.dispatch_ns == 0U ||
      packet.dispatch_ns > static_cast<std::uint64_t>(now_ns) ||
      packet.local_receive_ns > packet.processing_ns ||
      packet.processing_ns > packet.dispatch_ns ||
      Crc32(&packet, sizeof(packet) - sizeof(packet.crc32)) != packet.crc32) {
    return false;
  }
  for (double value : packet.payload) if (!std::isfinite(value)) return false;
  return true;
}

JointDynamicLimitsV1 Limits(const Options& options) noexcept {
  JointDynamicLimitsV1 limits{};
  limits.header = teleop_command_abi::MakeHeaderV1<JointDynamicLimitsV1>();
  limits.dof = 6;
  for (std::size_t i = 0; i < 6; ++i) {
    limits.minimum_position_rad[i] = kJointLower[i] + kJointMargin;
    limits.maximum_position_rad[i] = kJointUpper[i] - kJointMargin;
    limits.maximum_velocity_rad_s[i] = options.maximum_velocity[i];
    limits.maximum_acceleration_rad_s2[i] = options.maximum_acceleration;
    limits.maximum_jerk_rad_s3[i] = options.maximum_jerk;
  }
  return limits;
}

AcceptedJointTargetV1 AcceptedFromPacket(const TargetPacket& packet,
                                         std::uint64_t epoch,
                                         std::int64_t now_ns,
                                         bool accepted) noexcept {
  AcceptedJointTargetV1 target{};
  target.header = teleop_command_abi::MakeHeaderV1<AcceptedJointTargetV1>();
  target.sequence = packet.sequence;
  target.safety_epoch = epoch;
  target.source_monotonic_ns = packet.source_capture_ns == 0U
      ? static_cast<std::int64_t>(packet.local_receive_ns)
      : static_cast<std::int64_t>(packet.source_capture_ns);
  if (target.source_monotonic_ns > static_cast<std::int64_t>(packet.processing_ns)) {
    target.source_monotonic_ns = static_cast<std::int64_t>(packet.local_receive_ns);
  }
  target.accepted_monotonic_ns = static_cast<std::int64_t>(packet.processing_ns);
  target.valid_until_monotonic_ns = now_ns + 100'000'000;
  target.dof = 6;
  target.engagement = EngagementState::kEngaged;
  target.validity = accepted ? TargetValidity::kAccepted
                             : TargetValidity::kRejectedKeepPrevious;
  if (accepted) std::copy_n(packet.payload, 6, target.position_rad.begin());
  return target;
}

void WriteSix(std::ostream& out, const std::array<double, 6>& values) {
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0U) out << ',';
    out << values[i];
  }
  out << ']';
}

void WriteTelemetry(const Options& options,
                    const std::vector<TelemetryRow>& rows) {
  std::ofstream out(options.telemetry_file, std::ios::trunc);
  if (!out) throw std::runtime_error("cannot write telemetry");
  out << std::setprecision(17);
  for (const auto& row : rows) {
    out << "{\"tick_ns\":" << row.tick_ns
        << ",\"wake_lateness_ns\":" << row.wake_lateness_ns
        << ",\"send_duration_ns\":" << row.send_duration_ns
        << ",\"command_age_ns\":" << row.command_age_ns
        << ",\"feedback_call_start_ns\":" << row.feedback_call_start_ns
        << ",\"feedback_call_end_ns\":" << row.feedback_call_end_ns
        << ",\"feedback_sample_ns\":" << row.feedback_sample_ns
        << ",\"feedback_validation_ns\":" << row.feedback_validation_ns
        << ",\"feedback_call_duration_ns\":" << row.feedback_call_duration_ns
        << ",\"feedback_sample_age_ns\":" << row.feedback_sample_age_ns
        << ",\"input_sequence\":" << row.input_sequence
        << ",\"output_sequence\":" << row.output_sequence
        << ",\"safety_epoch\":" << row.epoch
        << ",\"transport_state\":" << static_cast<int>(row.transport_state)
        << ",\"output_mode\":" << static_cast<int>(row.output_mode)
        << ",\"shaped_q_rad\":"; WriteSix(out, row.shaped_q);
    out << ",\"shaped_dq_rad_s\":"; WriteSix(out, row.shaped_dq);
    out << ",\"shaped_ddq_rad_s2\":"; WriteSix(out, row.shaped_ddq);
    out << ",\"measured_q_rad\":"; WriteSix(out, row.measured_q);
    out << ",\"measured_dq_rad_s\":"; WriteSix(out, row.measured_dq);
    out << ",\"sdk_return\":" << row.sdk_return << "}\n";
  }
}

int Run(const Options& options) {
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  TargetSocket target_socket(options.target_socket);
  StatusSender status_sender(options.status_socket);
  research_thin_jaka::JakaSdkTranslation sdk(options.robot_ip,
                                             options.edg_state_ip);
  teleop_shaping::ThinJakaConfig transport_config{};
  transport_config.dof = 6;
  transport_config.pause_policy =
      teleop_shaping::PauseCommandPolicy::kRepeatStoppedPositionRequired;
  transport_config.resume_policy =
      teleop_shaping::ResumePreparationPolicy::kKeepPrepared;
  transport_config.measurement.dof = 6;
  // EDG supplies q and dq directly. One fresh sample is sufficient; the gate
  // still rejects residual velocity above the stationary threshold.
  transport_config.measurement.stable_sample_count = 1;
  transport_config.measurement.stationary_velocity_rad_s = 0.003;
  transport_config.maximum_status_age_ns = 32'000'000;
  transport_config.maximum_tick_interval_ns = 16'000'000;
  transport_config.status_poll_interval_ticks = 2;
  transport_config.servo_step_num = 1;
  transport_config.resume_position_tolerance_rad = 5e-5;
  ThinJakaTransportAdapter adapter(sdk.FunctionTable(), transport_config);
  const JointDynamicLimitsV1 limits = Limits(options);
  if (!teleop_command_abi::Validate(limits).ok) {
    throw std::runtime_error("invalid dynamic limits");
  }
  research_thin_jaka::ReadOnlyControllerSnapshot preflight{};
  std::vector<TelemetryRow> telemetry;
  telemetry.reserve(static_cast<std::size_t>(options.duration_s * 125.0) + 32U);
  EngagementJointShaperV1 shaper;
  std::optional<TargetPacket> pending_resume_target;
  std::optional<TargetPacket> deferred_target;
  std::uint64_t epoch = 0;
  std::uint64_t last_input_sequence = 0;
  std::uint64_t last_target_dispatch_ns = 0;
  std::uint64_t latest_accepted_sequence = 0;
  std::uint64_t superseded = 0;
  std::uint64_t accepted_count = 0;
  std::uint64_t rejected_count = 0;
  std::uint64_t pause_count = 0;
  std::uint64_t resume_count = 0;
  std::uint64_t first_resume_tick_count = 0;
  std::uint64_t rh56_command_count = 0;
  std::uint64_t deadline_miss_count = 0;
  std::uint64_t consecutive_deadline_misses = 0;
  std::int64_t maximum_wake_lateness_ns = 0;
  std::int64_t maximum_send_duration_ns = 0;
  std::int64_t maximum_command_age_ns = 0;
  double maximum_tracking_error = 0.0;
  std::array<double, 6> maximum_abs_velocity{};
  std::array<double, 6> maximum_abs_acceleration{};
  std::array<double, 6> maximum_abs_jerk{};
  std::array<double, 6> initial_q{};
  std::array<double, 6> measured_q{};
  std::array<double, 6> measured_dq{};
  ShapedJointCommandV1 last_shaped{};
  bool last_shaped_valid = false;
  bool initial_measurement_recorded = false;
  bool connected = false;
  bool servo_prepared = false;
  bool hard_fault = false;
  std::string outcome = "duration_complete";
  std::int64_t servo_start_ns = 0;
  bool automatic_stop_requested = false;
  try {
    if (adapter.Connect(NowNs()) != ThinJakaCode::kOk) {
      throw std::runtime_error("SDK login failed");
    }
    connected = true;
    if (!sdk.ReadAndVerifyPreflight(options.expected_tool_id,
                                    options.expected_user_frame_id,
                                    options.expected_payload_mass_kg,
                                    options.expected_com_mm, 0.01, 0.1,
                                    &preflight)) {
      throw std::runtime_error("read-only controller preflight rejected motion");
    }
    if (adapter.PrepareServo(NowNs()) != ThinJakaCode::kOk) {
      throw std::runtime_error("EDG/servo preparation failed");
    }
    servo_prepared = true;
    // Servo preparation owns no active shaper clock.  q/dq observation
    // continues below while the transport waits in ServoReady.  The first
    // clutch engagement and every re-engagement start a fresh measurement
    // refresh and initialize a new 8 ms shaper grid.
    servo_start_ns = NowNs();
    std::int64_t deadline = servo_start_ns;
    std::uint64_t loop_sequence = 0;
    while (!g_stop.load(std::memory_order_relaxed)) {
      deadline += kPeriodNs;
      SleepUntil(deadline);
      const std::int64_t cycle_start = NowNs();
      const std::int64_t elapsed_ns = cycle_start - servo_start_ns;
      const std::int64_t duration_ns =
          static_cast<std::int64_t>(options.duration_s * 1e9);
      const auto shaper_state_before_input = shaper.Snapshot().state;
      if (!automatic_stop_requested && duration_ns > 1'000'000'000 &&
          elapsed_ns >= duration_ns - 1'000'000'000 &&
          (shaper_state_before_input == EngagementShaperState::kArmed ||
           shaper_state_before_input == EngagementShaperState::kActiveTracking) &&
          latest_accepted_sequence != 0U) {
        const auto stop = shaper.RequestControlledStop(
            last_input_sequence + 1U, StopReason::kClutchRelease,
            shaper.Snapshot().next_tick_monotonic_ns);
        if (stop.code != OperationCode::kOk &&
            stop.code != OperationCode::kAlreadyRequested) {
          outcome = "duration_controlled_stop_planning_failed";
          hard_fault = true;
          adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
          break;
        }
        automatic_stop_requested = true;
        outcome = "duration_complete";
      }
      if (elapsed_ns >= duration_ns) {
        if (latest_accepted_sequence == 0U &&
            adapter.Snapshot().state == ThinJakaState::kServoReady) {
          outcome = "duration_complete_no_stream";
        } else if (adapter.Snapshot().state != ThinJakaState::kStoppedReady) {
          outcome = "duration_stop_timeout";
          hard_fault = true;
          adapter.HardStop(StopReason::kTimingFault, cycle_start);
        }
        break;
      }
      const std::int64_t wake_lateness = std::max<std::int64_t>(0, cycle_start - deadline);
      maximum_wake_lateness_ns = std::max(maximum_wake_lateness_ns, wake_lateness);
      if (wake_lateness > kPeriodNs) {
        ++deadline_miss_count;
        ++consecutive_deadline_misses;
        const auto timing_state = shaper.Snapshot().state;
        const bool active_grid =
            timing_state == EngagementShaperState::kArmed ||
            timing_state == EngagementShaperState::kActiveTracking ||
            timing_state == EngagementShaperState::kControlledBraking;
        if (active_grid || wake_lateness > 12'000'000 ||
            consecutive_deadline_misses >= 2U) {
          outcome = "hard_timing_fault";
          hard_fault = true;
          shaper.HardStop(StopReason::kTimingFault, cycle_start);
          adapter.HardStop(StopReason::kTimingFault, cycle_start);
          break;
        }
        deadline = cycle_start;
      } else {
        consecutive_deadline_misses = 0;
      }

      TargetPacket packet{};
      bool have_packet = false;
      if (deferred_target.has_value()) {
        packet = *deferred_target;
        deferred_target.reset();
        have_packet = true;
      }
      TargetPacket received_packet{};
      if (target_socket.ReceiveLatest(&received_packet, &superseded)) {
        if (have_packet) ++superseded;
        packet = received_packet;
        have_packet = true;
      }
      bool initialized_this_cycle = false;
      if (have_packet) {
        if (!ValidPacket(packet, last_input_sequence, cycle_start)) {
          outcome = "invalid_command";
          hard_fault = true;
          adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
          break;
        }
        last_target_dispatch_ns = packet.dispatch_ns;
        const auto kind = static_cast<TargetKind>(packet.kind);
        const auto current_shaper_state = shaper.Snapshot().state;
        const bool command_uses_active_grid =
            current_shaper_state == EngagementShaperState::kArmed ||
            current_shaper_state == EngagementShaperState::kActiveTracking ||
            current_shaper_state == EngagementShaperState::kControlledBraking;
        if (kind != TargetKind::kStop && command_uses_active_grid &&
            static_cast<std::int64_t>(packet.processing_ns) >
                shaper.Snapshot().next_tick_monotonic_ns) {
          // The packet became causal after this logical tick. Keep only this
          // latest packet and activate it on the following 8 ms grid point.
          deferred_target = packet;
          have_packet = false;
        }
        if (!have_packet) {
          // Continue with transport observation; no target is applied early.
        } else {
        last_input_sequence = packet.sequence;
        if (kind == TargetKind::kStop) {
          outcome = "terminal_operator_stop";
          shaper.HardStop(StopReason::kProducerFailure, cycle_start);
          adapter.HardStop(StopReason::kProducerFailure, cycle_start);
          break;
        }
        if (kind == TargetKind::kHoldCurrent) {
          if ((packet.flags != 0U || packet.frame_id != 0U)) {
            outcome = "invalid_pause_command";
            hard_fault = true;
            adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
            break;
          }
          const auto engagement_state = shaper.Snapshot().state;
          if (engagement_state == EngagementShaperState::kArmed ||
              engagement_state == EngagementShaperState::kActiveTracking) {
            const auto result = shaper.RequestControlledStop(
                packet.sequence, StopReason::kClutchRelease,
                shaper.Snapshot().next_tick_monotonic_ns);
            if (result.code != OperationCode::kOk &&
                result.code != OperationCode::kAlreadyRequested) {
              outcome = "controlled_brake_planning_failed";
              hard_fault = true;
              adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
              break;
            }
            ++pause_count;
          }
        } else if (kind == TargetKind::kJointPosition) {
          if (packet.flags != kAllowMotion || packet.frame_id != 0U) {
            outcome = "joint_target_contract_mismatch";
            hard_fault = true;
            adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
            break;
          }
          const bool initial_engagement =
              adapter.Snapshot().state == ThinJakaState::kServoReady &&
              shaper.Snapshot().state ==
                  EngagementShaperState::kWaitingForEngagement;
          if (initial_engagement ||
              adapter.Snapshot().state == ThinJakaState::kStoppedReady) {
            pending_resume_target = packet;
            ++epoch;
            if (adapter.BeginMeasuredStateRefresh(epoch, cycle_start) !=
                ThinJakaCode::kOk) {
              outcome = "resume_refresh_start_failed";
              hard_fault = true;
              break;
            }
          } else if (adapter.Snapshot().state ==
                     ThinJakaState::kMeasuredStateRefresh) {
            pending_resume_target = packet;
          } else if (shaper.Snapshot().state == EngagementShaperState::kArmed ||
                     shaper.Snapshot().state ==
                         EngagementShaperState::kActiveTracking) {
            const std::int64_t grid_now =
                shaper.Snapshot().next_tick_monotonic_ns;
            const auto accepted = AcceptedFromPacket(packet, epoch, grid_now, true);
            if (shaper.ReplaceTarget(accepted, grid_now).code != OperationCode::kOk) {
              outcome = "accepted_target_rejected_by_shaper";
              hard_fault = true;
              adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
              break;
            }
            latest_accepted_sequence = packet.sequence;
            ++accepted_count;
          }
        } else if (kind == TargetKind::kHeartbeat) {
          if (packet.flags != kAllowMotion || packet.frame_id != 0U) {
            outcome = "heartbeat_contract_mismatch";
            hard_fault = true;
            adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
            break;
          }
          if ((shaper.Snapshot().state == EngagementShaperState::kArmed ||
               shaper.Snapshot().state ==
                   EngagementShaperState::kActiveTracking) &&
              latest_accepted_sequence != 0U) {
            const std::int64_t grid_now =
                shaper.Snapshot().next_tick_monotonic_ns;
            const auto heartbeat = AcceptedFromPacket(packet, epoch, grid_now, false);
            if (shaper.ReplaceTarget(heartbeat, grid_now).code != OperationCode::kOk) {
              outcome = "heartbeat_rejected_by_shaper";
              hard_fault = true;
              adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
              break;
            }
            ++rejected_count;
          }
        } else {
          outcome = "non_joint_target_received";
          hard_fault = true;
          adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
          break;
        }
        }
      }

      if (adapter.Snapshot().state == ThinJakaState::kMeasuredStateRefresh) {
        MeasuredJointStateV1 refreshed{};
        const auto refresh = adapter.RefreshMeasuredState(cycle_start, &refreshed);
        if (refresh == ThinJakaCode::kOk) {
          const std::int64_t refresh_validation_ns =
              adapter.Snapshot().last_feedback_validation_ns;
          if (!pending_resume_target.has_value()) {
            outcome = "engagement_target_missing_after_refresh";
            hard_fault = true;
            adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
            break;
          }
          double delta = 0.0;
          for (std::size_t i = 0; i < 6; ++i) {
            delta = std::max(
                delta, std::abs(pending_resume_target->payload[i] -
                                refreshed.position_rad[i]));
          }
          if (delta > 0.001) {
            outcome = "engagement_target_not_continuous_with_measured_q";
            hard_fault = true;
            adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
            break;
          }
          const auto accepted = AcceptedFromPacket(
              *pending_resume_target, epoch, refresh_validation_ns, true);
          if (shaper.InitializeEngagement(refreshed, limits, accepted,
                                          refresh_validation_ns).code !=
              OperationCode::kOk) {
            outcome = "engagement_shaper_initialize_failed";
            hard_fault = true;
            adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
            break;
          }
          deadline = refresh_validation_ns;
          latest_accepted_sequence = pending_resume_target->sequence;
          ++accepted_count;
          if (epoch > 1U) ++resume_count;
          pending_resume_target.reset();
          initialized_this_cycle = true;
          last_shaped = {};
          last_shaped_valid = false;
          if (!initial_measurement_recorded) {
            std::copy_n(refreshed.position_rad.begin(), 6, initial_q.begin());
            initial_measurement_recorded = true;
          }
          std::copy_n(refreshed.position_rad.begin(), 6, measured_q.begin());
          std::copy_n(refreshed.velocity_rad_s.begin(), 6, measured_dq.begin());
        } else if (refresh != ThinJakaCode::kNeedMoreSamples &&
                   refresh != ThinJakaCode::kUnstableMeasurement) {
          outcome = "engagement_measured_state_failed";
          hard_fault = true;
          break;
        }
      }

      ShapedJointCommandV1 shaped{};
      bool shaped_this_tick = false;
      std::int64_t cycle_send_duration_ns = 0;
      const auto engagement_state = shaper.Snapshot().state;
      if (!initialized_this_cycle &&
          ((engagement_state == EngagementShaperState::kArmed &&
            latest_accepted_sequence != 0U) ||
           engagement_state == EngagementShaperState::kActiveTracking ||
           engagement_state == EngagementShaperState::kControlledBraking)) {
        const std::int64_t grid_now =
            shaper.Snapshot().next_tick_monotonic_ns;
        const auto tick = shaper.Tick(grid_now, &shaped);
        if (tick.code == OperationCode::kTerminalNoOutput ||
            tick.code == OperationCode::kPlanningFailed) {
          outcome = "shaper_terminal_failure";
          hard_fault = true;
          adapter.HardStop(StopReason::kInvalidCommand, cycle_start);
          break;
        }
        shaped_this_tick = true;
        const std::int64_t send_start = NowNs();
        ThinJakaCode send_result = ThinJakaCode::kInvalidState;
        if (adapter.Snapshot().state == ThinJakaState::kServoReady) {
          send_result = adapter.StartStreaming(shaped, cycle_start);
          if (send_result == ThinJakaCode::kOk) {
            ++first_resume_tick_count;
            // StartStreaming sends the first point directly. Advance the
            // adapter's caller-supplied tick clock in the same cycle so the
            // next 8 ms tick is not measured from the prior ServoReady tick.
            send_result = adapter.Tick(cycle_start);
          }
        } else {
          send_result = adapter.OfferLatest(shaped, cycle_start);
          if (send_result == ThinJakaCode::kOk) send_result = adapter.Tick(cycle_start);
        }
        cycle_send_duration_ns = NowNs() - send_start;
        maximum_send_duration_ns = std::max(maximum_send_duration_ns,
                                            cycle_send_duration_ns);
        if (send_result != ThinJakaCode::kOk) {
          outcome = "thin_adapter_send_or_health_failure";
          hard_fault = true;
          break;
        }
        if (last_shaped_valid) {
          for (std::size_t i = 0; i < 6; ++i) {
            const double jerk = (shaped.acceleration_rad_s2[i] -
                                 last_shaped.acceleration_rad_s2[i]) / 0.008;
            maximum_abs_jerk[i] = std::max(maximum_abs_jerk[i], std::abs(jerk));
          }
        }
        for (std::size_t i = 0; i < 6; ++i) {
          maximum_abs_velocity[i] = std::max(maximum_abs_velocity[i],
                                             std::abs(shaped.velocity_rad_s[i]));
          maximum_abs_acceleration[i] = std::max(
              maximum_abs_acceleration[i], std::abs(shaped.acceleration_rad_s2[i]));
        }
        last_shaped = shaped;
        last_shaped_valid = true;
      } else {
        const auto tick_result = adapter.Tick(cycle_start);
        if (tick_result != ThinJakaCode::kOk) {
          outcome = "thin_adapter_idle_or_pause_health_failure";
          hard_fault = true;
          break;
        }
      }

      teleop_shaping::JakaJointFeedback feedback{};
      const auto feedback_result = sdk.FunctionTable().read_joint_feedback(
          sdk.FunctionTable().context, &feedback);
      if (feedback_result != teleop_shaping::JakaFunctionResult::kOk) {
        outcome = "EDG_feedback_failure";
        hard_fault = true;
        adapter.HardStop(StopReason::kSdkFailure, cycle_start);
        break;
      }
      std::copy_n(feedback.position_rad.begin(), 6, measured_q.begin());
      std::copy_n(feedback.velocity_rad_s.begin(), 6, measured_dq.begin());
      if (last_shaped_valid) {
        for (std::size_t i = 0; i < 6; ++i) {
          const double error = std::abs(std::remainder(
              last_shaped.position_rad[i] - measured_q[i], 2.0 * M_PI));
          maximum_tracking_error = std::max(maximum_tracking_error, error);
          if (error > options.tracking_abort_rad) {
            outcome = "measured_tracking_error";
            hard_fault = true;
            adapter.HardStop(StopReason::kControllerAlarm, cycle_start);
            break;
          }
        }
        if (hard_fault) break;
      }
      const std::int64_t command_age = last_target_dispatch_ns == 0U ? 0 :
          std::max<std::int64_t>(0, cycle_start -
             static_cast<std::int64_t>(last_target_dispatch_ns));
      maximum_command_age_ns = std::max(maximum_command_age_ns, command_age);
      const auto snapshot = adapter.Snapshot();
      TelemetryRow row{};
      row.tick_ns = cycle_start;
      row.wake_lateness_ns = wake_lateness;
      row.send_duration_ns = cycle_send_duration_ns;
      row.command_age_ns = command_age;
      row.feedback_call_start_ns = feedback.sdk_call_start_monotonic_ns;
      row.feedback_call_end_ns = feedback.sdk_call_end_monotonic_ns;
      row.feedback_sample_ns = feedback.sampled_monotonic_ns;
      row.feedback_validation_ns = feedback.validation_monotonic_ns;
      row.feedback_call_duration_ns = feedback.sdk_call_end_monotonic_ns -
                                      feedback.sdk_call_start_monotonic_ns;
      row.feedback_sample_age_ns = feedback.validation_monotonic_ns -
                                   feedback.sampled_monotonic_ns;
      row.input_sequence = last_input_sequence;
      row.output_sequence = snapshot.last_output_sequence;
      row.epoch = snapshot.safety_epoch;
      row.transport_state = snapshot.state;
      row.output_mode = shaped_this_tick ? shaped.output_mode :
          (last_shaped_valid ? last_shaped.output_mode : OutputMode::kInactive);
      if (last_shaped_valid) {
        std::copy_n(last_shaped.position_rad.begin(), 6, row.shaped_q.begin());
        std::copy_n(last_shaped.velocity_rad_s.begin(), 6, row.shaped_dq.begin());
        std::copy_n(last_shaped.acceleration_rad_s2.begin(), 6, row.shaped_ddq.begin());
      }
      row.measured_q = measured_q;
      row.measured_dq = measured_dq;
      row.sdk_return = sdk.last_sdk_return_code();
      telemetry.push_back(row);

      if ((++loop_sequence % 13U) == 0U) {
        std::uint32_t flags = kStatusConnected | kStatusEdgActive;
        if (last_input_sequence != 0U) flags |= kStatusHasTarget;
        if (accepted_count != 0U) flags |= kStatusAccepted;
        if (rejected_count != 0U) flags |= kStatusRejected;
        if (command_age > 40'000'000) flags |= kStatusTargetWarning;
        if (snapshot.state == ThinJakaState::kControlledStopping)
          flags |= kStatusControlledBraking;
        if (snapshot.state == ThinJakaState::kStoppedReady)
          flags |= kStatusStoppedReady | kStatusHolding;
        if (snapshot.state == ThinJakaState::kMeasuredStateRefresh)
          flags |= kStatusMeasuredRefresh | kStatusHolding;
        if (snapshot.state == ThinJakaState::kServoReady)
          flags |= kStatusHolding;
        const auto wire_state = static_cast<std::uint16_t>(
            snapshot.state == ThinJakaState::kFaulted ? 8U :
            snapshot.state == ThinJakaState::kStreaming ? 6U : 5U);
        StatusPacket status{kStatusMagic, kWireVersion, wire_state,
                            flags, last_input_sequence, loop_sequence,
                            static_cast<std::uint64_t>(cycle_start),
                            shaped_this_tick ? static_cast<std::uint64_t>(cycle_start) : 0U,
                            static_cast<std::uint64_t>(feedback.sampled_monotonic_ns),
                            {}, 0, 0};
        std::copy(measured_q.begin(), measured_q.end(), status.joint_position_rad);
        status_sender.Send(status);
      }
    }
  } catch (const std::exception& error) {
    outcome = std::string("fault:") + error.what();
    hard_fault = true;
    if (connected) adapter.HardStop(StopReason::kSdkFailure, NowNs());
  }

  if (connected) {
    const auto state = adapter.Snapshot().state;
    const auto engagement_state = shaper.Snapshot().state;
    if (!hard_fault && state == ThinJakaState::kStreaming &&
        (engagement_state == EngagementShaperState::kArmed ||
         engagement_state == EngagementShaperState::kActiveTracking)) {
      const std::int64_t stop_grid_ns =
          shaper.Snapshot().next_tick_monotonic_ns;
      SleepUntil(stop_grid_ns);
      (void)shaper.RequestControlledStop(last_input_sequence + 1U,
                                         StopReason::kClutchRelease,
                                         stop_grid_ns);
    }
    if (!hard_fault &&
        shaper.Snapshot().state ==
            EngagementShaperState::kControlledBraking) {
      const std::int64_t stop_deadline = std::min(
          servo_start_ns + static_cast<std::int64_t>(options.duration_s * 1e9),
          NowNs() + 900'000'000);
      while (NowNs() < stop_deadline) {
        const std::int64_t grid_now =
            shaper.Snapshot().next_tick_monotonic_ns;
        SleepUntil(grid_now);
        const std::int64_t now = NowNs();
        ShapedJointCommandV1 command{};
        const auto result = shaper.Tick(grid_now, &command);
        if (result.code == OperationCode::kTerminalNoOutput) {
          hard_fault = true;
          adapter.HardStop(StopReason::kInvalidCommand, now);
          break;
        }
        if (adapter.OfferLatest(command, now) != ThinJakaCode::kOk ||
            adapter.Tick(now) != ThinJakaCode::kOk) {
          hard_fault = true;
          break;
        }
        if (command.output_mode == OutputMode::kStopped) break;
      }
    }
    if (adapter.Snapshot().state == ThinJakaState::kStreaming ||
        adapter.Snapshot().state == ThinJakaState::kControlledStopping ||
        adapter.Snapshot().state == ThinJakaState::kMeasuredStateRefresh) {
      hard_fault = true;
      adapter.HardStop(StopReason::kTimingFault, NowNs());
    }
    const auto cleanup = adapter.Cleanup(NowNs());
    if (cleanup != ThinJakaCode::kOk) {
      hard_fault = true;
      outcome = "cleanup_failure";
    }
  }

  WriteTelemetry(options, telemetry);
  const auto snapshot = adapter.Snapshot();
  std::ofstream metrics(options.metrics_file, std::ios::trunc);
  if (!metrics) throw std::runtime_error("cannot write metrics");
  metrics << std::setprecision(17)
          << "{\n  \"schema_version\":\"research_thin_jaka_gate.v1\",\n"
          << "  \"outcome\":\"" << outcome << "\",\n"
          << "  \"hardware_connected\":" << (connected ? "true" : "false") << ",\n"
          << "  \"servo_prepared\":" << (servo_prepared ? "true" : "false") << ",\n"
          << "  \"pause_policy\":\"repeat_stopped_position_required\",\n"
          << "  \"resume_policy\":\"keep_prepared\",\n"
          << "  \"payload_mass_kg\":" << preflight.payload_mass_kg << ",\n"
          << "  \"payload_com_mm\":[" << preflight.payload_com_mm[0] << ','
          << preflight.payload_com_mm[1] << ',' << preflight.payload_com_mm[2] << "],\n"
          << "  \"installation_rpy_rad\":[" << preflight.installation_rpy_rad[0] << ','
          << preflight.installation_rpy_rad[1] << ',' << preflight.installation_rpy_rad[2] << "],\n"
          << "  \"active_tcp_mm_rpy_rad\":[" << preflight.active_tcp_mm_rpy_rad[0] << ','
          << preflight.active_tcp_mm_rpy_rad[1] << ',' << preflight.active_tcp_mm_rpy_rad[2] << ','
          << preflight.active_tcp_mm_rpy_rad[3] << ',' << preflight.active_tcp_mm_rpy_rad[4] << ','
          << preflight.active_tcp_mm_rpy_rad[5] << "],\n"
          << "  \"collision_level\":" << preflight.collision_level << ",\n"
          << "  \"controller_error_code\":" << preflight.controller_error_code << ",\n"
          << "  \"controller_estop\":" << (preflight.estop ? "true" : "false") << ",\n"
          << "  \"controller_collision\":" << (preflight.collision ? "true" : "false") << ",\n"
          << "  \"accepted_target_count\":" << accepted_count << ",\n"
          << "  \"hold_rejected_count\":" << rejected_count << ",\n"
          << "  \"pause_count\":" << pause_count << ",\n"
          << "  \"resume_count\":" << resume_count << ",\n"
          << "  \"first_resume_tick_count\":" << first_resume_tick_count << ",\n"
          << "  \"rh56_command_count\":" << rh56_command_count << ",\n"
          << "  \"superseded_input_count\":" << superseded << ",\n"
          << "  \"deadline_miss_count\":" << deadline_miss_count << ",\n"
          << "  \"maximum_wake_lateness_ns\":" << maximum_wake_lateness_ns << ",\n"
          << "  \"maximum_send_duration_ns\":" << maximum_send_duration_ns << ",\n"
          << "  \"maximum_command_age_ns\":" << maximum_command_age_ns << ",\n"
          << "  \"maximum_tracking_error_rad\":" << maximum_tracking_error << ",\n"
          << "  \"maximum_resume_position_delta_rad\":"
          << snapshot.maximum_resume_position_delta_rad << ",\n"
          << "  \"maximum_feedback_call_duration_ns\":"
          << snapshot.maximum_feedback_call_duration_ns << ",\n"
          << "  \"maximum_feedback_sample_age_ns\":"
          << snapshot.maximum_feedback_sample_age_ns << ",\n"
          << "  \"maximum_velocity_rad_s\":"; WriteSix(metrics, maximum_abs_velocity);
  metrics << ",\n  \"maximum_acceleration_rad_s2\":"; WriteSix(metrics, maximum_abs_acceleration);
  metrics << ",\n  \"maximum_jerk_rad_s3\":"; WriteSix(metrics, maximum_abs_jerk);
  metrics << ",\n  \"initial_q_rad\":"; WriteSix(metrics, initial_q);
  metrics << ",\n  \"final_measured_q_rad\":"; WriteSix(metrics, measured_q);
  metrics << ",\n  \"final_measured_dq_rad_s\":"; WriteSix(metrics, measured_dq);
  metrics << ",\n  \"last_sdk_return\":" << sdk.last_sdk_return_code()
          << ",\n  \"last_sdk_operation\":\"" << sdk.last_operation() << "\"\n}\n";
  return hard_fault ? 2 : 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = ParseOptions(argc, argv);
    return options.preflight_only ? RunPreflight(options) : Run(options);
  } catch (const std::exception& error) {
    std::cerr << "research thin JAKA worker error: " << error.what() << '\n';
    return 64;
  }
}
