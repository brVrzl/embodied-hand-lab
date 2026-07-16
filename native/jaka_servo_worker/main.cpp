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
constexpr double kProbeMaximumVelocityRadS = 0.005;
constexpr double kProbeMaximumAccelerationRadS2 = 0.02;

enum class Mode { DryRun, StateRead, ZeroMotion, MinimalMotion };
enum class State : std::uint16_t {
  Disconnected, Connecting, Connected, Armed, EdgReady, Holding, Running,
  ControlledStop, Fault, Shutdown
};
enum class TargetKind : std::uint16_t { Heartbeat, HoldCurrent, JointPosition, CartesianPose, Stop };
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
  std::uint64_t fake_read_delay_ns = 0;
  std::uint64_t fake_write_delay_ns = 0;
  std::uint64_t fake_fail_after = 0;
  std::array<double, 3> workspace_min_mm{};
  std::array<double, 3> workspace_max_mm{};
  bool workspace_min_set = false;
  bool workspace_max_set = false;
};

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
    else if (a == "--fake-read-delay-us") o.fake_read_delay_ns = std::stoull(value_after(i, argc, argv)) * 1000;
    else if (a == "--fake-write-delay-us") o.fake_write_delay_ns = std::stoull(value_after(i, argc, argv)) * 1000;
    else if (a == "--fake-fail-after") o.fake_fail_after = std::stoull(value_after(i, argc, argv));
    else if (a == "--workspace-min-mm") { o.workspace_min_mm = parse_xyz(value_after(i, argc, argv)); o.workspace_min_set = true; }
    else if (a == "--workspace-max-mm") { o.workspace_max_mm = parse_xyz(value_after(i, argc, argv)); o.workspace_max_set = true; }
    else if (a == "--help") {
      std::cout << "jaka_servo_worker --mode dry-run|state-read|zero-motion|minimal-motion [options]\n";
      std::exit(0);
    } else throw std::runtime_error("unknown option: " + a);
  }
  if (!(o.duration_s > 0.0 && o.duration_s <= 2000.0)) throw std::runtime_error("duration must be in (0, 2000] s");
  if (!(o.warning_ns < o.hold_ns && o.hold_ns < o.stop_ns && o.stop_ns < o.fatal_ns))
    throw std::runtime_error("stale thresholds must be strictly increasing");
  if (o.mode != Mode::DryRun) {
    if (!o.hardware || o.acknowledgement != kHardwareAck || o.robot_ip.empty())
      throw std::runtime_error("connected modes require --hardware, --robot-ip, and exact acknowledgement");
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

class Backend {
 public:
  virtual ~Backend() = default;
  virtual void connect() = 0;
  virtual void verify(int tool, int user) = 0;
  virtual void enter_edg() = 0;
  virtual void validate_probe(const std::array<double, 6>& initial, const std::array<double, 6>& target) = 0;
  virtual void read(std::array<double, 6>& joints) = 0;
  virtual void command(const std::array<double, 6>& joints) = 0;
  virtual void cleanup() noexcept = 0;
  virtual int cleanup_error_code() const noexcept = 0;
};

class FakeBackend final : public Backend {
 public:
  explicit FakeBackend(const Options& o) : options_(o) {}
  ~FakeBackend() override { cleanup(); }
  void connect() override { connected_ = true; }
  void verify(int, int) override { if (!connected_) throw std::runtime_error("fake disconnected"); }
  void enter_edg() override { if (!connected_) throw std::runtime_error("fake disconnected"); edg_ = true; }
  void validate_probe(const std::array<double, 6>&, const std::array<double, 6>&) override {}
  void read(std::array<double, 6>& joints) override { delay(options_.fake_read_delay_ns); fail(); joints = joints_; }
  void command(const std::array<double, 6>& joints) override { delay(options_.fake_write_delay_ns); fail(); if (!edg_) throw std::runtime_error("fake EDG inactive"); joints_ = joints; }
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
  void command(const std::array<double, 6>& joints) override {
    JointValue value{};
    for (std::size_t i = 0; i < joints.size(); ++i) value.jVal[i] = joints[i];
    require_sdk(robot_.edg_servo_j(&value, ABS, 1), "edg_servo_j");
  }
  void cleanup() noexcept override {
    if (servo_) { record_cleanup_error(robot_.servo_move_enable(FALSE)); servo_ = false; }
    if (edg_) { record_cleanup_error(robot_.edg_init(FALSE, options_.edg_state_ip.c_str())); edg_ = false; }
    if (connected_) { record_cleanup_error(robot_.login_out()); connected_ = false; }
  }
  int cleanup_error_code() const noexcept override { return cleanup_error_code_; }
 private:
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

struct Samples {
  std::array<std::uint64_t, kMaximumSamples> periods{}, reads{}, writes{}, sdk{}, target_ages{}, command_ages{};
  std::size_t count = 0;
  std::uint64_t missed = 0, maximum_consecutive = 0;
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

void write_metrics(const Options& o, const Samples& s, std::uint64_t accepted, std::uint64_t rejected,
                   std::uint64_t warning_cycles,
                   double elapsed_s, double cpu_s, double maximum_command_delta_rad,
                   double maximum_observed_delta_rad, int error_code, int cleanup_error_code,
                   const std::string& outcome) {
  std::ofstream file;
  std::ostream* output = &std::cout;
  if (!o.metrics_file.empty()) { file.open(o.metrics_file); if (!file) throw std::runtime_error("cannot open metrics file"); output = &file; }
  auto& out = *output;
  out << std::setprecision(12) << "{\n  \"schema_version\":\"jaka_worker_metrics.v1\",\n"
      << "  \"mode\":\"" << (o.mode == Mode::DryRun ? "native_no_robot" : o.mode == Mode::StateRead ? "connected_state_read" : o.mode == Mode::ZeroMotion ? "zero_motion_edg" : "minimal_motion") << "\",\n"
      << "  \"outcome\":\"" << outcome << "\",\n  \"requested_period_ns\":" << kPeriodNs << ",\n"
      << "  \"elapsed_s\":" << elapsed_s << ",\n  \"worker_cpu_s\":" << cpu_s << ",\n  \"worker_cpu_percent\":" << (elapsed_s > 0 ? cpu_s / elapsed_s * 100.0 : 0.0) << ",\n"
      << "  \"loop_rate_hz\":" << (elapsed_s > 0 ? s.count / elapsed_s : 0.0) << ",\n"
      << "  \"accepted_target_rate_hz\":" << (elapsed_s > 0 ? accepted / elapsed_s : 0.0) << ",\n"
      << "  \"maximum_intentional_command_delta_rad\":" << maximum_command_delta_rad << ",\n"
      << "  \"maximum_observed_joint_delta_rad\":" << maximum_observed_delta_rad << ",\n"
      << "  \"accepted_targets\":" << accepted << ",\n  \"rejected_targets\":" << rejected << ",\n"
      << "  \"target_age_warning_cycles\":" << warning_cycles << ",\n"
      << "  \"error_code\":" << error_code << ",\n  \"cleanup_error_code\":" << cleanup_error_code << ",\n"
      << "  \"missed_deadlines\":" << s.missed << ",\n  \"max_consecutive_missed_deadlines\":" << s.maximum_consecutive << ",\n  \"statistics\":{\n";
  metric_json(out, "actual_cycle_period", s.periods, s.count, true);
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
  auto backend = o.mode == Mode::DryRun ? std::unique_ptr<Backend>(new FakeBackend(o)) : std::unique_ptr<Backend>(new RealBackend(o));
  TargetSocket target_socket(o.target_socket); StatusSender status_sender(o.status_socket);
  auto samples_storage = std::make_unique<Samples>();
  Samples& samples = *samples_storage;
  TargetPacket latest{}; bool ever_received = false;
  std::uint64_t accepted = 0, rejected = 0, last_sequence = 0, last_dispatch = 0, consecutive_overruns = 0;
  std::uint64_t warning_cycles = 0;
  std::uint64_t status_accepted = 0, status_rejected = 0;
  std::array<double, 6> initial{}, observed{}, target{};
  double maximum_command_delta_rad = 0.0, maximum_observed_delta_rad = 0.0;
  State state = State::Connecting; std::int32_t error_code = 0;
  const char* outcome = "completed";
  std::string fault_outcome;
  rusage usage_start{}, usage_end{}; getrusage(RUSAGE_SELF, &usage_start);
  const auto start = now_ns(); auto previous = start; auto deadline = start;
  try {
    backend->connect(); state = State::Connected;
    backend->verify(o.expected_tool_id, o.expected_user_frame_id); state = State::Armed;
    backend->read(initial); observed = initial; target = initial;
    if (!std::all_of(initial.begin(), initial.end(), [](double v) { return std::isfinite(v) && std::abs(v) <= 2.0 * M_PI; })) throw std::runtime_error("initial joint state failed radians/finiteness check");
    if (o.mode == Mode::MinimalMotion) {
      auto endpoint = initial; endpoint[static_cast<std::size_t>(o.probe_joint)] += o.probe_delta_rad;
      backend->validate_probe(initial, endpoint);
    }
    if (o.mode != Mode::StateRead) { backend->enter_edg(); state = State::EdgReady; backend->read(observed);
      double delta = 0; for (std::size_t i = 0; i < 6; ++i) delta = std::max(delta, std::abs(observed[i] - initial[i]));
      if (delta > 1e-4) throw std::runtime_error("near-zero initial command delta check failed");
    }
    state = State::Holding;
    while (!g_stop.load(std::memory_order_relaxed) && samples.count < kMaximumSamples) {
      deadline += kPeriodNs;
      timespec wake{static_cast<time_t>(deadline / 1'000'000'000), static_cast<long>(deadline % 1'000'000'000)};
      while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, nullptr) == EINTR && !g_stop.load()) {}
      const auto cycle_start = now_ns();
      if (cycle_start - start >= static_cast<std::uint64_t>(o.duration_s * 1e9)) break;
      TargetPacket packet{};
      bool invalid_command = false, transport_failure = false;
      if (target_socket.drain_newest(packet, last_sequence, ever_received, cycle_start, rejected,
                                     invalid_command, transport_failure)) {
        latest = packet; last_sequence = packet.sequence; last_dispatch = packet.dispatch_ns; ever_received = true; ++accepted;
        state = packet.kind == static_cast<std::uint16_t>(TargetKind::Stop) ? State::ControlledStop : State::Running;
      }
      if (transport_failure) { state = State::Fault; outcome = "target_transport_failure"; break; }
      if (invalid_command) { state = State::ControlledStop; outcome = "invalid_command"; break; }
      if (state == State::ControlledStop) { outcome = "operator_stop_command"; break; }
      const auto age = ever_received ? cycle_start - std::min(cycle_start, last_dispatch) : 0;
      if (ever_received && age >= o.warning_ns) ++warning_cycles;
      if (ever_received && age >= o.fatal_ns) { state = State::Fault; outcome = "fatal_target_timeout"; break; }
      if (ever_received && age >= o.stop_ns) { state = State::ControlledStop; outcome = "controlled_stop_target_timeout"; break; }
      if (!ever_received || age >= o.hold_ns) state = State::Holding;
      const auto read_start = now_ns(); backend->read(observed); const auto read_end = now_ns();
      for (std::size_t joint = 0; joint < 6; ++joint)
        maximum_observed_delta_rad = std::max(maximum_observed_delta_rad, std::abs(observed[joint] - initial[joint]));
      std::uint64_t write_duration = 0, command_time = 0;
      if (o.mode != Mode::StateRead) {
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
      const std::size_t i = samples.count++;
      samples.periods[i] = cycle_start - previous; previous = cycle_start;
      samples.reads[i] = read_end - read_start; samples.writes[i] = write_duration;
      samples.sdk[i] = samples.reads[i] + samples.writes[i];
      samples.target_ages[i] = ever_received ? age : 0;
      samples.command_ages[i] = command_time && last_dispatch ? command_time - std::min(command_time, last_dispatch) : 0;
      if (cycle_end > deadline + kPeriodNs) { ++samples.missed; ++consecutive_overruns; samples.maximum_consecutive = std::max(samples.maximum_consecutive, consecutive_overruns); }
      else consecutive_overruns = 0;
      if (consecutive_overruns >= o.max_consecutive_overruns) { state = State::Fault; outcome = "control_loop_overrun"; break; }
      if ((i % 13) == 0) {
        std::uint32_t flags = kStatusConnected;
        if (o.mode != Mode::StateRead) flags |= kStatusEdgActive;
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
                maximum_observed_delta_rad, error_code, cleanup_error_code, outcome);
  return error_code == 0 ? 0 : 2;
}
}  // namespace

int main(int argc, char** argv) {
  try { return run(parse_options(argc, argv)); }
  catch (const std::exception& e) { std::cerr << "configuration error: " << e.what() << '\n'; return 64; }
}
