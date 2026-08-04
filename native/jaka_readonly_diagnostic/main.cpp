#include "readonly_backend.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
constexpr int kSdkTimeoutCode = -61;
std::atomic<bool> g_stop{false};
void signal_handler(int) { g_stop.store(true, std::memory_order_relaxed); }

enum class Mode { DryRun, Fake, Connected };
struct Options {
  Mode mode = Mode::DryRun;
  std::string robot_ip;
  std::string metrics_file;
  double duration_s = 5.0;
  double poll_hz = 10.0;
  double slow_poll_hz = 1.0;
  std::size_t max_samples = 10'000;
  int sessions = 1;
  int max_consecutive_failures = 5;
  jaka_readonly::FakeOptions fake{};
};

std::string next_value(int& index, int argc, char** argv) {
  if (++index >= argc) throw std::runtime_error(std::string("missing value after ") + argv[index - 1]);
  return argv[index];
}

bool valid_ipv4(const std::string& value) {
  in_addr address{};
  return inet_pton(AF_INET, value.c_str(), &address) == 1;
}

Options parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--mode") {
      const auto value = next_value(i, argc, argv);
      if (value == "dry-run") options.mode = Mode::DryRun;
      else if (value == "fake") options.mode = Mode::Fake;
      else if (value == "connected") options.mode = Mode::Connected;
      else throw std::runtime_error("mode must be dry-run, fake, or connected");
    } else if (argument == "--robot-ip") options.robot_ip = next_value(i, argc, argv);
    else if (argument == "--metrics-file") options.metrics_file = next_value(i, argc, argv);
    else if (argument == "--duration-s") options.duration_s = std::stod(next_value(i, argc, argv));
    else if (argument == "--poll-hz") options.poll_hz = std::stod(next_value(i, argc, argv));
    else if (argument == "--slow-poll-hz") options.slow_poll_hz = std::stod(next_value(i, argc, argv));
    else if (argument == "--max-samples") options.max_samples = std::stoull(next_value(i, argc, argv));
    else if (argument == "--sessions") options.sessions = std::stoi(next_value(i, argc, argv));
    else if (argument == "--max-consecutive-failures") options.max_consecutive_failures = std::stoi(next_value(i, argc, argv));
    else if (argument == "--fake-delay-us") options.fake.call_delay_ns = std::stoull(next_value(i, argc, argv)) * 1000;
    else if (argument == "--fake-timeout-every") options.fake.timeout_every = std::stoull(next_value(i, argc, argv));
    else if (argument == "--fake-disconnect-after") options.fake.disconnect_after_fast_reads = std::stoull(next_value(i, argc, argv));
    else if (argument == "--fake-fail-login") options.fake.fail_login = true;
    else if (argument == "--fake-fail-logout") options.fake.fail_logout = true;
    else if (argument == "--help") {
      std::cout << "jaka_readonly_diagnostic --robot-ip IPV4 [--mode dry-run|fake|connected] [options]\n"
                   "Default mode is dry-run and never constructs the JAKA SDK client.\n";
      std::exit(0);
    } else throw std::runtime_error("unknown option: " + argument);
  }
  if (!valid_ipv4(options.robot_ip)) throw std::runtime_error("--robot-ip must be an explicit IPv4 address");
  if (!(options.duration_s > 0.0 && options.duration_s <= 3600.0)) throw std::runtime_error("duration must be in (0, 3600] seconds");
  if (!(options.poll_hz >= 0.1 && options.poll_hz <= 125.0)) throw std::runtime_error("poll rate must be within [0.1, 125] Hz");
  if (!(options.slow_poll_hz >= 0.1 && options.slow_poll_hz <= options.poll_hz)) throw std::runtime_error("slow poll rate must be within [0.1, poll rate] Hz");
  if (options.max_samples == 0 || options.max_samples > 100'000) throw std::runtime_error("max samples must be within [1, 100000]");
  if (options.sessions < 1 || options.sessions > 20) throw std::runtime_error("sessions must be within [1, 20]");
  if (options.max_consecutive_failures < 1 || options.max_consecutive_failures > 100) throw std::runtime_error("max consecutive failures must be within [1, 100]");
  return options;
}

struct Series {
  explicit Series(std::size_t capacity = 0) { values.reserve(capacity); }
  void add(std::uint64_t value) { if (values.size() < values.capacity()) values.push_back(value); }
  std::vector<std::uint64_t> values;
};

struct Stats {
  std::size_t count = 0;
  double mean = 0, median = 0, stddev = 0, minimum = 0, maximum = 0, p95 = 0, p99 = 0;
  bool has_p999 = false; double p999 = 0;
};

double percentile(const std::vector<std::uint64_t>& sorted, double quantile) {
  if (sorted.empty()) return 0.0;
  const double position = quantile * static_cast<double>(sorted.size() - 1);
  const auto lower = static_cast<std::size_t>(position);
  const auto upper = std::min(lower + 1, sorted.size() - 1);
  return sorted[lower] + (position - lower) * static_cast<double>(sorted[upper] - sorted[lower]);
}

Stats statistics(const Series& series) {
  Stats result; result.count = series.values.size();
  if (series.values.empty()) return result;
  std::vector<std::uint64_t> sorted = series.values; std::sort(sorted.begin(), sorted.end());
  const long double sum = std::accumulate(sorted.begin(), sorted.end(), static_cast<long double>(0));
  result.mean = static_cast<double>(sum / sorted.size());
  long double squared = 0;
  for (auto value : sorted) { const long double delta = value - result.mean; squared += delta * delta; }
  result.median = percentile(sorted, .5); result.stddev = std::sqrt(static_cast<double>(squared / sorted.size()));
  result.minimum = sorted.front(); result.maximum = sorted.back(); result.p95 = percentile(sorted, .95); result.p99 = percentile(sorted, .99);
  result.has_p999 = sorted.size() >= 1000; if (result.has_p999) result.p999 = percentile(sorted, .999);
  return result;
}

struct Results {
  explicit Results(std::size_t capacity)
      : initialization(capacity), first_read_latency(capacity), disconnect(capacity), periods(capacity), jitter(capacity),
        sdk_cycle(capacity), calls{Series(capacity), Series(capacity), Series(capacity), Series(capacity), Series(capacity),
        Series(capacity), Series(capacity), Series(capacity), Series(capacity), Series(capacity), Series(capacity),
        Series(capacity), Series(capacity), Series(capacity), Series(capacity), Series(capacity), Series(capacity)} {}
  Series initialization, first_read_latency, disconnect, periods, jitter, sdk_cycle;
  std::array<Series, jaka_readonly::kCallCount> calls;
  std::uint64_t failed_calls = 0, failed_reads = 0, connection_failures = 0, disconnect_failures = 0;
  std::uint64_t timeouts = 0, maximum_consecutive_failures = 0, reconnect_attempts = 0;
  std::uint64_t missed_poll_deadlines = 0, maximum_consecutive_missed_deadlines = 0;
  std::size_t cycles = 0, sessions_completed = 0;
  std::size_t baseline_thread_count = 0, post_cleanup_thread_count = 0;
  double polling_elapsed_s = 0.0;
};

void record_batch(const jaka_readonly::Batch& batch, Results& results) {
  for (std::size_t i = 0; i < jaka_readonly::kCallCount; ++i) {
    const auto& observation = batch.calls[i];
    if (!observation.attempted) continue;
    results.calls[i].add(observation.duration_ns);
    if (observation.code != 0) {
      ++results.failed_calls;
      if (i != static_cast<std::size_t>(jaka_readonly::Call::Login) &&
          i != static_cast<std::size_t>(jaka_readonly::Call::Logout)) ++results.failed_reads;
      if (observation.code == kSdkTimeoutCode) ++results.timeouts;
    }
  }
}

std::uint64_t batch_duration(const jaka_readonly::Batch& batch) {
  std::uint64_t sum = 0;
  for (const auto& observation : batch.calls) if (observation.attempted) sum += observation.duration_ns;
  return sum;
}

std::string escaped(const std::string& value) {
  std::string output;
  for (char character : value) {
    if (character == '\\' || character == '"') { output.push_back('\\'); output.push_back(character); }
    else if (character == '\n') output += "\\n";
    else if (static_cast<unsigned char>(character) >= 32) output.push_back(character);
  }
  return output;
}

void write_stats(std::ostream& output, const Series& series) {
  const auto stats = statistics(series);
  output << "{\"count\":" << stats.count << ",\"mean_ns\":" << stats.mean << ",\"median_ns\":" << stats.median
         << ",\"stddev_ns\":" << stats.stddev << ",\"min_ns\":" << stats.minimum << ",\"max_ns\":" << stats.maximum
         << ",\"p95_ns\":" << stats.p95 << ",\"p99_ns\":" << stats.p99 << ",\"p999_ns\":";
  if (stats.has_p999) output << stats.p999; else output << "null";
  output << "}";
}

template <std::size_t Size>
void write_array(std::ostream& output, const std::array<double, Size>& values) {
  output << '[';
  for (std::size_t i = 0; i < Size; ++i) { if (i) output << ','; output << values[i]; }
  output << ']';
}

double cpu_seconds(const rusage& usage) {
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 + usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}

std::size_t process_thread_count() {
  std::size_t count = 0;
  for ([[maybe_unused]] const auto& entry : std::filesystem::directory_iterator("/proc/self/task")) ++count;
  return count;
}

void write_results(const Options& options, const Results& results, const jaka_readonly::State& state,
                   const std::string& backend_name, const std::string& outcome, double elapsed_s, double cpu_s) {
  std::ofstream file; std::ostream* destination = &std::cout;
  if (!options.metrics_file.empty()) { file.open(options.metrics_file); if (!file) throw std::runtime_error("cannot open metrics file"); destination = &file; }
  auto& out = *destination; out << std::setprecision(12);
  out << "{\n  \"schema_version\":\"jaka_readonly_gate3a.v1\",\n  \"mode\":\""
      << (options.mode == Mode::Connected ? "connected_read_only" : options.mode == Mode::Fake ? "fake" : "dry_run")
      << "\",\n  \"backend\":\"" << backend_name << "\",\n  \"outcome\":\"" << escaped(outcome) << "\",\n"
      << "  \"robot_ip\":\"" << escaped(options.robot_ip) << "\",\n  \"credentials\":\"unsupported_by_installed_sdk\",\n"
      << "  \"requested_poll_hz\":" << options.poll_hz << ",\n  \"requested_slow_poll_hz\":" << options.slow_poll_hz
      << ",\n  \"bounded_sample_capacity\":" << options.max_samples << ",\n  \"cycles\":" << results.cycles
      << ",\n  \"sessions_completed\":" << results.sessions_completed << ",\n  \"reconnect_attempts\":" << results.reconnect_attempts
      << ",\n  \"failed_calls\":" << results.failed_calls << ",\n  \"failed_reads\":" << results.failed_reads
      << ",\n  \"connection_failures\":" << results.connection_failures
      << ",\n  \"disconnect_failures\":" << results.disconnect_failures << ",\n  \"timeouts\":" << results.timeouts
      << ",\n  \"max_consecutive_failed_cycles\":" << results.maximum_consecutive_failures
      << ",\n  \"missed_poll_deadlines\":" << results.missed_poll_deadlines
      << ",\n  \"max_consecutive_missed_poll_deadlines\":" << results.maximum_consecutive_missed_deadlines << ",\n"
      << "  \"elapsed_s\":" << elapsed_s << ",\n  \"worker_cpu_s\":" << cpu_s << ",\n  \"worker_cpu_percent\":"
      << (elapsed_s > 0 ? cpu_s / elapsed_s * 100.0 : 0.0) << ",\n  \"achieved_poll_hz\":"
      << (results.polling_elapsed_s > 0 ? results.cycles / results.polling_elapsed_s : 0.0)
      << ",\n  \"polling_elapsed_s\":" << results.polling_elapsed_s
      << ",\n  \"baseline_thread_count\":" << results.baseline_thread_count
      << ",\n  \"post_cleanup_thread_count\":" << results.post_cleanup_thread_count
      << ",\n  \"connection_duration\":null,\n"
      << "  \"connection_duration_note\":\"SDK exposes one combined login_in connection call\",\n  \"statistics\":{\n";
  out << "    \"sdk_initialization\":"; write_stats(out, results.initialization); out << ",\n";
  out << "    \"first_successful_state_read_latency\":"; write_stats(out, results.first_read_latency); out << ",\n";
  out << "    \"disconnect\":"; write_stats(out, results.disconnect); out << ",\n";
  out << "    \"actual_poll_period\":"; write_stats(out, results.periods); out << ",\n";
  out << "    \"absolute_scheduling_jitter\":"; write_stats(out, results.jitter); out << ",\n";
  out << "    \"sdk_calls_per_cycle\":"; write_stats(out, results.sdk_cycle); out << "\n  },\n  \"call_statistics\":{\n";
  for (std::size_t i = 0; i < jaka_readonly::kCallCount; ++i) {
    out << "    \"" << jaka_readonly::kCallNames[i] << "\":"; write_stats(out, results.calls[i]);
    out << (i + 1 == jaka_readonly::kCallCount ? "\n" : ",\n");
  }
  out << "  },\n  \"state_inventory\":{\n"
      << "    \"sdk_version\":\"" << escaped(state.sdk_version) << "\",\n"
      << "    \"controller_version\":null,\n    \"robot_model\":null,\n    \"robot_operating_mode\":null,\n"
      << "    \"controller_timestamp\":null,\n    \"controller_version_note\":\"not exposed by installed C++ API\",\n"
      << "    \"robot_model_note\":\"not exposed by installed C++ API\",\n"
      << "    \"robot_operating_mode_note\":\"program state and servo state are reported separately; neither is relabeled as operating mode\",\n"
      << "    \"controller_timestamp_note\":\"only EDG timestamp API exists and is prohibited in Gate 3A\",\n"
      << "    \"joint_position_rad\":"; if (state.joint_position_available) write_array(out, state.joint_position_rad); else out << "null"; out << ",\n"
      << "    \"joint_velocity_rad_s\":"; if (state.joint_velocity_available) write_array(out, state.joint_velocity_rad_s); else out << "null"; out << ",\n"
      << "    \"joint_velocity_note\":\"direct field from deprecated/config-dependent combined RobotStatus structure\",\n"
      << "    \"tcp_mm_rpy_rad\":"; if (state.tcp_available) write_array(out, state.tcp_mm_rpy_rad); else out << "null"; out << ",\n"
      << "    \"fault_code\":" << (state.status_available ? std::to_string(state.fault_code) : "null") << ",\n"
      << "    \"fault_message\":" << (state.status_available ? "\"" + escaped(state.fault_message) + "\"" : "null") << ",\n"
      << "    \"powered\":" << (state.status_available ? (state.powered ? "true" : "false") : "null") << ",\n"
      << "    \"enabled\":" << (state.status_available ? (state.enabled ? "true" : "false") : "null") << ",\n"
      << "    \"emergency_stop\":" << (state.emergency_stop_available ? (state.emergency_stop ? "true" : "false") : "null") << ",\n"
      << "    \"collision\":" << (state.collision_available ? (state.collision ? "true" : "false") : "null") << ",\n"
      << "    \"servo_move_active\":" << (state.servo_state_available ? (state.servo_move_active ? "true" : "false") : "null") << ",\n"
      << "    \"sdk_socket_connected\":" << (state.socket_connected_available ? (state.socket_connected ? "true" : "false") : "null") << ",\n"
      << "    \"tool_id\":" << (state.tool_id_available ? std::to_string(state.tool_id) : "null") << ",\n"
      << "    \"tool_frame_mm_rpy_rad\":"; if (state.tool_data_available) write_array(out, state.tool_mm_rpy_rad); else out << "null"; out << ",\n"
      << "    \"user_frame_id\":" << (state.user_frame_id_available ? std::to_string(state.user_frame_id) : "null") << ",\n"
      << "    \"user_frame_mm_rpy_rad\":"; if (state.user_frame_data_available) write_array(out, state.user_frame_mm_rpy_rad); else out << "null"; out << ",\n"
      << "    \"program_state\":" << (state.program_state_available ? std::to_string(state.program_state) : "null") << ",\n"
      << "    \"program_motion_line\":" << (state.program_info_available ? std::to_string(state.program_motion_line) : "null") << "\n  }\n}\n";
}

int dry_run(const Options& options) {
  std::cout << "{\"schema_version\":\"jaka_readonly_gate3a.v1\",\"mode\":\"dry_run\","
               "\"connection_opened\":false,\"configuration_valid\":true,\"robot_ip\":\""
            << escaped(options.robot_ip) << "\",\"credentials\":\"unsupported_by_installed_sdk\","
               "\"read_only_calls_only\":true}\n";
  return 0;
}

int execute(const Options& options) {
  if (options.mode == Mode::DryRun) return dry_run(options);
  if (options.mode == Mode::Connected) {
    std::cerr << "JAKA GATE 3A READ-ONLY DIAGNOSTIC\nTarget: " << options.robot_ip
              << "\nNo EDG entry. No command-writing APIs are exposed. No robot motion is intended.\n"
              << "Polling: " << options.poll_hz << " Hz fast, " << options.slow_poll_hz << " Hz status.\n";
  }
  std::signal(SIGINT, signal_handler); std::signal(SIGTERM, signal_handler); std::signal(SIGHUP, signal_handler);
  Results results(options.max_samples); jaka_readonly::State state;
  results.baseline_thread_count = process_thread_count();
  std::string backend_name, outcome = "completed";
  rusage usage_start{}, usage_end{}; getrusage(RUSAGE_SELF, &usage_start); const auto total_start = Clock::now();
  const auto requested_period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / options.poll_hz));
  const auto slow_period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / options.slow_poll_hz));
  std::uint64_t consecutive_failures = 0, consecutive_missed_deadlines = 0; bool fatal = false;
  for (int session = 0; session < options.sessions && !g_stop.load() && !fatal; ++session) {
    if (session > 0) ++results.reconnect_attempts;
    const auto initialization_start = Clock::now();
    auto backend = options.mode == Mode::Connected ? jaka_readonly::make_vendor_backend() : jaka_readonly::make_fake_backend(options.fake);
    results.initialization.add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - initialization_start).count()));
    backend_name = backend->name();
    jaka_readonly::Batch batch{}; backend->query_sdk_version(state, batch); record_batch(batch, results);
    batch = {}; const int connect_code = backend->connect(options.robot_ip, batch); record_batch(batch, results);
    if (connect_code != 0) { ++results.connection_failures; outcome = "connection_failed_code_" + std::to_string(connect_code); fatal = true; continue; }
    batch = {}; backend->read_static(state, batch); record_batch(batch, results);
    const auto login_complete = Clock::now(); auto next_wake = login_complete; auto next_slow = login_complete;
    auto previous_cycle = login_complete; bool first_success = false;
    const auto session_end = login_complete + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(options.duration_s));
    while (!g_stop.load() && Clock::now() < session_end && results.cycles < options.max_samples) {
      const auto cycle_start = Clock::now();
      if (results.cycles > 0) {
        const auto actual = std::chrono::duration_cast<std::chrono::nanoseconds>(cycle_start - previous_cycle).count();
        const auto requested = std::chrono::duration_cast<std::chrono::nanoseconds>(requested_period).count();
        results.periods.add(static_cast<std::uint64_t>(actual)); results.jitter.add(static_cast<std::uint64_t>(std::llabs(actual - requested)));
      }
      previous_cycle = cycle_start; batch = {};
      const bool fast_ok = backend->read_fast(state, batch);
      const auto fast_complete = Clock::now();
      if (fast_ok && !first_success) {
        results.first_read_latency.add(static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(fast_complete - login_complete).count()));
        first_success = true;
      }
      if (cycle_start >= next_slow) { backend->read_slow(state, batch); next_slow += slow_period; }
      results.sdk_cycle.add(batch_duration(batch)); record_batch(batch, results); ++results.cycles;
      if (fast_ok) {
        consecutive_failures = 0;
      } else {
        ++consecutive_failures; results.maximum_consecutive_failures = std::max(results.maximum_consecutive_failures, consecutive_failures);
        if (consecutive_failures >= static_cast<std::uint64_t>(options.max_consecutive_failures)) {
          outcome = "consecutive_read_failure"; fatal = true; break;
        }
      }
      next_wake += requested_period;
      if (Clock::now() > next_wake) {
        ++results.missed_poll_deadlines; ++consecutive_missed_deadlines;
        results.maximum_consecutive_missed_deadlines = std::max(
            results.maximum_consecutive_missed_deadlines, consecutive_missed_deadlines);
      } else consecutive_missed_deadlines = 0;
      std::this_thread::sleep_until(next_wake);
    }
    results.polling_elapsed_s += std::chrono::duration<double>(Clock::now() - login_complete).count();
    batch = {}; const auto disconnect_start = Clock::now(); const int disconnect_code = backend->disconnect(batch);
    results.disconnect.add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - disconnect_start).count()));
    record_batch(batch, results);
    if (disconnect_code != 0) { ++results.disconnect_failures; outcome = "disconnect_failed_code_" + std::to_string(disconnect_code); fatal = true; }
    else ++results.sessions_completed;
  }
  if (g_stop.load()) outcome = "operator_interrupted_cleanly";
  results.post_cleanup_thread_count = process_thread_count();
  getrusage(RUSAGE_SELF, &usage_end); const double elapsed = std::chrono::duration<double>(Clock::now() - total_start).count();
  write_results(options, results, state, backend_name, outcome, elapsed, cpu_seconds(usage_end) - cpu_seconds(usage_start));
  if (g_stop.load()) return 130;
  return fatal ? 2 : 0;
}
}  // namespace

int main(int argc, char** argv) {
  try { return execute(parse(argc, argv)); }
  catch (const std::exception& error) { std::cerr << "configuration error: " << error.what() << '\n'; return 64; }
}
