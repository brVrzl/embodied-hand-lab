#include "zero_motion_backend.hpp"

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
constexpr std::uint64_t kRequestedPeriodNs = 8'000'000;
constexpr std::uint64_t kWakeLatenessWarningNs = 2'000'000;
constexpr std::uint64_t kAccumulatedTimingDebtNs = 8'000'000;
constexpr std::uint64_t kPeriodOverrunThresholdNs = 8'800'000;
constexpr std::uint64_t kHardStartPeriodNs = 12'000'000;
constexpr std::uint64_t kHardCompletionFromReleaseNs = 12'000'000;
constexpr std::uint64_t kMaximumConsecutivePeriodOverruns = 2;
constexpr std::uint64_t kMaximumConsecutiveCompletionMisses = 2;
constexpr std::size_t kTimingCapacity = 700;
constexpr double kMaximumPriorStageJointDeltaRad = 0.001;
constexpr double kObservationWarningRad = 50e-6;
constexpr double kObservationAbortRad = 500e-6;
constexpr double kFirstCommandDifferenceRad = 100e-6;
constexpr int kOperatorCountdownSeconds = 3;
constexpr const char* kZeroMotionAck = "I_ACKNOWLEDGE_INVARIANT_JOINT_COMMAND";

std::atomic<bool> g_stop{false};
void signal_handler(int) { g_stop.store(true, std::memory_order_relaxed); }

enum class BackendKind { Fake, Vendor };
enum class Stage { DryRun, Preflight, EntryExit, RunOneSecond, RunFiveSeconds };

const char* stage_name(Stage stage) {
  switch (stage) {
    case Stage::DryRun: return "dry-run";
    case Stage::Preflight: return "preflight";
    case Stage::EntryExit: return "entry-exit";
    case Stage::RunOneSecond: return "run-1s";
    case Stage::RunFiveSeconds: return "run-5s";
  }
  return "unknown";
}

const char* required_stage_approval(Stage stage) {
  switch (stage) {
    case Stage::Preflight: return "I_APPROVE_GATE3B_STAGE_2_PREFLIGHT";
    case Stage::EntryExit: return "I_APPROVE_GATE3B_STAGE_3_ENTRY_EXIT";
    case Stage::RunOneSecond: return "I_APPROVE_GATE3B_STAGE_4_ONE_SECOND";
    case Stage::RunFiveSeconds: return "I_APPROVE_GATE3B_STAGE_5_FIVE_SECONDS";
    case Stage::DryRun: return "";
  }
  return "";
}

struct Options {
  BackendKind backend = BackendKind::Fake;
  Stage stage = Stage::DryRun;
  std::string robot_ip;
  std::string edg_state_ip;
  std::string zero_motion_ack;
  std::string stage_approval;
  std::string joint_units;
  std::string result_file;
  std::string raw_timing_file;
  std::string prior_stage_result;
  bool physical_hardware = false;
  bool estop_access_confirmed = false;
  bool workspace_clear_confirmed = false;
  int expected_joint_count = 6;
  int expected_tool_id = 0;
  int expected_user_frame_id = 0;
  bool expected_tool_id_set = false;
  bool expected_user_frame_id_set = false;
  jaka_zero::FakeOptions fake{};
  std::uint64_t fake_start_lateness_step_ns = 0;
  std::uint64_t fake_single_start_lateness_ns = 0;
};

std::string next_value(int& index, int argc, char** argv) {
  if (++index >= argc) throw std::runtime_error(std::string("missing value after ") + argv[index - 1]);
  return argv[index];
}

bool valid_ipv4(const std::string& value) {
  in_addr address{};
  return inet_pton(AF_INET, value.c_str(), &address) == 1;
}

Stage parse_stage(const std::string& value) {
  if (value == "dry-run") return Stage::DryRun;
  if (value == "preflight") return Stage::Preflight;
  if (value == "entry-exit") return Stage::EntryExit;
  if (value == "run-1s") return Stage::RunOneSecond;
  if (value == "run-5s") return Stage::RunFiveSeconds;
  throw std::runtime_error("invalid stage");
}

Options parse(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--backend") {
      const auto value = next_value(i, argc, argv);
      if (value == "fake") options.backend = BackendKind::Fake;
      else if (value == "vendor") options.backend = BackendKind::Vendor;
      else throw std::runtime_error("backend must be fake or vendor");
    } else if (argument == "--stage") options.stage = parse_stage(next_value(i, argc, argv));
    else if (argument == "--robot-ip") options.robot_ip = next_value(i, argc, argv);
    else if (argument == "--edg-state-ip") options.edg_state_ip = next_value(i, argc, argv);
    else if (argument == "--zero-motion-ack") options.zero_motion_ack = next_value(i, argc, argv);
    else if (argument == "--stage-approval") options.stage_approval = next_value(i, argc, argv);
    else if (argument == "--joint-units") options.joint_units = next_value(i, argc, argv);
    else if (argument == "--result-file") options.result_file = next_value(i, argc, argv);
    else if (argument == "--raw-timing-file") options.raw_timing_file = next_value(i, argc, argv);
    else if (argument == "--prior-stage-result") options.prior_stage_result = next_value(i, argc, argv);
    else if (argument == "--expected-joint-count") options.expected_joint_count = std::stoi(next_value(i, argc, argv));
    else if (argument == "--expected-tool-id") { options.expected_tool_id = std::stoi(next_value(i, argc, argv)); options.expected_tool_id_set = true; }
    else if (argument == "--expected-user-frame-id") { options.expected_user_frame_id = std::stoi(next_value(i, argc, argv)); options.expected_user_frame_id_set = true; }
    else if (argument == "--physical-hardware") options.physical_hardware = true;
    else if (argument == "--estop-access-confirmed") options.estop_access_confirmed = true;
    else if (argument == "--workspace-clear-confirmed") options.workspace_clear_confirmed = true;
    else if (argument == "--fake-nonfinite-target") options.fake.nonfinite_target = true;
    else if (argument == "--fake-servo-active") options.fake.servo_active = true;
    else if (argument == "--fake-entry-failure") options.fake.entry_failure = true;
    else if (argument == "--fake-servo-enable-failure") options.fake.servo_enable_failure = true;
    else if (argument == "--fake-servo-disable-failure") options.fake.servo_disable_failure = true;
    else if (argument == "--fake-exit-failure") options.fake.exit_failure = true;
    else if (argument == "--fake-logout-failure") options.fake.logout_failure = true;
    else if (argument == "--fake-read-failure-cycle") options.fake.read_failure_cycle = std::stoull(next_value(i, argc, argv));
    else if (argument == "--fake-command-failure-cycle") options.fake.command_failure_cycle = std::stoull(next_value(i, argc, argv));
    else if (argument == "--fake-read-delay-us") options.fake.read_delay_ns = std::stoull(next_value(i, argc, argv)) * 1000;
    else if (argument == "--fake-command-delay-us") options.fake.command_delay_ns = std::stoull(next_value(i, argc, argv)) * 1000;
    else if (argument == "--fake-observed-delta-rad") options.fake.observed_joint_delta_rad = std::stod(next_value(i, argc, argv));
    else if (argument == "--fake-start-lateness-step-us") options.fake_start_lateness_step_ns = std::stoull(next_value(i, argc, argv)) * 1000;
    else if (argument == "--fake-single-start-lateness-us") options.fake_single_start_lateness_ns = std::stoull(next_value(i, argc, argv)) * 1000;
    else if (argument == "--help") {
      std::cout << "jaka_zero_motion_probe --stage dry-run|preflight|entry-exit|run-1s|run-5s [options]\n"
                   "Default is fake dry-run: no SDK client, connection, EDG, or command.\n";
      std::exit(0);
    } else throw std::runtime_error("unknown option: " + argument);
  }
  if (options.stage == Stage::DryRun) return options;
#ifdef GATE3B_ENTRY_EXIT_ONLY
  if (options.stage == Stage::RunOneSecond || options.stage == Stage::RunFiveSeconds)
    throw std::runtime_error("this entry/exit-only binary is compiled without cyclic command capability");
#endif
  if (options.expected_joint_count != static_cast<int>(jaka_zero::kJointCount)) throw std::runtime_error("expected joint count must be exactly 6");
  if (options.joint_units != "radians") throw std::runtime_error("--joint-units must explicitly equal radians");
  if (options.backend == BackendKind::Vendor) {
    if (!options.physical_hardware || options.zero_motion_ack != kZeroMotionAck ||
        !options.estop_access_confirmed || !options.workspace_clear_confirmed)
      throw std::runtime_error("physical mode requires hardware, zero-motion, E-stop, and workspace confirmations");
    if (options.stage_approval != required_stage_approval(options.stage))
      throw std::runtime_error("physical mode requires the exact approval for this stage");
    if (!valid_ipv4(options.robot_ip) || !valid_ipv4(options.edg_state_ip))
      throw std::runtime_error("physical mode requires explicit controller and EDG-state IPv4 addresses");
    if (!options.expected_tool_id_set || !options.expected_user_frame_id_set)
      throw std::runtime_error("physical mode requires explicit expected tool and user frame IDs");
    if (options.result_file.empty()) throw std::runtime_error("physical mode requires --result-file");
    if ((options.stage == Stage::RunOneSecond || options.stage == Stage::RunFiveSeconds) && options.raw_timing_file.empty())
      throw std::runtime_error("cyclic physical stages require --raw-timing-file");
    if (options.stage != Stage::Preflight && options.prior_stage_result.empty())
      throw std::runtime_error("physical stages 3-5 require --prior-stage-result");
  }
  return options;
}

class FixedSeries {
 public:
  explicit FixedSeries(std::size_t capacity) : data_(std::make_unique<std::uint64_t[]>(capacity)), capacity_(capacity) {}
  void add(std::uint64_t value) noexcept { if (size_ < capacity_) data_[size_++] = value; }
  std::size_t size() const noexcept { return size_; }
  std::uint64_t operator[](std::size_t index) const noexcept { return data_[index]; }
  std::vector<std::uint64_t> copy() const { return {data_.get(), data_.get() + size_}; }
 private:
  std::unique_ptr<std::uint64_t[]> data_;
  std::size_t capacity_ = 0, size_ = 0;
};

struct Metrics {
  Metrics() : periods(kTimingCapacity), commands(kTimingCapacity), reads(kTimingCapacity), waits(kTimingCapacity),
              wake_lateness(kTimingCapacity), cycle_work(kTimingCapacity) {}
  FixedSeries periods, commands, reads, waits, wake_lateness, cycle_work;
  std::uint64_t cycle_count = 0, completion_misses = 0, maximum_consecutive_completion_misses = 0;
  std::uint64_t period_overruns = 0, maximum_consecutive_period_overruns = 0, largest_overrun_ns = 0;
  std::uint64_t sdk_failures = 0, warning_events = 0, hard_deadline_misses = 0, schedule_realignments = 0;
  std::uint64_t cpu_migrations = 0, minor_page_faults = 0, major_page_faults = 0;
  int initial_cpu = -1, final_cpu = -1;
  double maximum_intentional_command_delta_rad = 0.0;
  double maximum_observed_encoder_delta_rad = 0.0;
};

struct Stats {
  std::size_t count = 0; double mean = 0, median = 0, stddev = 0, min = 0, max = 0, p95 = 0, p99 = 0;
  bool has_p999 = false; double p999 = 0;
};

double percentile(const std::vector<std::uint64_t>& sorted, double q) {
  if (sorted.empty()) return 0;
  const double position = q * static_cast<double>(sorted.size() - 1);
  const auto lo = static_cast<std::size_t>(position), hi = std::min(lo + 1, sorted.size() - 1);
  return sorted[lo] + (position - lo) * static_cast<double>(sorted[hi] - sorted[lo]);
}

Stats stats(const FixedSeries& series) {
  Stats result; auto values = series.copy(); result.count = values.size(); if (values.empty()) return result;
  std::sort(values.begin(), values.end()); const long double sum = std::accumulate(values.begin(), values.end(), static_cast<long double>(0));
  result.mean = static_cast<double>(sum / values.size()); long double squared = 0;
  for (auto value : values) { const long double delta = value - result.mean; squared += delta * delta; }
  result.median = percentile(values, .5); result.stddev = std::sqrt(static_cast<double>(squared / values.size()));
  result.min = values.front(); result.max = values.back(); result.p95 = percentile(values, .95); result.p99 = percentile(values, .99);
  result.has_p999 = values.size() >= 1000; if (result.has_p999) result.p999 = percentile(values, .999); return result;
}

std::string read_file(const std::string& path) {
  std::ifstream input(path); if (!input) throw std::runtime_error("cannot read prior stage result");
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

bool json_string_field_equals(const std::string& contents, const std::string& key, const std::string& value) {
  const auto key_position = contents.find('"' + key + '"');
  if (key_position == std::string::npos) return false;
  auto cursor = contents.find(':', key_position + key.size() + 2);
  if (cursor == std::string::npos) return false;
  cursor = contents.find_first_not_of(" \t\r\n", cursor + 1);
  return cursor != std::string::npos && contents.compare(cursor, value.size() + 2, '"' + value + '"') == 0;
}

bool json_bool_field_equals(const std::string& contents, const std::string& key, bool value) {
  const auto key_position = contents.find('"' + key + '"');
  if (key_position == std::string::npos) return false;
  auto cursor = contents.find(':', key_position + key.size() + 2);
  if (cursor == std::string::npos) return false;
  cursor = contents.find_first_not_of(" \t\r\n", cursor + 1);
  const std::string expected = value ? "true" : "false";
  return cursor != std::string::npos && contents.compare(cursor, expected.size(), expected) == 0;
}

bool validate_prior_stage(const Options& options, jaka_zero::Joints& prior_target) {
  if (options.stage == Stage::Preflight || options.stage == Stage::DryRun ||
      (options.backend == BackendKind::Fake && options.prior_stage_result.empty())) return false;
  const char* expected = options.stage == Stage::EntryExit ? "preflight" :
                         options.stage == Stage::RunOneSecond ? "entry-exit" : "run-1s";
  const auto contents = read_file(options.prior_stage_result);
  if (!json_string_field_equals(contents, "stage", expected) ||
      !json_string_field_equals(contents, "outcome", "completed") ||
      !json_bool_field_equals(contents, "physical_execution", true))
    throw std::runtime_error("prior stage result is not a completed physical prerequisite");
  const auto key = contents.find("\"captured_invariant_joint_rad\"");
  const auto bracket = key == std::string::npos ? std::string::npos : contents.find('[', key);
  if (bracket == std::string::npos) throw std::runtime_error("prior stage result has no captured joint target");
  std::size_t cursor = bracket + 1;
  for (std::size_t i = 0; i < prior_target.size(); ++i) {
    cursor = contents.find_first_not_of(" \t\r\n", cursor);
    if (cursor == std::string::npos) throw std::runtime_error("prior stage joint target is incomplete");
    std::size_t consumed = 0;
    prior_target[i] = std::stod(contents.substr(cursor), &consumed);
    cursor += consumed;
    cursor = contents.find_first_not_of(" \t\r\n", cursor);
    const char expected_separator = i + 1 == prior_target.size() ? ']' : ',';
    if (cursor == std::string::npos || contents[cursor] != expected_separator)
      throw std::runtime_error("prior stage joint target has invalid dimension");
    ++cursor;
  }
  if (!jaka_zero::finite_joints(prior_target)) throw std::runtime_error("prior stage joint target is non-finite");
  return true;
}

std::size_t thread_count() {
  std::size_t count = 0;
  for ([[maybe_unused]] const auto& entry : std::filesystem::directory_iterator("/proc/self/task")) ++count;
  return count;
}

double cpu_seconds(const rusage& usage) {
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 + usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}

bool validate_preflight(const Options& options, const jaka_zero::PreflightState& state, std::string& reason) {
  if (state.fault_code != 0) reason = "active_sdk_fault";
  else if (!state.powered || !state.enabled) reason = "controller_not_powered_and_enabled";
  else if (state.emergency_stop) reason = "emergency_stop_active";
  else if (state.collision) reason = "collision_state_active";
  else if (state.tool_id != options.expected_tool_id || state.user_frame_id != options.expected_user_frame_id) reason = "frame_identity_mismatch";
  else if (!jaka_zero::finite_joints(state.captured_joint_rad)) reason = "invalid_nonfinite_joint_target";
  else if (state.servo_move_active) reason = "unexpected_external_servo_move_owner";
  else return true;
  return false;
}

struct RunResult {
  RunResult() { outcome.reserve(64); lifecycle_trace.reserve(128); }
  std::string outcome = "completed";
  int exit_code = 0;
  jaka_zero::PreflightState preflight{};
  jaka_zero::PreflightState precommand{};
  Metrics metrics{};
  jaka_zero::TimedResult login{}, preflight_call{}, precommand_call{}, edg_entry{}, initial_edg_read{}, final_edg_read{}, servo_enable{}, servo_disable{}, edg_exit{}, logout{};
  jaka_zero::Joints captured_start{}, invariant_target{}, intentional_command_delta{};
  jaka_zero::Joints historical_prior_stage_target{}, inter_run_observation_delta{};
  jaka_zero::Joints standard_state_observation{}, initial_edg_observation{}, final_edg_observation{};
  jaka_zero::Joints cross_api_observation_delta{}, encoder_drift_during_run{};
  double initial_edg_delta_rad = 0.0;
  double prior_stage_joint_delta_rad = 0.0;
  double inter_run_observation_delta_max_rad = 0.0;
  double elapsed_s = 0.0, command_run_duration_s = 0.0, cpu_s = 0.0;
  bool observation_warning = false, final_edg_observation_available = false;
  std::size_t baseline_threads = 0, post_cleanup_threads = 0;
  std::string lifecycle_trace;
};

#ifndef GATE3B_ENTRY_EXIT_ONLY
void warm_absolute_scheduler() {
  auto deadline = Clock::now();
  for (int cycle = 0; cycle < 25; ++cycle) {
    deadline += std::chrono::nanoseconds(kRequestedPeriodNs);
    std::this_thread::sleep_until(deadline);
  }
}

void run_cycles(Stage stage, jaka_zero::Backend& backend, const jaka_zero::Joints& target, Metrics& metrics,
                RunResult& result, std::uint64_t fake_start_lateness_step_ns,
                std::uint64_t fake_single_start_lateness_ns) {
  const std::uint64_t maximum_cycles = stage == Stage::RunOneSecond ? 125 : 625;
  const auto period = std::chrono::nanoseconds(kRequestedPeriodNs);
  auto scheduled = Clock::now(), previous_start = scheduled;
  const auto run_start = scheduled;
  std::uint64_t consecutive_completion_misses = 0, consecutive_period_overruns = 0;
  rusage before{}, after{}; getrusage(RUSAGE_SELF, &before);
  int previous_cpu = -1;
  for (std::uint64_t cycle = 0; cycle < maximum_cycles; ++cycle) {
    if (g_stop.load(std::memory_order_relaxed)) { result.outcome = "operator_interrupted"; result.exit_code = 130; break; }
    if (fake_single_start_lateness_ns && cycle == 1)
      std::this_thread::sleep_until(scheduled + std::chrono::nanoseconds(fake_single_start_lateness_ns));
    else if (fake_start_lateness_step_ns && cycle > 0)
      std::this_thread::sleep_until(scheduled + std::chrono::nanoseconds(fake_start_lateness_step_ns * cycle));
    const auto cycle_start = Clock::now();
    const auto wake_lateness = cycle_start > scheduled ? static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(cycle_start - scheduled).count()) : 0;
    metrics.wake_lateness.add(wake_lateness);
    const int cpu = sched_getcpu();
    if (cycle == 0) metrics.initial_cpu = cpu;
    if (previous_cpu >= 0 && cpu >= 0 && cpu != previous_cpu) ++metrics.cpu_migrations;
    previous_cpu = cpu; metrics.final_cpu = cpu;
    bool start_warning = false;
    if (cycle > 0) {
      const auto actual_period = static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(cycle_start - previous_start).count());
      metrics.periods.add(actual_period);
      if (actual_period > kRequestedPeriodNs)
        metrics.largest_overrun_ns = std::max(metrics.largest_overrun_ns, actual_period - kRequestedPeriodNs);
      if (actual_period > kHardStartPeriodNs || wake_lateness >= kAccumulatedTimingDebtNs) {
        ++metrics.hard_deadline_misses;
        result.outcome = actual_period > kHardStartPeriodNs ? "hard_start_period_miss" : "accumulated_timing_debt";
        result.exit_code = 2; break;
      }
      if (actual_period > kPeriodOverrunThresholdNs) {
        start_warning = true; ++metrics.period_overruns; ++consecutive_period_overruns;
        metrics.maximum_consecutive_period_overruns = std::max(metrics.maximum_consecutive_period_overruns, consecutive_period_overruns);
        if (consecutive_period_overruns >= kMaximumConsecutivePeriodOverruns) {
          ++metrics.hard_deadline_misses;
          result.outcome = "repeated_period_overrun"; result.exit_code = 2; break;
        }
      } else consecutive_period_overruns = 0;
      if (wake_lateness > kWakeLatenessWarningNs) start_warning = true;
      if (start_warning) ++metrics.warning_events;
    }
    previous_start = cycle_start;
    const jaka_zero::Joints issued_command = target;
    metrics.maximum_intentional_command_delta_rad = std::max(
        metrics.maximum_intentional_command_delta_rad,
        jaka_zero::maximum_absolute_delta(target, issued_command));
    if (!jaka_zero::finite_joints(issued_command)) { result.outcome = "nonfinite_command"; result.exit_code = 2; break; }
    const auto command = backend.command_invariant(issued_command); metrics.commands.add(command.duration_ns);
    if (command.code != 0) { ++metrics.sdk_failures; result.outcome = "edg_command_failure"; result.exit_code = 2; break; }
    ++metrics.cycle_count;
    const auto completion = Clock::now();
    metrics.cycle_work.add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(completion - cycle_start).count()));
    const auto next_release = scheduled + period;
    bool completion_warning = false;
    if (completion > next_release) {
      ++metrics.completion_misses; ++consecutive_completion_misses;
      metrics.maximum_consecutive_completion_misses = std::max(metrics.maximum_consecutive_completion_misses, consecutive_completion_misses);
      metrics.largest_overrun_ns = std::max(metrics.largest_overrun_ns, static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(completion - next_release).count()));
      completion_warning = true; ++metrics.warning_events;
      if (completion > scheduled + std::chrono::nanoseconds(kHardCompletionFromReleaseNs) ||
          consecutive_completion_misses >= kMaximumConsecutiveCompletionMisses) {
        ++metrics.hard_deadline_misses;
        result.outcome = completion > scheduled + std::chrono::nanoseconds(kHardCompletionFromReleaseNs)
            ? "hard_completion_deadline_miss" : "repeated_completion_deadline_miss";
        result.exit_code = 2; break;
      }
    } else consecutive_completion_misses = 0;
    if (start_warning || completion_warning) {
      scheduled = (completion_warning ? completion : cycle_start) + period;
      ++metrics.schedule_realignments;
    } else scheduled = next_release;
    if (cycle + 1 < maximum_cycles) {
      const auto wait_start = Clock::now(); std::this_thread::sleep_until(scheduled);
      metrics.waits.add(static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - wait_start).count()));
    }
  }
  getrusage(RUSAGE_SELF, &after);
  metrics.minor_page_faults = static_cast<std::uint64_t>(after.ru_minflt - before.ru_minflt);
  metrics.major_page_faults = static_cast<std::uint64_t>(after.ru_majflt - before.ru_majflt);
  result.command_run_duration_s = std::chrono::duration<double>(Clock::now() - run_start).count();
}
#endif

jaka_zero::Joints joint_delta(const jaka_zero::Joints& observation, const jaka_zero::Joints& reference) {
  jaka_zero::Joints result{};
  for (std::size_t i = 0; i < result.size(); ++i) result[i] = observation[i] - reference[i];
  return result;
}

void print_joints(std::ostream& out, const jaka_zero::Joints& joints) {
  out << std::setprecision(12) << '[';
  for (std::size_t i = 0; i < joints.size(); ++i) { if (i) out << ','; out << joints[i]; }
  out << ']';
}

void write_stats(std::ostream& out, const FixedSeries& series) {
  const auto value = stats(series);
  out << "{\"count\":" << value.count << ",\"mean_ns\":" << value.mean << ",\"median_ns\":" << value.median
      << ",\"stddev_ns\":" << value.stddev << ",\"min_ns\":" << value.min << ",\"max_ns\":" << value.max
      << ",\"p95_ns\":" << value.p95 << ",\"p99_ns\":" << value.p99 << ",\"p999_ns\":";
  if (value.has_p999) out << value.p999; else out << "null";
  out << '}';
}

void write_joints(std::ostream& out, const jaka_zero::Joints& joints) {
  out << '[';
  for (std::size_t i = 0; i < joints.size(); ++i) {
    if (i) out << ',';
    if (std::isfinite(joints[i])) out << joints[i]; else out << "null";
  }
  out << ']';
}

void write_result(const Options& options, const RunResult& result) {
  std::ofstream file; std::ostream* destination = &std::cout;
  if (!options.result_file.empty()) { file.open(options.result_file); if (!file) throw std::runtime_error("cannot open result file"); destination = &file; }
  auto& out = *destination; out << std::setprecision(12);
  out << "{\n  \"schema_version\":\"jaka_zero_motion_gate3b.v1\",\n  \"stage\":\"" << stage_name(options.stage)
      << "\",\n  \"backend\":\"" << (options.backend == BackendKind::Vendor ? "vendor" : "fake")
      << "\",\n  \"physical_execution\":" << (options.backend == BackendKind::Vendor ? "true" : "false")
      << ",\n  \"outcome\":\"" << result.outcome << "\",\n  \"requested_period_ns\":" << kRequestedPeriodNs
      << ",\n  \"wake_lateness_warning_ns\":" << kWakeLatenessWarningNs
      << ",\n  \"accumulated_timing_debt_ns\":" << kAccumulatedTimingDebtNs
      << ",\n  \"period_overrun_threshold_ns\":" << kPeriodOverrunThresholdNs
      << ",\n  \"hard_start_period_ns\":" << kHardStartPeriodNs
      << ",\n  \"hard_completion_from_release_ns\":" << kHardCompletionFromReleaseNs
      << ",\n  \"maximum_consecutive_completion_misses\":" << kMaximumConsecutiveCompletionMisses
      << ",\n  \"maximum_consecutive_period_overruns\":" << kMaximumConsecutivePeriodOverruns
      << ",\n  \"observation_warning_threshold_rad\":" << kObservationWarningRad
      << ",\n  \"observation_abort_threshold_rad\":" << kObservationAbortRad
      << ",\n  \"first_command_difference_threshold_rad\":" << kFirstCommandDifferenceRad
      << ",\n  \"cycle_count\":" << result.metrics.cycle_count
      << ",\n  \"elapsed_s\":" << result.elapsed_s << ",\n  \"sdk_failures\":" << result.metrics.sdk_failures
      << ",\n  \"timing_warning_events\":" << result.metrics.warning_events
      << ",\n  \"hard_deadline_misses\":" << result.metrics.hard_deadline_misses
      << ",\n  \"schedule_realignments\":" << result.metrics.schedule_realignments
      << ",\n  \"cpu_migrations\":" << result.metrics.cpu_migrations
      << ",\n  \"initial_cpu\":" << result.metrics.initial_cpu
      << ",\n  \"final_cpu\":" << result.metrics.final_cpu
      << ",\n  \"minor_page_faults_during_loop\":" << result.metrics.minor_page_faults
      << ",\n  \"major_page_faults_during_loop\":" << result.metrics.major_page_faults
      << ",\n  \"command_run_duration_s\":" << result.command_run_duration_s
      << ",\n  \"completion_misses\":" << result.metrics.completion_misses
      << ",\n  \"max_consecutive_completion_misses\":" << result.metrics.maximum_consecutive_completion_misses
      << ",\n  \"period_overruns\":" << result.metrics.period_overruns
      << ",\n  \"max_consecutive_period_overruns\":" << result.metrics.maximum_consecutive_period_overruns
      << ",\n  \"largest_overrun_ns\":" << result.metrics.largest_overrun_ns
      << ",\n  \"maximum_intentional_command_delta_rad\":" << result.metrics.maximum_intentional_command_delta_rad
      << ",\n  \"maximum_observed_encoder_delta_rad\":" << result.metrics.maximum_observed_encoder_delta_rad
      << ",\n  \"initial_edg_delta_rad\":" << result.initial_edg_delta_rad
      << ",\n  \"observation_warning\":" << (result.observation_warning ? "true" : "false")
      << ",\n  \"prior_stage_joint_delta_rad\":" << result.prior_stage_joint_delta_rad
      << ",\n  \"inter_run_observation_delta_max_rad\":" << result.inter_run_observation_delta_max_rad
      << ",\n  \"captured_invariant_joint_rad\":"; write_joints(out, result.captured_start); out << ",\n"
      << "  \"captured_start_joint_vector\":"; write_joints(out, result.captured_start); out << ",\n"
      << "  \"invariant_command_target\":"; write_joints(out, result.invariant_target); out << ",\n"
      << "  \"intentional_command_delta\":"; write_joints(out, result.intentional_command_delta); out << ",\n"
      << "  \"historical_prior_stage_target\":"; write_joints(out, result.historical_prior_stage_target); out << ",\n"
      << "  \"inter_run_observation_delta\":"; write_joints(out, result.inter_run_observation_delta); out << ",\n"
      << "  \"standard_state_observation\":"; write_joints(out, result.standard_state_observation); out << ",\n"
      << "  \"edg_state_observation\":"; write_joints(out, result.initial_edg_observation); out << ",\n"
      << "  \"cross_api_observation_delta\":"; write_joints(out, result.cross_api_observation_delta); out << ",\n"
      << "  \"final_edg_state_observation\":"; write_joints(out, result.final_edg_observation); out << ",\n"
      << "  \"final_edg_observation_available\":" << (result.final_edg_observation_available ? "true" : "false") << ",\n"
      << "  \"encoder_drift_during_run\":"; write_joints(out, result.encoder_drift_during_run); out << ",\n"
      << "  \"sdk_version\":\"" << result.preflight.sdk_version << "\",\n  \"fault_code\":" << result.preflight.fault_code
      << ",\n  \"powered\":" << (result.preflight.powered ? "true" : "false")
      << ",\n  \"enabled\":" << (result.preflight.enabled ? "true" : "false")
      << ",\n  \"emergency_stop\":" << (result.preflight.emergency_stop ? "true" : "false")
      << ",\n  \"collision\":" << (result.preflight.collision ? "true" : "false")
      << ",\n  \"servo_move_active\":" << (result.preflight.servo_move_active ? "true" : "false")
      << ",\n  \"tool_id\":" << result.preflight.tool_id << ",\n  \"user_frame_id\":" << result.preflight.user_frame_id
      << ",\n  \"login_duration_ns\":" << result.login.duration_ns
      << ",\n  \"preflight_duration_ns\":" << result.preflight_call.duration_ns
      << ",\n  \"precommand_check_duration_ns\":" << result.precommand_call.duration_ns
      << ",\n  \"edg_entry_duration_ns\":" << result.edg_entry.duration_ns
      << ",\n  \"initial_edg_read_duration_ns\":" << result.initial_edg_read.duration_ns
      << ",\n  \"final_edg_read_duration_ns\":" << result.final_edg_read.duration_ns
      << ",\n  \"servo_enable_duration_ns\":" << result.servo_enable.duration_ns
      << ",\n  \"servo_disable_duration_ns\":" << result.servo_disable.duration_ns
      << ",\n  \"edg_exit_duration_ns\":" << result.edg_exit.duration_ns
      << ",\n  \"logout_duration_ns\":" << result.logout.duration_ns
      << ",\n  \"edg_entry_code\":" << result.edg_entry.code
      << ",\n  \"servo_enable_code\":" << result.servo_enable.code
      << ",\n  \"servo_disable_code\":" << result.servo_disable.code
      << ",\n  \"edg_exit_code\":" << result.edg_exit.code
      << ",\n  \"logout_code\":" << result.logout.code
      << ",\n  \"process_cpu_s\":" << result.cpu_s
      << ",\n  \"process_cpu_percent\":" << (result.elapsed_s > 0 ? result.cpu_s / result.elapsed_s * 100.0 : 0.0)
      << ",\n  \"baseline_thread_count\":" << result.baseline_threads
      << ",\n  \"post_cleanup_thread_count\":" << result.post_cleanup_threads
      << ",\n  \"physical_motion_observation\":\"not_provided_by_software\",\n  \"lifecycle_trace\":\"" << result.lifecycle_trace
      << "\",\n  \"timing\":{\n    \"start_to_start_period\":"; write_stats(out, result.metrics.periods);
  out << ",\n    \"edg_read_call\":"; write_stats(out, result.metrics.reads);
  out << ",\n    \"command_call\":"; write_stats(out, result.metrics.commands);
  out << ",\n    \"wait_duration\":"; write_stats(out, result.metrics.waits);
  out << ",\n    \"wake_lateness\":"; write_stats(out, result.metrics.wake_lateness);
  out << ",\n    \"cycle_work\":"; write_stats(out, result.metrics.cycle_work); out << "\n  }\n}\n";
}

void write_raw(const Options& options, const RunResult& result) {
  if (options.raw_timing_file.empty()) return;
  std::ofstream out(options.raw_timing_file); if (!out) throw std::runtime_error("cannot open raw timing file");
  out << "index,start_period_ns,edg_read_ns,command_ns,wait_ns,wake_lateness_ns,cycle_work_ns\n";
  const auto count = std::max({result.metrics.periods.size() + (result.metrics.wake_lateness.size() ? 1u : 0u),
                               result.metrics.reads.size(), result.metrics.commands.size(),
                               result.metrics.wake_lateness.size(), result.metrics.cycle_work.size()});
  for (std::size_t i = 0; i < count; ++i) {
    out << i << ',' << (i == 0 || i - 1 >= result.metrics.periods.size() ? 0 : result.metrics.periods[i - 1]) << ','
        << (i < result.metrics.reads.size() ? result.metrics.reads[i] : 0) << ','
        << (i < result.metrics.commands.size() ? result.metrics.commands[i] : 0) << ','
        << (i < result.metrics.waits.size() ? result.metrics.waits[i] : 0) << ','
        << (i < result.metrics.wake_lateness.size() ? result.metrics.wake_lateness[i] : 0) << ','
        << (i < result.metrics.cycle_work.size() ? result.metrics.cycle_work[i] : 0) << '\n';
  }
}

int execute(const Options& options) {
  if (options.stage == Stage::DryRun) {
    std::cout << "{\"schema_version\":\"jaka_zero_motion_gate3b.v1\",\"stage\":\"dry-run\","
                 "\"connection_opened\":false,\"edg_entered\":false,\"commands_issued\":0}\n";
    return 0;
  }
  jaka_zero::Joints prior_stage_target{};
  const bool has_prior_stage_target = validate_prior_stage(options, prior_stage_target);
  if (options.backend == BackendKind::Vendor) {
    std::cerr << "JAKA GATE 3B ZERO-MOTION STAGE: " << stage_name(options.stage)
              << "\nTarget controller: " << options.robot_ip << "\nEDG state address: " << options.edg_state_ip
              << "\nJoint units: radians; invariant target only; no Cartesian target; no external input."
              << (options.stage == Stage::EntryExit
                    ? "\nEntry/exit binary exposes no servo enable/disable or motion API."
                    : "\nCommand stage owns paired servo-mode enable/disable; invariant joint target only.")
              << " Slow combined status is absent."
              << "\nE-stop access and clear workspace explicitly confirmed.\n";
  }
  g_stop.store(false); std::signal(SIGINT, signal_handler); std::signal(SIGTERM, signal_handler); std::signal(SIGHUP, signal_handler);
  RunResult result; result.baseline_threads = thread_count(); rusage usage_start{}, usage_end{}; getrusage(RUSAGE_SELF, &usage_start);
  if (has_prior_stage_target) result.historical_prior_stage_target = prior_stage_target;
  const auto process_start = Clock::now();
  auto backend = options.backend == BackendKind::Vendor ? jaka_zero::make_vendor_backend() : jaka_zero::make_fake_backend(options.fake);
  result.login = backend->initialize_and_login(options.robot_ip);
  if (result.login.code != 0) { result.outcome = "login_failure"; result.exit_code = 2; }
  if (result.exit_code == 0) {
    result.preflight_call = backend->preflight(result.preflight);
    std::string reason;
    if (result.preflight_call.code != 0) { result.outcome = "preflight_sdk_failure"; result.exit_code = 2; }
    else if (!validate_preflight(options, result.preflight, reason)) { result.outcome = reason; result.exit_code = 2; }
    else if (has_prior_stage_target) {
      result.prior_stage_joint_delta_rad = jaka_zero::maximum_absolute_delta(
          prior_stage_target, result.preflight.captured_joint_rad);
      if (result.prior_stage_joint_delta_rad > kMaximumPriorStageJointDeltaRad) {
        result.outcome = "prior_stage_joint_delta_exceeded"; result.exit_code = 2;
      }
    }
  }
  result.captured_start = result.preflight.captured_joint_rad;
  result.standard_state_observation = result.preflight.captured_joint_rad;
  if (g_stop.load(std::memory_order_relaxed) && result.exit_code == 0) {
    result.outcome = "operator_interrupted"; result.exit_code = 130;
  }
  const bool command_stage = options.stage == Stage::RunOneSecond || options.stage == Stage::RunFiveSeconds;
  if (result.exit_code == 0 && command_stage) {
    if (options.backend == BackendKind::Vendor) {
      std::cerr << "\nOPERATOR CHECKPOINT\nController: " << options.robot_ip
                << "\nStage: " << stage_name(options.stage)
                << "\nDuration: " << (options.stage == Stage::RunOneSecond ? 1 : 5) << " second(s)"
                << "\nPreliminary captured joints: ";
      print_joints(std::cerr, result.preflight.captured_joint_rad);
      std::cerr << "\nMaximum first-command difference: " << kFirstCommandDifferenceRad << " rad"
                << "\nRequested period: 0.008 s"
                << "\nServo-move mode will be enabled by this process and disabled during cleanup."
                << "\nCleanup: stop commands -> servo_move_enable(false) -> edg_init(false) -> logout."
                << "\nPrepare to stop the test now.\n";
      for (int second = kOperatorCountdownSeconds; second > 0; --second) {
        std::cerr << "Starting final validation in " << second << "...\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
#ifndef GATE3B_ENTRY_EXIT_ONLY
    warm_absolute_scheduler();
#endif
    result.precommand_call = backend->precommand_check(result.precommand);
    std::string reason;
    if (result.precommand_call.code != 0) { result.outcome = "precommand_sdk_failure"; result.exit_code = 2; }
    else if (!validate_preflight(options, result.precommand, reason)) { result.outcome = "precommand_" + reason; result.exit_code = 2; }
    else {
      result.captured_start = result.precommand.captured_joint_rad;
      result.standard_state_observation = result.precommand.captured_joint_rad;
      result.invariant_target = result.captured_start;
      result.intentional_command_delta = joint_delta(result.invariant_target, result.captured_start);
      if (has_prior_stage_target) {
        result.inter_run_observation_delta = joint_delta(result.captured_start, prior_stage_target);
        result.inter_run_observation_delta_max_rad = jaka_zero::maximum_absolute_delta(
            result.captured_start, prior_stage_target);
      }
      if (jaka_zero::maximum_absolute_delta(result.invariant_target, result.captured_start) != 0.0) {
        result.outcome = "nonzero_intentional_command_delta"; result.exit_code = 2;
      }
      if (options.backend == BackendKind::Vendor) {
        std::cerr << "Final captured invariant target: "; print_joints(std::cerr, result.invariant_target); std::cerr << '\n';
      }
    }
  }
  if (result.exit_code == 0 && options.stage != Stage::Preflight) {
    const jaka_zero::Joints invariant_target = command_stage ? result.invariant_target : result.captured_start;
    result.invariant_target = invariant_target;
    result.intentional_command_delta = joint_delta(result.invariant_target, result.captured_start);
    result.edg_entry = backend->enter_edg(options.edg_state_ip);
    if (result.edg_entry.code != 0) { result.outcome = "edg_entry_failure"; result.exit_code = 2; }
    else if (g_stop.load(std::memory_order_relaxed)) { result.outcome = "operator_interrupted"; result.exit_code = 130; }
    else {
      jaka_zero::EdgObservation initial{}; result.initial_edg_read = backend->read_edg(initial);
      if (result.initial_edg_read.code != 0 || !jaka_zero::finite_joints(initial.joint_position_rad)) {
        result.outcome = "initial_edg_read_failure"; result.exit_code = 2;
      } else if (g_stop.load(std::memory_order_relaxed)) {
        result.outcome = "operator_interrupted"; result.exit_code = 130;
      } else {
        result.initial_edg_observation = initial.joint_position_rad;
        result.cross_api_observation_delta = joint_delta(result.initial_edg_observation, result.standard_state_observation);
        result.initial_edg_delta_rad = jaka_zero::maximum_absolute_delta(invariant_target, initial.joint_position_rad);
        result.observation_warning = result.initial_edg_delta_rad > kObservationWarningRad;
        if (result.initial_edg_delta_rad > kObservationAbortRad) {
          result.outcome = "cross_api_observation_abort"; result.exit_code = 2;
        } else if (command_stage && result.initial_edg_delta_rad > kFirstCommandDifferenceRad) {
          result.outcome = "first_command_difference_exceeded"; result.exit_code = 2;
        } else if (options.stage == Stage::RunOneSecond || options.stage == Stage::RunFiveSeconds) {
#ifndef GATE3B_ENTRY_EXIT_ONLY
          result.servo_enable = backend->enable_servo_move();
          if (result.servo_enable.code != 0) {
            result.outcome = "servo_move_enable_failure"; result.exit_code = 2;
          } else {
            run_cycles(options.stage, *backend, invariant_target, result.metrics, result,
                       options.backend == BackendKind::Fake ? options.fake_start_lateness_step_ns : 0,
                       options.backend == BackendKind::Fake ? options.fake_single_start_lateness_ns : 0);
            if (result.exit_code == 0) {
              jaka_zero::EdgObservation final{};
              result.final_edg_read = backend->read_edg(final);
              if (result.final_edg_read.code != 0 || !jaka_zero::finite_joints(final.joint_position_rad)) {
                ++result.metrics.sdk_failures;
                result.outcome = "final_edg_read_failure"; result.exit_code = 2;
              } else {
                result.final_edg_observation = final.joint_position_rad;
                result.final_edg_observation_available = true;
                result.encoder_drift_during_run = joint_delta(result.final_edg_observation, result.initial_edg_observation);
                result.metrics.maximum_observed_encoder_delta_rad = jaka_zero::maximum_absolute_delta(
                    result.final_edg_observation, result.initial_edg_observation);
                result.observation_warning = result.observation_warning ||
                    result.metrics.maximum_observed_encoder_delta_rad > kObservationWarningRad;
                if (result.metrics.maximum_observed_encoder_delta_rad > kObservationAbortRad) {
                  result.outcome = "encoder_drift_abort"; result.exit_code = 2;
                }
              }
            }
          }
#else
          result.outcome = "command_capability_not_compiled"; result.exit_code = 2;
#endif
        }
      }
    }
  }
#ifndef GATE3B_ENTRY_EXIT_ONLY
  if (backend->servo_move_active()) result.servo_disable = backend->disable_servo_move();
  if (result.servo_disable.code != 0) { result.outcome = "servo_move_disable_failure"; result.exit_code = 2; }
#endif
  if (backend->edg_active()) result.edg_exit = backend->exit_edg();
  if (result.edg_exit.code != 0) { result.outcome = "edg_exit_failure"; result.exit_code = 2; }
  if (backend->logged_in()) result.logout = backend->logout();
  if (result.logout.code != 0) { result.outcome = "logout_failure"; result.exit_code = 2; }
  else if (g_stop.load(std::memory_order_relaxed) && result.exit_code == 0) {
    result.outcome = "operator_interrupted"; result.exit_code = 130;
  }
  result.lifecycle_trace = backend->lifecycle_trace();
  backend.reset(); result.post_cleanup_threads = thread_count();
  getrusage(RUSAGE_SELF, &usage_end); result.elapsed_s = std::chrono::duration<double>(Clock::now() - process_start).count();
  result.cpu_s = cpu_seconds(usage_end) - cpu_seconds(usage_start);
  write_raw(options, result); write_result(options, result);
  return result.exit_code;
}
}  // namespace

int main(int argc, char** argv) {
  try { return execute(parse(argc, argv)); }
  catch (const std::exception& error) { std::cerr << "configuration error: " << error.what() << '\n'; return 64; }
}
