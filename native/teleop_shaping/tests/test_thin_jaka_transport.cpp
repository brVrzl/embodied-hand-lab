#include "teleop_shaping/thin_jaka_transport_adapter.hpp"

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <new>

namespace {

using namespace teleop_command_abi;
using namespace teleop_shaping;

std::atomic<std::uint64_t> g_allocations{0};
int g_failures = 0;

#define CHECK(condition)                                                        \
  do {                                                                          \
    if (!(condition)) {                                                         \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "            \
                << #condition << '\n';                                          \
      ++g_failures;                                                             \
    }                                                                           \
  } while (false)

struct FakeTableContext {
  bool connected{false};
  bool edg{false};
  bool servo{false};
  bool powered{true};
  bool alarm{false};
  bool estop{false};
  bool collision{false};
  bool session_lost{false};
  bool stale_status{false};
  bool feedback_future{false};
  bool feedback_stale{false};
  bool feedback_clock_regression{false};
  std::int64_t now_ns{1'000'000'000};
  std::int64_t feedback_callback_duration_ns{1'000'000};
  std::int64_t feedback_validation_delay_ns{1'000};
  std::uint64_t feedback_sequence{0};
  std::uint64_t status_sequence{0};
  std::array<double, kMaxDof> q{};
  std::array<double, kMaxDof> dq{};
  JakaFunctionResult next_login{JakaFunctionResult::kOk};
  JakaFunctionResult next_edg{JakaFunctionResult::kOk};
  JakaFunctionResult next_servo{JakaFunctionResult::kOk};
  JakaFunctionResult next_send{JakaFunctionResult::kOk};
  JakaFunctionResult next_feedback{JakaFunctionResult::kOk};
  JakaFunctionResult next_status{JakaFunctionResult::kOk};
  JakaFunctionResult next_stop{JakaFunctionResult::kOk};
  JakaFunctionResult next_logout{JakaFunctionResult::kOk};
  std::uint64_t login_count{0};
  std::uint64_t edg_enable_count{0};
  std::uint64_t edg_disable_count{0};
  std::uint64_t servo_enable_count{0};
  std::uint64_t servo_disable_count{0};
  std::uint64_t send_count{0};
  std::uint64_t feedback_count{0};
  std::uint64_t status_count{0};
  std::uint64_t stop_count{0};
  std::uint64_t logout_count{0};
};

JakaFunctionResult Consume(JakaFunctionResult* value) noexcept {
  const auto result = *value;
  *value = JakaFunctionResult::kOk;
  return result;
}

JakaFunctionResult Login(void* opaque) noexcept {
  auto* context = static_cast<FakeTableContext*>(opaque);
  ++context->login_count;
  const auto result = Consume(&context->next_login);
  if (result == JakaFunctionResult::kOk) context->connected = true;
  return result;
}

JakaFunctionResult SetEdg(void* opaque, bool enabled) noexcept {
  auto* context = static_cast<FakeTableContext*>(opaque);
  if (enabled) {
    ++context->edg_enable_count;
  } else {
    ++context->edg_disable_count;
  }
  const auto result = Consume(&context->next_edg);
  if (result == JakaFunctionResult::kOk) context->edg = enabled;
  return result;
}

JakaFunctionResult SetServo(void* opaque, bool enabled) noexcept {
  auto* context = static_cast<FakeTableContext*>(opaque);
  if (enabled) {
    ++context->servo_enable_count;
  } else {
    ++context->servo_disable_count;
  }
  const auto result = Consume(&context->next_servo);
  if (result == JakaFunctionResult::kOk) context->servo = enabled;
  return result;
}

JakaFunctionResult Send(void* opaque, const double* position, std::uint8_t dof,
                        std::uint32_t step_num) noexcept {
  auto* context = static_cast<FakeTableContext*>(opaque);
  ++context->send_count;
  if (!context->connected || !context->edg || !context->servo ||
      position == nullptr || dof != 6U || step_num != 1U) {
    return JakaFunctionResult::kRejected;
  }
  const auto result = Consume(&context->next_send);
  if (result == JakaFunctionResult::kOk) {
    for (std::size_t i = 0; i < dof; ++i) context->q[i] = position[i];
  }
  return result;
}

JakaFunctionResult ReadFeedback(void* opaque, JakaJointFeedback* feedback) noexcept {
  auto* context = static_cast<FakeTableContext*>(opaque);
  ++context->feedback_count;
  const auto result = Consume(&context->next_feedback);
  if (result != JakaFunctionResult::kOk) return result;
  if (!context->connected || feedback == nullptr) {
    return JakaFunctionResult::kSessionLost;
  }
  *feedback = {};
  feedback->sequence = ++context->feedback_sequence;
  feedback->sdk_call_start_monotonic_ns =
      context->now_ns - (context->feedback_clock_regression ? 1 : 0);
  feedback->sdk_call_end_monotonic_ns =
      feedback->sdk_call_start_monotonic_ns +
      (context->feedback_stale ? 40'000'000
                               : context->feedback_callback_duration_ns);
  feedback->sampled_monotonic_ns = context->feedback_stale
      ? feedback->sdk_call_start_monotonic_ns
      : feedback->sdk_call_end_monotonic_ns;
  feedback->validation_monotonic_ns =
      feedback->sdk_call_end_monotonic_ns +
      context->feedback_validation_delay_ns;
  if (context->feedback_future) {
    feedback->sampled_monotonic_ns = feedback->validation_monotonic_ns + 1;
  }
  feedback->dof = 6;
  feedback->position_rad = context->q;
  feedback->velocity_rad_s = context->dq;
  return JakaFunctionResult::kOk;
}

JakaFunctionResult ReadStatus(void* opaque, JakaNormalizedStatus* status) noexcept {
  auto* context = static_cast<FakeTableContext*>(opaque);
  ++context->status_count;
  const auto result = Consume(&context->next_status);
  if (result != JakaFunctionResult::kOk) return result;
  if (status == nullptr) return JakaFunctionResult::kFailure;
  *status = {};
  status->sequence = ++context->status_sequence;
  status->sdk_call_start_monotonic_ns = context->now_ns;
  status->sdk_call_end_monotonic_ns = context->now_ns + 100'000;
  status->sampled_monotonic_ns =
      context->stale_status ? context->now_ns - 100'000'000
                            : status->sdk_call_end_monotonic_ns;
  status->validation_monotonic_ns = status->sdk_call_end_monotonic_ns + 1'000;
  status->session_alive = context->connected && !context->session_lost;
  status->powered_on = context->powered;
  status->servo_enabled = context->servo;
  status->edg_ready = context->edg;
  status->alarm = context->alarm;
  status->estop = context->estop;
  status->collision = context->collision;
  return JakaFunctionResult::kOk;
}

JakaFunctionResult Stop(void* opaque) noexcept {
  auto* context = static_cast<FakeTableContext*>(opaque);
  ++context->stop_count;
  return Consume(&context->next_stop);
}

JakaFunctionResult Logout(void* opaque) noexcept {
  auto* context = static_cast<FakeTableContext*>(opaque);
  ++context->logout_count;
  const auto result = Consume(&context->next_logout);
  if (result == JakaFunctionResult::kOk) context->connected = false;
  return result;
}

JakaSdkFunctionTable Table(FakeTableContext* context) {
  return {context, Login, SetEdg, SetServo, Send, ReadFeedback,
          ReadStatus, Stop, Logout};
}

ThinJakaConfig Config(
    PauseCommandPolicy pause = PauseCommandPolicy::kNoCommandRequired,
    ResumePreparationPolicy resume = ResumePreparationPolicy::kKeepPrepared) {
  ThinJakaConfig config{};
  config.pause_policy = pause;
  config.resume_policy = resume;
  config.status_poll_interval_ticks = 1;
  return config;
}

ShapedJointCommandV1 Command(std::uint64_t sequence, std::uint64_t epoch,
                             std::int64_t now_ns, OutputMode mode, double q) {
  ShapedJointCommandV1 command{};
  command.header = MakeHeaderV1<ShapedJointCommandV1>();
  command.output_sequence = sequence;
  command.source_sequence = sequence;
  command.safety_epoch = epoch;
  command.generated_monotonic_ns = now_ns;
  command.valid_until_monotonic_ns = now_ns + 100'000'000;
  command.dof = 6;
  command.output_mode = mode;
  command.stop_class = mode == OutputMode::kActiveTracking
                           ? StopClass::kNone
                           : StopClass::kControlled;
  command.stop_reason = mode == OutputMode::kActiveTracking
                            ? StopReason::kNone
                            : StopReason::kClutchRelease;
  command.position_rad[1] = q;
  return command;
}

void Advance(FakeTableContext* context, std::int64_t* now_ns) {
  *now_ns += 8'000'000;
  context->now_ns = *now_ns;
}

void InitializeStreaming(ThinJakaTransportAdapter* adapter,
                         FakeTableContext* context, std::uint64_t epoch,
                         std::int64_t* now_ns) {
  CHECK(adapter->Connect(*now_ns) == ThinJakaCode::kOk);
  CHECK(adapter->PrepareServo(*now_ns) == ThinJakaCode::kOk);
  CHECK(adapter->BeginMeasuredStateRefresh(epoch, *now_ns) == ThinJakaCode::kOk);
  MeasuredJointStateV1 measured{};
  for (int sample = 0; sample < 3; ++sample) {
    Advance(context, now_ns);
    const auto result = adapter->RefreshMeasuredState(*now_ns, &measured);
    CHECK(result == (sample < 2 ? ThinJakaCode::kNeedMoreSamples
                               : ThinJakaCode::kOk));
  }
  auto first = Command(1, epoch, *now_ns, OutputMode::kActiveTracking,
                       context->q[1]);
  first.velocity_rad_s = measured.velocity_rad_s;
  CHECK(adapter->StartStreaming(first, *now_ns) == ThinJakaCode::kOk);
}

void StopToReady(ThinJakaTransportAdapter* adapter, FakeTableContext* context,
                 std::uint64_t epoch, std::int64_t* now_ns,
                 std::uint64_t first_sequence = 2) {
  Advance(context, now_ns);
  CHECK(adapter->OfferLatest(
            Command(first_sequence, epoch, *now_ns,
                    OutputMode::kControlledBraking, context->q[1]),
            *now_ns) == ThinJakaCode::kOk);
  CHECK(adapter->Tick(*now_ns) == ThinJakaCode::kOk);
  Advance(context, now_ns);
  CHECK(adapter->OfferLatest(
            Command(first_sequence + 1, epoch, *now_ns, OutputMode::kStopped,
                    context->q[1]),
            *now_ns) == ThinJakaCode::kOk);
  CHECK(adapter->Tick(*now_ns) == ThinJakaCode::kOk);
  CHECK(adapter->Snapshot().state == ThinJakaState::kStoppedReady);
}

void RefreshAndResume(ThinJakaTransportAdapter* adapter,
                      FakeTableContext* context, std::uint64_t epoch,
                      std::int64_t* now_ns) {
  CHECK(adapter->BeginMeasuredStateRefresh(epoch, *now_ns) == ThinJakaCode::kOk);
  MeasuredJointStateV1 measured{};
  for (int sample = 0; sample < 3; ++sample) {
    Advance(context, now_ns);
    const auto result = adapter->RefreshMeasuredState(*now_ns, &measured);
    CHECK(result == (sample < 2 ? ThinJakaCode::kNeedMoreSamples
                               : ThinJakaCode::kOk));
    CHECK(adapter->Tick(*now_ns) == ThinJakaCode::kOk);
  }
  auto first = Command(1, epoch, *now_ns, OutputMode::kActiveTracking,
                       measured.position_rad[1]);
  first.velocity_rad_s = measured.velocity_rad_s;
  CHECK(adapter->StartStreaming(first, *now_ns) == ThinJakaCode::kOk);
}

void TestPoliciesDefaultClosedAndRepeatStopped() {
  std::int64_t now = 1'000'000'000;
  FakeTableContext unverified_context;
  ThinJakaTransportAdapter unverified(Table(&unverified_context), ThinJakaConfig{});
  CHECK(unverified.Connect(now) == ThinJakaCode::kOk);
  CHECK(unverified.PrepareServo(now) == ThinJakaCode::kInvalidConfiguration);
  CHECK(unverified_context.edg_enable_count == 0U);

  FakeTableContext incompatible_context;
  ThinJakaTransportAdapter incompatible(
      Table(&incompatible_context),
      Config(PauseCommandPolicy::kRepeatStoppedPositionRequired,
             ResumePreparationPolicy::kRestartEdg));
  CHECK(incompatible.Connect(now) == ThinJakaCode::kOk);
  CHECK(incompatible.PrepareServo(now) == ThinJakaCode::kInvalidConfiguration);

  FakeTableContext context;
  context.now_ns = now;
  ThinJakaTransportAdapter adapter(
      Table(&context),
      Config(PauseCommandPolicy::kRepeatStoppedPositionRequired));
  InitializeStreaming(&adapter, &context, 1, &now);
  StopToReady(&adapter, &context, 1, &now);
  const auto before = context.send_count;
  for (int tick = 0; tick < 4; ++tick) {
    Advance(&context, &now);
    CHECK(adapter.Tick(now) == ThinJakaCode::kOk);
  }
  CHECK(context.send_count == before + 4U);
  CHECK(adapter.Snapshot().repeated_stopped_command_count == 4U);
  CHECK(adapter.Snapshot().session_owned);
  CHECK(adapter.BeginMeasuredStateRefresh(2, now) == ThinJakaCode::kOk);
  MeasuredJointStateV1 measured{};
  Advance(&context, &now);
  CHECK(adapter.RefreshMeasuredState(now, &measured) ==
        ThinJakaCode::kNeedMoreSamples);
  const auto before_refresh_tick = context.send_count;
  CHECK(adapter.Tick(now) == ThinJakaCode::kOk);
  CHECK(context.send_count == before_refresh_tick + 1U);
  CHECK(adapter.Snapshot().repeated_stopped_command_count == 5U);
}

void TestFeedbackTimestampSemantics() {
  auto initialized_adapter = [](FakeTableContext* context,
                                ThinJakaConfig config,
                                std::int64_t now) {
    auto adapter = std::make_unique<ThinJakaTransportAdapter>(Table(context), config);
    CHECK(adapter->Connect(now) == ThinJakaCode::kOk);
    CHECK(adapter->PrepareServo(now) == ThinJakaCode::kOk);
    CHECK(adapter->BeginMeasuredStateRefresh(1, now) == ThinJakaCode::kOk);
    return adapter;
  };

  {
    std::int64_t now = 1'500'000'000;
    FakeTableContext context;
    context.now_ns = now;
    context.feedback_callback_duration_ns = 2'000'000;
    context.feedback_validation_delay_ns = 3'000;
    auto config = Config();
    config.measurement.stable_sample_count = 1;
    auto adapter = initialized_adapter(&context, config, now);
    MeasuredJointStateV1 measured{};
    CHECK(adapter->RefreshMeasuredState(now, &measured) == ThinJakaCode::kOk);
    const auto snapshot = adapter->Snapshot();
    CHECK(snapshot.last_feedback_call_duration_ns == 2'000'000);
    CHECK(snapshot.last_feedback_sample_age_ns == 3'000);
    CHECK(snapshot.last_feedback_sample_ns <=
          snapshot.last_feedback_validation_ns);
    CHECK(measured.position_rad == context.q);
    CHECK(measured.velocity_rad_s == context.dq);
  }
  {
    std::int64_t now = 1'600'000'000;
    FakeTableContext context;
    context.now_ns = now;
    context.feedback_future = true;
    auto adapter = initialized_adapter(&context, Config(), now);
    MeasuredJointStateV1 measured{};
    CHECK(adapter->RefreshMeasuredState(now, &measured) ==
          ThinJakaCode::kTimingFault);
    CHECK(adapter->Snapshot().state == ThinJakaState::kFaulted);
  }
  {
    std::int64_t now = 1'700'000'000;
    FakeTableContext context;
    context.now_ns = now;
    context.feedback_stale = true;
    auto adapter = initialized_adapter(&context, Config(), now);
    MeasuredJointStateV1 measured{};
    CHECK(adapter->RefreshMeasuredState(now, &measured) == ThinJakaCode::kStale);
    CHECK(adapter->Snapshot().state == ThinJakaState::kFaulted);
  }
  {
    std::int64_t now = 1'800'000'000;
    FakeTableContext context;
    context.now_ns = now;
    context.feedback_clock_regression = true;
    auto adapter = initialized_adapter(&context, Config(), now);
    MeasuredJointStateV1 measured{};
    CHECK(adapter->RefreshMeasuredState(now, &measured) ==
          ThinJakaCode::kTimingFault);
    CHECK(adapter->Snapshot().state == ThinJakaState::kFaulted);
  }
}

void TestThousandCycle125HzLatestOnly() {
  std::int64_t now = 2'000'000'000;
  FakeTableContext context;
  context.now_ns = now;
  ThinJakaTransportAdapter adapter(Table(&context), Config());
  InitializeStreaming(&adapter, &context, 1, &now);
  const auto allocations_before = g_allocations.load();
  std::uint64_t epoch = 1;
  for (int cycle = 0; cycle < 1000; ++cycle) {
    Advance(&context, &now);
    const double q = static_cast<double>(cycle % 5) * 1e-5;
    CHECK(adapter.OfferLatest(
              Command(2, epoch, now, OutputMode::kActiveTracking, q), now) ==
          ThinJakaCode::kOk);
    CHECK(adapter.OfferLatest(
              Command(3, epoch, now, OutputMode::kActiveTracking, q), now) ==
          ThinJakaCode::kOk);
    CHECK(adapter.Tick(now) == ThinJakaCode::kOk);
    StopToReady(&adapter, &context, epoch, &now, 4);
    Advance(&context, &now);
    CHECK(adapter.Tick(now) == ThinJakaCode::kOk);
    ++epoch;
    RefreshAndResume(&adapter, &context, epoch, &now);
  }
  const auto allocations_after = g_allocations.load();
  const auto snapshot = adapter.Snapshot();
  CHECK(allocations_after == allocations_before);
  CHECK(snapshot.state == ThinJakaState::kStreaming);
  CHECK(snapshot.clutch_cycle_count == 1000U);
  CHECK(snapshot.superseded_command_count == 1000U);
  CHECK(snapshot.skipped_output_sequence_count == 1000U);
  CHECK(snapshot.tick_deadline_miss_count == 0U);
  CHECK(snapshot.maximum_tick_interval_ns == 8'000'000);
  CHECK(snapshot.maximum_resume_position_delta_rad == 0.0);
  CHECK(snapshot.status_poll_count == snapshot.tick_count);
  CHECK(context.login_count == 1U);
  CHECK(context.logout_count == 0U);
  CHECK(context.stop_count == 0U);
  CHECK(adapter.Snapshot().session_owned);
  std::cout << "thin_adapter_clutch_cycles=" << snapshot.clutch_cycle_count
            << " ticks=" << snapshot.tick_count
            << " status_polls=" << snapshot.status_poll_count
            << " allocations=" << (allocations_after - allocations_before)
            << '\n';
}

void TestRestartPoliciesAndFirstFrameContinuity() {
  for (auto policy : {ResumePreparationPolicy::kRestartEdg,
                      ResumePreparationPolicy::kRestartEdgAndServo}) {
    std::int64_t now = 3'000'000'000;
    FakeTableContext context;
    context.now_ns = now;
    ThinJakaTransportAdapter adapter(Table(&context), Config(
        PauseCommandPolicy::kNoCommandRequired, policy));
    InitializeStreaming(&adapter, &context, 1, &now);
    StopToReady(&adapter, &context, 1, &now);
    CHECK(!context.edg);
    if (policy == ResumePreparationPolicy::kRestartEdgAndServo) {
      CHECK(!context.servo);
    } else {
      CHECK(context.servo);
    }
    const auto edg_enables = context.edg_enable_count;
    const auto servo_enables = context.servo_enable_count;
    RefreshAndResume(&adapter, &context, 2, &now);
    CHECK(context.edg_enable_count == edg_enables + 1U);
    CHECK(context.servo_enable_count ==
          servo_enables +
              (policy == ResumePreparationPolicy::kRestartEdgAndServo ? 1U : 0U));
    CHECK(adapter.Snapshot().maximum_resume_position_delta_rad == 0.0);
  }

  std::int64_t now = 3'500'000'000;
  FakeTableContext context;
  context.now_ns = now;
  ThinJakaTransportAdapter adapter(Table(&context), Config());
  InitializeStreaming(&adapter, &context, 1, &now);
  StopToReady(&adapter, &context, 1, &now);
  CHECK(adapter.BeginMeasuredStateRefresh(2, now) == ThinJakaCode::kOk);
  MeasuredJointStateV1 measured{};
  for (int sample = 0; sample < 3; ++sample) {
    Advance(&context, &now);
    (void)adapter.RefreshMeasuredState(now, &measured);
  }
  auto discontinuous = Command(1, 2, now, OutputMode::kActiveTracking,
                               measured.position_rad[1] + 0.01);
  CHECK(adapter.StartStreaming(discontinuous, now) ==
        ThinJakaCode::kContinuityFault);
  CHECK(adapter.Snapshot().state == ThinJakaState::kFaulted);
  CHECK(context.stop_count == 1U);
}

void TestFaultMatrixAndExplicitReset() {
  int cases = 0;
  auto status_fault = [&](int kind) {
    std::int64_t now = 4'000'000'000 + static_cast<std::int64_t>(kind) * 1'000'000;
    FakeTableContext context;
    context.now_ns = now;
    ThinJakaTransportAdapter adapter(Table(&context), Config());
    InitializeStreaming(&adapter, &context, 1, &now);
    if (kind == 0) context.stale_status = true;
    if (kind == 1) context.alarm = true;
    if (kind == 2) context.estop = true;
    if (kind == 3) context.collision = true;
    if (kind == 4) context.servo = false;
    if (kind == 5) context.powered = false;
    if (kind == 6) context.session_lost = true;
    Advance(&context, &now);
    CHECK(adapter.Tick(now) != ThinJakaCode::kOk);
    CHECK(adapter.Snapshot().state == ThinJakaState::kFaulted);
    CHECK(context.stop_count == 1U);
    CHECK(adapter.BeginMeasuredStateRefresh(2, now) == ThinJakaCode::kInvalidState);
    ++cases;
  };
  for (int kind = 0; kind < 7; ++kind) status_fault(kind);

  {
    std::int64_t now = 4'100'000'000;
    FakeTableContext context;
    context.now_ns = now;
    ThinJakaTransportAdapter adapter(Table(&context), Config());
    InitializeStreaming(&adapter, &context, 1, &now);
    context.next_send = JakaFunctionResult::kTimeout;
    Advance(&context, &now);
    CHECK(adapter.OfferLatest(
              Command(2, 1, now, OutputMode::kActiveTracking, 0.0), now) ==
          ThinJakaCode::kOk);
    CHECK(adapter.Tick(now) == ThinJakaCode::kTimingFault);
    CHECK(adapter.Snapshot().fault_reason == StopReason::kTimingFault);
    ++cases;
  }
  {
    std::int64_t now = 4'200'000'000;
    FakeTableContext context;
    context.now_ns = now;
    ThinJakaTransportAdapter adapter(Table(&context), Config());
    InitializeStreaming(&adapter, &context, 1, &now);
    StopToReady(&adapter, &context, 1, &now);
    context.alarm = true;
    Advance(&context, &now);
    CHECK(adapter.Tick(now) == ThinJakaCode::kControllerFault);
    CHECK(adapter.BeginMeasuredStateRefresh(2, now) == ThinJakaCode::kInvalidState);
    ++cases;
  }
  {
    std::int64_t now = 4'300'000'000;
    FakeTableContext context;
    context.now_ns = now;
    ThinJakaTransportAdapter adapter(Table(&context), Config());
    InitializeStreaming(&adapter, &context, 1, &now);
    adapter.HardStop(StopReason::kSdkFailure, now);
    context.next_logout = JakaFunctionResult::kFailure;
    CHECK(adapter.Cleanup(now) == ThinJakaCode::kCleanupFailure);
    CHECK(adapter.Snapshot().cleanup_failed);
    CHECK(adapter.ExplicitReset(now) == ThinJakaCode::kInvalidState);
    ++cases;
  }
  {
    std::int64_t now = 4'400'000'000;
    FakeTableContext context;
    context.now_ns = now;
    ThinJakaTransportAdapter adapter(Table(&context), Config());
    InitializeStreaming(&adapter, &context, 1, &now);
    adapter.HardStop(StopReason::kSdkFailure, now);
    CHECK(adapter.Cleanup(now) == ThinJakaCode::kOk);
    CHECK(adapter.Snapshot().state == ThinJakaState::kResetRequired);
    CHECK(adapter.ExplicitReset(now) == ThinJakaCode::kOk);
    CHECK(adapter.Snapshot().state == ThinJakaState::kDisconnected);
    ++cases;
  }
  {
    std::int64_t now = 4'500'000'000;
    FakeTableContext context;
    context.now_ns = now;
    ThinJakaTransportAdapter adapter(Table(&context), Config());
    InitializeStreaming(&adapter, &context, 1, &now);
    StopToReady(&adapter, &context, 1, &now);
    RefreshAndResume(&adapter, &context, 2, &now);
    Advance(&context, &now);
    auto old = Command(2, 1, now, OutputMode::kActiveTracking, context.q[1]);
    CHECK(adapter.OfferLatest(old, now) == ThinJakaCode::kEpochMismatch);
    CHECK(adapter.Snapshot().old_epoch_rejection_count == 1U);
    ++cases;
  }
  {
    std::int64_t now = 4'600'000'000;
    FakeTableContext context;
    context.now_ns = now;
    ThinJakaTransportAdapter adapter(Table(&context), Config());
    InitializeStreaming(&adapter, &context, 1, &now);
    StopToReady(&adapter, &context, 1, &now);
    CHECK(adapter.BeginMeasuredStateRefresh(2, now) == ThinJakaCode::kOk);
    context.next_feedback = JakaFunctionResult::kTimeout;
    Advance(&context, &now);
    MeasuredJointStateV1 measured{};
    CHECK(adapter.RefreshMeasuredState(now, &measured) == ThinJakaCode::kStale);
    CHECK(adapter.Snapshot().state == ThinJakaState::kFaulted);
    CHECK(adapter.BeginMeasuredStateRefresh(2, now) == ThinJakaCode::kInvalidState);
    ++cases;
  }
  for (auto policy : {ResumePreparationPolicy::kRestartEdg,
                      ResumePreparationPolicy::kRestartEdgAndServo}) {
    std::int64_t now = 4'700'000'000;
    FakeTableContext context;
    context.now_ns = now;
    ThinJakaTransportAdapter adapter(
        Table(&context),
        Config(PauseCommandPolicy::kNoCommandRequired, policy));
    InitializeStreaming(&adapter, &context, 1, &now);
    StopToReady(&adapter, &context, 1, &now);
    if (policy == ResumePreparationPolicy::kRestartEdg) {
      context.next_edg = JakaFunctionResult::kFailure;
    } else {
      context.next_servo = JakaFunctionResult::kFailure;
    }
    CHECK(adapter.BeginMeasuredStateRefresh(2, now) == ThinJakaCode::kSdkFailure);
    CHECK(adapter.Snapshot().state == ThinJakaState::kFaulted);
    ++cases;
  }
  CHECK(cases == 15);
  std::cout << "thin_adapter_fault_cases=" << cases << '\n';
}

}  // namespace

void* operator new(std::size_t size) {
  g_allocations.fetch_add(1, std::memory_order_relaxed);
  if (void* result = std::malloc(size)) return result;
  throw std::bad_alloc();
}
void* operator new[](std::size_t size) {
  g_allocations.fetch_add(1, std::memory_order_relaxed);
  if (void* result = std::malloc(size)) return result;
  throw std::bad_alloc();
}
void operator delete(void* pointer) noexcept { std::free(pointer); }
void operator delete[](void* pointer) noexcept { std::free(pointer); }
void operator delete(void* pointer, std::size_t) noexcept { std::free(pointer); }
void operator delete[](void* pointer, std::size_t) noexcept { std::free(pointer); }

int main() {
  TestPoliciesDefaultClosedAndRepeatStopped();
  TestFeedbackTimestampSemantics();
  TestThousandCycle125HzLatestOnly();
  TestRestartPoliciesAndFirstFrameContinuity();
  TestFaultMatrixAndExplicitReset();
  if (g_failures != 0) {
    std::cerr << g_failures << " test checks failed\n";
    return 1;
  }
  std::cout << "thin_jaka_transport_tests: passed\n";
  return 0;
}
