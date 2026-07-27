#include "teleop_command_abi/abi_v1.hpp"
#include "teleop_shaping/clutch_recovery_transport.hpp"
#include "teleop_shaping/fake_consumer.hpp"
#include "teleop_shaping/fake_jaka_lifecycle.hpp"
#include "teleop_shaping/joint_shaper.hpp"

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <new>
#include <random>

namespace {

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

using namespace teleop_command_abi;
using namespace teleop_shaping;

MeasuredJointStateV1 Measured(std::int64_t now_ns, double velocity = 0.0,
                              double acceleration = 0.0) {
  MeasuredJointStateV1 value{};
  value.header = MakeHeaderV1<MeasuredJointStateV1>();
  value.state_sequence = 1;
  value.safety_epoch = 7;
  value.measured_monotonic_ns = now_ns;
  value.dof = 6;
  value.validity = MeasurementValidity::kValid;
  value.velocity_rad_s[1] = velocity;
  value.acceleration_rad_s2[1] = acceleration;
  return value;
}

JointDynamicLimitsV1 Limits() {
  JointDynamicLimitsV1 value{};
  value.header = MakeHeaderV1<JointDynamicLimitsV1>();
  value.dof = 6;
  for (std::size_t i = 0; i < value.dof; ++i) {
    value.minimum_position_rad[i] = -3.0;
    value.maximum_position_rad[i] = 3.0;
    value.maximum_velocity_rad_s[i] = 3.141592653589793;
    value.maximum_acceleration_rad_s2[i] = 4.0 * 3.141592653589793;
    value.maximum_jerk_rad_s3[i] = 50.0;
  }
  return value;
}

AcceptedJointTargetV1 Target(std::uint64_t sequence, std::int64_t source_ns,
                             std::int64_t now_ns, double joint_two = 0.0) {
  AcceptedJointTargetV1 value{};
  value.header = MakeHeaderV1<AcceptedJointTargetV1>();
  value.sequence = sequence;
  value.safety_epoch = 7;
  value.source_monotonic_ns = source_ns;
  value.accepted_monotonic_ns = now_ns;
  value.valid_until_monotonic_ns = now_ns + 1'000'000'000;
  value.dof = 6;
  value.engagement = EngagementState::kEngaged;
  value.validity = TargetValidity::kAccepted;
  value.position_rad[1] = joint_two;
  return value;
}

void TestAbiLayoutAndValidation() {
  CHECK(sizeof(AbiHeader) == 16);
  CHECK(alignof(AbiHeader) == 4);
  CHECK(sizeof(AcceptedJointTargetV1) == 128);
  CHECK(sizeof(MeasuredJointStateV1) == 240);
  CHECK(sizeof(JointDynamicLimitsV1) == 344);
  CHECK(sizeof(ShapedJointCommandV1) == 256);
  CHECK(sizeof(TransportHealthV1) == 64);
  CHECK(alignof(AcceptedJointTargetV1) == 8);
  CHECK(offsetof(AcceptedJointTargetV1, position_rad) == 64);

  const std::int64_t now = 1'000'000'000;
  auto target = Target(1, now, now, 0.1);
  CHECK(Validate(target, ValidationContext{now, 6, 7, 0}).ok);
  target.header.schema_version = 2;
  CHECK(Validate(target).error == ValidationError::kUnsupportedVersion);
  target = Target(1, now, now, 0.1);
  target.dof = 9;
  CHECK(Validate(target).error == ValidationError::kInvalidDof);
  target = Target(1, now, now, 0.1);
  target.engagement = static_cast<EngagementState>(255);
  CHECK(Validate(target).error == ValidationError::kInvalidEnum);
  target = Target(1, now, now, 0.1);
  target.position_rad[2] = std::numeric_limits<double>::quiet_NaN();
  CHECK(Validate(target).error == ValidationError::kNonFinite);
  target = Target(1, now, now, 0.1);
  target.source_monotonic_ns = now + 1;
  CHECK(Validate(target).error == ValidationError::kTimestampOrdering);
  target = Target(1, now, now, 0.1);
  target.validity = TargetValidity::kRejectedKeepPrevious;
  CHECK(Validate(target).error == ValidationError::kTargetSmuggling);
  target.position_rad.fill(0.0);
  CHECK(Validate(target).ok);
  target.valid_until_monotonic_ns = now;
  CHECK(Validate(target, ValidationContext{now + 1, 6, 7, 0}).error ==
        ValidationError::kStaleValidityWindow);

  auto limits = Limits();
  CHECK(Validate(limits).ok);
  limits.maximum_jerk_rad_s3[3] = 0.0;
  CHECK(Validate(limits).error == ValidationError::kInvalidLimits);
  limits = Limits();
  limits.maximum_velocity_rad_s[7] = 1.0;
  CHECK(Validate(limits).error == ValidationError::kTargetSmuggling);

  auto measured = Measured(now);
  CHECK(Validate(measured, ValidationContext{now, 6, 7, 0}).ok);
  measured.acceleration_rad_s2[0] = std::numeric_limits<double>::infinity();
  CHECK(Validate(measured).error == ValidationError::kNonFinite);

  ShapedJointCommandV1 command{};
  command.header = MakeHeaderV1<ShapedJointCommandV1>();
  command.output_sequence = 1;
  command.source_sequence = 1;
  command.safety_epoch = 7;
  command.generated_monotonic_ns = now;
  command.valid_until_monotonic_ns = now + 1;
  command.dof = 6;
  command.output_mode = OutputMode::kActiveTracking;
  command.stop_class = StopClass::kNone;
  command.stop_reason = StopReason::kNone;
  CHECK(Validate(command, ValidationContext{now, 6, 7, 0}).ok);
  command.output_mode = static_cast<OutputMode>(255);
  CHECK(Validate(command).error == ValidationError::kInvalidEnum);

  TransportHealthV1 health{};
  health.header = MakeHeaderV1<TransportHealthV1>();
  health.health_sequence = 1;
  health.safety_epoch = 7;
  health.sampled_monotonic_ns = now;
  health.transport_state = TransportState::kReady;
  health.controller_state = ControllerState::kReady;
  health.servo_enabled = 1;
  CHECK(Validate(health, ValidationContext{now, 0, 7, 0}).ok);
  health.alarm = 2;
  CHECK(Validate(health).error == ValidationError::kInvalidBoolean);
}

void TestActiveReferenceAndHeartbeat() {
  const std::int64_t start = 1'000'000'000;
  ReferenceJointShaperV1 shaper;
  CHECK(shaper.Initialize(Measured(start), Limits(), start).code == OperationCode::kOk);
  auto target = Target(1, start, start, 0.1);
  CHECK(shaper.ReplaceTarget(target, start).code == OperationCode::kOk);
  ShapedJointCommandV1 output{};
  CHECK(shaper.Tick(start, &output).code == OperationCode::kOk);
  CHECK(output.output_mode == OutputMode::kActiveTracking);
  CHECK(std::abs(output.acceleration_rad_s2[1] - 0.4) < 1e-12);
  CHECK(std::abs(output.velocity_rad_s[1] - 0.0032) < 1e-12);
  CHECK(std::abs(output.position_rad[1] - 0.0000256) < 1e-12);

  auto rejected = Target(2, start + 1, start + 1, 1.5);
  rejected.validity = TargetValidity::kRejectedKeepPrevious;
  rejected.position_rad.fill(0.0);
  CHECK(shaper.ReplaceTarget(rejected, start + 1).code == OperationCode::kOk);
  CHECK(shaper.Snapshot().source_sequence == 1);
  CHECK(shaper.Snapshot().last_input_sequence == 2);
  CHECK(shaper.Snapshot().liveness_monotonic_ns == start + 1);

  auto wrong_epoch = Target(3, start + 2, start + 2, 0.2);
  wrong_epoch.safety_epoch = 8;
  CHECK(shaper.ReplaceTarget(wrong_epoch, start + 2).code ==
        OperationCode::kInvalidArgument);
  CHECK(shaper.Snapshot().mode == ShaperMode::kHardStopped);
  CHECK(shaper.Tick(start + kReferencePeriodNs, &output).code ==
        OperationCode::kTerminalNoOutput);
  CHECK(output.output_sequence == 0);
}

void TestControlledBrakingAndStateMachine() {
  const std::int64_t start = 2'000'000'000;
  ReferenceJointShaperV1 shaper;
  CHECK(shaper.Initialize(Measured(start, 0.25, 0.0), Limits(), start).code ==
        OperationCode::kOk);
  CHECK(shaper.ReplaceTarget(Target(1, start, start), start).code == OperationCode::kOk);
  CHECK(shaper.RequestControlledStop(2, StopReason::kClutchRelease, start).code ==
        OperationCode::kOk);
  CHECK(shaper.RequestControlledStop(2, StopReason::kClutchRelease, start).code ==
        OperationCode::kAlreadyRequested);
  double previous_position = 0.0;
  double maximum_velocity = 0.0;
  double maximum_acceleration = 0.0;
  ShapedJointCommandV1 output{};
  bool completed = false;
  const std::uint64_t allocations_before = g_allocations.load();
  for (int tick = 0; tick < 100; ++tick) {
    const auto result = shaper.Tick(start + tick * kReferencePeriodNs, &output);
    CHECK(result.code == OperationCode::kOk || result.code == OperationCode::kCompleted);
    CHECK(output.position_rad[1] + 1e-12 >= previous_position);
    CHECK(output.velocity_rad_s[1] >= -1e-10);
    maximum_velocity = std::max(maximum_velocity, std::abs(output.velocity_rad_s[1]));
    maximum_acceleration =
        std::max(maximum_acceleration, std::abs(output.acceleration_rad_s2[1]));
    previous_position = output.position_rad[1];
    if (output.output_mode == OutputMode::kStopped) {
      completed = true;
      CHECK(output.velocity_rad_s[1] == 0.0);
      CHECK(output.acceleration_rad_s2[1] == 0.0);
      break;
    }
  }
  CHECK(completed);
  CHECK(g_allocations.load() == allocations_before);
  CHECK(maximum_velocity <= 0.25 + 1e-10);
  CHECK(maximum_acceleration <= Limits().maximum_acceleration_rad_s2[1] + 1e-10);
  CHECK(shaper.ReplaceTarget(Target(3, start + 1, start + 1), start + 1).code ==
        OperationCode::kInvalidState);
  shaper.HardStop(StopReason::kControllerAlarm, start + 1);
  shaper.HardStop(StopReason::kSdkFailure, start + 2);
  CHECK(shaper.Snapshot().stop_reason == StopReason::kControllerAlarm);
}

ShapedJointCommandV1 Command(std::uint64_t output_sequence, std::int64_t now_ns) {
  ShapedJointCommandV1 value{};
  value.header = MakeHeaderV1<ShapedJointCommandV1>();
  value.output_sequence = output_sequence;
  value.source_sequence = output_sequence;
  value.safety_epoch = 7;
  value.generated_monotonic_ns = now_ns;
  value.valid_until_monotonic_ns = now_ns + 16'000'000;
  value.dof = 6;
  value.output_mode = OutputMode::kActiveTracking;
  value.stop_class = StopClass::kNone;
  value.stop_reason = StopReason::kNone;
  return value;
}

ShapedJointCommandV1 CommandForMode(std::uint64_t output_sequence,
                                    std::uint64_t epoch,
                                    std::int64_t now_ns,
                                    OutputMode mode) {
  auto value = Command(output_sequence, now_ns);
  value.safety_epoch = epoch;
  value.output_mode = mode;
  if (mode == OutputMode::kControlledBraking || mode == OutputMode::kStopped) {
    value.stop_class = StopClass::kControlled;
    value.stop_reason = StopReason::kClutchRelease;
  }
  return value;
}

TransportHealthV1 Health(std::uint64_t sequence, std::uint64_t epoch,
                         std::int64_t now_ns) {
  TransportHealthV1 value{};
  value.header = MakeHeaderV1<TransportHealthV1>();
  value.health_sequence = sequence;
  value.safety_epoch = epoch;
  value.sampled_monotonic_ns = now_ns;
  value.transport_state = TransportState::kReady;
  value.controller_state = ControllerState::kReady;
  value.servo_enabled = 1;
  value.vendor_status_category = VendorStatusCategory::kNone;
  return value;
}

FakeSdkJointSample JointSample(std::uint64_t sequence, std::int64_t now_ns,
                               JointSampleFields fields, double position,
                               double velocity = 0.0,
                               double acceleration = 0.0) {
  FakeSdkJointSample value{};
  value.sample_sequence = sequence;
  value.sampled_monotonic_ns = now_ns;
  value.dof = 6;
  value.fields = fields;
  value.position_rad[1] = position;
  if (fields != JointSampleFields::kPositionOnly) {
    value.velocity_rad_s[1] = velocity;
  }
  if (fields == JointSampleFields::kPositionVelocityAcceleration) {
    value.acceleration_rad_s2[1] = acceleration;
  }
  return value;
}

FakeJakaLifecycleAdapter StreamingAdapter(std::int64_t now, std::uint64_t epoch = 7) {
  FakeJakaLifecycleAdapter adapter;
  CHECK(adapter.BeginConnect(now) == FakeLifecycleCode::kOk);
  CHECK(adapter.CompleteConnect(true, now) == FakeLifecycleCode::kOk);
  CHECK(adapter.PrepareServo(now) == FakeLifecycleCode::kOk);
  auto measured = Measured(now);
  measured.safety_epoch = epoch;
  CHECK(adapter.ArmEpoch(measured, now) == FakeLifecycleCode::kOk);
  CHECK(adapter.StartStreaming(epoch, now) == FakeLifecycleCode::kOk);
  return adapter;
}

void TestFakeConsumerLatestWinsAndFaults() {
  const std::int64_t now = 3'000'000'000;
  InMemoryFakeConsumerV1 consumer(6, 7);
  CHECK(consumer.Offer(Command(1, now), now) == ConsumerCode::kOk);
  CHECK(consumer.Offer(Command(2, now), now) == ConsumerCode::kOk);
  CHECK(consumer.superseded_count() == 1);
  CHECK(consumer.ConsumeLatest(now) == ConsumerCode::kOk);
  CHECK(consumer.last_consumed_sequence() == 2);
  CHECK(consumer.Offer(Command(2, now), now) == ConsumerCode::kDuplicateOrOld);
  CHECK(!consumer.hard_stopped());
  auto stale = Command(3, now);
  CHECK(consumer.Offer(stale, now + 16'000'001) == ConsumerCode::kStaleCommand);
  CHECK(consumer.hard_stopped());

  InMemoryFakeConsumerV1 epoch_consumer(6, 7);
  auto mismatch = Command(1, now);
  mismatch.safety_epoch = 8;
  CHECK(epoch_consumer.Offer(mismatch, now) == ConsumerCode::kEpochMismatch);
  CHECK(epoch_consumer.hard_stopped());

  InMemoryFakeConsumerV1 skipped_consumer(6, 7);
  CHECK(skipped_consumer.Offer(Command(1, now), now) == ConsumerCode::kOk);
  CHECK(skipped_consumer.Offer(Command(3, now), now) == ConsumerCode::kOk);
  CHECK(skipped_consumer.superseded_count() == 1);
  CHECK(skipped_consumer.ConsumeLatest(now) == ConsumerCode::kOk);
  CHECK(skipped_consumer.last_consumed_sequence() == 3);
  skipped_consumer.InjectProducerDisappearance();
  CHECK(skipped_consumer.ConsumeLatest(now) == ConsumerCode::kHardStopped);

  InMemoryFakeConsumerV1 invalid_consumer(6, 7);
  auto invalid = Command(1, now);
  invalid.position_rad[0] = std::numeric_limits<double>::quiet_NaN();
  CHECK(invalid_consumer.Offer(invalid, now) == ConsumerCode::kInvalidCommand);
  CHECK(invalid_consumer.hard_stopped());
}

void TestHardFaultAndFreshnessPreemption() {
  const std::int64_t start = 3'500'000'000;
  for (StopReason reason : {StopReason::kControllerAlarm, StopReason::kSdkFailure,
                            StopReason::kEstop, StopReason::kCollision,
                            StopReason::kProducerFailure}) {
    ReferenceJointShaperV1 shaper;
    CHECK(shaper.Initialize(Measured(start), Limits(), start).code == OperationCode::kOk);
    CHECK(shaper.ReplaceTarget(Target(1, start, start, 0.01), start).code ==
          OperationCode::kOk);
    shaper.HardStop(reason, start);
    ShapedJointCommandV1 output{};
    CHECK(shaper.Tick(start, &output).code == OperationCode::kTerminalNoOutput);
    CHECK(output.output_sequence == 0U);
    CHECK(shaper.Snapshot().stop_reason == reason);
  }

  ReferenceJointShaperV1 stale;
  CHECK(stale.Initialize(Measured(start), Limits(), start).code == OperationCode::kOk);
  auto expiring = Target(1, start, start, 0.01);
  expiring.valid_until_monotonic_ns = start;
  CHECK(stale.ReplaceTarget(expiring, start).code == OperationCode::kOk);
  ShapedJointCommandV1 output{};
  CHECK(stale.Tick(start, &output).code == OperationCode::kOk);
  CHECK(stale.Tick(start + kReferencePeriodNs, &output).code ==
        OperationCode::kTerminalNoOutput);
  CHECK(stale.Snapshot().stop_reason == StopReason::kStaleInput);

  ReferenceJointShaperV1 timing;
  CHECK(timing.Initialize(Measured(start), Limits(), start).code == OperationCode::kOk);
  CHECK(timing.ReplaceTarget(Target(1, start, start, 0.01), start).code ==
        OperationCode::kOk);
  CHECK(timing.Tick(start + kReferencePeriodNs, &output).code ==
        OperationCode::kTerminalNoOutput);
  CHECK(timing.Snapshot().stop_reason == StopReason::kTimingFault);
}

void TestResidualAccelerationNeutralization() {
  const std::int64_t start = 3'750'000'000;
  int neutralized_cases = 0;
  for (double velocity : {0.0, 1e-6, -1e-6, 1e-4, -1e-4, 1e-3, -1e-3,
                          1e-2, -1e-2}) {
    for (double acceleration : {0.0, 0.1, -0.1, 0.5, -0.5, 1.0, -1.0,
                                4.0, -4.0, 12.0, -12.0}) {
      ReferenceJointShaperV1 shaper;
      CHECK(shaper.Initialize(Measured(start, velocity, acceleration), Limits(), start).code ==
            OperationCode::kOk);
      CHECK(shaper.ReplaceTarget(Target(1, start, start), start).code ==
            OperationCode::kOk);
      const auto request =
          shaper.RequestControlledStop(2, StopReason::kClutchRelease, start);
      CHECK(request.code == OperationCode::kOk);
      if (request.code != OperationCode::kOk) {
        continue;
      }
      neutralized_cases += shaper.Snapshot().acceleration_neutralization_axis_count > 0U;
      ShapedJointCommandV1 output{};
      std::array<double, kMaxDof> previous_acceleration{};
      previous_acceleration[1] = acceleration;
      bool completed = false;
      for (int tick = 0; tick < 500; ++tick) {
        const auto result = shaper.Tick(start + tick * kReferencePeriodNs, &output);
        CHECK(result.code == OperationCode::kOk || result.code == OperationCode::kCompleted);
        CHECK(std::isfinite(output.position_rad[1]));
        CHECK(std::abs(output.velocity_rad_s[1]) <=
              Limits().maximum_velocity_rad_s[1] + 1e-10);
        CHECK(std::abs(output.acceleration_rad_s2[1]) <=
              Limits().maximum_acceleration_rad_s2[1] + 1e-10);
        CHECK(std::abs(output.acceleration_rad_s2[1] - previous_acceleration[1]) /
                  (static_cast<double>(kReferencePeriodNs) / 1e9) <=
              Limits().maximum_jerk_rad_s3[1] + 1e-8);
        previous_acceleration[1] = output.acceleration_rad_s2[1];
        if (output.output_mode == OutputMode::kStopped) {
          completed = true;
          break;
        }
      }
      CHECK(completed);
    }
  }
  CHECK(neutralized_cases > 0);

  ReferenceJointShaperV1 limited;
  auto measured = Measured(start, 0.0, 1.0);
  measured.position_rad[1] = 2.9999;
  CHECK(limited.Initialize(measured, Limits(), start).code == OperationCode::kOk);
  auto target = Target(1, start, start);
  target.position_rad[1] = measured.position_rad[1];
  CHECK(limited.ReplaceTarget(target, start).code == OperationCode::kOk);
  CHECK(limited.RequestControlledStop(2, StopReason::kClutchRelease, start).code ==
        OperationCode::kPlanningFailed);
  CHECK(limited.Snapshot().brake_planning_failure ==
        BrakePlanningFailure::kPositionLimit);
  CHECK(limited.Snapshot().mode == ShaperMode::kHardStopped);
}

void TestFakeJakaLifecycleHappyPathAndRecovery() {
  const std::int64_t now = 3'900'000'000;
  auto adapter = StreamingAdapter(now);
  CHECK(adapter.Send(CommandForMode(1, 7, now, OutputMode::kActiveTracking),
                     FakeSendOutcome::kOk, now) == FakeLifecycleCode::kOk);
  CHECK(adapter.SampleHealth(Health(1, 7, now), now) == FakeLifecycleCode::kOk);
  CHECK(adapter.Send(CommandForMode(2, 7, now, OutputMode::kControlledBraking),
                     FakeSendOutcome::kOk, now) == FakeLifecycleCode::kOk);
  CHECK(adapter.Snapshot().lifecycle_state ==
        FakeJakaLifecycleState::kControlledStopping);
  CHECK(adapter.ArmEpoch(Measured(now), now) == FakeLifecycleCode::kInvalidState);
  CHECK(adapter.Send(CommandForMode(3, 7, now, OutputMode::kStopped),
                     FakeSendOutcome::kOk, now) == FakeLifecycleCode::kOk);
  CHECK(adapter.Snapshot().lifecycle_state == FakeJakaLifecycleState::kStopped);
  CHECK(adapter.Send(CommandForMode(4, 7, now, OutputMode::kStopped),
                     FakeSendOutcome::kOk, now) == FakeLifecycleCode::kInvalidState);
  CHECK(adapter.Snapshot().accepted_command_count == 3U);

  auto measured = Measured(now);
  measured.safety_epoch = 8;
  CHECK(adapter.ArmEpoch(measured, now) == FakeLifecycleCode::kOk);
  CHECK(adapter.StartStreaming(8, now) == FakeLifecycleCode::kOk);
  CHECK(adapter.Send(CommandForMode(1, 8, now, OutputMode::kActiveTracking),
                     FakeSendOutcome::kOk, now) == FakeLifecycleCode::kOk);
  CHECK(adapter.Send(CommandForMode(2, 7, now, OutputMode::kActiveTracking),
                     FakeSendOutcome::kOk, now) == FakeLifecycleCode::kEpochMismatch);
  CHECK(adapter.Snapshot().hard_stop_latched);
  CHECK(adapter.has_terminal_fault_record());
  CHECK(adapter.terminal_fault_record().safety_epoch == 7);
  CHECK(adapter.Snapshot().safety_epoch == 8);
  CHECK(adapter.BeginCleanup(now) == FakeLifecycleCode::kOk);
  CHECK(adapter.CompleteCleanup(now) == FakeLifecycleCode::kOk);
  CHECK(adapter.Snapshot().lifecycle_state == FakeJakaLifecycleState::kDisconnected);
  CHECK(!adapter.Snapshot().session_owned);
  CHECK(adapter.Snapshot().hard_stop_latched);
  CHECK(adapter.Snapshot().reset_required);
  CHECK(adapter.BeginConnect(now) == FakeLifecycleCode::kInvalidState);
  CHECK(adapter.ResetAfterCleanup(now) == FakeLifecycleCode::kOk);
  CHECK(!adapter.Snapshot().hard_stop_latched);
  CHECK(!adapter.Snapshot().reset_required);
  CHECK(adapter.BeginConnect(now) == FakeLifecycleCode::kOk);
}

void TestSdkFreeClutchRecoveryContract() {
  const std::int64_t now = 4'100'000'000;
  CHECK(!SupportsSessionHeldRecovery(PauseCommandPolicy::kUnverified));
  CHECK(SupportsSessionHeldRecovery(PauseCommandPolicy::kNoCommandRequired));
  CHECK(SupportsSessionHeldRecovery(
      PauseCommandPolicy::kRepeatStoppedPositionRequired));

  InMemoryFakeJakaSdkInterface sdk;
  sdk.SetPauseCommandPolicy(PauseCommandPolicy::kNoCommandRequired);
  CHECK(sdk.Connect() == FakeSdkIoCode::kOk);
  CHECK(sdk.PrepareStreaming() == FakeSdkIoCode::kOk);
  auto adapter = StreamingAdapter(now);

  auto old_active = CommandForMode(1, 7, now, OutputMode::kActiveTracking);
  old_active.position_rad[1] = 1.0;
  auto braking = CommandForMode(2, 7, now + 8'000'000,
                                 OutputMode::kControlledBraking);
  braking.position_rad[1] = 0.8;
  auto stopped = CommandForMode(3, 7, now + 16'000'000, OutputMode::kStopped);
  stopped.position_rad[1] = 0.7;
  for (auto* command : {&old_active, &braking, &stopped}) {
    const auto io = sdk.SendShaped(*command);
    CHECK(adapter.Send(*command, ClassifyFakeSdkSend(io),
                       command->generated_monotonic_ns) == FakeLifecycleCode::kOk);
  }
  CHECK(adapter.Snapshot().lifecycle_state == FakeJakaLifecycleState::kStopped);
  CHECK(sdk.session_alive());
  CHECK(sdk.connect_count() == 1U);
  CHECK(sdk.prepare_count() == 1U);
  CHECK(sdk.send_count() == 3U);
  CHECK(sdk.cleanup_count() == 0U);

  RecoveryMeasurementGate gate(RecoveryMeasurementPolicy{});
  MeasuredJointStateV1 measured{};
  for (std::uint64_t sequence = 1; sequence <= 3; ++sequence) {
    const std::int64_t sample_time =
        now + 24'000'000 + static_cast<std::int64_t>(sequence) * 8'000'000;
    sdk.SetJointSample(JointSample(sequence, sample_time,
                                   JointSampleFields::kPositionVelocity,
                                   0.25, 0.0005));
    const auto result =
        ReadRecoveryMeasurement(&sdk, &gate, 8, sample_time, &measured);
    CHECK(result.code == (sequence < 3
                              ? RecoveryMeasurementCode::kNeedMoreSamples
                              : RecoveryMeasurementCode::kReady));
    if (sequence == 3) {
      CHECK(result.quality == RecoveryMeasurementQuality::
                                  kDirectQVelocityZeroAccelerationAfterStable);
    }
  }
  CHECK(measured.safety_epoch == 8U);
  CHECK(measured.position_rad[1] == 0.25);
  CHECK(measured.velocity_rad_s[1] == 0.0005);
  CHECK(measured.acceleration_rad_s2[1] == 0.0);
  CHECK(adapter.ArmEpoch(measured, now + 48'000'000) == FakeLifecycleCode::kOk);
  CHECK(adapter.StartStreaming(8, now + 48'000'000) == FakeLifecycleCode::kOk);
  auto resumed = CommandForMode(1, 8, now + 48'000'000,
                                OutputMode::kActiveTracking);
  resumed.position_rad = measured.position_rad;
  resumed.velocity_rad_s = measured.velocity_rad_s;
  CHECK(sdk.SendShaped(resumed) == FakeSdkIoCode::kOk);
  CHECK(adapter.Send(resumed, FakeSendOutcome::kOk, now + 48'000'000) ==
        FakeLifecycleCode::kOk);
  CHECK(resumed.position_rad[1] == measured.position_rad[1]);
  CHECK(resumed.position_rad[1] != old_active.position_rad[1]);
  CHECK(sdk.connect_count() == 1U);
  CHECK(sdk.prepare_count() == 1U);
  CHECK(sdk.cleanup_count() == 0U);

  auto old_epoch = CommandForMode(2, 7, now + 56'000'000,
                                  OutputMode::kActiveTracking);
  CHECK(adapter.Send(old_epoch, FakeSendOutcome::kOk, now + 56'000'000) ==
        FakeLifecycleCode::kEpochMismatch);
  CHECK(adapter.Snapshot().hard_stop_latched);
  CHECK(adapter.BeginCleanup(now + 56'000'001) == FakeLifecycleCode::kOk);
  CHECK(sdk.Cleanup() == FakeSdkIoCode::kOk);
  CHECK(adapter.CompleteCleanup(now + 56'000'002) == FakeLifecycleCode::kOk);
  CHECK(adapter.BeginConnect(now + 56'000'003) == FakeLifecycleCode::kInvalidState);
  CHECK(adapter.ResetAfterCleanup(now + 56'000'004) == FakeLifecycleCode::kOk);
  CHECK(adapter.BeginConnect(now + 56'000'005) == FakeLifecycleCode::kOk);
}

void TestRecoveryMeasurementFallbacksAndFaults() {
  const std::int64_t now = 4'200'000'000;
  MeasuredJointStateV1 measured{};
  {
    RecoveryMeasurementGate gate(RecoveryMeasurementPolicy{});
    auto sample = JointSample(1, now,
                              JointSampleFields::kPositionVelocityAcceleration,
                              0.4, 0.03, -0.2);
    const auto result = gate.Observe(sample, 9, now, &measured);
    CHECK(result.code == RecoveryMeasurementCode::kReady);
    CHECK(result.quality ==
          RecoveryMeasurementQuality::kDirectQVelocityAcceleration);
    CHECK(measured.position_rad[1] == 0.4);
    CHECK(measured.velocity_rad_s[1] == 0.03);
    CHECK(measured.acceleration_rad_s2[1] == -0.2);
  }
  {
    RecoveryMeasurementGate gate(RecoveryMeasurementPolicy{});
    for (std::uint64_t sequence = 1; sequence <= 3; ++sequence) {
      const std::int64_t tick = now + static_cast<std::int64_t>(sequence) * 8'000'000;
      auto sample = JointSample(sequence, tick, JointSampleFields::kPositionOnly,
                                0.5 + static_cast<double>(sequence) * 4e-6);
      const auto result = gate.Observe(sample, 10, tick, &measured);
      CHECK(result.code == (sequence < 3
                                ? RecoveryMeasurementCode::kNeedMoreSamples
                                : RecoveryMeasurementCode::kReady));
      if (sequence == 3) {
        CHECK(result.quality == RecoveryMeasurementQuality::
                                    kEstimatedVelocityZeroAccelerationAfterStable);
        CHECK(std::abs(measured.velocity_rad_s[1] - 0.0005) < 1e-9);
      }
    }
  }
  {
    RecoveryMeasurementGate gate(RecoveryMeasurementPolicy{});
    auto sample = JointSample(1, now, JointSampleFields::kPositionOnly, 0.0);
    CHECK(gate.Observe(sample, 10, now + 32'000'001, &measured).code ==
          RecoveryMeasurementCode::kStale);
    sample.sampled_monotonic_ns = now + 40'000'000;
    CHECK(gate.Observe(sample, 10, now + 40'000'000, &measured).code ==
          RecoveryMeasurementCode::kNeedMoreSamples);
    sample.sample_sequence = 2;
    sample.sampled_monotonic_ns += 8'000'000;
    sample.position_rad[1] = 0.1;
    CHECK(gate.Observe(sample, 10, sample.sampled_monotonic_ns, &measured).code ==
          RecoveryMeasurementCode::kUnstable);
    CHECK(gate.Observe(sample, 10, sample.sampled_monotonic_ns, &measured).code ==
          RecoveryMeasurementCode::kSequenceError);
  }
  {
    InMemoryFakeJakaSdkInterface sdk;
    CHECK(sdk.Connect() == FakeSdkIoCode::kOk);
    CHECK(sdk.PrepareStreaming() == FakeSdkIoCode::kOk);
    sdk.SetNextReadResult(FakeSdkIoCode::kStale);
    RecoveryMeasurementGate gate(RecoveryMeasurementPolicy{});
    CHECK(ReadRecoveryMeasurement(&sdk, &gate, 11, now, &measured).code ==
          RecoveryMeasurementCode::kIoFailure);
    sdk.SetNextSendResult(FakeSdkIoCode::kTransportFailure);
    CHECK(ClassifyFakeSdkSend(sdk.SendShaped(Command(1, now))) ==
          FakeSendOutcome::kTransportFailure);
  }
}

void TestFakeSdkHealthFaultsCannotClutchRecover() {
  const std::int64_t now = 4'300'000'000;
  int cases = 0;
  for (int fault = 0; fault < 4; ++fault) {
    InMemoryFakeJakaSdkInterface sdk;
    CHECK(sdk.Connect() == FakeSdkIoCode::kOk);
    CHECK(sdk.PrepareStreaming() == FakeSdkIoCode::kOk);
    auto adapter = StreamingAdapter(now);
    auto health = Health(1, 7, now);
    if (fault == 0) health.alarm = 1;
    if (fault == 1) health.estop = 1;
    if (fault == 2) health.collision = 1;
    if (fault == 3) health.servo_enabled = 0;
    sdk.SetHealth(health);
    TransportHealthV1 sampled{};
    CHECK(sdk.ReadHealth(&sampled) == FakeSdkIoCode::kOk);
    CHECK(adapter.SampleHealth(sampled, now) != FakeLifecycleCode::kOk);
    auto new_measured = Measured(now);
    new_measured.safety_epoch = 8;
    CHECK(adapter.ArmEpoch(new_measured, now) == FakeLifecycleCode::kInvalidState);
    CHECK(adapter.StartStreaming(8, now) == FakeLifecycleCode::kInvalidState);
    CHECK(adapter.BeginCleanup(now + 1) == FakeLifecycleCode::kOk);
    CHECK(sdk.Cleanup() == FakeSdkIoCode::kOk);
    CHECK(adapter.CompleteCleanup(now + 2) == FakeLifecycleCode::kOk);
    CHECK(adapter.BeginConnect(now + 3) == FakeLifecycleCode::kInvalidState);
    CHECK(adapter.ResetAfterCleanup(now + 4) == FakeLifecycleCode::kOk);
    ++cases;
  }
  {
    InMemoryFakeJakaSdkInterface sdk;
    CHECK(sdk.Connect() == FakeSdkIoCode::kOk);
    CHECK(sdk.PrepareStreaming() == FakeSdkIoCode::kOk);
    auto adapter = StreamingAdapter(now);
    sdk.SetNextHealthResult(FakeSdkIoCode::kStale);
    TransportHealthV1 sampled{};
    CHECK(sdk.ReadHealth(&sampled) == FakeSdkIoCode::kStale);
    adapter.InjectFault(StopReason::kStaleInput, now);
    CHECK(adapter.Snapshot().hard_stop_latched);
    auto new_measured = Measured(now);
    new_measured.safety_epoch = 8;
    CHECK(adapter.ArmEpoch(new_measured, now) == FakeLifecycleCode::kInvalidState);
    ++cases;
  }
  CHECK(cases == 5);

  auto adapter = StreamingAdapter(now);
  InMemoryFakeJakaSdkInterface sdk;
  CHECK(sdk.Connect() == FakeSdkIoCode::kOk);
  CHECK(sdk.PrepareStreaming() == FakeSdkIoCode::kOk);
  sdk.SetNextSendResult(FakeSdkIoCode::kTransportFailure);
  auto command = CommandForMode(1, 7, now, OutputMode::kActiveTracking);
  CHECK(adapter.Send(command, ClassifyFakeSdkSend(sdk.SendShaped(command)), now) ==
        FakeLifecycleCode::kTransportFailure);
  auto new_measured = Measured(now);
  new_measured.safety_epoch = 8;
  CHECK(adapter.ArmEpoch(new_measured, now) == FakeLifecycleCode::kInvalidState);
}

void TestFakeJakaLifecycleFaultMatrixAndTelemetry() {
  const std::int64_t now = 4'000'000'000;
  int fault_cases = 0;
  {
    FakeJakaLifecycleAdapter adapter;
    CHECK(adapter.BeginConnect(now) == FakeLifecycleCode::kOk);
    CHECK(adapter.CompleteConnect(false, now) == FakeLifecycleCode::kTransportFailure);
    CHECK(adapter.Snapshot().hard_stop_latched);
    ++fault_cases;
  }
  {
    FakeJakaLifecycleAdapter adapter;
    CHECK(adapter.BeginConnect(now) == FakeLifecycleCode::kOk);
    CHECK(adapter.CompleteConnect(true, now) == FakeLifecycleCode::kOk);
    CHECK(adapter.PrepareServo(now) == FakeLifecycleCode::kOk);
    auto invalid_measured = Measured(now);
    invalid_measured.velocity_rad_s[0] = std::numeric_limits<double>::quiet_NaN();
    CHECK(adapter.ArmEpoch(invalid_measured, now) == FakeLifecycleCode::kInvalidCommand);
    CHECK(adapter.Snapshot().hard_stop_latched);
    ++fault_cases;
  }
  {
    FakeJakaLifecycleAdapter adapter;
    CHECK(adapter.BeginConnect(now) == FakeLifecycleCode::kOk);
    CHECK(adapter.CompleteConnect(true, now) == FakeLifecycleCode::kOk);
    CHECK(adapter.PrepareServo(now) == FakeLifecycleCode::kOk);
    CHECK(adapter.ArmEpoch(Measured(now), now) == FakeLifecycleCode::kOk);
    CHECK(adapter.ArmEpoch(Measured(now), now) == FakeLifecycleCode::kEpochMismatch);
    CHECK(adapter.Snapshot().hard_stop_latched);
    ++fault_cases;
  }
  auto expect_fault = [&](FakeJakaLifecycleAdapter adapter,
                          ShapedJointCommandV1 command,
                          FakeSendOutcome outcome,
                          std::int64_t send_now) {
    const auto result = adapter.Send(command, outcome, send_now);
    CHECK(result != FakeLifecycleCode::kOk);
    CHECK(adapter.Snapshot().hard_stop_latched);
    CHECK(adapter.has_terminal_fault_record());
    CHECK(adapter.terminal_fault_record().output_sequence == command.output_sequence);
    CHECK(adapter.terminal_fault_record().source_sequence == command.source_sequence);
    CHECK(adapter.terminal_fault_record().safety_epoch == command.safety_epoch);
    CHECK(adapter.terminal_fault_record().command_age_ns ==
          send_now - command.generated_monotonic_ns);
    CHECK(adapter.terminal_fault_record().deadline_slack_ns ==
          command.valid_until_monotonic_ns - send_now);
    ++fault_cases;
  };

  {
    auto adapter = StreamingAdapter(now);
    CHECK(adapter.Send(CommandForMode(1, 7, now, OutputMode::kActiveTracking),
                       FakeSendOutcome::kOk, now) == FakeLifecycleCode::kOk);
    expect_fault(adapter, CommandForMode(1, 7, now, OutputMode::kActiveTracking),
                 FakeSendOutcome::kOk, now);
  }
  expect_fault(StreamingAdapter(now),
               CommandForMode(1, 8, now, OutputMode::kActiveTracking),
               FakeSendOutcome::kOk, now);
  expect_fault(StreamingAdapter(now),
               CommandForMode(1, 7, now, OutputMode::kActiveTracking),
               FakeSendOutcome::kOk, now + 16'000'001);
  expect_fault(StreamingAdapter(now),
               CommandForMode(1, 7, now, OutputMode::kActiveTracking),
               FakeSendOutcome::kOk, now - 1);
  {
    auto invalid = CommandForMode(1, 7, now, OutputMode::kActiveTracking);
    invalid.position_rad[0] = std::numeric_limits<double>::quiet_NaN();
    expect_fault(StreamingAdapter(now), invalid, FakeSendOutcome::kOk, now);
  }
  expect_fault(StreamingAdapter(now),
               CommandForMode(1, 7, now, OutputMode::kStopped),
               FakeSendOutcome::kOk, now);
  for (FakeSendOutcome outcome : {FakeSendOutcome::kRejected,
                                  FakeSendOutcome::kTransportFailure,
                                  FakeSendOutcome::kControllerAlarm}) {
    expect_fault(StreamingAdapter(now),
                 CommandForMode(1, 7, now, OutputMode::kActiveTracking),
                 outcome, now);
  }

  auto expect_health_fault = [&](TransportHealthV1 health) {
    auto adapter = StreamingAdapter(now);
    CHECK(adapter.SampleHealth(health, now) != FakeLifecycleCode::kOk);
    CHECK(adapter.Snapshot().hard_stop_latched);
    CHECK(adapter.has_terminal_fault_record());
    ++fault_cases;
  };
  auto health = Health(1, 7, now);
  health.producer_stale = 1;
  expect_health_fault(health);
  health = Health(1, 7, now);
  health.command_stale = 1;
  expect_health_fault(health);
  health = Health(1, 7, now);
  health.deadline_missed = 1;
  expect_health_fault(health);
  health = Health(1, 7, now);
  health.alarm = 1;
  expect_health_fault(health);
  health = Health(1, 7, now);
  health.estop = 1;
  expect_health_fault(health);
  health = Health(1, 7, now);
  health.collision = 1;
  expect_health_fault(health);
  health = Health(1, 7, now);
  health.transport_state = TransportState::kFaulted;
  expect_health_fault(health);
  health = Health(1, 7, now);
  health.servo_enabled = 0;
  expect_health_fault(health);
  {
    auto adapter = StreamingAdapter(now);
    CHECK(adapter.SampleHealth(Health(1, 7, now), now) == FakeLifecycleCode::kOk);
    CHECK(adapter.SampleHealth(Health(1, 7, now), now) != FakeLifecycleCode::kOk);
    CHECK(adapter.Snapshot().hard_stop_latched);
    ++fault_cases;
  }
  CHECK(fault_cases == 21);

  auto latest_wins_adapter = StreamingAdapter(now);
  CHECK(latest_wins_adapter.Send(
            CommandForMode(1, 7, now, OutputMode::kActiveTracking),
            FakeSendOutcome::kOk, now) == FakeLifecycleCode::kOk);
  CHECK(latest_wins_adapter.Send(
            CommandForMode(4, 7, now, OutputMode::kActiveTracking),
            FakeSendOutcome::kOk, now) == FakeLifecycleCode::kOk);
  CHECK(latest_wins_adapter.Snapshot().accepted_command_count == 2U);
  CHECK(latest_wins_adapter.Snapshot().skipped_output_sequence_count == 2U);

  auto telemetry_adapter = StreamingAdapter(now);
  const std::uint64_t allocations_before = g_allocations.load();
  for (std::uint64_t sequence = 1; sequence <= 300; ++sequence) {
    const std::int64_t tick = now + static_cast<std::int64_t>(sequence - 1) * 1'000;
    CHECK(telemetry_adapter.Send(
              CommandForMode(sequence, 7, tick, OutputMode::kActiveTracking),
              FakeSendOutcome::kOk, tick) == FakeLifecycleCode::kOk);
  }
  CHECK(g_allocations.load() == allocations_before);
  CHECK(telemetry_adapter.telemetry_size() ==
        FakeJakaLifecycleAdapter::kTelemetryCapacity);
  CHECK(telemetry_adapter.Snapshot().telemetry_overflow_count > 0U);
  telemetry_adapter.InjectFault(StopReason::kControllerAlarm, now + 1'000'000);
  const auto terminal_sequence =
      telemetry_adapter.terminal_fault_record().record_sequence;
  CHECK(telemetry_adapter.BeginCleanup(now + 1'000'001) == FakeLifecycleCode::kOk);
  CHECK(telemetry_adapter.CompleteCleanup(now + 1'000'002) == FakeLifecycleCode::kOk);
  CHECK(telemetry_adapter.has_terminal_fault_record());
  CHECK(telemetry_adapter.terminal_fault_record().record_sequence == terminal_sequence);
  std::cout << "fake_lifecycle_fault_matrix_cases=" << fault_cases
            << " telemetry_capacity=" << FakeJakaLifecycleAdapter::kTelemetryCapacity
            << " overflow_count="
            << telemetry_adapter.Snapshot().telemetry_overflow_count << '\n';
}

void TestDeterministicPropertyAndNoAllocation() {
  constexpr std::uint64_t seed = 0x5EED1234ULL;
  std::mt19937_64 random(seed);
  const std::int64_t start = 4'000'000'000;
  for (int iteration = 0; iteration < 5000; ++iteration) {
    auto target = Target(1, start, start);
    target.header.schema_version = static_cast<std::uint16_t>(random() & 0xFFFFU);
    const auto result = Validate(target);
    if (target.header.schema_version != kSchemaVersionV1) CHECK(!result.ok);
    target = Target(1, start, start);
    target.dof = static_cast<std::uint8_t>(random() & 0xFFU);
    if (target.dof == 0U || target.dof > kMaxDof) CHECK(!Validate(target).ok);
    target = Target(1, start, start);
    target.validity = static_cast<TargetValidity>(random() & 0xFFU);
    if (static_cast<std::uint8_t>(target.validity) >
        static_cast<std::uint8_t>(TargetValidity::kRejectedKeepPrevious)) {
      CHECK(!Validate(target).ok);
    }
  }

  ReferenceJointShaperV1 shaper;
  CHECK(shaper.Initialize(Measured(start), Limits(), start).code == OperationCode::kOk);
  std::uint64_t target_sequence = 1;
  std::int64_t target_source = start;
  CHECK(shaper.ReplaceTarget(Target(target_sequence, target_source, start), start).code ==
        OperationCode::kOk);
  ShapedJointCommandV1 output{};
  std::array<double, kMaxDof> previous_acceleration{};
  const std::uint64_t before = g_allocations.load();
  for (int tick = 0; tick < 100'000; ++tick) {
    const std::int64_t now = start + static_cast<std::int64_t>(tick) * kReferencePeriodNs;
    if (tick > 0 && tick % 50 == 0) {
      ++target_sequence;
      target_source = now;
      const double target_position = (target_sequence % 2U == 0U) ? 0.01 : -0.01;
      auto target = Target(target_sequence, target_source, now, target_position);
      target.valid_until_monotonic_ns = now + 1'000'000'000;
      CHECK(shaper.ReplaceTarget(target, now).code == OperationCode::kOk);
    }
    CHECK(shaper.Tick(now, &output).code == OperationCode::kOk);
    for (std::size_t i = 0; i < output.dof; ++i) {
      CHECK(std::isfinite(output.position_rad[i]));
      CHECK(std::isfinite(output.velocity_rad_s[i]));
      CHECK(std::isfinite(output.acceleration_rad_s2[i]));
      CHECK(std::abs(output.velocity_rad_s[i]) <= Limits().maximum_velocity_rad_s[i] + 1e-12);
      CHECK(std::abs(output.acceleration_rad_s2[i]) <=
            Limits().maximum_acceleration_rad_s2[i] + 1e-12);
      CHECK(std::abs(output.acceleration_rad_s2[i] - previous_acceleration[i]) /
                (static_cast<double>(kReferencePeriodNs) / 1e9) <=
            Limits().maximum_jerk_rad_s3[i] + 1e-9);
      previous_acceleration[i] = output.acceleration_rad_s2[i];
    }
  }
  const std::uint64_t after = g_allocations.load();
  CHECK(after == before);
  std::cout << "property_seed=" << seed << " ticks=100000 allocations="
            << (after - before) << '\n';
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
  TestAbiLayoutAndValidation();
  TestActiveReferenceAndHeartbeat();
  TestControlledBrakingAndStateMachine();
  TestFakeConsumerLatestWinsAndFaults();
  TestHardFaultAndFreshnessPreemption();
  TestResidualAccelerationNeutralization();
  TestFakeJakaLifecycleHappyPathAndRecovery();
  TestFakeJakaLifecycleFaultMatrixAndTelemetry();
  TestSdkFreeClutchRecoveryContract();
  TestRecoveryMeasurementFallbacksAndFaults();
  TestFakeSdkHealthFaultsCannotClutchRecover();
  TestDeterministicPropertyAndNoAllocation();
  if (g_failures != 0) {
    std::cerr << g_failures << " test checks failed\n";
    return 1;
  }
  std::cout << "teleop_shaping_tests: passed\n";
  return 0;
}
