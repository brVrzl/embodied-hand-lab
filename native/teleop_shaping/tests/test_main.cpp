#include "teleop_command_abi/abi_v1.hpp"
#include "teleop_shaping/fake_consumer.hpp"
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
  TestDeterministicPropertyAndNoAllocation();
  if (g_failures != 0) {
    std::cerr << g_failures << " test checks failed\n";
    return 1;
  }
  std::cout << "teleop_shaping_tests: passed\n";
  return 0;
}
