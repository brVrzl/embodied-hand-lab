#include "teleop_shaping/engagement_joint_shaper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

using namespace teleop_command_abi;
using namespace teleop_shaping;

int g_failures = 0;

#define CHECK(condition)                                                        \
  do {                                                                          \
    if (!(condition)) {                                                         \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "            \
                << #condition << '\n';                                          \
      ++g_failures;                                                             \
    }                                                                           \
  } while (false)

JointDynamicLimitsV1 Limits() {
  JointDynamicLimitsV1 limits{};
  limits.header = MakeHeaderV1<JointDynamicLimitsV1>();
  limits.dof = 6;
  for (std::size_t i = 0; i < limits.dof; ++i) {
    limits.minimum_position_rad[i] = -2.0;
    limits.maximum_position_rad[i] = 2.0;
    limits.maximum_velocity_rad_s[i] = 0.5;
    limits.maximum_acceleration_rad_s2[i] = 2.0;
    limits.maximum_jerk_rad_s3[i] = 20.0;
  }
  return limits;
}

MeasuredJointStateV1 Measured(std::uint64_t sequence, std::uint64_t epoch,
                              std::int64_t now_ns, double q,
                              double dq = 0.0) {
  MeasuredJointStateV1 measured{};
  measured.header = MakeHeaderV1<MeasuredJointStateV1>();
  measured.state_sequence = sequence;
  measured.safety_epoch = epoch;
  measured.measured_monotonic_ns = now_ns;
  measured.dof = 6;
  measured.validity = MeasurementValidity::kValid;
  measured.position_rad.fill(0.0);
  measured.velocity_rad_s.fill(0.0);
  measured.acceleration_rad_s2.fill(0.0);
  measured.position_rad[0] = q;
  measured.velocity_rad_s[0] = dq;
  return measured;
}

AcceptedJointTargetV1 Target(std::uint64_t sequence, std::uint64_t epoch,
                             std::int64_t now_ns, double q) {
  AcceptedJointTargetV1 target{};
  target.header = MakeHeaderV1<AcceptedJointTargetV1>();
  target.sequence = sequence;
  target.safety_epoch = epoch;
  target.source_monotonic_ns = now_ns - 2'000'000;
  target.accepted_monotonic_ns = now_ns;
  target.valid_until_monotonic_ns = now_ns + 2'000'000'000;
  target.dof = 6;
  target.engagement = EngagementState::kEngaged;
  target.validity = TargetValidity::kAccepted;
  target.position_rad.fill(0.0);
  target.position_rad[0] = q;
  return target;
}

ShapedJointCommandV1 DelayedFirstOutput(std::int64_t wait_ns) {
  const std::int64_t preparation_ns = 1'000'000'000;
  const std::int64_t engagement_ns = preparation_ns + wait_ns;
  const auto measured = Measured(1, 1, engagement_ns, 0.25);
  const auto target = Target(1, 1, engagement_ns, 0.25);
  EngagementJointShaperV1 lifecycle;

  CHECK(lifecycle.Snapshot().state ==
        EngagementShaperState::kWaitingForEngagement);
  CHECK(lifecycle.InitializeEngagement(measured, Limits(), target,
                                       engagement_ns).code ==
        OperationCode::kOk);
  const auto armed = lifecycle.Snapshot();
  CHECK(armed.state == EngagementShaperState::kArmed);
  CHECK(armed.initialization_monotonic_ns == engagement_ns);
  CHECK(armed.next_tick_monotonic_ns ==
        engagement_ns + kEngagementShaperPeriodNs);
  CHECK(armed.shaper.position_rad[0] == measured.position_rad[0]);

  ShapedJointCommandV1 output{};
  CHECK(lifecycle.Tick(armed.next_tick_monotonic_ns, &output).code ==
        OperationCode::kOk);
  CHECK(output.generated_monotonic_ns - engagement_ns ==
        kEngagementShaperPeriodNs);
  CHECK(std::abs(output.position_rad[0] - measured.position_rad[0]) < 1e-12);
  CHECK(lifecycle.Snapshot().shaper.last_tick_ns ==
        armed.next_tick_monotonic_ns);
  return output;
}

void TestDelayedInitialEngagement() {
  const auto after_two_seconds = DelayedFirstOutput(2'000'000'000);
  const auto after_ten_seconds = DelayedFirstOutput(10'000'000'000);
  for (std::size_t i = 0; i < 6; ++i) {
    CHECK(std::abs(after_two_seconds.position_rad[i] -
                   after_ten_seconds.position_rad[i]) < 1e-12);
    CHECK(std::abs(after_two_seconds.velocity_rad_s[i] -
                   after_ten_seconds.velocity_rad_s[i]) < 1e-12);
    CHECK(std::abs(after_two_seconds.acceleration_rad_s2[i] -
                   after_ten_seconds.acceleration_rad_s2[i]) < 1e-12);
  }
}

void TestReleaseWaitAndReengageUsesNewMeasuredState() {
  EngagementJointShaperV1 lifecycle;
  std::int64_t now = 5'000'000'000;
  CHECK(lifecycle.InitializeEngagement(Measured(1, 1, now, 0.0), Limits(),
                                       Target(1, 1, now, 0.03), now).code ==
        OperationCode::kOk);
  ShapedJointCommandV1 output{};
  for (int i = 0; i < 24; ++i) {
    now = lifecycle.Snapshot().next_tick_monotonic_ns;
    CHECK(lifecycle.Tick(now, &output).code == OperationCode::kOk);
  }
  now = lifecycle.Snapshot().next_tick_monotonic_ns;
  CHECK(lifecycle.RequestControlledStop(2, StopReason::kClutchRelease, now).code ==
        OperationCode::kOk);
  for (int i = 0; i < 500; ++i) {
    now = lifecycle.Snapshot().next_tick_monotonic_ns;
    const auto result = lifecycle.Tick(now, &output);
    CHECK(result.code == OperationCode::kOk ||
          result.code == OperationCode::kCompleted);
    if (lifecycle.Snapshot().state == EngagementShaperState::kStopped) break;
  }
  CHECK(lifecycle.Snapshot().state == EngagementShaperState::kStopped);

  const std::int64_t reengagement_ns = now + 6'000'000'000;
  const auto refreshed = Measured(2, 2, reengagement_ns, -0.18, 0.002);
  CHECK(lifecycle.InitializeEngagement(
            refreshed, Limits(), Target(10, 2, reengagement_ns, -0.18),
            reengagement_ns).code == OperationCode::kOk);
  const auto rearmed = lifecycle.Snapshot();
  CHECK(rearmed.engagement_count == 2);
  CHECK(rearmed.safety_epoch == 2);
  CHECK(std::abs(rearmed.shaper.position_rad[0] + 0.18) < 1e-12);
  CHECK(std::abs(rearmed.shaper.velocity_rad_s[0] - 0.002) < 1e-12);
  CHECK(rearmed.shaper.source_sequence == 10);

  CHECK(lifecycle.Tick(rearmed.next_tick_monotonic_ns, &output).code ==
        OperationCode::kOk);
  CHECK(std::abs(output.position_rad[0] - refreshed.position_rad[0]) < 5e-5);
  CHECK(output.safety_epoch == 2);
  CHECK(output.source_sequence == 10);
}

void TestOldEpochRejectedAfterReengagement() {
  EngagementJointShaperV1 lifecycle;
  const std::int64_t now = 8'000'000'000;
  CHECK(lifecycle.InitializeEngagement(Measured(1, 2, now, 0.1), Limits(),
                                       Target(10, 2, now, 0.1), now).code ==
        OperationCode::kOk);
  const std::int64_t first_tick = lifecycle.Snapshot().next_tick_monotonic_ns;
  const auto old_epoch = Target(11, 1, first_tick, 0.1);
  CHECK(lifecycle.ReplaceTarget(old_epoch, first_tick).code !=
        OperationCode::kOk);
  CHECK(lifecycle.Snapshot().state == EngagementShaperState::kHardStopped);
}

void TestActiveGapAndClockRegressionFailClosed() {
  for (const std::int64_t invalid_offset :
       std::array<std::int64_t, 2>{2 * kEngagementShaperPeriodNs, -1}) {
    EngagementJointShaperV1 lifecycle;
    const std::int64_t now = 11'000'000'000;
    CHECK(lifecycle.InitializeEngagement(Measured(1, 1, now, 0.0), Limits(),
                                         Target(1, 1, now, 0.0), now).code ==
          OperationCode::kOk);
    ShapedJointCommandV1 output{};
    const std::int64_t invalid_tick =
        invalid_offset < 0 ? now - 1 : now + invalid_offset;
    CHECK(lifecycle.Tick(invalid_tick, &output).code ==
          OperationCode::kTerminalNoOutput);
    CHECK(lifecycle.Snapshot().state == EngagementShaperState::kHardStopped);
    CHECK(lifecycle.Snapshot().shaper.stop_reason == StopReason::kTimingFault);
  }
}

}  // namespace

int main() {
  TestDelayedInitialEngagement();
  TestReleaseWaitAndReengageUsesNewMeasuredState();
  TestOldEpochRejectedAfterReengagement();
  TestActiveGapAndClockRegressionFailClosed();
  if (g_failures != 0) {
    std::cerr << g_failures << " engagement lifecycle checks failed\n";
    return 1;
  }
  std::cout << "engagement lifecycle checks passed\n";
  return 0;
}
