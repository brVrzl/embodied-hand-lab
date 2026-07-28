#pragma once

#include <cstdint>
#include <optional>

#include "teleop_shaping/joint_shaper.hpp"

namespace teleop_shaping {

constexpr std::int64_t kEngagementShaperPeriodNs = 8'000'000;

enum class EngagementShaperState : std::uint8_t {
  kWaitingForEngagement = 0,
  kArmed = 1,
  kActiveTracking = 2,
  kControlledBraking = 3,
  kStopped = 4,
  kHardStopped = 5,
};

struct EngagementShaperSnapshot {
  EngagementShaperState state;
  std::uint64_t safety_epoch;
  std::uint64_t engagement_count;
  std::int64_t initialization_monotonic_ns;
  std::int64_t next_tick_monotonic_ns;
  ShaperSnapshot shaper;
};

// Owns only the robot-independent active/braking shaper lifecycle. Transport
// preparation and measured-state acquisition remain caller responsibilities.
// No active clock exists while waiting for engagement or between stopped and
// re-engagement.
class EngagementJointShaperV1 final {
 public:
  EngagementJointShaperV1() noexcept;

  OperationResult InitializeEngagement(
      const teleop_command_abi::MeasuredJointStateV1& measured,
      const teleop_command_abi::JointDynamicLimitsV1& limits,
      const teleop_command_abi::AcceptedJointTargetV1& first_target,
      std::int64_t validation_now_ns) noexcept;
  OperationResult ReplaceTarget(
      const teleop_command_abi::AcceptedJointTargetV1& target,
      std::int64_t grid_now_ns) noexcept;
  OperationResult RequestControlledStop(
      std::uint64_t release_sequence,
      teleop_command_abi::StopReason reason,
      std::int64_t grid_now_ns) noexcept;
  OperationResult Tick(
      std::int64_t grid_now_ns,
      teleop_command_abi::ShapedJointCommandV1* output) noexcept;
  void HardStop(teleop_command_abi::StopReason reason,
                std::int64_t now_ns) noexcept;

  EngagementShaperSnapshot Snapshot() const noexcept;

 private:
  OperationResult InvalidState() const noexcept;

  std::optional<ReferenceJointShaperV1> shaper_;
  EngagementShaperState state_;
  std::uint64_t safety_epoch_;
  std::uint64_t engagement_count_;
  std::int64_t initialization_monotonic_ns_;
  std::int64_t next_tick_monotonic_ns_;
};

}  // namespace teleop_shaping
