#pragma once

#include <array>
#include <cstdint>

#include "teleop_command_abi/abi_v1.hpp"

namespace teleop_shaping {

using teleop_command_abi::AcceptedJointTargetV1;
using teleop_command_abi::JointDynamicLimitsV1;
using teleop_command_abi::MeasuredJointStateV1;
using teleop_command_abi::OutputMode;
using teleop_command_abi::ShapedJointCommandV1;
using teleop_command_abi::StopReason;
using teleop_command_abi::ValidationResult;
using teleop_command_abi::kMaxDof;

constexpr std::int64_t kReferencePeriodNs = 8'000'000;

enum class OperationCode : std::uint8_t {
  kOk = 0,
  kCompleted = 1,
  kAlreadyRequested = 2,
  kInvalidArgument = 3,
  kInvalidState = 4,
  kPlanningFailed = 5,
  kTerminalNoOutput = 6,
};

struct OperationResult {
  OperationCode code;
  ValidationResult validation;
};

enum class ShaperMode : std::uint8_t {
  kUninitialized = 0,
  kActiveTracking = 1,
  kControlledBraking = 2,
  kStopped = 3,
  kHardStopped = 4,
};

enum class BrakePlanningFailure : std::uint8_t {
  kNone = 0,
  kPositionLimit = 1,
  kVelocityLimit = 2,
  kNumerical = 3,
  kInvalidDynamicState = 4,
};

struct ShaperSnapshot {
  ShaperMode mode;
  std::uint8_t dof;
  std::uint64_t safety_epoch;
  std::uint64_t last_input_sequence;
  std::uint64_t source_sequence;
  std::uint64_t output_sequence;
  std::uint64_t release_sequence;
  std::int64_t last_tick_ns;
  std::int64_t liveness_monotonic_ns;
  StopReason stop_reason;
  BrakePlanningFailure brake_planning_failure;
  std::uint8_t brake_planning_failure_axis;
  std::uint8_t acceleration_neutralization_axis_count;
  std::array<double, kMaxDof> position_rad;
  std::array<double, kMaxDof> velocity_rad_s;
  std::array<double, kMaxDof> acceleration_rad_s2;
};

class IJointShaper {
 public:
  virtual ~IJointShaper() = default;
  virtual OperationResult Initialize(const MeasuredJointStateV1& measured,
                                     const JointDynamicLimitsV1& limits,
                                     std::int64_t now_ns) noexcept = 0;
  virtual OperationResult ReplaceTarget(const AcceptedJointTargetV1& target,
                                        std::int64_t now_ns) noexcept = 0;
  virtual OperationResult Tick(std::int64_t now_ns,
                               ShapedJointCommandV1* output) noexcept = 0;
  virtual OperationResult RequestControlledStop(std::uint64_t release_sequence,
                                                StopReason reason,
                                                std::int64_t now_ns) noexcept = 0;
  virtual void HardStop(StopReason reason, std::int64_t now_ns) noexcept = 0;
  virtual ShaperSnapshot Snapshot() const noexcept = 0;
};

class ReferenceJointShaperV1 final : public IJointShaper {
 public:
  ReferenceJointShaperV1() noexcept;
  OperationResult Initialize(const MeasuredJointStateV1& measured,
                             const JointDynamicLimitsV1& limits,
                             std::int64_t now_ns) noexcept override;
  // Lifecycle-specific initialization whose first integration step is due on
  // the following 8 ms grid tick. The legacy Initialize contract is retained
  // for replay/conformance callers that intentionally tick at now_ns.
  OperationResult InitializeForNextTick(
      const MeasuredJointStateV1& measured,
      const JointDynamicLimitsV1& limits,
      std::int64_t now_ns) noexcept;
  OperationResult ReplaceTarget(const AcceptedJointTargetV1& target,
                                std::int64_t now_ns) noexcept override;
  OperationResult Tick(std::int64_t now_ns,
                       ShapedJointCommandV1* output) noexcept override;
  OperationResult RequestControlledStop(std::uint64_t release_sequence,
                                        StopReason reason,
                                        std::int64_t now_ns) noexcept override;
  void HardStop(StopReason reason, std::int64_t now_ns) noexcept override;
  ShaperSnapshot Snapshot() const noexcept override;

 private:
  struct BrakeAxis {
    std::array<double, 4> duration_s;
    std::array<double, 4> jerk_rad_s3;
    std::uint8_t phase;
    double phase_elapsed_s;
    bool complete;
    bool uses_acceleration_neutralization;
  };

  OperationResult FailClosed(teleop_command_abi::ValidationResult validation,
                             StopReason reason, std::int64_t now_ns) noexcept;
  bool PlanBrakeAxis(std::size_t axis, BrakeAxis* plan) noexcept;
  bool PlanAccelerationNeutralizedBrakeAxis(std::size_t axis,
                                            BrakeAxis* plan) noexcept;
  bool SynchronizeBrakeAxis(std::size_t axis, double duration_s,
                            BrakeAxis* plan) noexcept;
  void AdvanceBrakeAxis(std::size_t axis, double dt_s) noexcept;
  void Publish(OutputMode mode, teleop_command_abi::StopClass stop_class,
               StopReason reason, std::int64_t now_ns,
               ShapedJointCommandV1* output) noexcept;

  ShaperMode mode_;
  std::uint8_t dof_;
  std::uint64_t safety_epoch_;
  std::uint64_t last_input_sequence_;
  std::uint64_t source_sequence_;
  std::uint64_t output_sequence_;
  std::uint64_t release_sequence_;
  std::int64_t last_tick_ns_;
  std::int64_t liveness_monotonic_ns_;
  std::int64_t target_valid_until_ns_;
  std::int64_t last_target_source_ns_;
  StopReason stop_reason_;
  BrakePlanningFailure brake_planning_failure_;
  std::uint8_t brake_planning_failure_axis_;
  std::uint8_t acceleration_neutralization_axis_count_;
  JointDynamicLimitsV1 limits_;
  std::array<double, kMaxDof> position_;
  std::array<double, kMaxDof> velocity_;
  std::array<double, kMaxDof> acceleration_;
  std::array<double, kMaxDof> target_;
  std::array<double, kMaxDof> target_velocity_;
  std::array<BrakeAxis, kMaxDof> brake_;
};

}  // namespace teleop_shaping
