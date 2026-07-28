#include "teleop_shaping/engagement_joint_shaper.hpp"

namespace teleop_shaping {

namespace {

OperationResult Result(OperationCode code) noexcept {
  return OperationResult{code, teleop_command_abi::ValidationResult{
                                   code == OperationCode::kOk,
                                   code == OperationCode::kOk
                                       ? teleop_command_abi::ValidationError::kOk
                                       : teleop_command_abi::ValidationError::kInvalidModeContract,
                                   code == OperationCode::kOk
                                       ? teleop_command_abi::ValidationField::kNone
                                       : teleop_command_abi::ValidationField::kOutputMode,
                                   static_cast<std::uint8_t>(
                                       teleop_command_abi::kNoFieldIndex)}};
}

}  // namespace

EngagementJointShaperV1::EngagementJointShaperV1() noexcept
    : state_(EngagementShaperState::kWaitingForEngagement),
      safety_epoch_(0U),
      engagement_count_(0U),
      initialization_monotonic_ns_(-1),
      next_tick_monotonic_ns_(-1) {}

OperationResult EngagementJointShaperV1::InvalidState() const noexcept {
  return Result(OperationCode::kInvalidState);
}

OperationResult EngagementJointShaperV1::InitializeEngagement(
    const teleop_command_abi::MeasuredJointStateV1& measured,
    const teleop_command_abi::JointDynamicLimitsV1& limits,
    const teleop_command_abi::AcceptedJointTargetV1& first_target,
    std::int64_t validation_now_ns) noexcept {
  if (state_ != EngagementShaperState::kWaitingForEngagement &&
      state_ != EngagementShaperState::kStopped) {
    return InvalidState();
  }
  if (validation_now_ns < 0 || measured.safety_epoch == 0U ||
      first_target.safety_epoch != measured.safety_epoch) {
    state_ = EngagementShaperState::kHardStopped;
    return Result(OperationCode::kInvalidArgument);
  }

  shaper_.emplace();
  auto result =
      shaper_->InitializeForNextTick(measured, limits, validation_now_ns);
  if (result.code != OperationCode::kOk) {
    state_ = EngagementShaperState::kHardStopped;
    return result;
  }
  result = shaper_->ReplaceTarget(first_target, validation_now_ns);
  if (result.code != OperationCode::kOk) {
    state_ = EngagementShaperState::kHardStopped;
    return result;
  }

  state_ = EngagementShaperState::kArmed;
  safety_epoch_ = measured.safety_epoch;
  ++engagement_count_;
  initialization_monotonic_ns_ = validation_now_ns;
  next_tick_monotonic_ns_ = validation_now_ns + kEngagementShaperPeriodNs;
  return result;
}

OperationResult EngagementJointShaperV1::ReplaceTarget(
    const teleop_command_abi::AcceptedJointTargetV1& target,
    std::int64_t grid_now_ns) noexcept {
  if (!shaper_.has_value() ||
      (state_ != EngagementShaperState::kArmed &&
       state_ != EngagementShaperState::kActiveTracking) ||
      grid_now_ns != next_tick_monotonic_ns_) {
    return InvalidState();
  }
  const auto result = shaper_->ReplaceTarget(target, grid_now_ns);
  if (result.code != OperationCode::kOk) {
    state_ = EngagementShaperState::kHardStopped;
  }
  return result;
}

OperationResult EngagementJointShaperV1::RequestControlledStop(
    std::uint64_t release_sequence, teleop_command_abi::StopReason reason,
    std::int64_t grid_now_ns) noexcept {
  if (!shaper_.has_value() ||
      (state_ != EngagementShaperState::kArmed &&
       state_ != EngagementShaperState::kActiveTracking) ||
      grid_now_ns != next_tick_monotonic_ns_) {
    return InvalidState();
  }
  const auto result =
      shaper_->RequestControlledStop(release_sequence, reason, grid_now_ns);
  if (result.code == OperationCode::kOk ||
      result.code == OperationCode::kAlreadyRequested) {
    state_ = EngagementShaperState::kControlledBraking;
  } else if (result.code == OperationCode::kPlanningFailed) {
    state_ = EngagementShaperState::kHardStopped;
  }
  return result;
}

OperationResult EngagementJointShaperV1::Tick(
    std::int64_t grid_now_ns,
    teleop_command_abi::ShapedJointCommandV1* output) noexcept {
  if (!shaper_.has_value() ||
      (state_ != EngagementShaperState::kArmed &&
       state_ != EngagementShaperState::kActiveTracking &&
       state_ != EngagementShaperState::kControlledBraking)) {
    return InvalidState();
  }
  const auto result = shaper_->Tick(grid_now_ns, output);
  if (result.code != OperationCode::kOk &&
      result.code != OperationCode::kCompleted) {
    state_ = EngagementShaperState::kHardStopped;
    return result;
  }
  next_tick_monotonic_ns_ += kEngagementShaperPeriodNs;
  if (output->output_mode == teleop_command_abi::OutputMode::kStopped) {
    state_ = EngagementShaperState::kStopped;
    next_tick_monotonic_ns_ = -1;
  } else if (output->output_mode ==
             teleop_command_abi::OutputMode::kControlledBraking) {
    state_ = EngagementShaperState::kControlledBraking;
  } else {
    state_ = EngagementShaperState::kActiveTracking;
  }
  return result;
}

void EngagementJointShaperV1::HardStop(
    teleop_command_abi::StopReason reason, std::int64_t now_ns) noexcept {
  if (shaper_.has_value()) shaper_->HardStop(reason, now_ns);
  state_ = EngagementShaperState::kHardStopped;
  next_tick_monotonic_ns_ = -1;
}

EngagementShaperSnapshot EngagementJointShaperV1::Snapshot() const noexcept {
  ShaperSnapshot shaper_snapshot{};
  if (shaper_.has_value()) shaper_snapshot = shaper_->Snapshot();
  return EngagementShaperSnapshot{state_, safety_epoch_, engagement_count_,
                                  initialization_monotonic_ns_,
                                  next_tick_monotonic_ns_, shaper_snapshot};
}

}  // namespace teleop_shaping
