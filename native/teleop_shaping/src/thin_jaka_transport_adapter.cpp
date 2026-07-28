#include "teleop_shaping/thin_jaka_transport_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace teleop_shaping {
namespace {

bool IsSupportedPausePolicy(PauseCommandPolicy policy) noexcept {
  return policy == PauseCommandPolicy::kNoCommandRequired ||
         policy == PauseCommandPolicy::kRepeatStoppedPositionRequired;
}

bool IsSupportedResumePolicy(ResumePreparationPolicy policy) noexcept {
  return policy == ResumePreparationPolicy::kKeepPrepared ||
         policy == ResumePreparationPolicy::kRestartEdg ||
         policy == ResumePreparationPolicy::kRestartEdgAndServo;
}

bool SdkOk(JakaFunctionResult result) noexcept {
  return result == JakaFunctionResult::kOk;
}

}  // namespace

ThinJakaTransportAdapter::ThinJakaTransportAdapter(
    JakaSdkFunctionTable functions, ThinJakaConfig config) noexcept
    : functions_(functions),
      config_(config),
      state_(ThinJakaState::kDisconnected),
      fault_reason_(teleop_command_abi::StopReason::kNone),
      session_owned_(false),
      edg_enabled_(false),
      servo_enabled_(false),
      cleanup_failed_(false),
      pending_valid_(false),
      stopped_command_valid_(false),
      hard_stop_callback_called_(false),
      safety_epoch_(0),
      last_output_sequence_(0),
      last_status_sequence_(0),
      sent_command_count_(0),
      repeated_stopped_command_count_(0),
      status_poll_count_(0),
      superseded_command_count_(0),
      skipped_output_sequence_count_(0),
      tick_count_(0),
      tick_deadline_miss_count_(0),
      clutch_cycle_count_(0),
      old_epoch_rejection_count_(0),
      last_tick_ns_(-1),
      maximum_tick_interval_ns_(0),
      maximum_command_age_ns_(0),
      maximum_resume_position_delta_rad_(0.0),
      last_feedback_call_start_ns_(-1),
      last_feedback_call_end_ns_(-1),
      last_feedback_sample_ns_(-1),
      last_feedback_validation_ns_(-1),
      last_feedback_sample_age_ns_(0),
      last_feedback_call_duration_ns_(0),
      maximum_feedback_sample_age_ns_(0),
      maximum_feedback_call_duration_ns_(0),
      refresh_epoch_(0),
      measured_{},
      pending_{},
      stopped_command_{},
      measurement_gate_(config.measurement) {}

bool ThinJakaTransportAdapter::ValidFunctionTable() const noexcept {
  return functions_.context != nullptr && functions_.login != nullptr &&
         functions_.set_edg_enabled != nullptr &&
         functions_.set_servo_enabled != nullptr &&
         functions_.send_joint_position != nullptr &&
         functions_.read_joint_feedback != nullptr &&
         functions_.read_status != nullptr && functions_.stop_motion != nullptr &&
         functions_.logout != nullptr;
}

ThinJakaCode ThinJakaTransportAdapter::Fault(
    ThinJakaCode code, teleop_command_abi::StopReason reason) noexcept {
  if (state_ != ThinJakaState::kFaulted && state_ != ThinJakaState::kCleanup &&
      state_ != ThinJakaState::kResetRequired) {
    fault_reason_ = reason;
    state_ = ThinJakaState::kFaulted;
    pending_valid_ = false;
    if (session_owned_ && !hard_stop_callback_called_) {
      hard_stop_callback_called_ = true;
      (void)functions_.stop_motion(functions_.context);
    }
  }
  return code;
}

ThinJakaCode ThinJakaTransportAdapter::Connect(std::int64_t) noexcept {
  if (state_ != ThinJakaState::kDisconnected || !ValidFunctionTable() ||
      config_.dof == 0U || config_.dof > teleop_command_abi::kMaxDof ||
      config_.measurement.dof != config_.dof ||
      config_.status_poll_interval_ticks == 0U || config_.servo_step_num == 0U ||
      config_.maximum_status_age_ns < 0 ||
      config_.maximum_tick_interval_ns <= 0 ||
      !std::isfinite(config_.resume_position_tolerance_rad) ||
      config_.resume_position_tolerance_rad < 0.0) {
    return ThinJakaCode::kInvalidConfiguration;
  }
  state_ = ThinJakaState::kConnecting;
  const auto result = functions_.login(functions_.context);
  if (!SdkOk(result)) {
    return Fault(ThinJakaCode::kSdkFailure,
                 teleop_command_abi::StopReason::kSdkFailure);
  }
  session_owned_ = true;
  state_ = ThinJakaState::kConnected;
  return ThinJakaCode::kOk;
}

ThinJakaCode ThinJakaTransportAdapter::PrepareServo(std::int64_t) noexcept {
  if (state_ != ThinJakaState::kConnected || !session_owned_) {
    return ThinJakaCode::kInvalidState;
  }
  if (!IsSupportedPausePolicy(config_.pause_policy) ||
      !IsSupportedResumePolicy(config_.resume_policy)) {
    return ThinJakaCode::kInvalidConfiguration;
  }
  if (config_.pause_policy ==
          PauseCommandPolicy::kRepeatStoppedPositionRequired &&
      config_.resume_policy != ResumePreparationPolicy::kKeepPrepared) {
    return ThinJakaCode::kInvalidConfiguration;
  }
  if (!SdkOk(functions_.set_edg_enabled(functions_.context, true))) {
    return Fault(ThinJakaCode::kSdkFailure,
                 teleop_command_abi::StopReason::kSdkFailure);
  }
  edg_enabled_ = true;
  if (!SdkOk(functions_.set_servo_enabled(functions_.context, true))) {
    return Fault(ThinJakaCode::kSdkFailure,
                 teleop_command_abi::StopReason::kSdkFailure);
  }
  servo_enabled_ = true;
  state_ = ThinJakaState::kServoReady;
  return ThinJakaCode::kOk;
}

ThinJakaCode ThinJakaTransportAdapter::ApplyResumePreparation() noexcept {
  if (config_.resume_policy == ResumePreparationPolicy::kKeepPrepared) {
    if (!edg_enabled_ || !servo_enabled_) {
      return Fault(ThinJakaCode::kControllerFault,
                   teleop_command_abi::StopReason::kControllerAlarm);
    }
    return ThinJakaCode::kOk;
  }
  if (config_.resume_policy == ResumePreparationPolicy::kRestartEdg ||
      config_.resume_policy == ResumePreparationPolicy::kRestartEdgAndServo) {
    if (!SdkOk(functions_.set_edg_enabled(functions_.context, true))) {
      return Fault(ThinJakaCode::kSdkFailure,
                   teleop_command_abi::StopReason::kSdkFailure);
    }
    edg_enabled_ = true;
    if (config_.resume_policy == ResumePreparationPolicy::kRestartEdgAndServo) {
      if (!SdkOk(functions_.set_servo_enabled(functions_.context, true))) {
        return Fault(ThinJakaCode::kSdkFailure,
                     teleop_command_abi::StopReason::kSdkFailure);
      }
      servo_enabled_ = true;
    }
    return ThinJakaCode::kOk;
  }
  return ThinJakaCode::kInvalidConfiguration;
}

ThinJakaCode ThinJakaTransportAdapter::BeginMeasuredStateRefresh(
    std::uint64_t new_safety_epoch, std::int64_t) noexcept {
  if ((state_ != ThinJakaState::kServoReady &&
       state_ != ThinJakaState::kStoppedReady) ||
      !session_owned_ || new_safety_epoch == 0U ||
      new_safety_epoch <= safety_epoch_) {
    return ThinJakaCode::kInvalidState;
  }
  if (state_ == ThinJakaState::kStoppedReady) {
    const auto preparation = ApplyResumePreparation();
    if (preparation != ThinJakaCode::kOk) return preparation;
  }
  refresh_epoch_ = new_safety_epoch;
  measurement_gate_.Reset();
  pending_valid_ = false;
  // When resuming a retained session, keep the already completed stopped
  // command available while fresh q/dq samples are collected.  The vendor
  // ServoJ contract asks clients to keep sending without gaps; clearing it
  // here would create an avoidable multi-tick hole during reference refresh.
  if (state_ == ThinJakaState::kServoReady) stopped_command_valid_ = false;
  state_ = ThinJakaState::kMeasuredStateRefresh;
  return ThinJakaCode::kOk;
}

ThinJakaCode ThinJakaTransportAdapter::RefreshMeasuredState(
    std::int64_t now_ns,
    teleop_command_abi::MeasuredJointStateV1* measured) noexcept {
  if (state_ != ThinJakaState::kMeasuredStateRefresh || measured == nullptr) {
    return ThinJakaCode::kInvalidState;
  }
  JakaJointFeedback feedback{};
  const auto io = functions_.read_joint_feedback(functions_.context, &feedback);
  if (!SdkOk(io)) {
    return Fault(io == JakaFunctionResult::kTimeout ? ThinJakaCode::kStale
                                                    : ThinJakaCode::kSdkFailure,
                 io == JakaFunctionResult::kTimeout
                     ? teleop_command_abi::StopReason::kStaleInput
                     : teleop_command_abi::StopReason::kSdkFailure);
  }
  const bool clock_regression =
      feedback.sdk_call_start_monotonic_ns < now_ns ||
      (last_feedback_validation_ns_ >= 0 &&
       feedback.validation_monotonic_ns <= last_feedback_validation_ns_);
  const bool invalid_order =
      feedback.sdk_call_start_monotonic_ns < 0 ||
      feedback.sdk_call_end_monotonic_ns <
          feedback.sdk_call_start_monotonic_ns ||
      feedback.sampled_monotonic_ns <
          feedback.sdk_call_start_monotonic_ns ||
      feedback.sampled_monotonic_ns > feedback.sdk_call_end_monotonic_ns ||
      feedback.validation_monotonic_ns <
          feedback.sdk_call_end_monotonic_ns;
  if (clock_regression || invalid_order) {
    return Fault(ThinJakaCode::kTimingFault,
                 teleop_command_abi::StopReason::kTimingFault);
  }
  const std::int64_t sample_age =
      feedback.validation_monotonic_ns - feedback.sampled_monotonic_ns;
  if (sample_age < 0 || sample_age > config_.measurement.maximum_sample_age_ns) {
    return Fault(ThinJakaCode::kStale,
                 teleop_command_abi::StopReason::kStaleInput);
  }
  last_feedback_call_start_ns_ = feedback.sdk_call_start_monotonic_ns;
  last_feedback_call_end_ns_ = feedback.sdk_call_end_monotonic_ns;
  last_feedback_sample_ns_ = feedback.sampled_monotonic_ns;
  last_feedback_validation_ns_ = feedback.validation_monotonic_ns;
  last_feedback_sample_age_ns_ = sample_age;
  last_feedback_call_duration_ns_ =
      feedback.sdk_call_end_monotonic_ns -
      feedback.sdk_call_start_monotonic_ns;
  maximum_feedback_sample_age_ns_ =
      std::max(maximum_feedback_sample_age_ns_, sample_age);
  maximum_feedback_call_duration_ns_ = std::max(
      maximum_feedback_call_duration_ns_, last_feedback_call_duration_ns_);
  FakeSdkJointSample sample{};
  sample.sample_sequence = feedback.sequence;
  sample.sampled_monotonic_ns = feedback.sampled_monotonic_ns;
  sample.dof = feedback.dof;
  sample.fields = JointSampleFields::kPositionVelocity;
  sample.position_rad = feedback.position_rad;
  sample.velocity_rad_s = feedback.velocity_rad_s;
  const auto result =
      measurement_gate_.Observe(sample, refresh_epoch_,
                                feedback.validation_monotonic_ns, measured);
  if (result.code == RecoveryMeasurementCode::kNeedMoreSamples) {
    return ThinJakaCode::kNeedMoreSamples;
  }
  if (result.code == RecoveryMeasurementCode::kUnstable) {
    return ThinJakaCode::kUnstableMeasurement;
  }
  if (result.code != RecoveryMeasurementCode::kReady) {
    return Fault(result.code == RecoveryMeasurementCode::kStale
                     ? ThinJakaCode::kStale
                     : ThinJakaCode::kInvalidCommand,
                 result.code == RecoveryMeasurementCode::kStale
                     ? teleop_command_abi::StopReason::kStaleInput
                     : teleop_command_abi::StopReason::kInvalidCommand);
  }
  measured_ = *measured;
  safety_epoch_ = refresh_epoch_;
  refresh_epoch_ = 0;
  last_output_sequence_ = 0;
  last_status_sequence_ = 0;
  state_ = ThinJakaState::kServoReady;
  return ThinJakaCode::kOk;
}

ThinJakaCode ThinJakaTransportAdapter::StartStreaming(
    const teleop_command_abi::ShapedJointCommandV1& first_command,
    std::int64_t now_ns) noexcept {
  if (state_ != ThinJakaState::kServoReady || safety_epoch_ == 0U) {
    return ThinJakaCode::kInvalidState;
  }
  teleop_command_abi::ValidationContext context{};
  context.now_ns = now_ns;
  context.expected_dof = config_.dof;
  context.expected_epoch = safety_epoch_;
  const auto validation = teleop_command_abi::Validate(first_command, context);
  if (!validation.ok ||
      first_command.output_mode !=
          teleop_command_abi::OutputMode::kActiveTracking) {
    return Fault(ThinJakaCode::kInvalidCommand,
                 teleop_command_abi::StopReason::kInvalidCommand);
  }
  double maximum_delta = 0.0;
  for (std::size_t i = 0; i < config_.dof; ++i) {
    maximum_delta = std::max(
        maximum_delta,
        std::abs(first_command.position_rad[i] - measured_.position_rad[i]));
  }
  maximum_resume_position_delta_rad_ =
      std::max(maximum_resume_position_delta_rad_, maximum_delta);
  if (maximum_delta > config_.resume_position_tolerance_rad) {
    return Fault(ThinJakaCode::kContinuityFault,
                 teleop_command_abi::StopReason::kInvalidCommand);
  }
  state_ = ThinJakaState::kStreaming;
  return Send(first_command, now_ns);
}

ThinJakaCode ThinJakaTransportAdapter::OfferLatest(
    const teleop_command_abi::ShapedJointCommandV1& command,
    std::int64_t now_ns) noexcept {
  if (state_ != ThinJakaState::kStreaming &&
      state_ != ThinJakaState::kControlledStopping) {
    return ThinJakaCode::kInvalidState;
  }
  teleop_command_abi::ValidationContext context{};
  context.now_ns = now_ns;
  context.expected_dof = config_.dof;
  context.expected_epoch = safety_epoch_;
  context.previous_sequence = last_output_sequence_;
  const auto validation = teleop_command_abi::Validate(command, context);
  if (!validation.ok) {
    if (validation.error == teleop_command_abi::ValidationError::kEpochMismatch) {
      ++old_epoch_rejection_count_;
      return Fault(ThinJakaCode::kEpochMismatch,
                   teleop_command_abi::StopReason::kEpochMismatch);
    }
    return Fault(validation.error ==
                         teleop_command_abi::ValidationError::kStaleValidityWindow
                     ? ThinJakaCode::kStale
                     : ThinJakaCode::kInvalidCommand,
                 validation.error ==
                         teleop_command_abi::ValidationError::kStaleValidityWindow
                     ? teleop_command_abi::StopReason::kStaleInput
                     : teleop_command_abi::StopReason::kInvalidCommand);
  }
  if (pending_valid_) {
    if (command.output_sequence <= pending_.output_sequence) {
      return Fault(ThinJakaCode::kInvalidCommand,
                   teleop_command_abi::StopReason::kInvalidCommand);
    }
    ++superseded_command_count_;
  }
  pending_ = command;
  pending_valid_ = true;
  return ThinJakaCode::kOk;
}

ThinJakaCode ThinJakaTransportAdapter::ApplyStoppedPreparation() noexcept {
  if (config_.resume_policy == ResumePreparationPolicy::kRestartEdgAndServo) {
    if (!SdkOk(functions_.set_servo_enabled(functions_.context, false))) {
      return Fault(ThinJakaCode::kSdkFailure,
                   teleop_command_abi::StopReason::kSdkFailure);
    }
    servo_enabled_ = false;
  }
  if (config_.resume_policy == ResumePreparationPolicy::kRestartEdg ||
      config_.resume_policy == ResumePreparationPolicy::kRestartEdgAndServo) {
    if (!SdkOk(functions_.set_edg_enabled(functions_.context, false))) {
      return Fault(ThinJakaCode::kSdkFailure,
                   teleop_command_abi::StopReason::kSdkFailure);
    }
    edg_enabled_ = false;
  }
  return ThinJakaCode::kOk;
}

ThinJakaCode ThinJakaTransportAdapter::Send(
    const teleop_command_abi::ShapedJointCommandV1& command,
    std::int64_t now_ns) noexcept {
  const auto result = functions_.send_joint_position(
      functions_.context, command.position_rad.data(), command.dof,
      config_.servo_step_num);
  if (!SdkOk(result)) {
    return Fault(result == JakaFunctionResult::kTimeout
                     ? ThinJakaCode::kTimingFault
                     : ThinJakaCode::kSdkFailure,
                 result == JakaFunctionResult::kTimeout
                     ? teleop_command_abi::StopReason::kTimingFault
                     : teleop_command_abi::StopReason::kSdkFailure);
  }
  if (last_output_sequence_ != 0U &&
      command.output_sequence > last_output_sequence_ + 1U) {
    skipped_output_sequence_count_ +=
        command.output_sequence - last_output_sequence_ - 1U;
  }
  last_output_sequence_ = command.output_sequence;
  ++sent_command_count_;
  maximum_command_age_ns_ = std::max(
      maximum_command_age_ns_, now_ns - command.generated_monotonic_ns);
  if (command.output_mode ==
      teleop_command_abi::OutputMode::kControlledBraking) {
    state_ = ThinJakaState::kControlledStopping;
  } else if (command.output_mode == teleop_command_abi::OutputMode::kStopped) {
    if (state_ != ThinJakaState::kControlledStopping) {
      return Fault(ThinJakaCode::kInvalidCommand,
                   teleop_command_abi::StopReason::kInvalidCommand);
    }
    stopped_command_ = command;
    stopped_command_valid_ = true;
    ++clutch_cycle_count_;
    state_ = ThinJakaState::kStoppedReady;
    return ApplyStoppedPreparation();
  }
  return ThinJakaCode::kOk;
}

ThinJakaCode ThinJakaTransportAdapter::RepeatStoppedCommand() noexcept {
  if (!stopped_command_valid_) return ThinJakaCode::kInvalidState;
  const auto result = functions_.send_joint_position(
      functions_.context, stopped_command_.position_rad.data(),
      stopped_command_.dof, config_.servo_step_num);
  if (!SdkOk(result)) {
    return Fault(result == JakaFunctionResult::kTimeout
                     ? ThinJakaCode::kTimingFault
                     : ThinJakaCode::kSdkFailure,
                 result == JakaFunctionResult::kTimeout
                     ? teleop_command_abi::StopReason::kTimingFault
                     : teleop_command_abi::StopReason::kSdkFailure);
  }
  ++repeated_stopped_command_count_;
  return ThinJakaCode::kOk;
}

ThinJakaCode ThinJakaTransportAdapter::Tick(std::int64_t now_ns) noexcept {
  if (state_ == ThinJakaState::kFaulted || state_ == ThinJakaState::kCleanup ||
      state_ == ThinJakaState::kResetRequired ||
      state_ == ThinJakaState::kDisconnected) {
    return ThinJakaCode::kInvalidState;
  }
  if (last_tick_ns_ >= 0) {
    if (now_ns <= last_tick_ns_) {
      ++tick_deadline_miss_count_;
      return Fault(ThinJakaCode::kTimingFault,
                   teleop_command_abi::StopReason::kTimingFault);
    }
    const auto interval = now_ns - last_tick_ns_;
    maximum_tick_interval_ns_ = std::max(maximum_tick_interval_ns_, interval);
    if (interval > config_.maximum_tick_interval_ns) {
      ++tick_deadline_miss_count_;
      return Fault(ThinJakaCode::kTimingFault,
                   teleop_command_abi::StopReason::kTimingFault);
    }
  }
  last_tick_ns_ = now_ns;
  ++tick_count_;

  if (tick_count_ % config_.status_poll_interval_ticks == 0U) {
    const auto status = PollStatus(now_ns);
    if (status != ThinJakaCode::kOk) return status;
  }
  if ((state_ == ThinJakaState::kStoppedReady ||
       state_ == ThinJakaState::kMeasuredStateRefresh) &&
      config_.pause_policy ==
          PauseCommandPolicy::kRepeatStoppedPositionRequired &&
      stopped_command_valid_) {
    return RepeatStoppedCommand();
  }
  if (pending_valid_) {
    const auto command = pending_;
    pending_valid_ = false;
    return Send(command, now_ns);
  }
  return ThinJakaCode::kOk;
}

ThinJakaCode ThinJakaTransportAdapter::PollStatus(std::int64_t now_ns) noexcept {
  if (!session_owned_) return ThinJakaCode::kInvalidState;
  JakaNormalizedStatus status{};
  const auto result = functions_.read_status(functions_.context, &status);
  ++status_poll_count_;
  if (!SdkOk(result)) {
    return Fault(result == JakaFunctionResult::kTimeout
                     ? ThinJakaCode::kStale
                     : ThinJakaCode::kSdkFailure,
                 result == JakaFunctionResult::kTimeout
                     ? teleop_command_abi::StopReason::kStaleInput
                     : teleop_command_abi::StopReason::kSdkFailure);
  }
  const bool invalid_timing =
      status.sdk_call_start_monotonic_ns < now_ns ||
      status.sdk_call_end_monotonic_ns < status.sdk_call_start_monotonic_ns ||
      status.sampled_monotonic_ns < status.sdk_call_start_monotonic_ns ||
      status.sampled_monotonic_ns > status.sdk_call_end_monotonic_ns ||
      status.validation_monotonic_ns < status.sdk_call_end_monotonic_ns;
  if (invalid_timing) {
    return Fault(ThinJakaCode::kTimingFault,
                 teleop_command_abi::StopReason::kTimingFault);
  }
  if (status.sequence == 0U || status.sequence <= last_status_sequence_ ||
      status.sampled_monotonic_ns < 0 ||
      status.sampled_monotonic_ns > status.validation_monotonic_ns ||
      status.validation_monotonic_ns - status.sampled_monotonic_ns >
          config_.maximum_status_age_ns) {
    return Fault(ThinJakaCode::kStale,
                 teleop_command_abi::StopReason::kStaleInput);
  }
  last_status_sequence_ = status.sequence;
  if (!status.session_alive) {
    return Fault(ThinJakaCode::kSdkFailure,
                 teleop_command_abi::StopReason::kSdkFailure);
  }
  if (status.estop) {
    return Fault(ThinJakaCode::kControllerFault,
                 teleop_command_abi::StopReason::kEstop);
  }
  if (status.collision) {
    return Fault(ThinJakaCode::kControllerFault,
                 teleop_command_abi::StopReason::kCollision);
  }
  if (status.alarm || !status.powered_on ||
      (servo_enabled_ && !status.servo_enabled) ||
      (edg_enabled_ && !status.edg_ready)) {
    return Fault(ThinJakaCode::kControllerFault,
                 teleop_command_abi::StopReason::kControllerAlarm);
  }
  return ThinJakaCode::kOk;
}

void ThinJakaTransportAdapter::HardStop(
    teleop_command_abi::StopReason reason, std::int64_t) noexcept {
  (void)Fault(ThinJakaCode::kControllerFault,
              reason == teleop_command_abi::StopReason::kNone
                  ? teleop_command_abi::StopReason::kInvalidCommand
                  : reason);
}

ThinJakaCode ThinJakaTransportAdapter::Cleanup(std::int64_t) noexcept {
  if (state_ != ThinJakaState::kFaulted &&
      state_ != ThinJakaState::kStoppedReady &&
      state_ != ThinJakaState::kServoReady &&
      state_ != ThinJakaState::kConnected) {
    return ThinJakaCode::kInvalidState;
  }
  state_ = ThinJakaState::kCleanup;
  bool failed = false;
  if (servo_enabled_) {
    const bool disabled =
        SdkOk(functions_.set_servo_enabled(functions_.context, false));
    failed = !disabled || failed;
    if (disabled) servo_enabled_ = false;
  }
  if (edg_enabled_) {
    const bool disabled =
        SdkOk(functions_.set_edg_enabled(functions_.context, false));
    failed = !disabled || failed;
    if (disabled) edg_enabled_ = false;
  }
  if (session_owned_) {
    const bool logged_out = SdkOk(functions_.logout(functions_.context));
    failed = !logged_out || failed;
    if (logged_out) session_owned_ = false;
  }
  cleanup_failed_ = failed;
  state_ = ThinJakaState::kResetRequired;
  return failed ? ThinJakaCode::kCleanupFailure : ThinJakaCode::kOk;
}

ThinJakaCode ThinJakaTransportAdapter::ExplicitReset(std::int64_t) noexcept {
  if (state_ != ThinJakaState::kResetRequired || cleanup_failed_ ||
      session_owned_) {
    return ThinJakaCode::kInvalidState;
  }
  state_ = ThinJakaState::kDisconnected;
  fault_reason_ = teleop_command_abi::StopReason::kNone;
  hard_stop_callback_called_ = false;
  safety_epoch_ = 0;
  refresh_epoch_ = 0;
  last_output_sequence_ = 0;
  last_status_sequence_ = 0;
  pending_valid_ = false;
  stopped_command_valid_ = false;
  last_tick_ns_ = -1;
  return ThinJakaCode::kOk;
}

ThinJakaSnapshot ThinJakaTransportAdapter::Snapshot() const noexcept {
  return {state_,
          fault_reason_,
          session_owned_,
          edg_enabled_,
          servo_enabled_,
          cleanup_failed_,
          safety_epoch_,
          last_output_sequence_,
          sent_command_count_,
          repeated_stopped_command_count_,
          status_poll_count_,
          superseded_command_count_,
          skipped_output_sequence_count_,
          tick_count_,
          tick_deadline_miss_count_,
          clutch_cycle_count_,
          old_epoch_rejection_count_,
          maximum_tick_interval_ns_,
          maximum_command_age_ns_,
          maximum_resume_position_delta_rad_,
          last_feedback_call_start_ns_,
          last_feedback_call_end_ns_,
          last_feedback_sample_ns_,
          last_feedback_validation_ns_,
          last_feedback_sample_age_ns_,
          last_feedback_call_duration_ns_,
          maximum_feedback_sample_age_ns_,
          maximum_feedback_call_duration_ns_};
}

}  // namespace teleop_shaping
