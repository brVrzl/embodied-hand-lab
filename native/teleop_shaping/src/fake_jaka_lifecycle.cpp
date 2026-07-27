#include "teleop_shaping/fake_jaka_lifecycle.hpp"

#include <cassert>

namespace teleop_shaping {
namespace {

constexpr teleop_command_abi::ValidationResult Valid() noexcept {
  return {true, teleop_command_abi::ValidationError::kOk,
          teleop_command_abi::ValidationField::kNone,
          static_cast<std::uint8_t>(teleop_command_abi::kNoFieldIndex)};
}

bool IsActiveLifecycle(FakeJakaLifecycleState state) noexcept {
  return state == FakeJakaLifecycleState::kConnected ||
         state == FakeJakaLifecycleState::kServoReady ||
         state == FakeJakaLifecycleState::kStreaming ||
         state == FakeJakaLifecycleState::kControlledStopping ||
         state == FakeJakaLifecycleState::kStopped;
}

}  // namespace

FakeJakaLifecycleAdapter::FakeJakaLifecycleAdapter() noexcept
    : lifecycle_state_(FakeJakaLifecycleState::kDisconnected),
      dof_(0),
      session_owned_(false),
      hard_stop_latched_(false),
      safety_epoch_(0),
      last_output_sequence_(0),
      last_health_sequence_(0),
      accepted_command_count_(0),
      rejected_command_count_(0),
      skipped_output_sequence_count_(0),
      telemetry_overflow_count_(0),
      telemetry_record_sequence_(0),
      fault_reason_(teleop_command_abi::StopReason::kNone),
      telemetry_{},
      telemetry_size_(0),
      telemetry_next_(0),
      has_terminal_fault_record_(false),
      terminal_fault_record_{} {}

void FakeJakaLifecycleAdapter::Record(
    FakeTelemetryEvent event, FakeLifecycleCode result, std::int64_t now_ns,
    const teleop_command_abi::ShapedJointCommandV1* command,
    teleop_command_abi::ValidationResult validation) noexcept {
  FakeJakaTelemetryRecord record{};
  record.record_sequence = ++telemetry_record_sequence_;
  record.monotonic_ns = now_ns;
  record.lifecycle_state = lifecycle_state_;
  record.event = event;
  record.result = result;
  record.output_mode = command == nullptr
                           ? teleop_command_abi::OutputMode::kInactive
                           : command->output_mode;
  record.output_sequence = command == nullptr ? 0U : command->output_sequence;
  record.source_sequence = command == nullptr ? 0U : command->source_sequence;
  record.safety_epoch = command == nullptr ? safety_epoch_ : command->safety_epoch;
  record.command_age_ns = command == nullptr ? 0 : now_ns - command->generated_monotonic_ns;
  record.deadline_slack_ns =
      command == nullptr ? 0 : command->valid_until_monotonic_ns - now_ns;
  record.stop_reason = fault_reason_;
  record.validation_error = validation.error;
  record.validation_index = validation.index;
  telemetry_[telemetry_next_] = record;
  telemetry_next_ = (telemetry_next_ + 1U) % kTelemetryCapacity;
  if (telemetry_size_ < kTelemetryCapacity) {
    ++telemetry_size_;
  } else {
    ++telemetry_overflow_count_;
  }
  if (event == FakeTelemetryEvent::kFaultLatched) {
    terminal_fault_record_ = record;
    has_terminal_fault_record_ = true;
  }
}

FakeLifecycleCode FakeJakaLifecycleAdapter::Fault(
    FakeLifecycleCode code, teleop_command_abi::StopReason reason,
    std::int64_t now_ns,
    teleop_command_abi::ValidationResult validation,
    const teleop_command_abi::ShapedJointCommandV1* command) noexcept {
  if (!hard_stop_latched_) {
    hard_stop_latched_ = true;
    lifecycle_state_ = FakeJakaLifecycleState::kFaulted;
    fault_reason_ = reason;
    ++rejected_command_count_;
    Record(FakeTelemetryEvent::kFaultLatched, code, now_ns, command, validation);
  }
  return code;
}

FakeLifecycleCode FakeJakaLifecycleAdapter::BeginConnect(std::int64_t now_ns) noexcept {
  if (lifecycle_state_ != FakeJakaLifecycleState::kDisconnected || session_owned_ ||
      hard_stop_latched_) {
    return FakeLifecycleCode::kInvalidState;
  }
  lifecycle_state_ = FakeJakaLifecycleState::kConnecting;
  Record(FakeTelemetryEvent::kLifecycle, FakeLifecycleCode::kOk, now_ns, nullptr,
         Valid());
  return FakeLifecycleCode::kOk;
}

FakeLifecycleCode FakeJakaLifecycleAdapter::CompleteConnect(
    bool success, std::int64_t now_ns) noexcept {
  if (lifecycle_state_ != FakeJakaLifecycleState::kConnecting) {
    return FakeLifecycleCode::kInvalidState;
  }
  if (!success) {
    return Fault(FakeLifecycleCode::kTransportFailure,
                 teleop_command_abi::StopReason::kSdkFailure, now_ns, Valid());
  }
  session_owned_ = true;
  lifecycle_state_ = FakeJakaLifecycleState::kConnected;
  Record(FakeTelemetryEvent::kLifecycle, FakeLifecycleCode::kOk, now_ns, nullptr,
         Valid());
  return FakeLifecycleCode::kOk;
}

FakeLifecycleCode FakeJakaLifecycleAdapter::PrepareServo(std::int64_t now_ns) noexcept {
  if (lifecycle_state_ != FakeJakaLifecycleState::kConnected || !session_owned_ ||
      hard_stop_latched_) {
    return FakeLifecycleCode::kInvalidState;
  }
  lifecycle_state_ = FakeJakaLifecycleState::kServoReady;
  Record(FakeTelemetryEvent::kLifecycle, FakeLifecycleCode::kOk, now_ns, nullptr,
         Valid());
  return FakeLifecycleCode::kOk;
}

FakeLifecycleCode FakeJakaLifecycleAdapter::ArmEpoch(
    const teleop_command_abi::MeasuredJointStateV1& measured,
    std::int64_t now_ns) noexcept {
  if ((lifecycle_state_ != FakeJakaLifecycleState::kServoReady &&
       lifecycle_state_ != FakeJakaLifecycleState::kStopped) ||
      hard_stop_latched_ || !session_owned_) {
    return FakeLifecycleCode::kInvalidState;
  }
  teleop_command_abi::ValidationContext context{};
  context.now_ns = now_ns;
  const auto validation = teleop_command_abi::Validate(measured, context);
  if (!validation.ok) {
    return Fault(FakeLifecycleCode::kInvalidCommand,
                 teleop_command_abi::StopReason::kInvalidCommand, now_ns,
                 validation);
  }
  if (measured.safety_epoch <= safety_epoch_) {
    return Fault(FakeLifecycleCode::kEpochMismatch,
                 teleop_command_abi::StopReason::kEpochMismatch, now_ns,
                 validation);
  }
  dof_ = measured.dof;
  safety_epoch_ = measured.safety_epoch;
  last_output_sequence_ = 0;
  last_health_sequence_ = 0;
  lifecycle_state_ = FakeJakaLifecycleState::kServoReady;
  Record(FakeTelemetryEvent::kEpochArmed, FakeLifecycleCode::kOk, now_ns, nullptr,
         Valid());
  return FakeLifecycleCode::kOk;
}

FakeLifecycleCode FakeJakaLifecycleAdapter::StartStreaming(
    std::uint64_t safety_epoch, std::int64_t now_ns) noexcept {
  if (lifecycle_state_ != FakeJakaLifecycleState::kServoReady ||
      hard_stop_latched_ || safety_epoch == 0U || safety_epoch != safety_epoch_) {
    return FakeLifecycleCode::kInvalidState;
  }
  lifecycle_state_ = FakeJakaLifecycleState::kStreaming;
  Record(FakeTelemetryEvent::kLifecycle, FakeLifecycleCode::kOk, now_ns, nullptr,
         Valid());
  return FakeLifecycleCode::kOk;
}

FakeLifecycleCode FakeJakaLifecycleAdapter::Send(
    const teleop_command_abi::ShapedJointCommandV1& command,
    FakeSendOutcome outcome, std::int64_t now_ns) noexcept {
  if (hard_stop_latched_ || lifecycle_state_ == FakeJakaLifecycleState::kFaulted) {
    return FakeLifecycleCode::kHardStopped;
  }
  if (lifecycle_state_ != FakeJakaLifecycleState::kStreaming &&
      lifecycle_state_ != FakeJakaLifecycleState::kControlledStopping) {
    return FakeLifecycleCode::kInvalidState;
  }
  teleop_command_abi::ValidationContext context{};
  context.now_ns = now_ns;
  context.expected_dof = dof_;
  context.expected_epoch = safety_epoch_;
  context.previous_sequence = last_output_sequence_;
  const auto validation = teleop_command_abi::Validate(command, context);
  if (!validation.ok) {
    FakeLifecycleCode code = FakeLifecycleCode::kInvalidCommand;
    teleop_command_abi::StopReason reason =
        teleop_command_abi::StopReason::kInvalidCommand;
    if (validation.error == teleop_command_abi::ValidationError::kEpochMismatch) {
      code = FakeLifecycleCode::kEpochMismatch;
      reason = teleop_command_abi::StopReason::kEpochMismatch;
    } else if (validation.error ==
               teleop_command_abi::ValidationError::kStaleValidityWindow) {
      code = FakeLifecycleCode::kStaleCommand;
      reason = teleop_command_abi::StopReason::kStaleInput;
    } else if (validation.error ==
               teleop_command_abi::ValidationError::kInvalidSequence) {
      code = FakeLifecycleCode::kDuplicateOrOldSequence;
    }
    return Fault(code, reason, now_ns, validation, &command);
  }
  if (command.output_mode == teleop_command_abi::OutputMode::kActiveTracking) {
    if (lifecycle_state_ != FakeJakaLifecycleState::kStreaming) {
      return Fault(FakeLifecycleCode::kInvalidCommand,
                   teleop_command_abi::StopReason::kInvalidCommand, now_ns,
                   validation, &command);
    }
  } else if (command.output_mode ==
             teleop_command_abi::OutputMode::kControlledBraking) {
    lifecycle_state_ = FakeJakaLifecycleState::kControlledStopping;
  } else if (command.output_mode == teleop_command_abi::OutputMode::kStopped) {
    if (lifecycle_state_ != FakeJakaLifecycleState::kControlledStopping) {
      return Fault(FakeLifecycleCode::kInvalidCommand,
                   teleop_command_abi::StopReason::kInvalidCommand, now_ns,
                   validation, &command);
    }
    lifecycle_state_ = FakeJakaLifecycleState::kStopped;
  } else {
    return Fault(FakeLifecycleCode::kInvalidCommand,
                 teleop_command_abi::StopReason::kInvalidCommand, now_ns,
                 validation, &command);
  }
  if (outcome != FakeSendOutcome::kOk) {
    if (outcome == FakeSendOutcome::kRejected) {
      return Fault(FakeLifecycleCode::kSendRejected,
                   teleop_command_abi::StopReason::kInvalidCommand, now_ns,
                   validation, &command);
    }
    if (outcome == FakeSendOutcome::kControllerAlarm) {
      return Fault(FakeLifecycleCode::kControllerFault,
                   teleop_command_abi::StopReason::kControllerAlarm, now_ns,
                   validation, &command);
    }
    return Fault(FakeLifecycleCode::kTransportFailure,
                 teleop_command_abi::StopReason::kSdkFailure, now_ns,
                 validation, &command);
  }
  if (last_output_sequence_ != 0U &&
      command.output_sequence > last_output_sequence_ + 1U) {
    skipped_output_sequence_count_ +=
        command.output_sequence - last_output_sequence_ - 1U;
  }
  last_output_sequence_ = command.output_sequence;
  ++accepted_command_count_;
  Record(FakeTelemetryEvent::kCommandAccepted, FakeLifecycleCode::kOk, now_ns,
         &command, validation);
  return FakeLifecycleCode::kOk;
}

FakeLifecycleCode FakeJakaLifecycleAdapter::SampleHealth(
    const teleop_command_abi::TransportHealthV1& health,
    std::int64_t now_ns) noexcept {
  if (hard_stop_latched_) return FakeLifecycleCode::kHardStopped;
  if (!IsActiveLifecycle(lifecycle_state_)) return FakeLifecycleCode::kInvalidState;
  teleop_command_abi::ValidationContext context{};
  context.now_ns = now_ns;
  context.expected_epoch = safety_epoch_;
  context.previous_sequence = last_health_sequence_;
  const auto validation = teleop_command_abi::Validate(health, context);
  if (!validation.ok) {
    return Fault(FakeLifecycleCode::kInvalidCommand,
                 validation.error == teleop_command_abi::ValidationError::kEpochMismatch
                     ? teleop_command_abi::StopReason::kEpochMismatch
                     : teleop_command_abi::StopReason::kInvalidCommand,
                 now_ns, validation);
  }
  last_health_sequence_ = health.health_sequence;
  if (health.estop != 0U) {
    return Fault(FakeLifecycleCode::kControllerFault,
                 teleop_command_abi::StopReason::kEstop, now_ns, validation);
  }
  if (health.collision != 0U) {
    return Fault(FakeLifecycleCode::kControllerFault,
                 teleop_command_abi::StopReason::kCollision, now_ns, validation);
  }
  if (health.alarm != 0U ||
      health.controller_state == teleop_command_abi::ControllerState::kAlarm) {
    return Fault(FakeLifecycleCode::kControllerFault,
                 teleop_command_abi::StopReason::kControllerAlarm, now_ns,
                 validation);
  }
  if (health.producer_stale != 0U || health.command_stale != 0U) {
    return Fault(FakeLifecycleCode::kStaleCommand,
                 teleop_command_abi::StopReason::kStaleInput, now_ns, validation);
  }
  if (health.deadline_missed != 0U) {
    return Fault(FakeLifecycleCode::kStaleCommand,
                 teleop_command_abi::StopReason::kTimingFault, now_ns, validation);
  }
  if (health.transport_state == teleop_command_abi::TransportState::kFaulted) {
    return Fault(FakeLifecycleCode::kTransportFailure,
                 teleop_command_abi::StopReason::kSdkFailure, now_ns, validation);
  }
  if (lifecycle_state_ == FakeJakaLifecycleState::kStreaming &&
      (health.controller_state != teleop_command_abi::ControllerState::kReady ||
       health.transport_state != teleop_command_abi::TransportState::kReady ||
       health.servo_enabled == 0U)) {
    return Fault(FakeLifecycleCode::kControllerFault,
                 teleop_command_abi::StopReason::kControllerAlarm, now_ns,
                 validation);
  }
  Record(FakeTelemetryEvent::kHealthAccepted, FakeLifecycleCode::kOk, now_ns,
         nullptr, validation);
  return FakeLifecycleCode::kOk;
}

void FakeJakaLifecycleAdapter::InjectFault(
    teleop_command_abi::StopReason reason, std::int64_t now_ns) noexcept {
  Fault(FakeLifecycleCode::kControllerFault,
        reason == teleop_command_abi::StopReason::kNone
            ? teleop_command_abi::StopReason::kInvalidCommand
            : reason,
        now_ns, Valid());
}

FakeLifecycleCode FakeJakaLifecycleAdapter::BeginCleanup(std::int64_t now_ns) noexcept {
  if (lifecycle_state_ != FakeJakaLifecycleState::kFaulted &&
      lifecycle_state_ != FakeJakaLifecycleState::kStopped &&
      lifecycle_state_ != FakeJakaLifecycleState::kConnected &&
      lifecycle_state_ != FakeJakaLifecycleState::kServoReady) {
    return FakeLifecycleCode::kInvalidState;
  }
  lifecycle_state_ = FakeJakaLifecycleState::kCleaningUp;
  Record(FakeTelemetryEvent::kCleanup, FakeLifecycleCode::kOk, now_ns, nullptr,
         Valid());
  return FakeLifecycleCode::kOk;
}

FakeLifecycleCode FakeJakaLifecycleAdapter::CompleteCleanup(
    std::int64_t now_ns) noexcept {
  if (lifecycle_state_ != FakeJakaLifecycleState::kCleaningUp) {
    return FakeLifecycleCode::kInvalidState;
  }
  session_owned_ = false;
  lifecycle_state_ = FakeJakaLifecycleState::kDisconnected;
  hard_stop_latched_ = false;
  safety_epoch_ = 0;
  dof_ = 0;
  last_output_sequence_ = 0;
  last_health_sequence_ = 0;
  fault_reason_ = teleop_command_abi::StopReason::kNone;
  Record(FakeTelemetryEvent::kCleanup, FakeLifecycleCode::kOk, now_ns, nullptr,
         Valid());
  return FakeLifecycleCode::kOk;
}

FakeJakaLifecycleSnapshot FakeJakaLifecycleAdapter::Snapshot() const noexcept {
  return {lifecycle_state_,
          dof_,
          session_owned_,
          hard_stop_latched_,
          safety_epoch_,
          last_output_sequence_,
          last_health_sequence_,
          accepted_command_count_,
          rejected_command_count_,
          skipped_output_sequence_count_,
          telemetry_overflow_count_,
          fault_reason_};
}

const FakeJakaTelemetryRecord& FakeJakaLifecycleAdapter::telemetry(
    std::size_t index) const noexcept {
  assert(index < telemetry_size_);
  const std::size_t oldest =
      telemetry_size_ < kTelemetryCapacity ? 0U : telemetry_next_;
  return telemetry_[(oldest + index) % kTelemetryCapacity];
}

const FakeJakaTelemetryRecord&
FakeJakaLifecycleAdapter::terminal_fault_record() const noexcept {
  assert(has_terminal_fault_record_);
  return terminal_fault_record_;
}

}  // namespace teleop_shaping
