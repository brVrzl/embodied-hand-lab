#include "teleop_shaping/joint_shaper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace teleop_shaping {
namespace {

using teleop_command_abi::StopClass;
using teleop_command_abi::TargetValidity;
using teleop_command_abi::ValidationContext;
using teleop_command_abi::ValidationError;
using teleop_command_abi::ValidationField;

constexpr double kPeriodS = static_cast<double>(kReferencePeriodNs) / 1e9;
constexpr double kNumericalEpsilon = 1e-12;

constexpr ValidationResult Valid() noexcept {
  return {true, ValidationError::kOk, ValidationField::kNone,
          static_cast<std::uint8_t>(teleop_command_abi::kNoFieldIndex)};
}

constexpr ValidationResult Invalid(ValidationError error, ValidationField field,
                                   std::size_t index = teleop_command_abi::kNoFieldIndex) noexcept {
  return {false, error, field, static_cast<std::uint8_t>(index)};
}

constexpr OperationResult Result(OperationCode code,
                                 ValidationResult validation = Valid()) noexcept {
  return {code, validation};
}

double Clip(double value, double magnitude) noexcept {
  return std::max(-magnitude, std::min(magnitude, value));
}

bool IsControlledReason(StopReason reason) noexcept {
  return reason == StopReason::kClutchRelease;
}

void IntegrateConstantJerk(double jerk, double dt, double* position, double* velocity,
                          double* acceleration) noexcept {
  *position += *velocity * dt + 0.5 * *acceleration * dt * dt +
               jerk * dt * dt * dt / 6.0;
  *velocity += *acceleration * dt + 0.5 * jerk * dt * dt;
  *acceleration += jerk * dt;
}

}  // namespace

ReferenceJointShaperV1::ReferenceJointShaperV1() noexcept
    : mode_(ShaperMode::kUninitialized),
      dof_(0),
      safety_epoch_(0),
      last_input_sequence_(0),
      source_sequence_(0),
      output_sequence_(0),
      release_sequence_(0),
      last_tick_ns_(-1),
      liveness_monotonic_ns_(-1),
      target_valid_until_ns_(-1),
      last_target_source_ns_(-1),
      stop_reason_(StopReason::kNone),
      limits_{},
      position_{},
      velocity_{},
      acceleration_{},
      target_{},
      target_velocity_{},
      brake_{} {}

OperationResult ReferenceJointShaperV1::Initialize(
    const MeasuredJointStateV1& measured, const JointDynamicLimitsV1& limits,
    std::int64_t now_ns) noexcept {
  ValidationContext context{};
  context.now_ns = now_ns;
  ValidationResult validation = teleop_command_abi::Validate(measured, context);
  if (!validation.ok) return Result(OperationCode::kInvalidArgument, validation);
  validation = teleop_command_abi::Validate(limits);
  if (!validation.ok) return Result(OperationCode::kInvalidArgument, validation);
  if (limits.dof != measured.dof) {
    return Result(OperationCode::kInvalidArgument,
                  Invalid(ValidationError::kInvalidDof, ValidationField::kDof));
  }
  for (std::size_t i = 0; i < measured.dof; ++i) {
    if (measured.position_rad[i] < limits.minimum_position_rad[i] ||
        measured.position_rad[i] > limits.maximum_position_rad[i]) {
      return Result(OperationCode::kInvalidArgument,
                    Invalid(ValidationError::kInvalidLimits, ValidationField::kPosition, i));
    }
    if (std::abs(measured.velocity_rad_s[i]) > limits.maximum_velocity_rad_s[i] ||
        std::abs(measured.acceleration_rad_s2[i]) >
            limits.maximum_acceleration_rad_s2[i]) {
      return Result(OperationCode::kInvalidArgument,
                    Invalid(ValidationError::kInvalidLimits, ValidationField::kVelocity, i));
    }
  }

  mode_ = ShaperMode::kActiveTracking;
  dof_ = measured.dof;
  safety_epoch_ = measured.safety_epoch;
  last_input_sequence_ = 0;
  source_sequence_ = 0;
  output_sequence_ = 0;
  release_sequence_ = 0;
  last_tick_ns_ = now_ns - kReferencePeriodNs;
  liveness_monotonic_ns_ = measured.measured_monotonic_ns;
  target_valid_until_ns_ = -1;
  last_target_source_ns_ = -1;
  stop_reason_ = StopReason::kNone;
  limits_ = limits;
  position_ = measured.position_rad;
  velocity_ = measured.velocity_rad_s;
  acceleration_ = measured.acceleration_rad_s2;
  target_ = measured.position_rad;
  target_velocity_.fill(0.0);
  brake_.fill(BrakeAxis{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0U, 0.0, true});
  return Result(OperationCode::kOk);
}

OperationResult ReferenceJointShaperV1::FailClosed(ValidationResult validation,
                                                   StopReason reason,
                                                   std::int64_t now_ns) noexcept {
  HardStop(reason, now_ns);
  return Result(OperationCode::kInvalidArgument, validation);
}

OperationResult ReferenceJointShaperV1::ReplaceTarget(
    const AcceptedJointTargetV1& target, std::int64_t now_ns) noexcept {
  if (mode_ != ShaperMode::kActiveTracking) {
    return Result(OperationCode::kInvalidState);
  }
  ValidationContext context{};
  context.now_ns = now_ns;
  context.expected_dof = dof_;
  context.expected_epoch = safety_epoch_;
  context.previous_sequence = last_input_sequence_;
  ValidationResult validation = teleop_command_abi::Validate(target, context);
  if (!validation.ok) {
    const StopReason reason = validation.error == ValidationError::kEpochMismatch
                                  ? StopReason::kEpochMismatch
                                  : StopReason::kInvalidCommand;
    return FailClosed(validation, reason, now_ns);
  }

  last_input_sequence_ = target.sequence;
  liveness_monotonic_ns_ = target.accepted_monotonic_ns;
  if (target.validity == TargetValidity::kRejectedKeepPrevious) {
    return Result(OperationCode::kOk);
  }
  if (target.validity != TargetValidity::kAccepted) {
    return FailClosed(
        Invalid(ValidationError::kInvalidModeContract, ValidationField::kValidity),
        StopReason::kInvalidCommand, now_ns);
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    if (target.position_rad[i] < limits_.minimum_position_rad[i] ||
        target.position_rad[i] > limits_.maximum_position_rad[i]) {
      return FailClosed(
          Invalid(ValidationError::kInvalidLimits, ValidationField::kPosition, i),
          StopReason::kInvalidCommand, now_ns);
    }
  }
  if (last_target_source_ns_ >= 0) {
    const std::int64_t delta_ns = target.source_monotonic_ns - last_target_source_ns_;
    if (delta_ns <= 0) {
      return FailClosed(
          Invalid(ValidationError::kTimestampOrdering, ValidationField::kSourceTimestamp),
          StopReason::kInvalidCommand, now_ns);
    }
    const double delta_s = static_cast<double>(delta_ns) / 1e9;
    for (std::size_t i = 0; i < dof_; ++i) {
      target_velocity_[i] = (target.position_rad[i] - target_[i]) / delta_s;
    }
  } else {
    target_velocity_.fill(0.0);
  }
  target_ = target.position_rad;
  source_sequence_ = target.sequence;
  target_valid_until_ns_ = target.valid_until_monotonic_ns;
  last_target_source_ns_ = target.source_monotonic_ns;
  return Result(OperationCode::kOk);
}

bool ReferenceJointShaperV1::PlanBrakeAxis(std::size_t axis, BrakeAxis* plan) noexcept {
  if (plan == nullptr) return false;
  *plan = BrakeAxis{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0U, 0.0, false};
  const double velocity = velocity_[axis];
  const double acceleration = acceleration_[axis];
  if (std::abs(velocity) <= kNumericalEpsilon) {
    if (std::abs(acceleration) <= kNumericalEpsilon) {
      plan->complete = true;
      return true;
    }
    return false;
  }

  const double sign = std::copysign(1.0, velocity);
  const double normalized_velocity = sign * velocity;
  const double normalized_acceleration = sign * acceleration;
  const double maximum_acceleration = limits_.maximum_acceleration_rad_s2[axis];
  const double maximum_jerk = limits_.maximum_jerk_rad_s3[axis];
  const double triangular_peak =
      std::sqrt(0.5 * normalized_acceleration * normalized_acceleration +
                maximum_jerk * normalized_velocity);
  double first_duration = 0.0;
  double hold_duration = 0.0;
  double final_duration = 0.0;
  if (triangular_peak <= maximum_acceleration) {
    first_duration = (normalized_acceleration + triangular_peak) / maximum_jerk;
    final_duration = triangular_peak / maximum_jerk;
  } else {
    first_duration = (normalized_acceleration + maximum_acceleration) / maximum_jerk;
    final_duration = maximum_acceleration / maximum_jerk;
    const double first_delta_velocity =
        0.5 * (normalized_acceleration - maximum_acceleration) * first_duration;
    const double final_delta_velocity = -0.5 * maximum_acceleration * final_duration;
    hold_duration =
        (normalized_velocity + first_delta_velocity + final_delta_velocity) /
        maximum_acceleration;
  }
  if (!std::isfinite(first_duration) || !std::isfinite(hold_duration) ||
      !std::isfinite(final_duration) || first_duration < -kNumericalEpsilon ||
      hold_duration < -kNumericalEpsilon || final_duration < -kNumericalEpsilon) {
    return false;
  }
  plan->duration_s = {std::max(0.0, first_duration), std::max(0.0, hold_duration),
                      std::max(0.0, final_duration)};
  plan->jerk_rad_s3 = {-sign * maximum_jerk, 0.0, sign * maximum_jerk};

  double predicted_position = position_[axis];
  double predicted_velocity = velocity;
  double predicted_acceleration = acceleration;
  for (std::size_t phase = 0; phase < plan->duration_s.size(); ++phase) {
    IntegrateConstantJerk(plan->jerk_rad_s3[phase], plan->duration_s[phase],
                          &predicted_position, &predicted_velocity,
                          &predicted_acceleration);
  }
  const double velocity_tolerance = 1e-8 * std::max(1.0, std::abs(velocity));
  const double acceleration_tolerance =
      1e-8 * std::max(1.0, std::abs(acceleration));
  if (!std::isfinite(predicted_position) ||
      std::abs(predicted_velocity) > velocity_tolerance ||
      std::abs(predicted_acceleration) > acceleration_tolerance ||
      predicted_position < limits_.minimum_position_rad[axis] ||
      predicted_position > limits_.maximum_position_rad[axis]) {
    return false;
  }
  return true;
}

bool ReferenceJointShaperV1::SynchronizeBrakeAxis(std::size_t axis, double duration_s,
                                                  BrakeAxis* plan) noexcept {
  if (plan == nullptr || !(duration_s > 0.0) || !std::isfinite(duration_s)) return false;
  const double velocity = velocity_[axis];
  const double acceleration = acceleration_[axis];
  if (std::abs(velocity) <= kNumericalEpsilon) {
    return std::abs(acceleration) <= kNumericalEpsilon;
  }
  const double sign = std::copysign(1.0, velocity);
  const double normalized_velocity = sign * velocity;
  const double normalized_acceleration = sign * acceleration;
  const double linear = 2.0 * duration_s * normalized_acceleration +
                        4.0 * normalized_velocity;
  const double discriminant =
      linear * linear + 4.0 * duration_s * duration_s *
                            normalized_acceleration * normalized_acceleration;
  const double jerk =
      (linear + std::sqrt(std::max(0.0, discriminant))) /
      (2.0 * duration_s * duration_s);
  if (!(jerk > 0.0) || jerk > limits_.maximum_jerk_rad_s3[axis] + 1e-9) return false;
  const double peak_acceleration = 0.5 * (jerk * duration_s - normalized_acceleration);
  const double first_duration =
      (normalized_acceleration + peak_acceleration) / jerk;
  const double final_duration = peak_acceleration / jerk;
  if (peak_acceleration > limits_.maximum_acceleration_rad_s2[axis] + 1e-9 ||
      first_duration < -kNumericalEpsilon || final_duration < -kNumericalEpsilon ||
      std::abs(first_duration + final_duration - duration_s) > 1e-8) {
    return false;
  }
  *plan = BrakeAxis{{std::max(0.0, first_duration), 0.0,
                     std::max(0.0, final_duration)},
                    {-sign * jerk, 0.0, sign * jerk}, 0U, 0.0, false};

  double predicted_position = position_[axis];
  double predicted_velocity = velocity;
  double predicted_acceleration = acceleration;
  for (std::size_t phase = 0; phase < plan->duration_s.size(); ++phase) {
    IntegrateConstantJerk(plan->jerk_rad_s3[phase], plan->duration_s[phase],
                          &predicted_position, &predicted_velocity,
                          &predicted_acceleration);
  }
  return std::isfinite(predicted_position) && std::abs(predicted_velocity) <= 1e-8 &&
         std::abs(predicted_acceleration) <= 1e-8 &&
         predicted_position >= limits_.minimum_position_rad[axis] &&
         predicted_position <= limits_.maximum_position_rad[axis];
}

OperationResult ReferenceJointShaperV1::RequestControlledStop(
    std::uint64_t release_sequence, StopReason reason, std::int64_t now_ns) noexcept {
  if (mode_ == ShaperMode::kControlledBraking || mode_ == ShaperMode::kStopped) {
    return Result(OperationCode::kAlreadyRequested);
  }
  if (mode_ != ShaperMode::kActiveTracking || source_sequence_ == 0U ||
      !IsControlledReason(reason) || release_sequence <= last_input_sequence_ ||
      now_ns < last_tick_ns_) {
    return Result(OperationCode::kInvalidState);
  }
  for (std::size_t i = 0; i < dof_; ++i) {
    if (!PlanBrakeAxis(i, &brake_[i])) {
      HardStop(StopReason::kInvalidCommand, now_ns);
      return Result(OperationCode::kPlanningFailed);
    }
  }
  double synchronized_duration_s = 0.0;
  for (std::size_t i = 0; i < dof_; ++i) {
    synchronized_duration_s = std::max(
        synchronized_duration_s,
        brake_[i].duration_s[0] + brake_[i].duration_s[1] + brake_[i].duration_s[2]);
  }
  synchronized_duration_s =
      std::ceil(synchronized_duration_s / kPeriodS - 1e-12) * kPeriodS;
  for (std::size_t i = 0; i < dof_; ++i) {
    if (!brake_[i].complete &&
        !SynchronizeBrakeAxis(i, synchronized_duration_s, &brake_[i])) {
      HardStop(StopReason::kInvalidCommand, now_ns);
      return Result(OperationCode::kPlanningFailed);
    }
  }
  for (std::size_t i = dof_; i < kMaxDof; ++i) {
    brake_[i] = BrakeAxis{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0U, 0.0, true};
  }
  last_input_sequence_ = release_sequence;
  release_sequence_ = release_sequence;
  liveness_monotonic_ns_ = now_ns;
  target_velocity_.fill(0.0);
  stop_reason_ = reason;
  mode_ = ShaperMode::kControlledBraking;
  return Result(OperationCode::kOk);
}

void ReferenceJointShaperV1::AdvanceBrakeAxis(std::size_t axis, double dt_s) noexcept {
  BrakeAxis& plan = brake_[axis];
  double remaining = dt_s;
  std::size_t bounded_iterations = 0;
  while (!plan.complete && remaining > kNumericalEpsilon && bounded_iterations < 4U) {
    ++bounded_iterations;
    while (plan.phase < plan.duration_s.size() &&
           plan.duration_s[plan.phase] - plan.phase_elapsed_s <= kNumericalEpsilon) {
      ++plan.phase;
      plan.phase_elapsed_s = 0.0;
    }
    if (plan.phase >= plan.duration_s.size()) {
      plan.complete = true;
      velocity_[axis] = 0.0;
      acceleration_[axis] = 0.0;
      break;
    }
    const double phase_remaining = plan.duration_s[plan.phase] - plan.phase_elapsed_s;
    const double step = std::min(remaining, phase_remaining);
    IntegrateConstantJerk(plan.jerk_rad_s3[plan.phase], step, &position_[axis],
                          &velocity_[axis], &acceleration_[axis]);
    plan.phase_elapsed_s += step;
    remaining -= step;
  }
  if (!plan.complete && plan.phase >= plan.duration_s.size()) {
    plan.complete = true;
  }
  if (plan.complete) {
    velocity_[axis] = 0.0;
    acceleration_[axis] = 0.0;
  }
}

void ReferenceJointShaperV1::Publish(OutputMode mode, StopClass stop_class,
                                     StopReason reason, std::int64_t now_ns,
                                     ShapedJointCommandV1* output) noexcept {
  *output = ShapedJointCommandV1{};
  output->header = teleop_command_abi::MakeHeaderV1<ShapedJointCommandV1>();
  output->output_sequence = ++output_sequence_;
  output->source_sequence = source_sequence_;
  output->safety_epoch = safety_epoch_;
  output->generated_monotonic_ns = now_ns;
  output->valid_until_monotonic_ns = now_ns + 2 * kReferencePeriodNs;
  output->dof = dof_;
  output->output_mode = mode;
  output->stop_class = stop_class;
  output->stop_reason = reason;
  output->reason_code = 0U;
  output->position_rad = position_;
  output->velocity_rad_s = velocity_;
  output->acceleration_rad_s2 = acceleration_;
}

OperationResult ReferenceJointShaperV1::Tick(std::int64_t now_ns,
                                             ShapedJointCommandV1* output) noexcept {
  if (output == nullptr) return Result(OperationCode::kInvalidArgument);
  *output = ShapedJointCommandV1{};
  if (mode_ == ShaperMode::kHardStopped) {
    return Result(OperationCode::kTerminalNoOutput);
  }
  if (mode_ == ShaperMode::kUninitialized) {
    return Result(OperationCode::kInvalidState);
  }
  if (now_ns < 0 || now_ns - last_tick_ns_ != kReferencePeriodNs) {
    HardStop(StopReason::kTimingFault, now_ns);
    return Result(OperationCode::kTerminalNoOutput);
  }
  if (mode_ == ShaperMode::kActiveTracking) {
    if (source_sequence_ == 0U) {
      HardStop(StopReason::kInvalidCommand, now_ns);
      return Result(OperationCode::kTerminalNoOutput);
    }
    if (now_ns > target_valid_until_ns_) {
      HardStop(StopReason::kStaleInput, now_ns);
      return Result(OperationCode::kTerminalNoOutput);
    }
    for (std::size_t i = 0; i < dof_; ++i) {
      const double maximum_acceleration = limits_.maximum_acceleration_rad_s2[i];
      const double desired_acceleration =
          Clip(36.0 * (target_[i] - position_[i]) +
                   10.0 * (target_velocity_[i] - velocity_[i]),
               maximum_acceleration);
      acceleration_[i] +=
          Clip(desired_acceleration - acceleration_[i],
               limits_.maximum_jerk_rad_s3[i] * kPeriodS);
      velocity_[i] = Clip(velocity_[i] + acceleration_[i] * kPeriodS,
                          limits_.maximum_velocity_rad_s[i]);
      position_[i] += velocity_[i] * kPeriodS;
      if (!std::isfinite(position_[i]) || !std::isfinite(velocity_[i]) ||
          !std::isfinite(acceleration_[i]) ||
          position_[i] < limits_.minimum_position_rad[i] ||
          position_[i] > limits_.maximum_position_rad[i]) {
        HardStop(StopReason::kInvalidCommand, now_ns);
        return Result(OperationCode::kTerminalNoOutput);
      }
    }
    target_velocity_.fill(0.0);
    last_tick_ns_ = now_ns;
    Publish(OutputMode::kActiveTracking, StopClass::kNone, StopReason::kNone,
            now_ns, output);
    return Result(OperationCode::kOk);
  }
  if (mode_ == ShaperMode::kControlledBraking) {
    bool complete = true;
    for (std::size_t i = 0; i < dof_; ++i) {
      AdvanceBrakeAxis(i, kPeriodS);
      complete = complete && brake_[i].complete;
      if (!std::isfinite(position_[i]) || position_[i] < limits_.minimum_position_rad[i] ||
          position_[i] > limits_.maximum_position_rad[i]) {
        HardStop(StopReason::kInvalidCommand, now_ns);
        return Result(OperationCode::kTerminalNoOutput);
      }
    }
    last_tick_ns_ = now_ns;
    if (complete) mode_ = ShaperMode::kStopped;
    Publish(complete ? OutputMode::kStopped : OutputMode::kControlledBraking,
            StopClass::kControlled, stop_reason_, now_ns, output);
    return Result(complete ? OperationCode::kCompleted : OperationCode::kOk);
  }

  last_tick_ns_ = now_ns;
  Publish(OutputMode::kStopped, StopClass::kControlled, stop_reason_, now_ns, output);
  return Result(OperationCode::kCompleted);
}

void ReferenceJointShaperV1::HardStop(StopReason reason, std::int64_t now_ns) noexcept {
  if (mode_ == ShaperMode::kHardStopped) return;
  mode_ = ShaperMode::kHardStopped;
  stop_reason_ = reason == StopReason::kNone ? StopReason::kInvalidCommand : reason;
  target_velocity_.fill(0.0);
  liveness_monotonic_ns_ = now_ns;
}

ShaperSnapshot ReferenceJointShaperV1::Snapshot() const noexcept {
  return ShaperSnapshot{mode_,
                        dof_,
                        safety_epoch_,
                        last_input_sequence_,
                        source_sequence_,
                        output_sequence_,
                        release_sequence_,
                        last_tick_ns_,
                        liveness_monotonic_ns_,
                        stop_reason_,
                        position_,
                        velocity_,
                        acceleration_};
}

}  // namespace teleop_shaping
