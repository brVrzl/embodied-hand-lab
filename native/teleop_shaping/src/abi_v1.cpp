#include "teleop_command_abi/abi_v1.hpp"

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

namespace teleop_command_abi {
namespace {

constexpr ValidationResult Ok() noexcept {
  return {true, ValidationError::kOk, ValidationField::kNone,
          static_cast<std::uint8_t>(kNoFieldIndex)};
}

constexpr ValidationResult Error(ValidationError error, ValidationField field,
                                 std::size_t index = kNoFieldIndex) noexcept {
  return {false, error, field, static_cast<std::uint8_t>(index)};
}

template <typename T>
ValidationResult ValidateHeader(const AbiHeader& header) noexcept {
  if (header.magic != kAbiMagic) {
    return Error(ValidationError::kBadMagic, ValidationField::kHeader);
  }
  if (header.schema_version != kSchemaVersionV1) {
    return Error(ValidationError::kUnsupportedVersion, ValidationField::kHeader);
  }
  if (header.struct_size != sizeof(T)) {
    return Error(ValidationError::kWrongStructSize, ValidationField::kHeader);
  }
  if (header.host_endianness != kLittleEndian) {
    return Error(ValidationError::kUnsupportedEndianness, ValidationField::kHeader);
  }
  for (std::size_t i = 0; i < header.reserved.size(); ++i) {
    if (header.reserved[i] != 0U) {
      return Error(ValidationError::kNonzeroReserved, ValidationField::kHeader, i);
    }
  }
  return Ok();
}

template <typename Enum>
constexpr bool EnumAtMost(Enum value, Enum maximum) noexcept {
  using Raw = std::underlying_type_t<Enum>;
  const Raw raw = static_cast<Raw>(value);
  return raw >= static_cast<Raw>(0) && raw <= static_cast<Raw>(maximum);
}

bool ValidStopReason(StopReason value) noexcept {
  return EnumAtMost(value, StopReason::kInvalidCommand);
}

template <typename Array>
ValidationResult ValidateFiniteActiveZeroUnused(const Array& values, std::uint8_t dof,
                                                ValidationField field) noexcept {
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i < dof) {
      if (!std::isfinite(values[i])) {
        return Error(ValidationError::kNonFinite, field, i);
      }
    } else if (values[i] != 0.0) {
      return Error(ValidationError::kTargetSmuggling, ValidationField::kUnusedAxis, i);
    }
  }
  return Ok();
}

template <typename Array>
ValidationResult ValidateAllZero(const Array& values, ValidationField field) noexcept {
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!std::isfinite(values[i])) {
      return Error(ValidationError::kNonFinite, field, i);
    }
    if (values[i] != 0.0) {
      return Error(ValidationError::kTargetSmuggling, field, i);
    }
  }
  return Ok();
}

ValidationResult ValidateDof(std::uint8_t dof, const ValidationContext& context) noexcept {
  if (dof == 0U || dof > kMaxDof) {
    return Error(ValidationError::kInvalidDof, ValidationField::kDof);
  }
  if (context.expected_dof != 0U && dof != context.expected_dof) {
    return Error(ValidationError::kInvalidDof, ValidationField::kDof);
  }
  return Ok();
}

ValidationResult ValidateIdentity(std::uint64_t sequence, std::uint64_t epoch,
                                  const ValidationContext& context) noexcept {
  if (sequence == 0U || sequence <= context.previous_sequence) {
    return Error(ValidationError::kInvalidSequence, ValidationField::kSequence);
  }
  if (epoch == 0U) {
    return Error(ValidationError::kEpochMismatch, ValidationField::kSafetyEpoch);
  }
  if (context.expected_epoch != 0U && epoch != context.expected_epoch) {
    return Error(ValidationError::kEpochMismatch, ValidationField::kSafetyEpoch);
  }
  return Ok();
}

bool ValidFlag(std::uint8_t value) noexcept { return value <= 1U; }

}  // namespace

ValidationResult Validate(const AcceptedJointTargetV1& value,
                          ValidationContext context) noexcept {
  ValidationResult result = ValidateHeader<AcceptedJointTargetV1>(value.header);
  if (!result.ok) return result;
  result = ValidateIdentity(value.sequence, value.safety_epoch, context);
  if (!result.ok) return result;
  result = ValidateDof(value.dof, context);
  if (!result.ok) return result;
  if (value.reserved0 != 0U) {
    return Error(ValidationError::kNonzeroReserved, ValidationField::kHeader);
  }
  if (!EnumAtMost(value.engagement, EngagementState::kEngaged)) {
    return Error(ValidationError::kInvalidEnum, ValidationField::kEngagement);
  }
  if (!EnumAtMost(value.validity, TargetValidity::kRejectedKeepPrevious)) {
    return Error(ValidationError::kInvalidEnum, ValidationField::kValidity);
  }
  if (value.source_monotonic_ns < 0 || value.accepted_monotonic_ns < 0 ||
      value.valid_until_monotonic_ns < 0 ||
      value.source_monotonic_ns > value.accepted_monotonic_ns ||
      value.accepted_monotonic_ns > value.valid_until_monotonic_ns) {
    return Error(ValidationError::kTimestampOrdering, ValidationField::kAcceptedTimestamp);
  }
  if (context.now_ns >= 0) {
    if (value.accepted_monotonic_ns > context.now_ns) {
      return Error(ValidationError::kTimestampOrdering, ValidationField::kAcceptedTimestamp);
    }
    if (value.valid_until_monotonic_ns < context.now_ns) {
      return Error(ValidationError::kStaleValidityWindow,
                   ValidationField::kValidUntilTimestamp);
    }
  }
  if (value.validity == TargetValidity::kAccepted) {
    if (value.engagement != EngagementState::kEngaged) {
      return Error(ValidationError::kInvalidModeContract, ValidationField::kEngagement);
    }
    return ValidateFiniteActiveZeroUnused(value.position_rad, value.dof,
                                          ValidationField::kPosition);
  }
  return ValidateAllZero(value.position_rad, ValidationField::kPosition);
}

ValidationResult Validate(const MeasuredJointStateV1& value,
                          ValidationContext context) noexcept {
  ValidationResult result = ValidateHeader<MeasuredJointStateV1>(value.header);
  if (!result.ok) return result;
  result = ValidateIdentity(value.state_sequence, value.safety_epoch, context);
  if (!result.ok) return result;
  result = ValidateDof(value.dof, context);
  if (!result.ok) return result;
  for (std::size_t i = 0; i < value.reserved0.size(); ++i) {
    if (value.reserved0[i] != 0U) {
      return Error(ValidationError::kNonzeroReserved, ValidationField::kHeader, i);
    }
  }
  if (!EnumAtMost(value.validity, MeasurementValidity::kInvalid)) {
    return Error(ValidationError::kInvalidEnum, ValidationField::kValidity);
  }
  if (value.validity != MeasurementValidity::kValid) {
    return Error(ValidationError::kInvalidMeasurement, ValidationField::kValidity);
  }
  if (value.measured_monotonic_ns < 0 ||
      (context.now_ns >= 0 && value.measured_monotonic_ns > context.now_ns)) {
    return Error(ValidationError::kTimestampOrdering, ValidationField::kMeasuredTimestamp);
  }
  for (const auto& [values, field] : {
           std::pair{&value.position_rad, ValidationField::kPosition},
           std::pair{&value.velocity_rad_s, ValidationField::kVelocity},
           std::pair{&value.acceleration_rad_s2, ValidationField::kAcceleration}}) {
    result = ValidateFiniteActiveZeroUnused(*values, value.dof, field);
    if (!result.ok) return result;
  }
  return Ok();
}

ValidationResult Validate(const JointDynamicLimitsV1& value) noexcept {
  ValidationResult result = ValidateHeader<JointDynamicLimitsV1>(value.header);
  if (!result.ok) return result;
  ValidationContext context{};
  result = ValidateDof(value.dof, context);
  if (!result.ok) return result;
  for (std::size_t i = 0; i < value.reserved0.size(); ++i) {
    if (value.reserved0[i] != 0U) {
      return Error(ValidationError::kNonzeroReserved, ValidationField::kHeader, i);
    }
  }
  for (std::size_t i = 0; i < kMaxDof; ++i) {
    if (i >= value.dof) {
      if (value.minimum_position_rad[i] != 0.0 || value.maximum_position_rad[i] != 0.0 ||
          value.maximum_velocity_rad_s[i] != 0.0 ||
          value.maximum_acceleration_rad_s2[i] != 0.0 ||
          value.maximum_jerk_rad_s3[i] != 0.0) {
        return Error(ValidationError::kTargetSmuggling, ValidationField::kUnusedAxis, i);
      }
      continue;
    }
    const std::array<std::pair<double, ValidationField>, 5> fields{
        std::pair{value.minimum_position_rad[i], ValidationField::kMinimumPosition},
        std::pair{value.maximum_position_rad[i], ValidationField::kMaximumPosition},
        std::pair{value.maximum_velocity_rad_s[i], ValidationField::kMaximumVelocity},
        std::pair{value.maximum_acceleration_rad_s2[i],
                  ValidationField::kMaximumAcceleration},
        std::pair{value.maximum_jerk_rad_s3[i], ValidationField::kMaximumJerk}};
    for (const auto& [entry, field] : fields) {
      if (!std::isfinite(entry)) {
        return Error(ValidationError::kNonFinite, field, i);
      }
    }
    if (!(value.minimum_position_rad[i] < value.maximum_position_rad[i])) {
      return Error(ValidationError::kInvalidLimits, ValidationField::kMinimumPosition, i);
    }
    if (!(value.maximum_velocity_rad_s[i] > 0.0)) {
      return Error(ValidationError::kInvalidLimits, ValidationField::kMaximumVelocity, i);
    }
    if (!(value.maximum_acceleration_rad_s2[i] > 0.0)) {
      return Error(ValidationError::kInvalidLimits, ValidationField::kMaximumAcceleration, i);
    }
    if (!(value.maximum_jerk_rad_s3[i] > 0.0)) {
      return Error(ValidationError::kInvalidLimits, ValidationField::kMaximumJerk, i);
    }
  }
  return Ok();
}

ValidationResult Validate(const ShapedJointCommandV1& value,
                          ValidationContext context) noexcept {
  ValidationResult result = ValidateHeader<ShapedJointCommandV1>(value.header);
  if (!result.ok) return result;
  result = ValidateIdentity(value.output_sequence, value.safety_epoch, context);
  if (!result.ok) return result;
  result = ValidateDof(value.dof, context);
  if (!result.ok) return result;
  if (!EnumAtMost(value.output_mode, OutputMode::kHardStopped)) {
    return Error(ValidationError::kInvalidEnum, ValidationField::kOutputMode);
  }
  if (!EnumAtMost(value.stop_class, StopClass::kImmediate)) {
    return Error(ValidationError::kInvalidEnum, ValidationField::kStopClass);
  }
  if (!ValidStopReason(value.stop_reason)) {
    return Error(ValidationError::kInvalidEnum, ValidationField::kStopReason);
  }
  if (value.generated_monotonic_ns < 0 || value.valid_until_monotonic_ns < 0 ||
      value.generated_monotonic_ns > value.valid_until_monotonic_ns) {
    return Error(ValidationError::kTimestampOrdering, ValidationField::kGeneratedTimestamp);
  }
  if (context.now_ns >= 0) {
    if (value.generated_monotonic_ns > context.now_ns) {
      return Error(ValidationError::kTimestampOrdering, ValidationField::kGeneratedTimestamp);
    }
    if (value.valid_until_monotonic_ns < context.now_ns) {
      return Error(ValidationError::kStaleValidityWindow,
                   ValidationField::kValidUntilTimestamp);
    }
  }
  const bool motion = value.output_mode == OutputMode::kActiveTracking ||
                      value.output_mode == OutputMode::kControlledBraking;
  if (motion && value.source_sequence == 0U) {
    return Error(ValidationError::kInvalidSequence, ValidationField::kSequence);
  }
  if (value.output_mode == OutputMode::kActiveTracking &&
      (value.stop_class != StopClass::kNone || value.stop_reason != StopReason::kNone)) {
    return Error(ValidationError::kInvalidModeContract, ValidationField::kOutputMode);
  }
  if (value.output_mode == OutputMode::kControlledBraking &&
      (value.stop_class != StopClass::kControlled || value.stop_reason == StopReason::kNone)) {
    return Error(ValidationError::kInvalidModeContract, ValidationField::kOutputMode);
  }
  if (value.output_mode == OutputMode::kStopped &&
      (value.stop_class != StopClass::kControlled || value.stop_reason == StopReason::kNone)) {
    return Error(ValidationError::kInvalidModeContract, ValidationField::kOutputMode);
  }
  if (value.output_mode == OutputMode::kHardStopped &&
      (value.stop_class != StopClass::kImmediate || value.stop_reason == StopReason::kNone)) {
    return Error(ValidationError::kInvalidModeContract, ValidationField::kOutputMode);
  }
  if (value.output_mode == OutputMode::kInactive ||
      value.output_mode == OutputMode::kHardStopped) {
    if (value.output_mode == OutputMode::kInactive &&
        (value.stop_class != StopClass::kNone || value.stop_reason != StopReason::kNone)) {
      return Error(ValidationError::kInvalidModeContract, ValidationField::kOutputMode);
    }
    for (const auto& [values, field] : {
             std::pair{&value.position_rad, ValidationField::kPosition},
             std::pair{&value.velocity_rad_s, ValidationField::kVelocity},
             std::pair{&value.acceleration_rad_s2, ValidationField::kAcceleration}}) {
      result = ValidateAllZero(*values, field);
      if (!result.ok) return result;
    }
    return Ok();
  }
  result = ValidateFiniteActiveZeroUnused(value.position_rad, value.dof,
                                          ValidationField::kPosition);
  if (!result.ok) return result;
  result = ValidateFiniteActiveZeroUnused(value.velocity_rad_s, value.dof,
                                          ValidationField::kVelocity);
  if (!result.ok) return result;
  result = ValidateFiniteActiveZeroUnused(value.acceleration_rad_s2, value.dof,
                                          ValidationField::kAcceleration);
  if (!result.ok) return result;
  if (value.output_mode == OutputMode::kStopped) {
    for (std::size_t i = 0; i < value.dof; ++i) {
      if (value.velocity_rad_s[i] != 0.0 || value.acceleration_rad_s2[i] != 0.0) {
        return Error(ValidationError::kInvalidModeContract, ValidationField::kVelocity, i);
      }
    }
  }
  return Ok();
}

ValidationResult Validate(const TransportHealthV1& value,
                          ValidationContext context) noexcept {
  ValidationResult result = ValidateHeader<TransportHealthV1>(value.header);
  if (!result.ok) return result;
  result = ValidateIdentity(value.health_sequence, value.safety_epoch, context);
  if (!result.ok) return result;
  if (value.sampled_monotonic_ns < 0 ||
      (context.now_ns >= 0 && value.sampled_monotonic_ns > context.now_ns)) {
    return Error(ValidationError::kTimestampOrdering, ValidationField::kSampledTimestamp);
  }
  if (!EnumAtMost(value.transport_state, TransportState::kFaulted)) {
    return Error(ValidationError::kInvalidEnum, ValidationField::kTransportState);
  }
  if (!EnumAtMost(value.controller_state, ControllerState::kCollision)) {
    return Error(ValidationError::kInvalidEnum, ValidationField::kControllerState);
  }
  if (!EnumAtMost(value.vendor_status_category, VendorStatusCategory::kUnknownFault)) {
    return Error(ValidationError::kInvalidEnum, ValidationField::kFlags);
  }
  const std::array<std::uint8_t, 7> flags{
      value.producer_stale, value.command_stale, value.deadline_missed, value.alarm,
      value.estop, value.collision, value.servo_enabled};
  for (std::size_t i = 0; i < flags.size(); ++i) {
    if (!ValidFlag(flags[i])) {
      return Error(ValidationError::kInvalidBoolean, ValidationField::kFlags, i);
    }
  }
  for (std::size_t i = 0; i < value.reserved0.size(); ++i) {
    if (value.reserved0[i] != 0U) {
      return Error(ValidationError::kNonzeroReserved, ValidationField::kHeader, i);
    }
  }
  return Ok();
}

}  // namespace teleop_command_abi
