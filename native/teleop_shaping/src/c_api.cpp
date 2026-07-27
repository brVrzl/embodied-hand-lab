#include "teleop_shaping/c_api.h"

#include <new>

namespace {

constexpr TeleopValidationResult NullValidation() noexcept {
  return {false, teleop_command_abi::ValidationError::kWrongStructSize,
          teleop_command_abi::ValidationField::kHeader,
          static_cast<std::uint8_t>(teleop_command_abi::kNoFieldIndex)};
}

void SetValidation(TeleopValidationResult* destination,
                   TeleopValidationResult value) noexcept {
  if (destination != nullptr) *destination = value;
}

teleop_command_abi::ValidationContext ContextOrDefault(
    const TeleopValidationContext* context) noexcept {
  return context == nullptr ? teleop_command_abi::ValidationContext{} : *context;
}

teleop_shaping::ReferenceJointShaperV1* Shaper(void* handle) noexcept {
  return static_cast<teleop_shaping::ReferenceJointShaperV1*>(handle);
}

}  // namespace

extern "C" {

int teleop_abi_is_little_endian_host(void) noexcept {
  const std::uint16_t value = 1U;
  return *reinterpret_cast<const std::uint8_t*>(&value) == 1U ? 1 : 0;
}

size_t teleop_abi_sizeof(int kind) noexcept {
  switch (kind) {
    case TELEOP_ABI_HEADER:
      return sizeof(teleop_command_abi::AbiHeader);
    case TELEOP_ABI_ACCEPTED_TARGET:
      return sizeof(TeleopAcceptedJointTargetV1);
    case TELEOP_ABI_MEASURED_STATE:
      return sizeof(TeleopMeasuredJointStateV1);
    case TELEOP_ABI_DYNAMIC_LIMITS:
      return sizeof(TeleopJointDynamicLimitsV1);
    case TELEOP_ABI_SHAPED_COMMAND:
      return sizeof(TeleopShapedJointCommandV1);
    case TELEOP_ABI_TRANSPORT_HEALTH:
      return sizeof(TeleopTransportHealthV1);
    default:
      return 0U;
  }
}

size_t teleop_abi_alignof(int kind) noexcept {
  switch (kind) {
    case TELEOP_ABI_HEADER:
      return alignof(teleop_command_abi::AbiHeader);
    case TELEOP_ABI_ACCEPTED_TARGET:
      return alignof(TeleopAcceptedJointTargetV1);
    case TELEOP_ABI_MEASURED_STATE:
      return alignof(TeleopMeasuredJointStateV1);
    case TELEOP_ABI_DYNAMIC_LIMITS:
      return alignof(TeleopJointDynamicLimitsV1);
    case TELEOP_ABI_SHAPED_COMMAND:
      return alignof(TeleopShapedJointCommandV1);
    case TELEOP_ABI_TRANSPORT_HEALTH:
      return alignof(TeleopTransportHealthV1);
    default:
      return 0U;
  }
}

TeleopValidationResult teleop_validate_target(
    const TeleopAcceptedJointTargetV1* value,
    const TeleopValidationContext* context) noexcept {
  return value == nullptr ? NullValidation()
                          : teleop_command_abi::Validate(*value, ContextOrDefault(context));
}

TeleopValidationResult teleop_validate_measured(
    const TeleopMeasuredJointStateV1* value,
    const TeleopValidationContext* context) noexcept {
  return value == nullptr ? NullValidation()
                          : teleop_command_abi::Validate(*value, ContextOrDefault(context));
}

TeleopValidationResult teleop_validate_limits(
    const TeleopJointDynamicLimitsV1* value) noexcept {
  return value == nullptr ? NullValidation() : teleop_command_abi::Validate(*value);
}

TeleopValidationResult teleop_validate_command(
    const TeleopShapedJointCommandV1* value,
    const TeleopValidationContext* context) noexcept {
  return value == nullptr ? NullValidation()
                          : teleop_command_abi::Validate(*value, ContextOrDefault(context));
}

TeleopValidationResult teleop_validate_health(
    const TeleopTransportHealthV1* value,
    const TeleopValidationContext* context) noexcept {
  return value == nullptr ? NullValidation()
                          : teleop_command_abi::Validate(*value, ContextOrDefault(context));
}

void* teleop_reference_shaper_create(void) noexcept {
  return new (std::nothrow) teleop_shaping::ReferenceJointShaperV1();
}

void teleop_reference_shaper_destroy(void* handle) noexcept { delete Shaper(handle); }

int teleop_reference_shaper_initialize(void* handle,
                                       const TeleopMeasuredJointStateV1* measured,
                                       const TeleopJointDynamicLimitsV1* limits,
                                       int64_t now_ns,
                                       TeleopValidationResult* validation) noexcept {
  if (handle == nullptr || measured == nullptr || limits == nullptr) {
    SetValidation(validation, NullValidation());
    return static_cast<int>(teleop_shaping::OperationCode::kInvalidArgument);
  }
  const auto result = Shaper(handle)->Initialize(*measured, *limits, now_ns);
  SetValidation(validation, result.validation);
  return static_cast<int>(result.code);
}

int teleop_reference_shaper_replace_target(void* handle,
                                           const TeleopAcceptedJointTargetV1* target,
                                           int64_t now_ns,
                                           TeleopValidationResult* validation) noexcept {
  if (handle == nullptr || target == nullptr) {
    SetValidation(validation, NullValidation());
    return static_cast<int>(teleop_shaping::OperationCode::kInvalidArgument);
  }
  const auto result = Shaper(handle)->ReplaceTarget(*target, now_ns);
  SetValidation(validation, result.validation);
  return static_cast<int>(result.code);
}

int teleop_reference_shaper_tick(void* handle, int64_t now_ns,
                                 TeleopShapedJointCommandV1* output,
                                 TeleopValidationResult* validation) noexcept {
  if (handle == nullptr || output == nullptr) {
    SetValidation(validation, NullValidation());
    return static_cast<int>(teleop_shaping::OperationCode::kInvalidArgument);
  }
  const auto result = Shaper(handle)->Tick(now_ns, output);
  SetValidation(validation, result.validation);
  return static_cast<int>(result.code);
}

int teleop_reference_shaper_request_stop(void* handle, uint64_t release_sequence,
                                         uint8_t stop_reason, int64_t now_ns,
                                         TeleopValidationResult* validation) noexcept {
  if (handle == nullptr || stop_reason >
                               static_cast<std::uint8_t>(teleop_command_abi::StopReason::kInvalidCommand)) {
    SetValidation(validation, NullValidation());
    return static_cast<int>(teleop_shaping::OperationCode::kInvalidArgument);
  }
  const auto result = Shaper(handle)->RequestControlledStop(
      release_sequence, static_cast<teleop_command_abi::StopReason>(stop_reason), now_ns);
  SetValidation(validation, result.validation);
  return static_cast<int>(result.code);
}

void teleop_reference_shaper_hard_stop(void* handle, uint8_t stop_reason,
                                       int64_t now_ns) noexcept {
  if (handle == nullptr) return;
  const auto reason = stop_reason <=
                              static_cast<std::uint8_t>(teleop_command_abi::StopReason::kInvalidCommand)
                          ? static_cast<teleop_command_abi::StopReason>(stop_reason)
                          : teleop_command_abi::StopReason::kInvalidCommand;
  Shaper(handle)->HardStop(reason, now_ns);
}

int teleop_reference_shaper_snapshot(void* handle,
                                     TeleopShaperSnapshot* snapshot) noexcept {
  if (handle == nullptr || snapshot == nullptr) return 0;
  *snapshot = Shaper(handle)->Snapshot();
  return 1;
}

}  // extern "C"
