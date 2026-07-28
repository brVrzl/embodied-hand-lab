#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
#include "teleop_command_abi/abi_v1.hpp"
#include "teleop_shaping/joint_shaper.hpp"
extern "C" {
typedef teleop_command_abi::AcceptedJointTargetV1 TeleopAcceptedJointTargetV1;
typedef teleop_command_abi::MeasuredJointStateV1 TeleopMeasuredJointStateV1;
typedef teleop_command_abi::JointDynamicLimitsV1 TeleopJointDynamicLimitsV1;
typedef teleop_command_abi::ShapedJointCommandV1 TeleopShapedJointCommandV1;
typedef teleop_command_abi::TransportHealthV1 TeleopTransportHealthV1;
typedef teleop_command_abi::ValidationContext TeleopValidationContext;
typedef teleop_command_abi::ValidationResult TeleopValidationResult;
typedef teleop_shaping::ShaperSnapshot TeleopShaperSnapshot;
#else
/* The stable command structures are C++ PODs in v1. C consumers use an explicit
 * serializer or a separately reviewed C layout binding; this test bridge is C++ only. */
#error "teleop_shaping/c_api.h currently requires C++17"
#endif

enum TeleopAbiStructKind {
  TELEOP_ABI_HEADER = 0,
  TELEOP_ABI_ACCEPTED_TARGET = 1,
  TELEOP_ABI_MEASURED_STATE = 2,
  TELEOP_ABI_DYNAMIC_LIMITS = 3,
  TELEOP_ABI_SHAPED_COMMAND = 4,
  TELEOP_ABI_TRANSPORT_HEALTH = 5,
};

int teleop_abi_is_little_endian_host(void) noexcept;
size_t teleop_abi_sizeof(int kind) noexcept;
size_t teleop_abi_alignof(int kind) noexcept;

TeleopValidationResult teleop_validate_target(
    const TeleopAcceptedJointTargetV1* value,
    const TeleopValidationContext* context) noexcept;
TeleopValidationResult teleop_validate_measured(
    const TeleopMeasuredJointStateV1* value,
    const TeleopValidationContext* context) noexcept;
TeleopValidationResult teleop_validate_limits(
    const TeleopJointDynamicLimitsV1* value) noexcept;
TeleopValidationResult teleop_validate_command(
    const TeleopShapedJointCommandV1* value,
    const TeleopValidationContext* context) noexcept;
TeleopValidationResult teleop_validate_health(
    const TeleopTransportHealthV1* value,
    const TeleopValidationContext* context) noexcept;

void* teleop_reference_shaper_create(void) noexcept;
void teleop_reference_shaper_destroy(void* handle) noexcept;
int teleop_reference_shaper_initialize(void* handle,
                                       const TeleopMeasuredJointStateV1* measured,
                                       const TeleopJointDynamicLimitsV1* limits,
                                       int64_t now_ns,
                                       TeleopValidationResult* validation) noexcept;
int teleop_reference_shaper_replace_target(void* handle,
                                           const TeleopAcceptedJointTargetV1* target,
                                           int64_t now_ns,
                                           TeleopValidationResult* validation) noexcept;
int teleop_reference_shaper_tick(void* handle, int64_t now_ns,
                                 TeleopShapedJointCommandV1* output,
                                 TeleopValidationResult* validation) noexcept;
int teleop_reference_shaper_request_stop(void* handle, uint64_t release_sequence,
                                         uint8_t stop_reason, int64_t now_ns,
                                         TeleopValidationResult* validation) noexcept;
void teleop_reference_shaper_hard_stop(void* handle, uint8_t stop_reason,
                                       int64_t now_ns) noexcept;
int teleop_reference_shaper_snapshot(void* handle,
                                     TeleopShaperSnapshot* snapshot) noexcept;

#ifdef __cplusplus
}
#endif
