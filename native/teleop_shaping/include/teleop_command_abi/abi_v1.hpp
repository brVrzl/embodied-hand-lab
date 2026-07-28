#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace teleop_command_abi {

constexpr std::uint32_t kAbiMagic = 0x54434D44U;  // "TCMD"
constexpr std::uint16_t kSchemaVersionV1 = 1U;
constexpr std::uint8_t kLittleEndian = 1U;
constexpr std::size_t kMaxDof = 8U;
constexpr std::size_t kNoFieldIndex = 0xFFU;

enum class EngagementState : std::uint8_t { kDisengaged = 0, kEngaged = 1 };
enum class TargetValidity : std::uint8_t {
  kNoTarget = 0,
  kAccepted = 1,
  kRejectedKeepPrevious = 2,
};
enum class OutputMode : std::uint8_t {
  kInactive = 0,
  kActiveTracking = 1,
  kControlledBraking = 2,
  kStopped = 3,
  kHardStopped = 4,
};
enum class StopClass : std::uint8_t { kNone = 0, kControlled = 1, kImmediate = 2 };
enum class StopReason : std::uint8_t {
  kNone = 0,
  kClutchRelease = 1,
  kStaleInput = 2,
  kTimingFault = 3,
  kControllerAlarm = 4,
  kSdkFailure = 5,
  kEstop = 6,
  kCollision = 7,
  kProducerFailure = 8,
  kEpochMismatch = 9,
  kInvalidCommand = 10,
};
enum class MeasurementValidity : std::uint8_t {
  kNoMeasurement = 0,
  kValid = 1,
  kInvalid = 2,
};
enum class TransportState : std::uint8_t {
  kDisconnected = 0,
  kStarting = 1,
  kReady = 2,
  kFaulted = 3,
};
enum class ControllerState : std::uint8_t {
  kUnknown = 0,
  kReady = 1,
  kDisabled = 2,
  kAlarm = 3,
  kEstop = 4,
  kCollision = 5,
};
enum class VendorStatusCategory : std::int32_t {
  kNone = 0,
  kTransient = 1,
  kCommandRejected = 2,
  kControllerFault = 3,
  kTransportFault = 4,
  kUnknownFault = 5,
};

enum class ValidationError : std::uint16_t {
  kOk = 0,
  kBadMagic,
  kUnsupportedVersion,
  kWrongStructSize,
  kUnsupportedEndianness,
  kNonzeroReserved,
  kInvalidDof,
  kInvalidEnum,
  kNonFinite,
  kTimestampOrdering,
  kInvalidSequence,
  kInvalidLimits,
  kEpochMismatch,
  kTargetSmuggling,
  kStaleValidityWindow,
  kInvalidModeContract,
  kInvalidBoolean,
  kInvalidMeasurement,
};

enum class ValidationField : std::uint16_t {
  kNone = 0,
  kHeader,
  kSequence,
  kSafetyEpoch,
  kSourceTimestamp,
  kAcceptedTimestamp,
  kValidUntilTimestamp,
  kMeasuredTimestamp,
  kGeneratedTimestamp,
  kSampledTimestamp,
  kDof,
  kEngagement,
  kValidity,
  kOutputMode,
  kStopClass,
  kStopReason,
  kPosition,
  kVelocity,
  kAcceleration,
  kMinimumPosition,
  kMaximumPosition,
  kMaximumVelocity,
  kMaximumAcceleration,
  kMaximumJerk,
  kTransportState,
  kControllerState,
  kFlags,
  kUnusedAxis,
};

struct ValidationResult {
  bool ok;
  ValidationError error;
  ValidationField field;
  std::uint8_t index;
};

struct ValidationContext {
  std::int64_t now_ns{-1};
  std::uint8_t expected_dof{0};
  std::uint64_t expected_epoch{0};
  std::uint64_t previous_sequence{0};
};

struct AbiHeader {
  std::uint32_t magic;
  std::uint16_t schema_version;
  std::uint16_t struct_size;
  std::uint8_t host_endianness;
  std::array<std::uint8_t, 7> reserved;
};

struct alignas(8) AcceptedJointTargetV1 {
  AbiHeader header;
  std::uint64_t sequence;
  std::uint64_t safety_epoch;
  std::int64_t source_monotonic_ns;
  std::int64_t accepted_monotonic_ns;
  std::int64_t valid_until_monotonic_ns;
  std::uint8_t dof;
  EngagementState engagement;
  TargetValidity validity;
  std::uint8_t reserved0;
  std::uint32_t reason_code;
  std::array<double, kMaxDof> position_rad;
};

struct alignas(8) MeasuredJointStateV1 {
  AbiHeader header;
  std::uint64_t state_sequence;
  std::uint64_t safety_epoch;
  std::int64_t measured_monotonic_ns;
  std::uint8_t dof;
  MeasurementValidity validity;
  std::array<std::uint8_t, 6> reserved0;
  std::array<double, kMaxDof> position_rad;
  std::array<double, kMaxDof> velocity_rad_s;
  std::array<double, kMaxDof> acceleration_rad_s2;
};

struct alignas(8) JointDynamicLimitsV1 {
  AbiHeader header;
  std::uint8_t dof;
  std::array<std::uint8_t, 7> reserved0;
  std::array<double, kMaxDof> minimum_position_rad;
  std::array<double, kMaxDof> maximum_position_rad;
  std::array<double, kMaxDof> maximum_velocity_rad_s;
  std::array<double, kMaxDof> maximum_acceleration_rad_s2;
  std::array<double, kMaxDof> maximum_jerk_rad_s3;
};

struct alignas(8) ShapedJointCommandV1 {
  AbiHeader header;
  std::uint64_t output_sequence;
  std::uint64_t source_sequence;
  std::uint64_t safety_epoch;
  std::int64_t generated_monotonic_ns;
  std::int64_t valid_until_monotonic_ns;
  std::uint8_t dof;
  OutputMode output_mode;
  StopClass stop_class;
  StopReason stop_reason;
  std::uint32_t reason_code;
  std::array<double, kMaxDof> position_rad;
  std::array<double, kMaxDof> velocity_rad_s;
  std::array<double, kMaxDof> acceleration_rad_s2;
};

struct alignas(8) TransportHealthV1 {
  AbiHeader header;
  std::uint64_t health_sequence;
  std::uint64_t last_consumed_output_sequence;
  std::uint64_t safety_epoch;
  std::int64_t sampled_monotonic_ns;
  TransportState transport_state;
  ControllerState controller_state;
  std::uint8_t producer_stale;
  std::uint8_t command_stale;
  std::uint8_t deadline_missed;
  std::uint8_t alarm;
  std::uint8_t estop;
  std::uint8_t collision;
  std::uint8_t servo_enabled;
  std::array<std::uint8_t, 3> reserved0;
  VendorStatusCategory vendor_status_category;
};

template <typename T>
constexpr AbiHeader MakeHeaderV1() noexcept {
  return AbiHeader{kAbiMagic, kSchemaVersionV1, static_cast<std::uint16_t>(sizeof(T)),
                   kLittleEndian, {0, 0, 0, 0, 0, 0, 0}};
}

ValidationResult Validate(const AcceptedJointTargetV1& value,
                          ValidationContext context = {}) noexcept;
ValidationResult Validate(const MeasuredJointStateV1& value,
                          ValidationContext context = {}) noexcept;
ValidationResult Validate(const JointDynamicLimitsV1& value) noexcept;
ValidationResult Validate(const ShapedJointCommandV1& value,
                          ValidationContext context = {}) noexcept;
ValidationResult Validate(const TransportHealthV1& value,
                          ValidationContext context = {}) noexcept;

static_assert(std::is_standard_layout_v<AbiHeader> && std::is_trivially_copyable_v<AbiHeader>);
static_assert(std::is_standard_layout_v<AcceptedJointTargetV1> &&
              std::is_trivially_copyable_v<AcceptedJointTargetV1>);
static_assert(std::is_standard_layout_v<MeasuredJointStateV1> &&
              std::is_trivially_copyable_v<MeasuredJointStateV1>);
static_assert(std::is_standard_layout_v<JointDynamicLimitsV1> &&
              std::is_trivially_copyable_v<JointDynamicLimitsV1>);
static_assert(std::is_standard_layout_v<ShapedJointCommandV1> &&
              std::is_trivially_copyable_v<ShapedJointCommandV1>);
static_assert(std::is_standard_layout_v<TransportHealthV1> &&
              std::is_trivially_copyable_v<TransportHealthV1>);
static_assert(sizeof(AbiHeader) == 16 && alignof(AbiHeader) == 4);
static_assert(sizeof(AcceptedJointTargetV1) == 128);
static_assert(sizeof(MeasuredJointStateV1) == 240);
static_assert(sizeof(JointDynamicLimitsV1) == 344);
static_assert(sizeof(ShapedJointCommandV1) == 256);
static_assert(sizeof(TransportHealthV1) == 64);
static_assert(offsetof(AcceptedJointTargetV1, sequence) == 16);
static_assert(offsetof(AcceptedJointTargetV1, position_rad) == 64);
static_assert(offsetof(MeasuredJointStateV1, position_rad) == 48);
static_assert(offsetof(JointDynamicLimitsV1, minimum_position_rad) == 24);
static_assert(offsetof(ShapedJointCommandV1, position_rad) == 64);
static_assert(offsetof(TransportHealthV1, vendor_status_category) == 60);

}  // namespace teleop_command_abi
