#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "teleop_command_abi/abi_v1.hpp"

namespace teleop_shaping {

// SDK-free lifecycle-shaped test double. It intentionally contains no vendor
// header, handle, return code, transport, or hardware operation.
enum class FakeJakaLifecycleState : std::uint8_t {
  kDisconnected = 0,
  kConnecting = 1,
  kConnected = 2,
  kServoReady = 3,
  kStreaming = 4,
  kControlledStopping = 5,
  kStopped = 6,
  kFaulted = 7,
  kCleaningUp = 8,
};

enum class FakeLifecycleCode : std::uint8_t {
  kOk = 0,
  kInvalidState = 1,
  kInvalidCommand = 2,
  kStaleCommand = 3,
  kEpochMismatch = 4,
  kDuplicateOrOldSequence = 5,
  kSendRejected = 6,
  kTransportFailure = 7,
  kControllerFault = 8,
  kHardStopped = 9,
};

enum class FakeSendOutcome : std::uint8_t {
  kOk = 0,
  kRejected = 1,
  kTransportFailure = 2,
  kControllerAlarm = 3,
};

enum class FakeTelemetryEvent : std::uint8_t {
  kLifecycle = 0,
  kEpochArmed = 1,
  kCommandAccepted = 2,
  kCommandRejected = 3,
  kHealthAccepted = 4,
  kFaultLatched = 5,
  kCleanup = 6,
};

struct FakeJakaTelemetryRecord {
  std::uint64_t record_sequence;
  std::int64_t monotonic_ns;
  FakeJakaLifecycleState lifecycle_state;
  FakeTelemetryEvent event;
  FakeLifecycleCode result;
  teleop_command_abi::OutputMode output_mode;
  std::uint64_t output_sequence;
  std::uint64_t source_sequence;
  std::uint64_t safety_epoch;
  std::int64_t command_age_ns;
  std::int64_t deadline_slack_ns;
  teleop_command_abi::StopReason stop_reason;
  teleop_command_abi::ValidationError validation_error;
  std::uint8_t validation_index;
};

struct FakeJakaLifecycleSnapshot {
  FakeJakaLifecycleState lifecycle_state;
  std::uint8_t dof;
  bool session_owned;
  bool hard_stop_latched;
  bool reset_required;
  std::uint64_t safety_epoch;
  std::uint64_t last_output_sequence;
  std::uint64_t last_health_sequence;
  std::uint64_t accepted_command_count;
  std::uint64_t rejected_command_count;
  std::uint64_t skipped_output_sequence_count;
  std::uint64_t telemetry_overflow_count;
  teleop_command_abi::StopReason fault_reason;
};

class FakeJakaLifecycleAdapter final {
 public:
  static constexpr std::size_t kTelemetryCapacity = 256;

  FakeJakaLifecycleAdapter() noexcept;
  FakeLifecycleCode BeginConnect(std::int64_t now_ns) noexcept;
  FakeLifecycleCode CompleteConnect(bool success, std::int64_t now_ns) noexcept;
  FakeLifecycleCode PrepareServo(std::int64_t now_ns) noexcept;
  FakeLifecycleCode ArmEpoch(
      const teleop_command_abi::MeasuredJointStateV1& measured,
      std::int64_t now_ns) noexcept;
  FakeLifecycleCode StartStreaming(std::uint64_t safety_epoch,
                                   std::int64_t now_ns) noexcept;
  FakeLifecycleCode Send(
      const teleop_command_abi::ShapedJointCommandV1& command,
      FakeSendOutcome outcome, std::int64_t now_ns) noexcept;
  FakeLifecycleCode SampleHealth(
      const teleop_command_abi::TransportHealthV1& health,
      std::int64_t now_ns) noexcept;
  void InjectFault(teleop_command_abi::StopReason reason,
                   std::int64_t now_ns) noexcept;
  FakeLifecycleCode BeginCleanup(std::int64_t now_ns) noexcept;
  FakeLifecycleCode CompleteCleanup(std::int64_t now_ns) noexcept;
  FakeLifecycleCode ResetAfterCleanup(std::int64_t now_ns) noexcept;

  FakeJakaLifecycleSnapshot Snapshot() const noexcept;
  std::size_t telemetry_size() const noexcept { return telemetry_size_; }
  const FakeJakaTelemetryRecord& telemetry(std::size_t index) const noexcept;
  bool has_terminal_fault_record() const noexcept { return has_terminal_fault_record_; }
  const FakeJakaTelemetryRecord& terminal_fault_record() const noexcept;

 private:
  FakeLifecycleCode Fault(FakeLifecycleCode code,
                          teleop_command_abi::StopReason reason,
                          std::int64_t now_ns,
                          teleop_command_abi::ValidationResult validation,
                          const teleop_command_abi::ShapedJointCommandV1* command =
                              nullptr) noexcept;
  void Record(FakeTelemetryEvent event, FakeLifecycleCode result,
              std::int64_t now_ns,
              const teleop_command_abi::ShapedJointCommandV1* command,
              teleop_command_abi::ValidationResult validation) noexcept;

  FakeJakaLifecycleState lifecycle_state_;
  std::uint8_t dof_;
  bool session_owned_;
  bool hard_stop_latched_;
  bool reset_required_;
  std::uint64_t safety_epoch_;
  std::uint64_t last_output_sequence_;
  std::uint64_t last_health_sequence_;
  std::uint64_t accepted_command_count_;
  std::uint64_t rejected_command_count_;
  std::uint64_t skipped_output_sequence_count_;
  std::uint64_t telemetry_overflow_count_;
  std::uint64_t telemetry_record_sequence_;
  teleop_command_abi::StopReason fault_reason_;
  std::array<FakeJakaTelemetryRecord, kTelemetryCapacity> telemetry_;
  std::size_t telemetry_size_;
  std::size_t telemetry_next_;
  bool has_terminal_fault_record_;
  FakeJakaTelemetryRecord terminal_fault_record_;
};

}  // namespace teleop_shaping
