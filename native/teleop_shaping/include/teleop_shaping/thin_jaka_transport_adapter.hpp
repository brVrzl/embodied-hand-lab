#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "teleop_command_abi/abi_v1.hpp"
#include "teleop_shaping/clutch_recovery_transport.hpp"

namespace teleop_shaping {

// SDK-free seam for a future sole-session JAKA transport. A real translation
// unit may populate this table, but no vendor type or symbol crosses it.
enum class JakaFunctionResult : std::uint8_t {
  kOk = 0,
  kTimeout = 1,
  kFailure = 2,
  kSessionLost = 3,
  kRejected = 4,
};

struct JakaJointFeedback {
  std::uint64_t sequence;
  std::int64_t sampled_monotonic_ns;
  std::uint8_t dof;
  std::array<double, teleop_command_abi::kMaxDof> position_rad;
  std::array<double, teleop_command_abi::kMaxDof> velocity_rad_s;
};

struct JakaNormalizedStatus {
  std::uint64_t sequence;
  std::int64_t sampled_monotonic_ns;
  bool session_alive;
  bool powered_on;
  bool servo_enabled;
  bool edg_ready;
  bool alarm;
  bool estop;
  bool collision;
};

struct JakaSdkFunctionTable {
  void* context;
  JakaFunctionResult (*login)(void*) noexcept;
  JakaFunctionResult (*set_edg_enabled)(void*, bool) noexcept;
  JakaFunctionResult (*set_servo_enabled)(void*, bool) noexcept;
  JakaFunctionResult (*send_joint_position)(
      void*, const double*, std::uint8_t, std::uint32_t) noexcept;
  JakaFunctionResult (*read_joint_feedback)(void*, JakaJointFeedback*) noexcept;
  JakaFunctionResult (*read_status)(void*, JakaNormalizedStatus*) noexcept;
  JakaFunctionResult (*stop_motion)(void*) noexcept;
  JakaFunctionResult (*logout)(void*) noexcept;
};

enum class ResumePreparationPolicy : std::uint8_t {
  kUnverified = 0,
  kKeepPrepared = 1,
  kRestartEdg = 2,
  kRestartEdgAndServo = 3,
};

enum class ThinJakaState : std::uint8_t {
  kDisconnected = 0,
  kConnecting = 1,
  kConnected = 2,
  kServoReady = 3,
  kStreaming = 4,
  kControlledStopping = 5,
  kStoppedReady = 6,
  kMeasuredStateRefresh = 7,
  kFaulted = 8,
  kCleanup = 9,
  kResetRequired = 10,
};

enum class ThinJakaCode : std::uint8_t {
  kOk = 0,
  kNeedMoreSamples = 1,
  kUnstableMeasurement = 2,
  kInvalidState = 3,
  kInvalidConfiguration = 4,
  kInvalidCommand = 5,
  kEpochMismatch = 6,
  kStale = 7,
  kTimingFault = 8,
  kSdkFailure = 9,
  kControllerFault = 10,
  kContinuityFault = 11,
  kCleanupFailure = 12,
};

struct ThinJakaConfig {
  std::uint8_t dof{6};
  PauseCommandPolicy pause_policy{PauseCommandPolicy::kUnverified};
  ResumePreparationPolicy resume_policy{ResumePreparationPolicy::kUnverified};
  RecoveryMeasurementPolicy measurement{};
  std::int64_t maximum_status_age_ns{32'000'000};
  std::int64_t maximum_tick_interval_ns{16'000'000};
  std::uint32_t status_poll_interval_ticks{2};
  std::uint32_t servo_step_num{1};
  double resume_position_tolerance_rad{1e-6};
};

struct ThinJakaSnapshot {
  ThinJakaState state;
  teleop_command_abi::StopReason fault_reason;
  bool session_owned;
  bool edg_enabled;
  bool servo_enabled;
  bool cleanup_failed;
  std::uint64_t safety_epoch;
  std::uint64_t last_output_sequence;
  std::uint64_t sent_command_count;
  std::uint64_t repeated_stopped_command_count;
  std::uint64_t status_poll_count;
  std::uint64_t superseded_command_count;
  std::uint64_t skipped_output_sequence_count;
  std::uint64_t tick_count;
  std::uint64_t tick_deadline_miss_count;
  std::uint64_t clutch_cycle_count;
  std::uint64_t old_epoch_rejection_count;
  std::int64_t maximum_tick_interval_ns;
  std::int64_t maximum_command_age_ns;
  double maximum_resume_position_delta_rad;
};

class ThinJakaTransportAdapter final {
 public:
  ThinJakaTransportAdapter(JakaSdkFunctionTable functions,
                           ThinJakaConfig config) noexcept;
  ThinJakaTransportAdapter(const ThinJakaTransportAdapter&) = delete;
  ThinJakaTransportAdapter& operator=(const ThinJakaTransportAdapter&) = delete;

  ThinJakaCode Connect(std::int64_t now_ns) noexcept;
  ThinJakaCode PrepareServo(std::int64_t now_ns) noexcept;
  ThinJakaCode BeginMeasuredStateRefresh(std::uint64_t new_safety_epoch,
                                         std::int64_t now_ns) noexcept;
  ThinJakaCode RefreshMeasuredState(
      std::int64_t now_ns,
      teleop_command_abi::MeasuredJointStateV1* measured) noexcept;
  ThinJakaCode StartStreaming(
      const teleop_command_abi::ShapedJointCommandV1& first_command,
      std::int64_t now_ns) noexcept;
  ThinJakaCode OfferLatest(
      const teleop_command_abi::ShapedJointCommandV1& command,
      std::int64_t now_ns) noexcept;
  ThinJakaCode Tick(std::int64_t now_ns) noexcept;
  ThinJakaCode PollStatus(std::int64_t now_ns) noexcept;
  ThinJakaCode Cleanup(std::int64_t now_ns) noexcept;
  ThinJakaCode ExplicitReset(std::int64_t now_ns) noexcept;
  void HardStop(teleop_command_abi::StopReason reason,
                std::int64_t now_ns) noexcept;

  ThinJakaSnapshot Snapshot() const noexcept;

 private:
  bool ValidFunctionTable() const noexcept;
  ThinJakaCode Fault(ThinJakaCode code,
                     teleop_command_abi::StopReason reason) noexcept;
  ThinJakaCode Send(
      const teleop_command_abi::ShapedJointCommandV1& command,
      std::int64_t now_ns) noexcept;
  ThinJakaCode ApplyResumePreparation() noexcept;
  ThinJakaCode ApplyStoppedPreparation() noexcept;
  ThinJakaCode RepeatStoppedCommand() noexcept;

  JakaSdkFunctionTable functions_;
  ThinJakaConfig config_;
  ThinJakaState state_;
  teleop_command_abi::StopReason fault_reason_;
  bool session_owned_;
  bool edg_enabled_;
  bool servo_enabled_;
  bool cleanup_failed_;
  bool pending_valid_;
  bool stopped_command_valid_;
  bool hard_stop_callback_called_;
  std::uint64_t safety_epoch_;
  std::uint64_t last_output_sequence_;
  std::uint64_t last_status_sequence_;
  std::uint64_t sent_command_count_;
  std::uint64_t repeated_stopped_command_count_;
  std::uint64_t status_poll_count_;
  std::uint64_t superseded_command_count_;
  std::uint64_t skipped_output_sequence_count_;
  std::uint64_t tick_count_;
  std::uint64_t tick_deadline_miss_count_;
  std::uint64_t clutch_cycle_count_;
  std::uint64_t old_epoch_rejection_count_;
  std::int64_t last_tick_ns_;
  std::int64_t maximum_tick_interval_ns_;
  std::int64_t maximum_command_age_ns_;
  double maximum_resume_position_delta_rad_;
  std::uint64_t refresh_epoch_;
  teleop_command_abi::MeasuredJointStateV1 measured_;
  teleop_command_abi::ShapedJointCommandV1 pending_;
  teleop_command_abi::ShapedJointCommandV1 stopped_command_;
  RecoveryMeasurementGate measurement_gate_;
};

}  // namespace teleop_shaping
