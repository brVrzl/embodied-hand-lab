#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "teleop_command_abi/abi_v1.hpp"
#include "teleop_shaping/fake_jaka_lifecycle.hpp"

namespace teleop_shaping {

// The real JAKA policy remains kUnverified until a separately authorized
// no-motion hardware gate establishes what the controller requires while
// stopped. Only kNoCommandRequired is supported by this reference contract.
enum class PauseCommandPolicy : std::uint8_t {
  kUnverified = 0,
  kNoCommandRequired = 1,
  kRepeatStoppedPositionRequired = 2,
};

bool SupportsSessionHeldRecovery(PauseCommandPolicy policy) noexcept;

enum class JointSampleFields : std::uint8_t {
  kPositionOnly = 0,
  kPositionVelocity = 1,
  kPositionVelocityAcceleration = 2,
};

enum class FakeSdkIoCode : std::uint8_t {
  kOk = 0,
  kTransportFailure = 1,
  kControllerFault = 2,
  kStale = 3,
  kInvalidState = 4,
};

struct FakeSdkJointSample {
  std::uint64_t sample_sequence;
  std::int64_t sampled_monotonic_ns;
  std::uint8_t dof;
  JointSampleFields fields;
  std::array<double, teleop_command_abi::kMaxDof> position_rad;
  std::array<double, teleop_command_abi::kMaxDof> velocity_rad_s;
  std::array<double, teleop_command_abi::kMaxDof> acceleration_rad_s2;
};

// Deliberately SDK-free. A future real adapter may implement an equivalent
// internal seam, but vendor types must not cross the command ABI boundary.
class IFakeJakaSdkInterface {
 public:
  virtual ~IFakeJakaSdkInterface() = default;
  virtual FakeSdkIoCode Connect() noexcept = 0;
  virtual FakeSdkIoCode PrepareStreaming() noexcept = 0;
  virtual FakeSdkIoCode SendShaped(
      const teleop_command_abi::ShapedJointCommandV1& command) noexcept = 0;
  virtual FakeSdkIoCode ReadJointSample(FakeSdkJointSample* sample) noexcept = 0;
  virtual FakeSdkIoCode ReadHealth(
      teleop_command_abi::TransportHealthV1* health) noexcept = 0;
  virtual FakeSdkIoCode Cleanup() noexcept = 0;
  virtual PauseCommandPolicy pause_command_policy() const noexcept = 0;
  virtual bool session_alive() const noexcept = 0;
};

class InMemoryFakeJakaSdkInterface final : public IFakeJakaSdkInterface {
 public:
  InMemoryFakeJakaSdkInterface() noexcept;

  FakeSdkIoCode Connect() noexcept override;
  FakeSdkIoCode PrepareStreaming() noexcept override;
  FakeSdkIoCode SendShaped(
      const teleop_command_abi::ShapedJointCommandV1& command) noexcept override;
  FakeSdkIoCode ReadJointSample(FakeSdkJointSample* sample) noexcept override;
  FakeSdkIoCode ReadHealth(
      teleop_command_abi::TransportHealthV1* health) noexcept override;
  FakeSdkIoCode Cleanup() noexcept override;
  PauseCommandPolicy pause_command_policy() const noexcept override {
    return pause_policy_;
  }
  bool session_alive() const noexcept override { return session_alive_; }

  void SetJointSample(const FakeSdkJointSample& sample) noexcept { sample_ = sample; }
  void SetHealth(const teleop_command_abi::TransportHealthV1& health) noexcept {
    health_ = health;
  }
  void SetPauseCommandPolicy(PauseCommandPolicy policy) noexcept {
    pause_policy_ = policy;
  }
  void SetNextConnectResult(FakeSdkIoCode code) noexcept { connect_result_ = code; }
  void SetNextPrepareResult(FakeSdkIoCode code) noexcept { prepare_result_ = code; }
  void SetNextSendResult(FakeSdkIoCode code) noexcept { send_result_ = code; }
  void SetNextReadResult(FakeSdkIoCode code) noexcept { read_result_ = code; }
  void SetNextHealthResult(FakeSdkIoCode code) noexcept { health_result_ = code; }
  void SetNextCleanupResult(FakeSdkIoCode code) noexcept { cleanup_result_ = code; }

  std::uint64_t connect_count() const noexcept { return connect_count_; }
  std::uint64_t prepare_count() const noexcept { return prepare_count_; }
  std::uint64_t send_count() const noexcept { return send_count_; }
  std::uint64_t read_count() const noexcept { return read_count_; }
  std::uint64_t health_count() const noexcept { return health_count_; }
  std::uint64_t cleanup_count() const noexcept { return cleanup_count_; }

 private:
  static FakeSdkIoCode ConsumeOneShot(FakeSdkIoCode* value) noexcept;

  bool session_alive_;
  bool streaming_prepared_;
  PauseCommandPolicy pause_policy_;
  FakeSdkJointSample sample_;
  teleop_command_abi::TransportHealthV1 health_;
  FakeSdkIoCode connect_result_;
  FakeSdkIoCode prepare_result_;
  FakeSdkIoCode send_result_;
  FakeSdkIoCode read_result_;
  FakeSdkIoCode health_result_;
  FakeSdkIoCode cleanup_result_;
  std::uint64_t connect_count_;
  std::uint64_t prepare_count_;
  std::uint64_t send_count_;
  std::uint64_t read_count_;
  std::uint64_t health_count_;
  std::uint64_t cleanup_count_;
};

enum class RecoveryMeasurementCode : std::uint8_t {
  kReady = 0,
  kNeedMoreSamples = 1,
  kUnstable = 2,
  kStale = 3,
  kInvalidSample = 4,
  kSequenceError = 5,
  kEpochError = 6,
  kIoFailure = 7,
};

enum class RecoveryMeasurementQuality : std::uint8_t {
  kNone = 0,
  kDirectQVelocityAcceleration = 1,
  kDirectQVelocityZeroAccelerationAfterStable = 2,
  kEstimatedVelocityZeroAccelerationAfterStable = 3,
};

struct RecoveryMeasurementPolicy {
  std::uint8_t dof{6};
  std::uint8_t stable_sample_count{3};
  std::int64_t maximum_sample_age_ns{32'000'000};
  std::int64_t maximum_sample_interval_ns{32'000'000};
  double stationary_velocity_rad_s{0.002};
};

struct RecoveryMeasurementResult {
  RecoveryMeasurementCode code;
  RecoveryMeasurementQuality quality;
  std::uint8_t stable_samples;
  std::uint8_t offending_index;
};

class RecoveryMeasurementGate final {
 public:
  explicit RecoveryMeasurementGate(RecoveryMeasurementPolicy policy) noexcept;
  void Reset() noexcept;
  RecoveryMeasurementResult Observe(
      const FakeSdkJointSample& sample, std::uint64_t safety_epoch,
      std::int64_t now_ns,
      teleop_command_abi::MeasuredJointStateV1* measured) noexcept;

 private:
  RecoveryMeasurementPolicy policy_;
  bool have_previous_;
  FakeSdkJointSample previous_;
  std::uint8_t stable_samples_;
};

RecoveryMeasurementResult ReadRecoveryMeasurement(
    IFakeJakaSdkInterface* sdk, RecoveryMeasurementGate* gate,
    std::uint64_t safety_epoch, std::int64_t now_ns,
    teleop_command_abi::MeasuredJointStateV1* measured) noexcept;

FakeSendOutcome ClassifyFakeSdkSend(FakeSdkIoCode code) noexcept;

}  // namespace teleop_shaping
