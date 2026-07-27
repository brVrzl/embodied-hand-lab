#include "teleop_shaping/clutch_recovery_transport.hpp"

#include <cmath>
#include <limits>

namespace teleop_shaping {
namespace {

constexpr std::uint8_t kNoIndex =
    static_cast<std::uint8_t>(teleop_command_abi::kNoFieldIndex);

bool IsKnownFields(JointSampleFields fields) noexcept {
  return fields == JointSampleFields::kPositionOnly ||
         fields == JointSampleFields::kPositionVelocity ||
         fields == JointSampleFields::kPositionVelocityAcceleration;
}

RecoveryMeasurementResult Result(RecoveryMeasurementCode code,
                                 RecoveryMeasurementQuality quality,
                                 std::uint8_t count,
                                 std::uint8_t index = kNoIndex) noexcept {
  return {code, quality, count, index};
}

}  // namespace

bool SupportsSessionHeldRecovery(PauseCommandPolicy policy) noexcept {
  return policy == PauseCommandPolicy::kNoCommandRequired;
}

InMemoryFakeJakaSdkInterface::InMemoryFakeJakaSdkInterface() noexcept
    : session_alive_(false),
      streaming_prepared_(false),
      pause_policy_(PauseCommandPolicy::kUnverified),
      sample_{},
      health_{},
      connect_result_(FakeSdkIoCode::kOk),
      prepare_result_(FakeSdkIoCode::kOk),
      send_result_(FakeSdkIoCode::kOk),
      read_result_(FakeSdkIoCode::kOk),
      health_result_(FakeSdkIoCode::kOk),
      cleanup_result_(FakeSdkIoCode::kOk),
      connect_count_(0),
      prepare_count_(0),
      send_count_(0),
      read_count_(0),
      health_count_(0),
      cleanup_count_(0) {}

FakeSdkIoCode InMemoryFakeJakaSdkInterface::ConsumeOneShot(
    FakeSdkIoCode* value) noexcept {
  const FakeSdkIoCode result = *value;
  *value = FakeSdkIoCode::kOk;
  return result;
}

FakeSdkIoCode InMemoryFakeJakaSdkInterface::Connect() noexcept {
  ++connect_count_;
  if (session_alive_) return FakeSdkIoCode::kInvalidState;
  const auto result = ConsumeOneShot(&connect_result_);
  if (result == FakeSdkIoCode::kOk) session_alive_ = true;
  return result;
}

FakeSdkIoCode InMemoryFakeJakaSdkInterface::PrepareStreaming() noexcept {
  ++prepare_count_;
  if (!session_alive_) return FakeSdkIoCode::kInvalidState;
  const auto result = ConsumeOneShot(&prepare_result_);
  if (result == FakeSdkIoCode::kOk) streaming_prepared_ = true;
  return result;
}

FakeSdkIoCode InMemoryFakeJakaSdkInterface::SendShaped(
    const teleop_command_abi::ShapedJointCommandV1&) noexcept {
  ++send_count_;
  if (!session_alive_ || !streaming_prepared_) {
    return FakeSdkIoCode::kInvalidState;
  }
  return ConsumeOneShot(&send_result_);
}

FakeSdkIoCode InMemoryFakeJakaSdkInterface::ReadJointSample(
    FakeSdkJointSample* sample) noexcept {
  ++read_count_;
  if (!session_alive_ || sample == nullptr) return FakeSdkIoCode::kInvalidState;
  const auto result = ConsumeOneShot(&read_result_);
  if (result == FakeSdkIoCode::kOk) *sample = sample_;
  return result;
}

FakeSdkIoCode InMemoryFakeJakaSdkInterface::ReadHealth(
    teleop_command_abi::TransportHealthV1* health) noexcept {
  ++health_count_;
  if (!session_alive_ || health == nullptr) return FakeSdkIoCode::kInvalidState;
  const auto result = ConsumeOneShot(&health_result_);
  if (result == FakeSdkIoCode::kOk) *health = health_;
  return result;
}

FakeSdkIoCode InMemoryFakeJakaSdkInterface::Cleanup() noexcept {
  ++cleanup_count_;
  const auto result = ConsumeOneShot(&cleanup_result_);
  if (result == FakeSdkIoCode::kOk) {
    streaming_prepared_ = false;
    session_alive_ = false;
  }
  return result;
}

RecoveryMeasurementGate::RecoveryMeasurementGate(
    RecoveryMeasurementPolicy policy) noexcept
    : policy_(policy), have_previous_(false), previous_{}, stable_samples_(0) {}

void RecoveryMeasurementGate::Reset() noexcept {
  have_previous_ = false;
  previous_ = {};
  stable_samples_ = 0;
}

RecoveryMeasurementResult RecoveryMeasurementGate::Observe(
    const FakeSdkJointSample& sample, std::uint64_t safety_epoch,
    std::int64_t now_ns,
    teleop_command_abi::MeasuredJointStateV1* measured) noexcept {
  if (measured == nullptr || safety_epoch == 0U) {
    return Result(RecoveryMeasurementCode::kEpochError,
                  RecoveryMeasurementQuality::kNone, stable_samples_);
  }
  if (policy_.dof == 0U || policy_.dof > teleop_command_abi::kMaxDof ||
      policy_.stable_sample_count == 0U || policy_.maximum_sample_age_ns < 0 ||
      policy_.maximum_sample_interval_ns <= 0 ||
      !std::isfinite(policy_.stationary_velocity_rad_s) ||
      policy_.stationary_velocity_rad_s < 0.0 || sample.dof != policy_.dof ||
      sample.sample_sequence == 0U || sample.sampled_monotonic_ns < 0 ||
      !IsKnownFields(sample.fields)) {
    return Result(RecoveryMeasurementCode::kInvalidSample,
                  RecoveryMeasurementQuality::kNone, stable_samples_);
  }
  if (sample.sampled_monotonic_ns > now_ns ||
      now_ns - sample.sampled_monotonic_ns > policy_.maximum_sample_age_ns) {
    return Result(RecoveryMeasurementCode::kStale,
                  RecoveryMeasurementQuality::kNone, stable_samples_);
  }
  for (std::size_t i = 0; i < teleop_command_abi::kMaxDof; ++i) {
    const bool used = i < sample.dof;
    if (!std::isfinite(sample.position_rad[i]) ||
        !std::isfinite(sample.velocity_rad_s[i]) ||
        !std::isfinite(sample.acceleration_rad_s2[i])) {
      return Result(RecoveryMeasurementCode::kInvalidSample,
                    RecoveryMeasurementQuality::kNone, stable_samples_,
                    static_cast<std::uint8_t>(i));
    }
    if ((!used && (sample.position_rad[i] != 0.0 ||
                   sample.velocity_rad_s[i] != 0.0 ||
                   sample.acceleration_rad_s2[i] != 0.0)) ||
        (sample.fields == JointSampleFields::kPositionOnly &&
         (sample.velocity_rad_s[i] != 0.0 ||
          sample.acceleration_rad_s2[i] != 0.0)) ||
        (sample.fields == JointSampleFields::kPositionVelocity &&
         sample.acceleration_rad_s2[i] != 0.0)) {
      return Result(RecoveryMeasurementCode::kInvalidSample,
                    RecoveryMeasurementQuality::kNone, stable_samples_,
                    static_cast<std::uint8_t>(i));
    }
  }

  if (have_previous_ &&
      (sample.sample_sequence <= previous_.sample_sequence ||
       sample.sampled_monotonic_ns <= previous_.sampled_monotonic_ns)) {
    return Result(RecoveryMeasurementCode::kSequenceError,
                  RecoveryMeasurementQuality::kNone, stable_samples_);
  }

  std::array<double, teleop_command_abi::kMaxDof> velocity{};
  RecoveryMeasurementQuality quality = RecoveryMeasurementQuality::kNone;
  if (sample.fields == JointSampleFields::kPositionVelocityAcceleration) {
    velocity = sample.velocity_rad_s;
    quality = RecoveryMeasurementQuality::kDirectQVelocityAcceleration;
  } else if (sample.fields == JointSampleFields::kPositionVelocity) {
    velocity = sample.velocity_rad_s;
    quality = RecoveryMeasurementQuality::kDirectQVelocityZeroAccelerationAfterStable;
  } else {
    quality = RecoveryMeasurementQuality::kEstimatedVelocityZeroAccelerationAfterStable;
    if (!have_previous_) {
      previous_ = sample;
      have_previous_ = true;
      stable_samples_ = 1U;
      return Result(RecoveryMeasurementCode::kNeedMoreSamples, quality,
                    stable_samples_);
    }
    const std::int64_t interval_ns =
        sample.sampled_monotonic_ns - previous_.sampled_monotonic_ns;
    if (interval_ns > policy_.maximum_sample_interval_ns) {
      previous_ = sample;
      stable_samples_ = 1U;
      return Result(RecoveryMeasurementCode::kNeedMoreSamples, quality,
                    stable_samples_);
    }
    const double interval_s = static_cast<double>(interval_ns) * 1e-9;
    for (std::size_t i = 0; i < sample.dof; ++i) {
      velocity[i] = (sample.position_rad[i] - previous_.position_rad[i]) / interval_s;
    }
  }

  if (sample.fields != JointSampleFields::kPositionVelocityAcceleration) {
    bool stable = true;
    for (std::size_t i = 0; i < sample.dof; ++i) {
      stable = stable &&
               std::abs(velocity[i]) <= policy_.stationary_velocity_rad_s;
    }
    if (!stable) {
      previous_ = sample;
      have_previous_ = true;
      stable_samples_ = 0U;
      return Result(RecoveryMeasurementCode::kUnstable, quality, stable_samples_);
    }
    if (stable_samples_ < std::numeric_limits<std::uint8_t>::max()) {
      stable_samples_ = static_cast<std::uint8_t>(stable_samples_ + 1U);
    }
    previous_ = sample;
    have_previous_ = true;
    if (stable_samples_ < policy_.stable_sample_count) {
      return Result(RecoveryMeasurementCode::kNeedMoreSamples, quality,
                    stable_samples_);
    }
  }

  teleop_command_abi::MeasuredJointStateV1 result{};
  result.header =
      teleop_command_abi::MakeHeaderV1<teleop_command_abi::MeasuredJointStateV1>();
  result.state_sequence = sample.sample_sequence;
  result.safety_epoch = safety_epoch;
  result.measured_monotonic_ns = sample.sampled_monotonic_ns;
  result.dof = sample.dof;
  result.validity = teleop_command_abi::MeasurementValidity::kValid;
  result.position_rad = sample.position_rad;
  result.velocity_rad_s = velocity;
  if (sample.fields == JointSampleFields::kPositionVelocityAcceleration) {
    result.acceleration_rad_s2 = sample.acceleration_rad_s2;
  }
  const teleop_command_abi::ValidationContext context{
      now_ns, policy_.dof, safety_epoch, 0U};
  if (!teleop_command_abi::Validate(result, context).ok) {
    return Result(RecoveryMeasurementCode::kInvalidSample, quality,
                  stable_samples_);
  }
  previous_ = sample;
  have_previous_ = true;
  *measured = result;
  return Result(RecoveryMeasurementCode::kReady, quality, stable_samples_);
}

RecoveryMeasurementResult ReadRecoveryMeasurement(
    IFakeJakaSdkInterface* sdk, RecoveryMeasurementGate* gate,
    std::uint64_t safety_epoch, std::int64_t now_ns,
    teleop_command_abi::MeasuredJointStateV1* measured) noexcept {
  if (sdk == nullptr || gate == nullptr || measured == nullptr) {
    return Result(RecoveryMeasurementCode::kIoFailure,
                  RecoveryMeasurementQuality::kNone, 0U);
  }
  FakeSdkJointSample sample{};
  if (sdk->ReadJointSample(&sample) != FakeSdkIoCode::kOk) {
    return Result(RecoveryMeasurementCode::kIoFailure,
                  RecoveryMeasurementQuality::kNone, 0U);
  }
  return gate->Observe(sample, safety_epoch, now_ns, measured);
}

FakeSendOutcome ClassifyFakeSdkSend(FakeSdkIoCode code) noexcept {
  if (code == FakeSdkIoCode::kOk) return FakeSendOutcome::kOk;
  if (code == FakeSdkIoCode::kControllerFault) {
    return FakeSendOutcome::kControllerAlarm;
  }
  if (code == FakeSdkIoCode::kTransportFailure || code == FakeSdkIoCode::kStale) {
    return FakeSendOutcome::kTransportFailure;
  }
  return FakeSendOutcome::kRejected;
}

}  // namespace teleop_shaping
