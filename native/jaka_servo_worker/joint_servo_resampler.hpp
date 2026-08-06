#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace jaka_servo {

constexpr std::uint64_t kControllerServoPeriodNs = 8'000'000;
constexpr std::array<double, 6> kJointLower{-6.28, -2.09, -2.27, -6.28, -2.09, -6.28};
constexpr std::array<double, 6> kJointUpper{6.28, 2.09, 2.27, 6.28, 2.09, 6.28};

// Numerical comparison envelope for the final jerk boundary only.  The
// nominal jerk limit is never changed.  The absolute 1e-6 rad/s^3 term and
// relative 2.5e-7 term cover the observed 1.3e-5 rad/s^3 finite-difference
// round-off at the configured 62.831853 rad/s^3 boundary while remaining
// below any meaningful command excursion.  This tolerance is shared by the
// native worker and the simulation-facing resampler C ABI.
constexpr double kOutputJerkHardBoundaryToleranceAbsoluteRadS3 = 1e-6;
constexpr double kOutputJerkHardBoundaryToleranceRelative = 2.5e-7;

// The final output assertion below remains at the configured project limit.
// This smaller internal target is only for the transition shaper, so the
// finite-difference jerk of a shaped point has deterministic room below the
// final assertion instead of repeatedly landing on it through round-off.
constexpr double kOutputJerkTransitionHeadroomFraction = 0.005;

inline double output_jerk_transition_target(double limit_rad_s3) {
  return std::abs(limit_rad_s3) *
      (1.0 - kOutputJerkTransitionHeadroomFraction);
}

inline double output_jerk_hard_boundary_tolerance(double limit_rad_s3) {
  return kOutputJerkHardBoundaryToleranceAbsoluteRadS3 +
      kOutputJerkHardBoundaryToleranceRelative * std::abs(limit_rad_s3);
}

inline double output_jerk_hard_boundary_with_tolerance(double limit_rad_s3) {
  return std::abs(limit_rad_s3) +
      output_jerk_hard_boundary_tolerance(limit_rad_s3);
}

inline bool output_jerk_within_hard_boundary(double raw_jerk_rad_s3,
                                             double limit_rad_s3) {
  return std::isfinite(raw_jerk_rad_s3) && std::isfinite(limit_rad_s3) &&
      std::abs(raw_jerk_rad_s3) <=
          output_jerk_hard_boundary_with_tolerance(limit_rad_s3);
}

inline void validate_manufacturer_joint_position_limits(
    const std::array<double, 6>& target) {
  for (std::size_t joint = 0; joint < target.size(); ++joint) {
    if (!std::isfinite(target[joint]) || target[joint] < kJointLower[joint] ||
        target[joint] > kJointUpper[joint])
      throw std::runtime_error(
          "joint target violates JAKA manufacturer position limits");
  }
}

struct ResampledServoPoint {
  std::array<double, 6> position{};
  std::array<double, 6> segment_velocity_rad_s{};
  std::uint64_t servo_time_ns = 0;
  std::uint64_t from_sequence = 0;
  std::uint64_t to_sequence = 0;
  std::uint64_t from_accepted_ns = 0;
  std::uint64_t to_accepted_ns = 0;
  double alpha = 0.0;
  bool endpoint = false;
};

inline ResampledServoPoint transition_limited_point(
    const ResampledServoPoint& proposed,
    const std::array<double, 6>& previous_position,
    const std::array<double, 6>& previous_velocity,
    const std::array<double, 6>& previous_acceleration,
    double dt_s, double acceleration_boundary_rad_s2,
    double jerk_boundary_rad_s3) {
  if (!(std::isfinite(dt_s) && dt_s > 0.0))
    throw std::runtime_error("output transition limiter timestep is invalid");
  if (!(std::isfinite(acceleration_boundary_rad_s2) &&
        acceleration_boundary_rad_s2 > 0.0 &&
        std::isfinite(jerk_boundary_rad_s3) && jerk_boundary_rad_s3 > 0.0))
    throw std::runtime_error("output transition limits are invalid");
  const double acceleration_boundary = acceleration_boundary_rad_s2 -
      std::max(1e-9, acceleration_boundary_rad_s2 * 1e-9);
  const double jerk_boundary =
      output_jerk_transition_target(jerk_boundary_rad_s3);
  const double transition_frequency =
      jerk_boundary / acceleration_boundary;
  ResampledServoPoint limited = proposed;
  limited.endpoint = false;
  for (std::size_t joint = 0; joint < limited.position.size(); ++joint) {
    // Track the PWL point with a critically damped transition.  Slewing only
    // toward the one-tick endpoint velocity has no braking term: after a
    // destination replacement it can cross the destination and keep
    // integrating away while jerk limits delay the sign change.  The
    // position/velocity feedback supplies that braking without changing a
    // PWL point which already satisfies all output boundaries.
    const double desired_acceleration =
        transition_frequency * transition_frequency *
            (proposed.position[joint] - previous_position[joint]) -
        2.0 * transition_frequency * previous_velocity[joint];
    const double maximum_acceleration_change = jerk_boundary * dt_s;
    const double jerk_limited_acceleration = std::clamp(
        desired_acceleration,
        previous_acceleration[joint] - maximum_acceleration_change,
        previous_acceleration[joint] + maximum_acceleration_change);
    const double acceleration = std::clamp(
        jerk_limited_acceleration, -acceleration_boundary,
        acceleration_boundary);
    const double velocity = previous_velocity[joint] + acceleration * dt_s;
    limited.position[joint] = previous_position[joint] + velocity * dt_s;
  }
  validate_manufacturer_joint_position_limits(limited.position);
  return limited;
}

// Production causal latest-destination/PWL resampler. Accepted target time
// defines segment duration; a replacement starts at the last emitted point.
class JointServoResampler {
 public:
  explicit JointServoResampler(
      std::uint64_t servo_period_ns = kControllerServoPeriodNs)
      : servo_period_ns_(servo_period_ns) {
    if (servo_period_ns_ == 0 ||
        servo_period_ns_ % kControllerServoPeriodNs != 0)
      throw std::invalid_argument(
          "servo period must be a positive multiple of 8 ms");
  }

  void initialize(const std::array<double, 6>& measured,
                  std::uint64_t servo_time_ns) {
    validate_manufacturer_joint_position_limits(measured);
    if (servo_time_ns == 0)
      throw std::runtime_error("resampler initialization time is invalid");
    emitted_ = start_ = destination_ = measured;
    last_servo_time_ns_ = segment_start_ns_ = segment_end_ns_ = servo_time_ns;
    initialized_ = true;
    has_accepted_ = false;
    internal_hold_ = false;
    active_ = false;
  }

  void hold(const std::array<double, 6>& position, std::uint64_t accepted_ns,
            std::uint64_t sequence) {
    if (!initialized_)
      throw std::runtime_error("resampler is not initialized");
    validate_manufacturer_joint_position_limits(position);
    if (accepted_ns == 0)
      throw std::runtime_error("hold target has an invalid timestamp");
    emitted_ = start_ = destination_ = position;
    segment_start_ns_ = segment_end_ns_ = last_servo_time_ns_;
    last_accepted_ns_ = from_accepted_ns_ = to_accepted_ns_ = accepted_ns;
    from_sequence_ = to_sequence_ = sequence;
    has_accepted_ = true;
    internal_hold_ = true;
    active_ = false;
  }

  void accept(const std::array<double, 6>& destination,
              std::uint64_t accepted_ns, std::uint64_t sequence) {
    if (!initialized_)
      throw std::runtime_error("resampler is not initialized");
    validate_manufacturer_joint_position_limits(destination);
    if (accepted_ns == 0 || sequence == 0)
      throw std::runtime_error(
          "accepted target has an invalid resampling timestamp or sequence");
    if (!has_accepted_) {
      last_accepted_ns_ = accepted_ns;
      from_accepted_ns_ = to_accepted_ns_ = accepted_ns;
      from_sequence_ = to_sequence_ = sequence;
      start_ = emitted_;
      destination_ = destination;
      segment_start_ns_ = last_servo_time_ns_;
      segment_end_ns_ = segment_start_ns_ + servo_period_ns_;
      maximum_segment_duration_ns_ = servo_period_ns_;
      has_accepted_ = true;
      active_ = true;
      return;
    }
    if (accepted_ns <= last_accepted_ns_)
      throw std::runtime_error(
          "accepted target resampling timestamps are not strictly monotonic");
    if (sequence <= to_sequence_)
      throw std::runtime_error(
          "accepted target resampling sequences are not strictly monotonic");
    if (internal_hold_) {
      last_accepted_ns_ = from_accepted_ns_ = to_accepted_ns_ = accepted_ns;
      from_sequence_ = to_sequence_ = sequence;
      start_ = emitted_;
      destination_ = destination;
      segment_start_ns_ = last_servo_time_ns_;
      segment_end_ns_ = segment_start_ns_ + servo_period_ns_;
      maximum_segment_duration_ns_ =
          std::max(maximum_segment_duration_ns_, servo_period_ns_);
      internal_hold_ = false;
      active_ = true;
      return;
    }
    if (active_ && last_servo_time_ns_ < segment_end_ns_) ++preemptions_;
    const std::uint64_t duration_ns = accepted_ns - last_accepted_ns_;
    maximum_segment_duration_ns_ =
        std::max(maximum_segment_duration_ns_, duration_ns);
    start_ = emitted_;
    destination_ = destination;
    segment_start_ns_ = last_servo_time_ns_;
    if (duration_ns >
        std::numeric_limits<std::uint64_t>::max() - segment_start_ns_)
      throw std::runtime_error(
          "accepted target segment duration overflows servo time");
    segment_end_ns_ = segment_start_ns_ + duration_ns;
    from_sequence_ = to_sequence_;
    to_sequence_ = sequence;
    from_accepted_ns_ = last_accepted_ns_;
    to_accepted_ns_ = accepted_ns;
    last_accepted_ns_ = accepted_ns;
    active_ = true;
    ++destination_switches_;
  }

  ResampledServoPoint evaluate(std::uint64_t servo_time_ns) const {
    if (!initialized_ || !has_accepted_)
      throw std::runtime_error("resampler has no accepted target");
    if (servo_time_ns < last_servo_time_ns_)
      throw std::runtime_error("servo evaluation time moved backwards");
    ResampledServoPoint point{};
    point.servo_time_ns = servo_time_ns;
    point.from_sequence = from_sequence_;
    point.to_sequence = to_sequence_;
    point.from_accepted_ns = from_accepted_ns_;
    point.to_accepted_ns = to_accepted_ns_;
    if (!active_ || segment_end_ns_ <= segment_start_ns_) {
      point.position = emitted_;
      point.alpha = active_ ? 1.0 : 0.0;
      point.endpoint = active_;
      return point;
    }
    point.alpha = std::clamp(
        static_cast<double>(servo_time_ns - segment_start_ns_) /
            static_cast<double>(segment_end_ns_ - segment_start_ns_),
        0.0, 1.0);
    for (std::size_t joint = 0; joint < point.position.size(); ++joint) {
      point.position[joint] =
          start_[joint] + point.alpha * (destination_[joint] - start_[joint]);
      point.segment_velocity_rad_s[joint] =
          (destination_[joint] - start_[joint]) * 1e9 /
          static_cast<double>(segment_end_ns_ - segment_start_ns_);
    }
    point.endpoint = point.alpha >= 1.0;
    return point;
  }

  void commit(const ResampledServoPoint& point, std::uint64_t command_ns) {
    if (point.servo_time_ns < last_servo_time_ns_)
      throw std::runtime_error("committed servo point moved backwards in time");
    commit_emitted(point);
    if (point.endpoint && active_) {
      active_ = false;
      ++endpoint_points_;
      if (command_ns >= to_accepted_ns_)
        maximum_endpoint_latency_ns_ = std::max(
            maximum_endpoint_latency_ns_, command_ns - to_accepted_ns_);
    }
  }

  void commit_transition_limited(const ResampledServoPoint& point) {
    if (point.servo_time_ns < last_servo_time_ns_)
      throw std::runtime_error(
          "transition-limited servo point moved backwards in time");
    commit_emitted(point);
    start_ = emitted_;
    segment_start_ns_ = last_servo_time_ns_;
    if (servo_period_ns_ >
        std::numeric_limits<std::uint64_t>::max() - segment_start_ns_)
      throw std::runtime_error(
          "transition-limited segment duration overflows servo time");
    segment_end_ns_ = segment_start_ns_ + servo_period_ns_;
    from_sequence_ = to_sequence_;
    from_accepted_ns_ = to_accepted_ns_;
    active_ = true;
    ++transition_limited_points_;
  }

  const std::array<double, 6>& emitted() const { return emitted_; }
  std::uint64_t emitted_points() const { return emitted_points_; }
  std::uint64_t repeated_points() const { return repeated_points_; }
  std::uint64_t destination_switches() const { return destination_switches_; }
  std::uint64_t preemptions() const { return preemptions_; }
  std::uint64_t endpoint_points() const { return endpoint_points_; }
  std::uint64_t maximum_segment_duration_ns() const {
    return maximum_segment_duration_ns_;
  }
  std::uint64_t maximum_endpoint_latency_ns() const {
    return maximum_endpoint_latency_ns_;
  }
  std::uint64_t transition_limited_points() const {
    return transition_limited_points_;
  }
  bool active() const { return active_; }

 private:
  void commit_emitted(const ResampledServoPoint& point) {
    bool repeated = true;
    for (std::size_t joint = 0; joint < emitted_.size(); ++joint)
      repeated = repeated && point.position[joint] == emitted_[joint];
    repeated_points_ += repeated ? 1 : 0;
    emitted_ = point.position;
    last_servo_time_ns_ = point.servo_time_ns;
    ++emitted_points_;
  }

  std::array<double, 6> emitted_{}, start_{}, destination_{};
  std::uint64_t servo_period_ns_ = kControllerServoPeriodNs;
  std::uint64_t last_servo_time_ns_ = 0;
  std::uint64_t segment_start_ns_ = 0, segment_end_ns_ = 0;
  std::uint64_t last_accepted_ns_ = 0;
  std::uint64_t from_accepted_ns_ = 0, to_accepted_ns_ = 0;
  std::uint64_t from_sequence_ = 0, to_sequence_ = 0;
  std::uint64_t emitted_points_ = 0, repeated_points_ = 0;
  std::uint64_t destination_switches_ = 0, preemptions_ = 0,
                endpoint_points_ = 0;
  std::uint64_t transition_limited_points_ = 0;
  std::uint64_t maximum_segment_duration_ns_ = 0,
                maximum_endpoint_latency_ns_ = 0;
  bool initialized_ = false, has_accepted_ = false, internal_hold_ = false,
       active_ = false;
};

}  // namespace jaka_servo
