#include "resampler_c_api.h"

#include "joint_servo_resampler.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <new>
#include <string>

namespace {
thread_local std::string last_error;

struct ResamplerHandle {
  jaka_servo::JointServoResampler resampler;
  std::array<double, 6> maximum_velocity{};
  std::array<double, 6> previous_position{};
  std::array<double, 6> previous_velocity{};
  std::array<double, 6> previous_acceleration{};
  double recoverable_acceleration = 0.0;
  double hard_acceleration = 0.0;
  double maximum_jerk = 0.0;
  std::uint64_t previous_command_ns = 0;
  bool transition_configured = false;
  bool previous_transition_limited = false;
};

struct MotionSample {
  std::array<double, 6> velocity{};
  std::array<double, 6> acceleration{};
  std::array<double, 6> jerk{};
};

std::array<double, 6> six(const double values[6]) {
  if (values == nullptr) throw std::invalid_argument("six-joint input is null");
  std::array<double, 6> result{};
  std::copy_n(values, result.size(), result.begin());
  return result;
}

template <typename Operation>
int guarded(Operation&& operation) {
  try {
    operation();
    last_error.clear();
    return 0;
  } catch (const std::exception& error) {
    last_error = error.what();
    return -1;
  }
}

ResamplerHandle& instance(void* handle) {
  if (handle == nullptr) throw std::invalid_argument("resampler handle is null");
  return *static_cast<ResamplerHandle*>(handle);
}

MotionSample measure(const ResamplerHandle& handle,
                     const jaka_servo::ResampledServoPoint& point,
                     std::uint64_t command_ns) {
  MotionSample sample{};
  if (command_ns == handle.previous_command_ns) return sample;
  if (command_ns < handle.previous_command_ns)
    throw std::runtime_error("output command timestamps moved backwards");
  const double dt_s =
      static_cast<double>(command_ns - handle.previous_command_ns) / 1e9;
  for (std::size_t joint = 0; joint < point.position.size(); ++joint) {
    sample.velocity[joint] =
        (point.position[joint] - handle.previous_position[joint]) / dt_s;
    sample.acceleration[joint] =
        (sample.velocity[joint] - handle.previous_velocity[joint]) / dt_s;
    sample.jerk[joint] =
        (sample.acceleration[joint] - handle.previous_acceleration[joint]) /
        dt_s;
  }
  return sample;
}

bool transition_required(const ResamplerHandle& handle,
                         const MotionSample& sample) {
  for (std::size_t joint = 0; joint < sample.velocity.size(); ++joint) {
    if (std::abs(sample.velocity[joint]) > handle.maximum_velocity[joint] + 1e-12 ||
        std::abs(sample.acceleration[joint]) >
            handle.recoverable_acceleration + 1e-12 ||
        std::abs(sample.jerk[joint]) > handle.maximum_jerk + 1e-12)
      return true;
  }
  return false;
}

void require_final_boundaries(const ResamplerHandle& handle,
                              const MotionSample& sample) {
  for (std::size_t joint = 0; joint < sample.velocity.size(); ++joint) {
    if (std::abs(sample.velocity[joint]) > handle.maximum_velocity[joint] + 1e-12)
      throw std::runtime_error(
          "selected output exceeds velocity boundary at J" +
          std::to_string(joint + 1) + ": value=" +
          std::to_string(sample.velocity[joint]) + " limit=" +
          std::to_string(handle.maximum_velocity[joint]));
    if (std::abs(sample.acceleration[joint]) > handle.hard_acceleration + 1e-12)
      throw std::runtime_error(
          "selected output exceeds acceleration boundary at J" +
          std::to_string(joint + 1) + ": value=" +
          std::to_string(sample.acceleration[joint]) + " limit=" +
          std::to_string(handle.hard_acceleration));
    if (!jaka_servo::output_jerk_within_hard_boundary(
            sample.jerk[joint], handle.maximum_jerk))
      throw std::runtime_error(
          "selected output exceeds jerk boundary at J" +
          std::to_string(joint + 1) + ": value=" +
          std::to_string(sample.jerk[joint]) + " limit=" +
          std::to_string(handle.maximum_jerk));
  }
}

void copy_point(const jaka_servo::ResampledServoPoint& result,
                const MotionSample& motion, bool limited, bool recovered,
                jaka_resampler_point_v1* point) {
  std::copy(result.position.begin(), result.position.end(), point->position_rad);
  std::copy(result.segment_velocity_rad_s.begin(),
            result.segment_velocity_rad_s.end(),
            point->segment_velocity_rad_s);
  std::copy(motion.velocity.begin(), motion.velocity.end(),
            point->emitted_velocity_rad_s);
  std::copy(motion.acceleration.begin(), motion.acceleration.end(),
            point->emitted_acceleration_rad_s2);
  std::copy(motion.jerk.begin(), motion.jerk.end(), point->emitted_jerk_rad_s3);
  point->servo_time_ns = result.servo_time_ns;
  point->from_sequence = result.from_sequence;
  point->to_sequence = result.to_sequence;
  point->from_accepted_ns = result.from_accepted_ns;
  point->to_accepted_ns = result.to_accepted_ns;
  point->alpha = result.alpha;
  point->endpoint = result.endpoint ? 1 : 0;
  point->transition_limited = limited ? 1 : 0;
  point->recovered_from_transition = recovered ? 1 : 0;
}

}  // namespace

extern "C" void* jaka_resampler_create(void) {
  try {
    last_error.clear();
    return new ResamplerHandle();
  } catch (const std::exception& error) {
    last_error = error.what();
    return nullptr;
  }
}

extern "C" void jaka_resampler_destroy(void* handle) {
  delete static_cast<ResamplerHandle*>(handle);
}

extern "C" int jaka_resampler_initialize(void* handle,
                                          const double position_rad[6],
                                          uint64_t servo_time_ns) {
  return guarded([&] {
    auto& value = instance(handle);
    value.previous_position = six(position_rad);
    value.previous_velocity.fill(0.0);
    value.previous_acceleration.fill(0.0);
    value.previous_command_ns = servo_time_ns;
    value.previous_transition_limited = false;
    value.resampler.initialize(value.previous_position, servo_time_ns);
  });
}

extern "C" int jaka_resampler_hold(void* handle,
                                    const double position_rad[6],
                                    uint64_t accepted_ns,
                                    uint64_t sequence) {
  return guarded([&] {
    instance(handle).resampler.hold(six(position_rad), accepted_ns, sequence);
  });
}

extern "C" int jaka_resampler_configure_transition(
    void* handle, const double maximum_velocity_rad_s[6],
    double recoverable_acceleration_rad_s2,
    double hard_acceleration_rad_s2, double maximum_jerk_rad_s3) {
  return guarded([&] {
    auto& value = instance(handle);
    value.maximum_velocity = six(maximum_velocity_rad_s);
    if (!std::all_of(value.maximum_velocity.begin(), value.maximum_velocity.end(),
                     [](double limit) {
                       return std::isfinite(limit) && limit > 0.0;
                     }) ||
        !std::isfinite(recoverable_acceleration_rad_s2) ||
        recoverable_acceleration_rad_s2 <= 0.0 ||
        !std::isfinite(hard_acceleration_rad_s2) ||
        hard_acceleration_rad_s2 < recoverable_acceleration_rad_s2 ||
        !std::isfinite(maximum_jerk_rad_s3) || maximum_jerk_rad_s3 <= 0.0)
      throw std::invalid_argument("invalid production transition limits");
    value.recoverable_acceleration = recoverable_acceleration_rad_s2;
    value.hard_acceleration = hard_acceleration_rad_s2;
    value.maximum_jerk = maximum_jerk_rad_s3;
    value.transition_configured = true;
  });
}

extern "C" int jaka_resampler_accept(void* handle,
                                      const double destination_rad[6],
                                      uint64_t accepted_ns,
                                      uint64_t sequence) {
  return guarded([&] {
    instance(handle).resampler.accept(six(destination_rad), accepted_ns, sequence);
  });
}

extern "C" int jaka_resampler_evaluate_selected(
    void* handle, uint64_t servo_time_ns, jaka_resampler_point_v1* point) {
  return guarded([&] {
    if (point == nullptr) throw std::invalid_argument("resampler point is null");
    auto& value = instance(handle);
    if (!value.transition_configured)
      throw std::runtime_error("production transition limits are not configured");
    const auto proposed = value.resampler.evaluate(servo_time_ns);
    const auto proposed_motion = measure(value, proposed, servo_time_ns);
    const bool limited = transition_required(value, proposed_motion);
    auto selected = proposed;
    if (limited && servo_time_ns > value.previous_command_ns) {
      const double dt_s =
          static_cast<double>(servo_time_ns - value.previous_command_ns) / 1e9;
      selected = jaka_servo::transition_limited_point(
          proposed, value.previous_position, value.previous_velocity,
          value.previous_acceleration, dt_s,
          std::min(value.recoverable_acceleration, value.hard_acceleration),
          value.maximum_jerk);
    }
    const auto selected_motion = measure(value, selected, servo_time_ns);
    require_final_boundaries(value, selected_motion);
    if (limited && servo_time_ns > value.previous_command_ns)
      value.resampler.commit_transition_limited(selected);
    else
      value.resampler.commit(selected, servo_time_ns);
    const bool recovered = value.previous_transition_limited && !limited;
    value.previous_position = selected.position;
    value.previous_velocity = selected_motion.velocity;
    value.previous_acceleration = selected_motion.acceleration;
    value.previous_command_ns = servo_time_ns;
    value.previous_transition_limited = limited;
    copy_point(selected, selected_motion, limited, recovered, point);
  });
}

extern "C" const char* jaka_resampler_last_error(void) {
  return last_error.c_str();
}
