#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct jaka_resampler_point_v1 {
  double position_rad[6];
  double segment_velocity_rad_s[6];
  double emitted_velocity_rad_s[6];
  double emitted_acceleration_rad_s2[6];
  double emitted_jerk_rad_s3[6];
  uint64_t servo_time_ns;
  uint64_t from_sequence;
  uint64_t to_sequence;
  uint64_t from_accepted_ns;
  uint64_t to_accepted_ns;
  double alpha;
  uint8_t endpoint;
  uint8_t transition_limited;
  uint8_t recovered_from_transition;
} jaka_resampler_point_v1;

void* jaka_resampler_create(void);
void jaka_resampler_destroy(void* handle);
int jaka_resampler_initialize(void* handle, const double position_rad[6],
                              uint64_t servo_time_ns);
int jaka_resampler_hold(void* handle, const double position_rad[6],
                        uint64_t accepted_ns, uint64_t sequence);
int jaka_resampler_configure_transition(
    void* handle, const double maximum_velocity_rad_s[6],
    double recoverable_acceleration_rad_s2,
    double hard_acceleration_rad_s2, double maximum_jerk_rad_s3);
int jaka_resampler_accept(void* handle, const double destination_rad[6],
                          uint64_t accepted_ns, uint64_t sequence);
int jaka_resampler_evaluate_selected(void* handle, uint64_t servo_time_ns,
                                     jaka_resampler_point_v1* point);
const char* jaka_resampler_last_error(void);

#ifdef __cplusplus
}
#endif
