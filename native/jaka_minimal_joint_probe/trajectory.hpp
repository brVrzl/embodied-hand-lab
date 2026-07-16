#pragma once

#include <algorithm>
#include <cmath>

namespace jaka_gate3c {

struct TrajectorySample {
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double jerk = 0.0;
};

inline TrajectorySample septic_state(double start, double displacement, double duration, double time) {
  const double s = std::clamp(time / duration, 0.0, 1.0);
  const double s2 = s * s, s3 = s2 * s, s4 = s3 * s;
  const double s5 = s4 * s, s6 = s5 * s, s7 = s6 * s;
  const double p = 35.0 * s4 - 84.0 * s5 + 70.0 * s6 - 20.0 * s7;
  const double dp = 140.0 * s3 - 420.0 * s4 + 420.0 * s5 - 140.0 * s6;
  const double ddp = 420.0 * s2 - 1680.0 * s3 + 2100.0 * s4 - 840.0 * s5;
  const double dddp = 840.0 * s - 5040.0 * s2 + 8400.0 * s3 - 4200.0 * s4;
  return {start + displacement * p, displacement * dp / duration,
          displacement * ddp / (duration * duration),
          displacement * dddp / (duration * duration * duration)};
}

}  // namespace jaka_gate3c
