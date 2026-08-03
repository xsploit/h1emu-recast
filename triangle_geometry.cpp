#include "triangle_geometry.h"

#include <cmath>

bool trianglePlaneHeightXZ(const float *a, const float *b, const float *c,
                           float x, float z, float &height) {
  const float denominator =
      (b[2] - c[2]) * (a[0] - c[0]) +
      (c[0] - b[0]) * (a[2] - c[2]);
  if (std::fabs(denominator) <= 1e-8f)
    return false;
  const float wa = ((b[2] - c[2]) * (x - c[0]) +
                    (c[0] - b[0]) * (z - c[2])) /
                   denominator;
  const float wb = ((c[2] - a[2]) * (x - c[0]) +
                    (a[0] - c[0]) * (z - c[2])) /
                   denominator;
  const float wc = 1.0f - wa - wb;
  constexpr float barycentricTolerance = 1e-4f;
  if (wa < -barycentricTolerance || wb < -barycentricTolerance ||
      wc < -barycentricTolerance || wa > 1.0f + barycentricTolerance ||
      wb > 1.0f + barycentricTolerance ||
      wc > 1.0f + barycentricTolerance)
    return false;
  height = wa * a[1] + wb * b[1] + wc * c[1];
  return true;
}
