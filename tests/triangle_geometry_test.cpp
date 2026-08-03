#include "triangle_geometry.h"

#include <cmath>
#include <cstdio>

namespace {

bool nearlyEqual(float left, float right, float tolerance = 1e-5f) {
  return std::fabs(left - right) <= tolerance;
}

} // namespace

int main() {
  {
    const float a[3] = {0.0f, 0.0f, 0.0f};
    const float b[3] = {2.0f, 2.0f, 0.0f};
    const float c[3] = {0.0f, 4.0f, 2.0f};
    float height = -1.0f;
    if (!trianglePlaneHeightXZ(a, b, c, 0.5f, 0.5f, height) ||
        !nearlyEqual(height, 1.5f)) {
      std::fprintf(stderr, "inside-triangle height interpolation failed\n");
      return 1;
    }
  }

  {
    // This cell-center sample lies inside the projected AABB of a thin,
    // near-vertical triangle but outside the triangle itself. Evaluating the
    // infinite plane here invents a surface height and can carve an unrelated
    // walkable span in a neighbouring voxel.
    const float a[3] = {0.0f, 0.0f, 0.0f};
    const float b[3] = {0.01f, 0.26f, 1.0f};
    const float c[3] = {0.02f, 0.0f, 0.0f};
    float height = -1.0f;
    if (trianglePlaneHeightXZ(a, b, c, 0.015f, 0.9f, height)) {
      std::fprintf(stderr, "outside-triangle plane extrapolation accepted\n");
      return 1;
    }
  }

  {
    const float a[3] = {0.0f, 0.0f, 0.0f};
    const float b[3] = {0.0f, 1.0f, 0.0f};
    const float c[3] = {0.0f, 2.0f, 1.0f};
    float height = -1.0f;
    if (trianglePlaneHeightXZ(a, b, c, 0.0f, 0.5f, height)) {
      std::fprintf(stderr, "degenerate projected triangle accepted\n");
      return 1;
    }
  }

  std::puts("triangle geometry tests PASS");
  return 0;
}
