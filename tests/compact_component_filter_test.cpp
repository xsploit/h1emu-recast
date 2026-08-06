#include "compact_component_filter.h"
#include "nav_semantic.h"
#include "RecastAlloc.h"

#include <cstdio>
#include <cstdlib>
#include <utility>
#include <vector>

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", message);
    std::exit(1);
  }
}

int directionBetween(int x, int z, int neighborX, int neighborZ) {
  for (int direction = 0; direction < 4; ++direction) {
    if (x + rcGetDirOffsetX(direction) == neighborX &&
        z + rcGetDirOffsetY(direction) == neighborZ)
      return direction;
  }
  return -1;
}

struct Fixture {
  static constexpr int width = 12;
  static constexpr int height = 12;
  rcCompactHeightfield *field = rcAllocCompactHeightfield();
  std::vector<int> spanAt = std::vector<int>(width * height, -1);

  Fixture(const std::vector<std::pair<int, int>> &cells) {
    require(field != nullptr, "heightfield allocation");
    field->width = width;
    field->height = height;
    field->spanCount = static_cast<int>(cells.size());
    // Deliberately leave this at zero. The filter must use the explicit build
    // border passed by its caller, not this field before partitioning sets it.
    field->borderSize = 0;
    field->cells = static_cast<rcCompactCell *>(
        rcAlloc(sizeof(rcCompactCell) * width * height, RC_ALLOC_PERM));
    field->spans = static_cast<rcCompactSpan *>(
        rcAlloc(sizeof(rcCompactSpan) * cells.size(), RC_ALLOC_PERM));
    field->areas = static_cast<unsigned char *>(
        rcAlloc(sizeof(unsigned char) * cells.size(), RC_ALLOC_PERM));
    require(field->cells && field->spans && field->areas,
            "compact arrays allocation");

    for (int i = 0; i < width * height; ++i) {
      field->cells[i].index = 0;
      field->cells[i].count = 0;
    }
    for (std::size_t i = 0; i < cells.size(); ++i) {
      const auto [x, z] = cells[i];
      field->cells[x + z * width].index = static_cast<unsigned int>(i);
      field->cells[x + z * width].count = 1;
      spanAt[x + z * width] = static_cast<int>(i);
      field->spans[i] = {};
      for (int direction = 0; direction < 4; ++direction)
        rcSetCon(field->spans[i], direction, RC_NOT_CONNECTED);
      field->areas[i] = NAV_AREA_TERRAIN;
    }
  }

  ~Fixture() { rcFreeCompactHeightfield(field); }

  int span(int x, int z) const { return spanAt[x + z * width]; }

  void connect(int x, int z, int neighborX, int neighborZ) {
    const int left = span(x, z);
    const int right = span(neighborX, neighborZ);
    require(left >= 0 && right >= 0, "connected cells must exist");
    const int direction = directionBetween(x, z, neighborX, neighborZ);
    const int reverse = directionBetween(neighborX, neighborZ, x, z);
    require(direction >= 0 && reverse >= 0, "cells must be adjacent");
    rcSetCon(field->spans[left], direction, 0);
    rcSetCon(field->spans[right], reverse, 0);
  }
};

} // namespace

int main() {
  const std::vector<std::pair<int, int>> cells = {
      // Three-cell interior island: prune.
      {3, 3}, {4, 3}, {5, 3},
      // Five-cell interior island: preserve.
      {3, 6}, {4, 6}, {5, 6}, {6, 6}, {7, 6},
      // Two-cell island touching the explicit one-cell tile border: preserve.
      {0, 9}, {1, 9},
      // Protected semantic singletons: preserve.
      {9, 3}, {9, 5}, {9, 7}};
  Fixture fixture(cells);
  fixture.connect(3, 3, 4, 3);
  fixture.connect(4, 3, 5, 3);
  fixture.connect(3, 6, 4, 6);
  fixture.connect(4, 6, 5, 6);
  fixture.connect(5, 6, 6, 6);
  fixture.connect(6, 6, 7, 6);
  fixture.connect(0, 9, 1, 9);
  fixture.field->areas[fixture.span(9, 3)] = NAV_AREA_STAIR;
  fixture.field->areas[fixture.span(9, 5)] = NAV_AREA_RAMP;
  fixture.field->areas[fixture.span(9, 7)] = NAV_AREA_THRESHOLD;

  const unsigned char protectedAreas[] = {
      NAV_AREA_STAIR, NAV_AREA_RAMP, NAV_AREA_THRESHOLD};
  const CompactComponentFilterResult result =
      pruneTinyInteriorCompactComponents(*fixture.field, 1, 4,
                                         protectedAreas, 3);

  require(result.componentsVisited == 6, "six components visited");
  require(result.componentsPruned == 1, "only tiny interior island pruned");
  require(result.spansPruned == 3, "three tiny spans pruned");
  require(fixture.field->areas[fixture.span(3, 3)] == RC_NULL_AREA,
          "tiny interior component removed");
  require(fixture.field->areas[fixture.span(7, 6)] == NAV_AREA_TERRAIN,
          "five-cell component retained");
  require(fixture.field->areas[fixture.span(1, 9)] == NAV_AREA_TERRAIN,
          "component connected to border retained");
  require(fixture.field->areas[fixture.span(9, 3)] == NAV_AREA_STAIR,
          "stair component retained");
  require(fixture.field->areas[fixture.span(9, 5)] == NAV_AREA_RAMP,
          "ramp component retained");
  require(fixture.field->areas[fixture.span(9, 7)] == NAV_AREA_THRESHOLD,
          "threshold component retained");

  std::puts("compact component filter contract PASS");
  return 0;
}
