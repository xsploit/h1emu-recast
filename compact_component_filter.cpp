#include "compact_component_filter.h"

#include <algorithm>
#include <vector>

namespace {

bool isProtectedArea(unsigned char area, const unsigned char *protectedAreas,
                     std::size_t protectedAreaCount) {
  if (!protectedAreas || protectedAreaCount == 0)
    return false;
  return std::find(protectedAreas, protectedAreas + protectedAreaCount, area) !=
         protectedAreas + protectedAreaCount;
}

} // namespace

CompactComponentFilterResult pruneTinyInteriorCompactComponents(
    rcCompactHeightfield &heightfield, int borderSize, int maxCells,
    const unsigned char *protectedAreas, std::size_t protectedAreaCount) {
  CompactComponentFilterResult result;
  if (heightfield.spanCount <= 0 || maxCells < 1 || borderSize < 0)
    return result;

  std::vector<int> spanX(static_cast<std::size_t>(heightfield.spanCount), -1);
  std::vector<int> spanZ(static_cast<std::size_t>(heightfield.spanCount), -1);
  for (int z = 0; z < heightfield.height; ++z) {
    for (int x = 0; x < heightfield.width; ++x) {
      const rcCompactCell &cell =
          heightfield.cells[x + z * heightfield.width];
      for (unsigned int offset = 0; offset < cell.count; ++offset) {
        const int span = static_cast<int>(cell.index + offset);
        spanX[static_cast<std::size_t>(span)] = x;
        spanZ[static_cast<std::size_t>(span)] = z;
      }
    }
  }

  std::vector<unsigned char> visited(
      static_cast<std::size_t>(heightfield.spanCount), 0);
  std::vector<int> stack;
  std::vector<int> component;
  for (int seed = 0; seed < heightfield.spanCount; ++seed) {
    if (visited[static_cast<std::size_t>(seed)] ||
        heightfield.areas[seed] == RC_NULL_AREA)
      continue;

    ++result.componentsVisited;
    bool touchesBorder = false;
    bool containsProtectedArea = false;
    stack.clear();
    component.clear();
    stack.push_back(seed);
    visited[static_cast<std::size_t>(seed)] = 1;

    while (!stack.empty()) {
      const int span = stack.back();
      stack.pop_back();
      component.push_back(span);

      const int x = spanX[static_cast<std::size_t>(span)];
      const int z = spanZ[static_cast<std::size_t>(span)];
      touchesBorder =
          touchesBorder || x < borderSize || z < borderSize ||
          x >= heightfield.width - borderSize ||
          z >= heightfield.height - borderSize;
      containsProtectedArea =
          containsProtectedArea ||
          isProtectedArea(heightfield.areas[span], protectedAreas,
                          protectedAreaCount);

      const rcCompactSpan &compactSpan = heightfield.spans[span];
      for (int direction = 0; direction < 4; ++direction) {
        const int connection = rcGetCon(compactSpan, direction);
        if (connection == RC_NOT_CONNECTED)
          continue;
        const int neighborX = x + rcGetDirOffsetX(direction);
        const int neighborZ = z + rcGetDirOffsetY(direction);
        if (neighborX < 0 || neighborZ < 0 ||
            neighborX >= heightfield.width ||
            neighborZ >= heightfield.height)
          continue;
        const rcCompactCell &neighborCell =
            heightfield.cells[neighborX + neighborZ * heightfield.width];
        const int neighbor =
            static_cast<int>(neighborCell.index) + connection;
        if (neighbor < 0 || neighbor >= heightfield.spanCount ||
            visited[static_cast<std::size_t>(neighbor)] ||
            heightfield.areas[neighbor] == RC_NULL_AREA)
          continue;
        visited[static_cast<std::size_t>(neighbor)] = 1;
        stack.push_back(neighbor);
      }
    }

    if (static_cast<int>(component.size()) > maxCells || touchesBorder ||
        containsProtectedArea)
      continue;

    ++result.componentsPruned;
    result.spansPruned += static_cast<int>(component.size());
    for (int span : component)
      heightfield.areas[span] = RC_NULL_AREA;
  }

  return result;
}
