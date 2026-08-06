#pragma once

#include <cstddef>

#include "Recast.h"

struct CompactComponentFilterResult {
  int componentsVisited = 0;
  int componentsPruned = 0;
  int spansPruned = 0;
};

// Removes only tiny, disconnected walkable islands that are wholly inside a
// tile. Components touching the expanded tile border are retained because
// they may connect to the neighboring tile. Components containing a protected
// semantic area are also retained, regardless of size.
CompactComponentFilterResult pruneTinyInteriorCompactComponents(
    rcCompactHeightfield &heightfield, int borderSize, int maxCells,
    const unsigned char *protectedAreas, std::size_t protectedAreaCount);
