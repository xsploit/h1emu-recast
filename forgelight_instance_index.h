#pragma once

#include <cstdint>
#include <vector>

#include "forgelight_nav_source.h"

namespace h1emu::nav {

// CSR-style spatial index over H1Col2Document instances, bucketed by their
// own stored world-space AABB (CollisionInstance::transform[10..15], no
// mesh-geometry re-derivation needed) into fixed-size XZ cells. Mirrors the
// same two-pass count/prefix-sum/scatter shape TriGrid (main.cpp) uses for
// triangles, but flattened into CSR arrays rather than a dense
// vector<vector<int>> grid -- appropriate at instance-count scale (hundreds
// of thousands), where per-cell heap allocations would otherwise dominate.
class InstanceIndex {
public:
  // worldBminXZ/worldBmaxXZ fix this index's origin and grid dimensions once;
  // every later query() reuses them unchanged, matching TriGrid's contract
  // that the lattice is never re-anchored per query. Pass the caller's
  // authoritative world bounds (e.g. the heightmap's declared extent), not a
  // bound recomputed from the instances alone, so the index lines up with
  // whatever else shares that same world frame.
  void build(const H1Col2Document &collision, const float worldBminXZ[2],
            const float worldBmaxXZ[2], float cellSize = 25.6f);

  // Returns instance indices (into H1Col2Document::instances) whose stored
  // world AABB overlaps [qmin, qmax] in XZ, sorted ascending and deduplicated
  // (an instance whose AABB spans multiple cells would otherwise be returned
  // once per covered cell).
  std::vector<std::uint32_t> query(const float qmin[2],
                                   const float qmax[2]) const;

  int cellCountX() const { return nx_; }
  int cellCountZ() const { return nz_; }

private:
  std::vector<std::uint32_t> cellOffsets_;   // size nx_*nz_ + 1 (CSR row ptrs)
  std::vector<std::uint32_t> cellInstances_; // flattened membership
  int nx_ = 0;
  int nz_ = 0;
  float origin_[2] = {0.0f, 0.0f};
  float cellSize_ = 25.6f;

  int clampedCellX(float worldX) const;
  int clampedCellZ(float worldZ) const;
};

} // namespace h1emu::nav
