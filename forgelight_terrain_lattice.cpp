#include "forgelight_terrain_lattice.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

using h1emu::nav::HeightmapRgb;

TerrainLattice::TerrainLattice(const HeightmapRgb &heightmap,
                               const float worldBminXZ[2],
                               const float worldBmaxXZ[2], float spacing)
    : heightmap_(heightmap), spacing_(spacing) {
  if (!(spacing_ > 0.0f))
    throw std::runtime_error("TerrainLattice spacing must be positive");
  if (!(worldBmaxXZ[0] > worldBminXZ[0]) ||
      !(worldBmaxXZ[1] > worldBminXZ[1]))
    throw std::runtime_error(
        "TerrainLattice world bounds must be non-degenerate");
  origin_[0] = worldBminXZ[0];
  origin_[1] = worldBminXZ[1];
  nx_ = std::max(
      1, static_cast<int>(
             std::ceil((worldBmaxXZ[0] - worldBminXZ[0]) / spacing_)));
  nz_ = std::max(
      1, static_cast<int>(
             std::ceil((worldBmaxXZ[1] - worldBminXZ[1]) / spacing_)));
}

void TerrainLattice::appendTile(const float qmin[2], const float qmax[2],
                                TileRasterInput &out) const {
  const int cx0 = std::max(
      0, static_cast<int>(std::floor((qmin[0] - origin_[0]) / spacing_)));
  const int cz0 = std::max(
      0, static_cast<int>(std::floor((qmin[1] - origin_[1]) / spacing_)));
  const int cx1 = std::min(
      nx_ - 1,
      static_cast<int>(std::floor((qmax[0] - origin_[0]) / spacing_)));
  const int cz1 = std::min(
      nz_ - 1,
      static_cast<int>(std::floor((qmax[1] - origin_[1]) / spacing_)));
  if (cx1 < cx0 || cz1 < cz0)
    return;

  const auto emitTri = [&](const float p0[3], const float p1[3],
                           const float p2[3]) {
    const int base = static_cast<int>(out.verts.size() / 3);
    for (const float *p : {p0, p1, p2}) {
      out.verts.push_back(p[0]);
      out.verts.push_back(p[1]);
      out.verts.push_back(p[2]);
    }
    out.tris.push_back(base);
    out.tris.push_back(base + 1);
    out.tris.push_back(base + 2);
    out.nonWalkableTris.push_back(0);
    out.triangleSemantics.push_back(NavSemantic::Terrain);
    // No H1COL2 instance owns a lattice cell; -1 keeps terrain out of
    // applyNonWalkableCarves' per-object walkable/obstacle grouping, which
    // is keyed by triangleObjects and would otherwise see terrain triangles
    // as if they belonged to whichever real instance index happened to
    // collide with a reused non-negative value.
    out.triangleObjects.push_back(-1);
  };

  for (int cz = cz0; cz <= cz1; ++cz) {
    for (int cx = cx0; cx <= cx1; ++cx) {
      const float x0 = origin_[0] + static_cast<float>(cx) * spacing_;
      const float z0 = origin_[1] + static_cast<float>(cz) * spacing_;
      const float x1 = x0 + spacing_;
      const float z1 = z0 + spacing_;

      const float a[3] = {x0, heightmap_.sampleNearestWorld(x0, z0), z0};
      const float b[3] = {x1, heightmap_.sampleNearestWorld(x1, z0), z0};
      const float c[3] = {x1, heightmap_.sampleNearestWorld(x1, z1), z1};
      const float d[3] = {x0, heightmap_.sampleNearestWorld(x0, z1), z1};

      // Hand-verified upward (+Y) winding for a flat quad: cross(d-a, b-a)
      // and cross(d-b, c-b) both yield a positive Y component.
      emitTri(a, d, b);
      emitTri(b, d, c);
    }
  }
}
