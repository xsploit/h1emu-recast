#pragma once

#include "forgelight_nav_source.h"
#include "tile_raster_input.h"

// Fixed-origin terrain quad grid sampled from a HeightmapRgb. The lattice's
// origin and spacing are fixed once at construction (matching InstanceIndex's
// contract) and never re-anchored per tile query -- appendTile() only ever
// selects which already-fixed cells overlap the given bounds.
//
// Every emitted triangle is generated fresh with a deliberately upward (+Y)
// winding: unlike H1COL2 instance geometry, this class controls vertex order
// completely, so there is no winding-correction step to apply here -- that
// concern is specific to consuming externally-authored collision.
class TerrainLattice {
public:
  // heightmap is referenced, not owned; must outlive this object.
  // worldBminXZ/worldBmaxXZ fix the lattice's origin and cell-count once.
  // spacing is the fixed quad size in world units.
  TerrainLattice(const h1emu::nav::HeightmapRgb &heightmap,
                const float worldBminXZ[2], const float worldBmaxXZ[2],
                float spacing);

  // Appends terrain triangles (NavSemantic::Terrain, upward-wound) for every
  // fixed lattice cell overlapping [qmin, qmax] into out. Does NOT clear out
  // first -- callers that already populated instance-derived triangles keep
  // them; this only adds more.
  void appendTile(const float qmin[2], const float qmax[2],
                  TileRasterInput &out) const;

  float spacing() const { return spacing_; }

private:
  const h1emu::nav::HeightmapRgb &heightmap_;
  float origin_[2];
  int nx_;
  int nz_;
  float spacing_;
};
