#pragma once

#include "forgelight_instance_index.h"
#include "forgelight_nav_source.h"
#include "geometry_source.h"

// Tile-local GeometrySource over native ForgeLight collision (H1COL2
// instances + H1SEM1 per-triangle semantics). Does not build a whole-world
// Mesh: queryTile() transforms only the instances the InstanceIndex reports
// as overlapping the expanded tile bounds.
//
// Both documents must already be mutually validated (loadH1Sem1's own
// validateH1Sem1Compatibility, called during loading) before construction;
// this class re-derives nothing about kind/semantic compatibility, it only
// consumes H1SEM1 semantics directly by triangle order, never H1COL2 actor
// kind, per the project's non-negotiable semantic-provenance rule.
class ForgelightGeometrySource : public GeometrySource {
public:
  // worldBmin/worldBmax are the full 3D world bounds this source reports via
  // worldBounds() (typically the caller's --global-bounds) -- fixed once,
  // matching InstanceIndex's contract that the lattice is never re-anchored
  // per tile query.
  ForgelightGeometrySource(const h1emu::nav::H1Col2Document &collision,
                           const h1emu::nav::H1Sem1Document &semantics,
                           const float worldBmin[3], const float worldBmax[3],
                           float instanceCellSize = 25.6f);

  void worldBounds(float bmin[3], float bmax[3]) const override;
  bool semanticInput() const override { return true; }
  bool queryTile(const float qmin[2], const float qmax[2],
                TileRasterInput &out) const override;

private:
  const h1emu::nav::H1Col2Document &collision_;
  const h1emu::nav::H1Sem1Document &semantics_;
  h1emu::nav::InstanceIndex instanceIndex_;
  float worldBmin_[3];
  float worldBmax_[3];
};
