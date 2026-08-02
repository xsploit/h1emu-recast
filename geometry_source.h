#pragma once

#include "tile_raster_input.h"

// A source of navmesh-bake geometry, queried one expanded Recast tile at a
// time. Every implementation (ObjGeometrySource today; a future ForgeLight
// native-collision source) returns triangles already classified into
// NavSemantic space (nav_semantic.h) via a TileRasterInput. The shared
// prepareRasterTriangles/applyNonWalkableCarves mapping (main.cpp) is the
// only place semantics turn into Recast areas/flags, so no GeometrySource
// may invent areas directly -- this keeps direct-build and TileCache output
// from drifting no matter which source produced the input geometry.
class GeometrySource {
public:
  virtual ~GeometrySource() = default;

  // Whole-source world bounds (equivalent to Mesh::bmin/bmax today). Must be
  // cheap -- no full geometry materialization.
  virtual void worldBounds(float bmin[3], float bmax[3]) const = 0;

  // True forces cfg.minRegionArea = 0 and skips the legacy low-hanging
  // walkable-obstacle filter in buildTile/buildTileCacheLayers, exactly as
  // Mesh::semanticInput does today for classified OBJ input.
  virtual bool semanticInput() const = 0;

  // qmin/qmax are the EXPANDED (bordered) tile query bounds in world XZ --
  // exactly what buildTile/buildTileCacheLayers already compute today before
  // querying geometry. Returns false (leaving out untouched) if nothing
  // overlaps this tile.
  virtual bool queryTile(const float qmin[2], const float qmax[2],
                         TileRasterInput &out) const = 0;
};
