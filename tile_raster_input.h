#pragma once

#include <vector>

#include "nav_semantic.h"

struct NonWalkableVolume {
  float bmin[3];
  float bmax[3];
};

// A geometry-source-agnostic batch of triangles ready for the shared
// prepareRasterTriangles/applyNonWalkableCarves mapping (main.cpp). Every
// GeometrySource implementation (OBJ today, ForgeLight later) produces one of
// these; the two canonical raster/carve functions only ever see this type, so
// the semantic-to-area mapping cannot drift between geometry sources.
//
// Field shapes intentionally mirror Mesh's per-triangle arrays (main.cpp) so
// that code written against one compiles unchanged against the other.
struct TileRasterInput {
  std::vector<float> verts; // x,y,z triples
  std::vector<int> tris;    // 0-based triangle indices into verts
  std::vector<unsigned char> nonWalkableTris; // parallel to tris
  std::vector<NavSemantic> triangleSemantics; // parallel to tris
  std::vector<int> triangleObjects;           // parallel to tris
  std::vector<NonWalkableVolume> nonWalkableVolumes;
  bool semanticInput = false;
};
