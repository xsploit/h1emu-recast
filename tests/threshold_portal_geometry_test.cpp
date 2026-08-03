#include "threshold_portal_geometry.h"

#include <algorithm>
#include <cstdio>

int main() {
  TileRasterInput mesh;
  mesh.verts = {0.0f, 0.0f, 0.0f, 0.17f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.1f, 0.17f, 0.0f, 1.1f};
  mesh.tris = {0, 1, 2, 2, 1, 3};
  mesh.nonWalkableTris = {0, 0};
  mesh.triangleSemantics = {NavSemantic::Threshold, NavSemantic::Threshold};
  mesh.triangleObjects = {42, 42};

  if (appendThresholdPortalPads(mesh, 0.4f) != 2) {
    std::fprintf(stderr, "expected two threshold portal pads\n");
    return 1;
  }
  if (mesh.tris.size() != 12 || mesh.triangleSemantics.size() != 4 ||
      mesh.nonWalkableTris.size() != 4 || mesh.triangleObjects.size() != 4) {
    std::fprintf(stderr, "threshold portal arrays lost cardinality\n");
    return 1;
  }
  float minX = mesh.verts[4 * 3 + 0];
  float maxX = minX;
  float minZ = mesh.verts[4 * 3 + 2];
  float maxZ = minZ;
  for (int vertex = 4; vertex < (int)(mesh.verts.size() / 3); ++vertex) {
    minX = std::min(minX, mesh.verts[vertex * 3 + 0]);
    maxX = std::max(maxX, mesh.verts[vertex * 3 + 0]);
    minZ = std::min(minZ, mesh.verts[vertex * 3 + 2]);
    maxZ = std::max(maxZ, mesh.verts[vertex * 3 + 2]);
  }
  if (minX > -0.399f || maxX < 0.569f) {
    std::fprintf(stderr, "threshold pad did not extend along its narrow axis\n");
    return 1;
  }
  if (minZ < -0.001f || maxZ > 1.101f) {
    std::fprintf(stderr, "threshold pad expanded into the jamb axis\n");
    return 1;
  }
  if (mesh.triangleObjects[2] != 42 || mesh.triangleObjects[3] != 42) {
    std::fprintf(stderr, "synthetic pads lost source carve precedence\n");
    return 1;
  }
  if (mesh.triangleSemantics[0] != NavSemantic::Threshold ||
      mesh.triangleSemantics[1] != NavSemantic::Threshold ||
      mesh.triangleSemantics[2] != NavSemantic::Threshold ||
      mesh.triangleSemantics[3] != NavSemantic::Threshold ||
      mesh.nonWalkableTris[0] != 0 || mesh.nonWalkableTris[1] != 0 ||
      mesh.nonWalkableTris[2] != 0 || mesh.nonWalkableTris[3] != 0 ||
      mesh.tris[0] != 0 || mesh.tris[1] != 1 || mesh.tris[2] != 2 ||
      mesh.tris[3] != 2 || mesh.tris[4] != 1 || mesh.tris[5] != 3) {
    std::fprintf(stderr, "threshold pads changed source triangle contracts\n");
    return 1;
  }

  TileRasterInput square;
  square.verts = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                  0.0f, 0.0f, 1.0f};
  square.tris = {0, 1, 2};
  square.nonWalkableTris = {0};
  square.triangleSemantics = {NavSemantic::Threshold};
  square.triangleObjects = {7};
  if (appendThresholdPortalPads(square, 0.4f) != 0) {
    std::fprintf(stderr, "ambiguous threshold surface was padded\n");
    return 1;
  }

  TileRasterInput ordinary = square;
  ordinary.triangleSemantics = {NavSemantic::FloorInterior};
  if (appendThresholdPortalPads(ordinary, 0.4f) != 0 ||
      ordinary.tris.size() != 3 || ordinary.verts.size() != 9) {
    std::fprintf(stderr, "ordinary walkable surface was padded\n");
    return 1;
  }
  TileRasterInput disabled = mesh;
  if (appendThresholdPortalPads(disabled, 0.0f) != 0 ||
      appendThresholdPortalPads(disabled, -0.4f) != 0 ||
      disabled.tris.size() != mesh.tris.size() ||
      disabled.verts.size() != mesh.verts.size()) {
    std::fprintf(stderr, "disabled threshold padding mutated geometry\n");
    return 1;
  }

  std::puts("threshold portal geometry tests PASS");
  return 0;
}
