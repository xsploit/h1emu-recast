#include "threshold_portal_geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

int appendThresholdPortalPads(TileRasterInput &mesh, float extension) {
  if (extension <= 0.0f)
    return 0;
  const int originalTriangleCount = (int)mesh.triangleSemantics.size();
  int pads = 0;
  for (int triId = 0; triId < originalTriangleCount; ++triId) {
    if (mesh.triangleSemantics[triId] != NavSemantic::Threshold)
      continue;
    const int sourceIndices[3] = {mesh.tris[triId * 3 + 0],
                                  mesh.tris[triId * 3 + 1],
                                  mesh.tris[triId * 3 + 2]};
    float edgeLengths[3]{};
    for (int edge = 0; edge < 3; ++edge) {
      const int from = sourceIndices[edge];
      const int to = sourceIndices[(edge + 1) % 3];
      const float dx = mesh.verts[to * 3 + 0] - mesh.verts[from * 3 + 0];
      const float dz = mesh.verts[to * 3 + 2] - mesh.verts[from * 3 + 2];
      edgeLengths[edge] = std::sqrt(dx * dx + dz * dz);
    }
    const int shortEdge = (int)(std::min_element(
                                      edgeLengths, edgeLengths + 3) -
                                  edgeLengths);
    const float shortLength = edgeLengths[shortEdge];
    const float longLength =
        *std::max_element(edgeLengths, edgeLengths + 3);
    // A threshold portal must have a clear narrow travel axis. Refuse to
    // synthesize geometry for degenerate or roughly square authored surfaces.
    if (shortLength <= 1e-4f || longLength <= shortLength * 2.0f)
      continue;

    const int axisFrom = sourceIndices[shortEdge];
    const int axisTo = sourceIndices[(shortEdge + 1) % 3];
    float axisX = mesh.verts[axisTo * 3 + 0] - mesh.verts[axisFrom * 3 + 0];
    float axisZ = mesh.verts[axisTo * 3 + 2] - mesh.verts[axisFrom * 3 + 2];
    axisX /= shortLength;
    axisZ /= shortLength;
    float minProjection = std::numeric_limits<float>::max();
    float maxProjection = std::numeric_limits<float>::lowest();
    for (int sourceIndex : sourceIndices) {
      const float projection = mesh.verts[sourceIndex * 3 + 0] * axisX +
                               mesh.verts[sourceIndex * 3 + 2] * axisZ;
      minProjection = std::min(minProjection, projection);
      maxProjection = std::max(maxProjection, projection);
    }
    const float midpoint = (minProjection + maxProjection) * 0.5f;
    const int firstVertex = (int)(mesh.verts.size() / 3);
    for (int sourceIndex : sourceIndices) {
      const float x = mesh.verts[sourceIndex * 3 + 0];
      const float y = mesh.verts[sourceIndex * 3 + 1];
      const float z = mesh.verts[sourceIndex * 3 + 2];
      const float projection = x * axisX + z * axisZ;
      const float direction = projection < midpoint ? -1.0f : 1.0f;
      mesh.verts.push_back(x + axisX * extension * direction);
      mesh.verts.push_back(y);
      mesh.verts.push_back(z + axisZ * extension * direction);
    }
    mesh.tris.push_back(firstVertex + 0);
    mesh.tris.push_back(firstVertex + 1);
    mesh.tris.push_back(firstVertex + 2);
    mesh.nonWalkableTris.push_back(0);
    mesh.triangleSemantics.push_back(NavSemantic::Threshold);
    // The portal is a more specific authored surface on the same object. This
    // lets it win the existing same-object precedence check against a coarse
    // sill or wall face. It is extended only along the threshold's narrow
    // travel axis, so jambs outside the authored opening still carve normally.
    mesh.triangleObjects.push_back(mesh.triangleObjects[triId]);
    pads++;
  }
  return pads;
}
