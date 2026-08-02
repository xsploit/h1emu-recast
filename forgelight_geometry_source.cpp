#include "forgelight_geometry_source.h"

#include <cstring>
#include <stdexcept>

#include "nav_semantic.h"

namespace {

using h1emu::nav::CollisionInstance;
using h1emu::nav::CollisionMesh;
using h1emu::nav::H1Col2Document;
using h1emu::nav::H1Sem1Document;
using h1emu::nav::SemanticId;

void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

// Standard optimized quaternion-vector rotation (no explicit quat*quat):
// t = 2 * cross(q.xyz, v); v' = v + q.w * t + cross(q.xyz, t).
void rotateByQuaternion(const float qxyz[3], float qw, const float v[3],
                        float out[3]) {
  float t[3];
  cross3(qxyz, v, t);
  t[0] *= 2.0f;
  t[1] *= 2.0f;
  t[2] *= 2.0f;
  float crossQt[3];
  cross3(qxyz, t, crossQt);
  out[0] = v[0] + qw * t[0] + crossQt[0];
  out[1] = v[1] + qw * t[1] + crossQt[1];
  out[2] = v[2] + qw * t[2] + crossQt[2];
}

// H1SEM1 SemanticId -> the same NavSemantic space prepareRasterTriangles/
// applyNonWalkableCarves (main.cpp) already consume from OBJ input. Kept as
// an explicit table, not arithmetic on the enum value: the two enums are
// defined independently in separate headers and any numeric alignment
// between them is coincidental, not a contract.
NavSemantic mapSemanticId(SemanticId id) {
  switch (id) {
  case SemanticId::Terrain:
    return NavSemantic::Terrain;
  case SemanticId::Road:
    return NavSemantic::Road;
  case SemanticId::FloorExterior:
    return NavSemantic::FloorExterior;
  case SemanticId::FloorInterior:
    return NavSemantic::FloorInterior;
  case SemanticId::Stair:
    return NavSemantic::Stair;
  case SemanticId::Ramp:
    return NavSemantic::Ramp;
  case SemanticId::Threshold:
    return NavSemantic::Threshold;
  case SemanticId::ObstacleStatic:
    return NavSemantic::ObstacleStatic;
  case SemanticId::DoorPanelDynamic:
    return NavSemantic::DoorPanelDynamic;
  case SemanticId::Exclude:
    return NavSemantic::Exclude;
  case SemanticId::Unknown:
    return NavSemantic::Unknown;
  case SemanticId::Invalid:
    break;
  }
  // parseH1Sem1 (forgelight_nav_source.cpp) unconditionally rejects
  // SemanticId::Invalid for every triangle, so a validly-parsed
  // H1Sem1Document can never reach this. Fail closed rather than silently
  // treating an impossible value as walkable or excluded.
  throw std::runtime_error("H1SEM1 semantic id is structurally invalid");
}

} // namespace

ForgelightGeometrySource::ForgelightGeometrySource(
    const H1Col2Document &collision, const H1Sem1Document &semantics,
    const float worldBmin[3], const float worldBmax[3],
    float instanceCellSize, const h1emu::nav::HeightmapRgb *heightmap,
    float terrainSpacing)
    : collision_(collision), semantics_(semantics) {
  if (semantics_.meshCount() != collision_.meshes.size())
    throw std::runtime_error(
        "ForgelightGeometrySource: H1SEM1/H1COL2 mesh count mismatch");
  std::memcpy(worldBmin_, worldBmin, sizeof(worldBmin_));
  std::memcpy(worldBmax_, worldBmax, sizeof(worldBmax_));
  const float worldBminXZ[2] = {worldBmin[0], worldBmin[2]};
  const float worldBmaxXZ[2] = {worldBmax[0], worldBmax[2]};
  instanceIndex_.build(collision_, worldBminXZ, worldBmaxXZ,
                       instanceCellSize);
  if (heightmap) {
    terrainLattice_ = std::make_unique<TerrainLattice>(
        *heightmap, worldBminXZ, worldBmaxXZ, terrainSpacing);
  }
}

void ForgelightGeometrySource::worldBounds(float bmin[3],
                                           float bmax[3]) const {
  std::memcpy(bmin, worldBmin_, sizeof(worldBmin_));
  std::memcpy(bmax, worldBmax_, sizeof(worldBmax_));
}

bool ForgelightGeometrySource::queryTile(const float qmin[2],
                                         const float qmax[2],
                                         TileRasterInput &out) const {
  const std::vector<std::uint32_t> instanceIds =
      instanceIndex_.query(qmin, qmax);
  // Unlike instances, terrain covers the world uniformly -- a tile with no
  // overlapping instances can still be entirely valid ground.
  if (instanceIds.empty() && !terrainLattice_)
    return false;

  out.verts.clear();
  out.tris.clear();
  out.nonWalkableTris.clear();
  out.triangleSemantics.clear();
  out.triangleObjects.clear();
  out.nonWalkableVolumes.clear();
  out.semanticInput = true;

  for (std::uint32_t instanceIndex : instanceIds) {
    const CollisionInstance &instance = collision_.instances[instanceIndex];
    const CollisionMesh &mesh = collision_.meshes[instance.meshIndex];
    const std::uint32_t semanticStart =
        semantics_.meshOffsets[instance.meshIndex];

    // transform layout (forgelight_nav_source.h): tx,ty,tz, qx,qy,qz,qw,
    // sx,sy,sz, minX,minY,minZ,maxX,maxY,maxZ.
    const float translation[3] = {instance.transform[0], instance.transform[1],
                                  instance.transform[2]};
    const float quatXyz[3] = {instance.transform[3], instance.transform[4],
                              instance.transform[5]};
    const float quatW = instance.transform[6];
    const float scale[3] = {instance.transform[7], instance.transform[8],
                            instance.transform[9]};

    const std::size_t triangleCount = mesh.triangleCount();
    for (std::size_t tri = 0; tri < triangleCount; ++tri) {
      std::uint32_t localIndices[3] = {mesh.indices[tri * 3 + 0],
                                       mesh.indices[tri * 3 + 1],
                                       mesh.indices[tri * 3 + 2]};

      float world[3][3];
      for (int corner = 0; corner < 3; ++corner) {
        const float *local = &mesh.vertices[localIndices[corner] * 3];
        const float scaled[3] = {local[0] * scale[0], local[1] * scale[1],
                                 local[2] * scale[2]};
        float rotated[3];
        rotateByQuaternion(quatXyz, quatW, scaled, rotated);
        world[corner][0] = rotated[0] + translation[0];
        world[corner][1] = rotated[1] + translation[1];
        world[corner][2] = rotated[2] + translation[2];
      }

      const SemanticId semanticId =
          semantics_.semantics[semanticStart + tri];
      const NavSemantic semantic = mapSemanticId(semanticId);
      const SemanticSpec *spec = semanticSpec(semantic);
      const bool isWalkable =
          spec && !spec->rasterizeNull && spec->area != RC_NULL_AREA;

      // Winding correction: applied ONLY to triangles already classified as
      // walkable by H1SEM1, and only after that classification -- never as
      // a blanket flip, and never used to decide walkability itself. The
      // corrected native collision corpus uses outward winding, so many
      // true traversable top faces present a negative-Y world normal; the
      // canonical prepareRasterTriangles (main.cpp) calls Recast's
      // rcMarkWalkableTriangles unmodified, which is winding-order
      // dependent (calcTriNormal from vertex order, promotes only
      // norm[1] > cos(slopeAngle)) and cannot itself distinguish this from
      // a genuine downward-facing surface.
      int order[3] = {0, 1, 2};
      if (isWalkable) {
        float edge1[3] = {world[1][0] - world[0][0], world[1][1] - world[0][1],
                          world[1][2] - world[0][2]};
        float edge2[3] = {world[2][0] - world[0][0], world[2][1] - world[0][1],
                          world[2][2] - world[0][2]};
        float normal[3];
        cross3(edge1, edge2, normal);
        if (normal[1] < 0.0f) {
          order[1] = 2;
          order[2] = 1;
        }
      }

      const int base = static_cast<int>(out.verts.size() / 3);
      for (int corner = 0; corner < 3; ++corner) {
        const float *w = world[order[corner]];
        out.verts.push_back(w[0]);
        out.verts.push_back(w[1]);
        out.verts.push_back(w[2]);
        out.tris.push_back(base + corner);
      }
      out.nonWalkableTris.push_back(0);
      out.triangleSemantics.push_back(semantic);
      out.triangleObjects.push_back(static_cast<int>(instanceIndex));
    }
  }

  if (terrainLattice_)
    terrainLattice_->appendTile(qmin, qmax, out);

  return !out.tris.empty();
}
