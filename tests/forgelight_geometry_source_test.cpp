#include "forgelight_geometry_source.h"

#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace h1emu::nav;

namespace {

void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}

void expectFailure(const std::string &name,
                   const std::function<void()> &operation) {
  try {
    operation();
  } catch (const std::exception &) {
    return;
  }
  throw std::runtime_error(name + " unexpectedly succeeded");
}

// One flat local triangle, corners (0,0,0) (1,0,0) (0,0,1). Order {0,1,2}
// yields a downward (-Y) world normal under an identity transform; order
// {0,2,1} yields an upward (+Y) normal. Everything else about the instance
// (translation zero, identity quaternion, unit scale) stays fixed so the
// winding-correction behavior is the only thing under test.
H1Col2Document makeSingleTriangleCollision(bool alreadyUpwardWinding,
                                           const std::array<float, 16>
                                               &transform = {0, 0, 0, 0, 0, 0,
                                                             1, 1, 1, 1, -100,
                                                             -10, -100, 100,
                                                             10, 100}) {
  H1Col2Document doc;
  CollisionMesh mesh;
  mesh.kind = CollisionKind::Walkable; // ignored entirely by this source
  mesh.vertices = {0, 0, 0, 1, 0, 0, 0, 0, 1};
  mesh.indices = alreadyUpwardWinding
                    ? std::vector<std::uint32_t>{0, 2, 1}
                    : std::vector<std::uint32_t>{0, 1, 2};
  doc.meshes.push_back(mesh);

  CollisionInstance instance;
  instance.meshIndex = 0;
  instance.transform = transform;
  doc.instances.push_back(instance);
  return doc;
}

H1Sem1Document makeSingleTriangleSemantics(SemanticId id) {
  H1Sem1Document doc;
  doc.meshOffsets = {0, 1};
  doc.semantics = {id};
  return doc;
}

} // namespace

int main() {
  try {
    const float worldBmin[3] = {-1000.0f, -100.0f, -1000.0f};
    const float worldBmax[3] = {1000.0f, 100.0f, 1000.0f};
    const float qmin[2] = {-5.0f, -5.0f};
    const float qmax[2] = {5.0f, 5.0f};

    // Downward-facing input + walkable semantic -> must flip to upward.
    {
      const H1Col2Document collision = makeSingleTriangleCollision(false);
      const H1Sem1Document semantics =
          makeSingleTriangleSemantics(SemanticId::Road);
      ForgelightGeometrySource source(collision, semantics, worldBmin,
                                      worldBmax);
      TileRasterInput out;
      require(source.queryTile(qmin, qmax, out),
              "expected geometry for downward walkable tile");
      require(out.tris.size() == 3 && out.verts.size() == 9,
              "expected exactly one output triangle");
      require(out.triangleSemantics.size() == 1 &&
                  out.triangleSemantics[0] == NavSemantic::Road,
              "semantic id mapping mismatch (Road)");
      // Flipped order swaps corners 1 and 2: world verts become
      // v0=(0,0,0), v2=(0,0,1), v1=(1,0,0).
      require(out.verts[0] == 0.0f && out.verts[1] == 0.0f &&
                  out.verts[2] == 0.0f,
              "flipped triangle corner 0 mismatch");
      require(out.verts[3] == 0.0f && out.verts[4] == 0.0f &&
                  out.verts[5] == 1.0f,
              "flipped triangle corner 1 mismatch (expected original v2)");
      require(out.verts[6] == 1.0f && out.verts[7] == 0.0f &&
                  out.verts[8] == 0.0f,
              "flipped triangle corner 2 mismatch (expected original v1)");
      require(out.nonWalkableTris[0] == 0, "nonWalkableTris must stay 0");
      require(out.triangleObjects[0] == 0, "triangleObjects should be instance index");
      require(out.semanticInput, "semanticInput must always be true");
    }

    // Already-upward input + walkable semantic -> must NOT flip; converges
    // to the exact same final geometry as the flipped case above.
    {
      const H1Col2Document collision = makeSingleTriangleCollision(true);
      const H1Sem1Document semantics =
          makeSingleTriangleSemantics(SemanticId::Road);
      ForgelightGeometrySource source(collision, semantics, worldBmin,
                                      worldBmax);
      TileRasterInput out;
      require(source.queryTile(qmin, qmax, out),
              "expected geometry for upward walkable tile");
      require(out.verts[0] == 0.0f && out.verts[1] == 0.0f &&
                  out.verts[2] == 0.0f,
              "already-upward corner 0 mismatch");
      require(out.verts[3] == 0.0f && out.verts[4] == 0.0f &&
                  out.verts[5] == 1.0f,
              "already-upward corner 1 mismatch");
      require(out.verts[6] == 1.0f && out.verts[7] == 0.0f &&
                  out.verts[8] == 0.0f,
              "already-upward corner 2 mismatch");
    }

    // Downward-facing input + NON-walkable semantic -> must NOT flip; the
    // winding correction only ever applies after walkable classification.
    {
      const H1Col2Document collision = makeSingleTriangleCollision(false);
      const H1Sem1Document semantics =
          makeSingleTriangleSemantics(SemanticId::Unknown);
      ForgelightGeometrySource source(collision, semantics, worldBmin,
                                      worldBmax);
      TileRasterInput out;
      require(source.queryTile(qmin, qmax, out),
              "expected geometry for downward non-walkable tile");
      require(out.triangleSemantics[0] == NavSemantic::Unknown,
              "semantic id mapping mismatch (Unknown)");
      // Unflipped: raw {0,1,2} order preserved verbatim.
      require(out.verts[0] == 0.0f && out.verts[1] == 0.0f &&
                  out.verts[2] == 0.0f,
              "unflipped corner 0 mismatch");
      require(out.verts[3] == 1.0f && out.verts[4] == 0.0f &&
                  out.verts[5] == 0.0f,
              "unflipped corner 1 mismatch (should stay original v1)");
      require(out.verts[6] == 0.0f && out.verts[7] == 0.0f &&
                  out.verts[8] == 1.0f,
              "unflipped corner 2 mismatch (should stay original v2)");
    }

    // Instance transform: translate + 90-degree rotation about Y + uniform
    // scale. Hand-verified expectation: rotating local point (1,0,0) by 90
    // degrees about +Y gives (0,0,-1); scaled by 2 gives (0,0,-2); then
    // translated by (10,5,-3) gives (10,5,-5).
    {
      const float half = std::sqrt(0.5f); // sin(45deg) == cos(45deg)
      const std::array<float, 16> transform = {
          10, 5, -3,           // translation
          0,  half, 0,  half,  // 90 degree rotation about Y (qx,qy,qz,qw)
          2,  2,  2,           // uniform scale
          -1000, -1000, -1000, 1000, 1000, 1000};
      const H1Col2Document collision =
          makeSingleTriangleCollision(true, transform);
      const H1Sem1Document semantics =
          makeSingleTriangleSemantics(SemanticId::ObstacleStatic);
      const float bigMin[3] = {-2000.0f, -2000.0f, -2000.0f};
      const float bigMax[3] = {2000.0f, 2000.0f, 2000.0f};
      ForgelightGeometrySource source(collision, semantics, bigMin, bigMax);
      TileRasterInput out;
      const float wideMin[2] = {-500.0f, -500.0f};
      const float wideMax[2] = {500.0f, 500.0f};
      require(source.queryTile(wideMin, wideMax, out),
              "expected geometry for transformed instance");
      // Corner order for this fixture is {0,2,1} (already-upward local
      // winding) and ObstacleStatic is non-walkable, so no flip applies:
      // emitted corners are local vertex 0, then 2, then 1, in that order.
      // Local vertex 0 = (0,0,0) trivially maps to just the translation
      // regardless of rotation/scale, so it doesn't exercise the rotation
      // math -- check the third emitted corner instead, local vertex 1 =
      // (1,0,0), which is where the hand-verified 90-degree-about-Y
      // rotation, x2 scale, then translate gives (10,5,-5).
      const float expectedCorner2[3] = {10.0f, 5.0f, -5.0f};
      require(std::fabs(out.verts[6] - expectedCorner2[0]) < 1e-4f &&
                  std::fabs(out.verts[7] - expectedCorner2[1]) < 1e-4f &&
                  std::fabs(out.verts[8] - expectedCorner2[2]) < 1e-4f,
              "transformed corner 2 (local v1, rotated+scaled+translated) mismatch");
      // Local vertex 2 = (0,0,1): rotates to (1,0,0), scales to (2,0,0),
      // translates to (12,5,-3) -- the second emitted corner.
      const float expectedCorner1[3] = {12.0f, 5.0f, -3.0f};
      require(std::fabs(out.verts[3] - expectedCorner1[0]) < 1e-4f &&
                  std::fabs(out.verts[4] - expectedCorner1[1]) < 1e-4f &&
                  std::fabs(out.verts[5] - expectedCorner1[2]) < 1e-4f,
              "transformed corner 1 (local v2, rotated+scaled+translated) mismatch");
    }

    // No overlapping instances -> queryTile must return false.
    {
      const H1Col2Document collision = makeSingleTriangleCollision(false);
      const H1Sem1Document semantics =
          makeSingleTriangleSemantics(SemanticId::Road);
      ForgelightGeometrySource source(collision, semantics, worldBmin,
                                      worldBmax);
      TileRasterInput out;
      const float farMin[2] = {900.0f, 900.0f};
      const float farMax[2] = {950.0f, 950.0f};
      require(!source.queryTile(farMin, farMax, out),
              "queryTile should report no geometry outside instance AABB");
    }

    // worldBounds() must return exactly what was passed to the constructor.
    {
      const H1Col2Document collision = makeSingleTriangleCollision(false);
      const H1Sem1Document semantics =
          makeSingleTriangleSemantics(SemanticId::Road);
      ForgelightGeometrySource source(collision, semantics, worldBmin,
                                      worldBmax);
      float bmin[3];
      float bmax[3];
      source.worldBounds(bmin, bmax);
      for (int i = 0; i < 3; ++i) {
        require(bmin[i] == worldBmin[i], "worldBounds bmin mismatch");
        require(bmax[i] == worldBmax[i], "worldBounds bmax mismatch");
      }
      require(source.semanticInput(), "semanticInput must be true");
    }

    // Mesh-count mismatch between H1COL2 and H1SEM1 must fail closed.
    {
      const H1Col2Document collision = makeSingleTriangleCollision(false);
      H1Sem1Document semantics = makeSingleTriangleSemantics(SemanticId::Road);
      semantics.meshOffsets = {0, 1, 2}; // claims two meshes; collision has one
      expectFailure("mesh count mismatch", [&] {
        ForgelightGeometrySource source(collision, semantics, worldBmin,
                                        worldBmax);
      });
    }

    std::cout << "forgelight geometry source contract tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "forgelight geometry source contract test failed: "
              << error.what() << '\n';
    return 1;
  }
}
