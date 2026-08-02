#include "forgelight_nav_source.h"

#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace h1emu::nav;

namespace {

void appendU32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
  bytes.push_back(static_cast<std::uint8_t>(value));
  bytes.push_back(static_cast<std::uint8_t>(value >> 8));
  bytes.push_back(static_cast<std::uint8_t>(value >> 16));
  bytes.push_back(static_cast<std::uint8_t>(value >> 24));
}

void appendF32(std::vector<std::uint8_t> &bytes, float value) {
  std::uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  appendU32(bytes, bits);
}

void setU32(std::vector<std::uint8_t> &bytes, std::size_t offset,
            std::uint32_t value) {
  if (offset + 4 > bytes.size())
    throw std::runtime_error("test mutation outside fixture");
  bytes[offset] = static_cast<std::uint8_t>(value);
  bytes[offset + 1] = static_cast<std::uint8_t>(value >> 8);
  bytes[offset + 2] = static_cast<std::uint8_t>(value >> 16);
  bytes[offset + 3] = static_cast<std::uint8_t>(value >> 24);
}

void appendMesh(std::vector<std::uint8_t> &bytes, std::uint8_t kind,
                float y) {
  bytes.push_back(kind);
  appendU32(bytes, 3);
  appendU32(bytes, 3);
  const float vertices[9] = {0, y, 0, 1, y, 0, 0, y, 1};
  for (float value : vertices)
    appendF32(bytes, value);
  appendU32(bytes, 0);
  appendU32(bytes, 1);
  appendU32(bytes, 2);
}

std::vector<std::uint8_t> collisionFixture() {
  std::vector<std::uint8_t> bytes{'H', '1', 'C', 'O', 'L', '2', 0, 0};
  appendU32(bytes, 2);
  appendU32(bytes, 2);
  appendU32(bytes, 2);
  appendMesh(bytes, 0, 0.0f);
  appendMesh(bytes, 1, 1.0f);
  appendU32(bytes, 0);
  appendU32(bytes, 1);
  for (int instance = 0; instance < 2; ++instance) {
    const float values[16] = {
        static_cast<float>(instance), 0, 0, 0, 0, 0, 1,
        instance == 0 ? 1.0f : -2.0f,
        instance == 0 ? 1.0f : 0.5f,
        instance == 0 ? 1.0f : 3.0f,
        0, 0, 0, 1, 1, 1};
    for (float value : values)
      appendF32(bytes, value);
  }
  return bytes;
}

std::vector<std::uint8_t> semanticFixture(const Sha256Digest &digest,
                                          std::uint8_t first = 2,
                                          std::uint8_t second = 8) {
  std::vector<std::uint8_t> bytes{'H', '1', 'S', 'E', 'M', '1', 0, 0};
  appendU32(bytes, 1);
  appendU32(bytes, 64);
  appendU32(bytes, 1);
  appendU32(bytes, 2);
  appendU32(bytes, 2);
  appendU32(bytes, 0);
  bytes.insert(bytes.end(), digest.begin(), digest.end());
  appendU32(bytes, 0);
  appendU32(bytes, 1);
  appendU32(bytes, 2);
  bytes.push_back(first);
  bytes.push_back(second);
  return bytes;
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

void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}

} // namespace

int main() {
  try {
    const std::vector<std::uint8_t> abc{'a', 'b', 'c'};
    require(sha256Hex(sha256(abc)) ==
                "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
            "SHA256 golden mismatch");
    require(sha256Hex(sha256({})) ==
                "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
            "empty SHA256 golden mismatch");

    const auto collisionBytes = collisionFixture();
    const H1Col2Document collision = parseH1Col2(collisionBytes);
    require(collision.version == 2 && collision.meshes.size() == 2 &&
                collision.instances.size() == 2,
            "H1COL2 counts mismatch");
    require(collision.meshes[0].triangleCount() == 1 &&
                collision.meshes[1].kind == CollisionKind::Solid,
            "H1COL2 mesh decode mismatch");
    require(collision.instances[1].meshIndex == 1 &&
                collision.instances[1].transform[0] == 1.0f &&
                collision.instances[1].transform[7] == -2.0f &&
                collision.instances[1].transform[8] == 0.5f &&
                collision.instances[1].transform[9] == 3.0f,
            "H1COL2 instance decode mismatch");

    auto trailingCollision = collisionBytes;
    trailingCollision.push_back(0);
    expectFailure("trailing H1COL2", [&] { parseH1Col2(trailingCollision); });
    auto truncatedCollision = collisionBytes;
    truncatedCollision.pop_back();
    expectFailure("truncated H1COL2", [&] { parseH1Col2(truncatedCollision); });
    auto badKind = collisionBytes;
    badKind[20] = 4;
    expectFailure("invalid H1COL2 kind", [&] { parseH1Col2(badKind); });
    auto badIndex = collisionBytes;
    // First index begins after header(20), mesh header(9), vertices(36).
    badIndex[65] = 3;
    expectFailure("invalid H1COL2 index", [&] { parseH1Col2(badIndex); });
    auto nonFiniteVertex = collisionBytes;
    setU32(nonFiniteVertex, 29, 0x7fc00000u);
    expectFailure("non-finite H1COL2 vertex",
                  [&] { parseH1Col2(nonFiniteVertex); });
    auto badInstanceMesh = collisionBytes;
    // Two 57-byte meshes follow the 20-byte file header.
    setU32(badInstanceMesh, 134, 2);
    expectFailure("invalid H1COL2 instance mesh",
                  [&] { parseH1Col2(badInstanceMesh); });
    auto zeroQuaternion = collisionBytes;
    // Instance mesh indices end at 142; identity quaternion w is component 6.
    setU32(zeroQuaternion, 142 + 6 * 4, 0);
    expectFailure("zero H1COL2 quaternion",
                  [&] { parseH1Col2(zeroQuaternion); });
    auto invertedAabb = collisionBytes;
    setU32(invertedAabb, 142 + 10 * 4, 0x40000000u);
    expectFailure("inverted H1COL2 AABB",
                  [&] { parseH1Col2(invertedAabb); });

    const auto semanticBytes = semanticFixture(collision.sha256);
    const H1Sem1Document semantics = parseH1Sem1(
        semanticBytes, collision.sha256, collision.meshTriangleCounts(), true);
    require(semantics.meshCount() == 2 && semantics.triangleCount() == 2 &&
                semantics.semantics[0] == SemanticId::Road &&
                semantics.semantics[1] == SemanticId::ObstacleStatic,
            "H1SEM1 decode mismatch");

    auto badDigest = semanticBytes;
    badDigest[32] ^= 1;
    expectFailure("H1SEM1 digest mismatch", [&] {
      parseH1Sem1(badDigest, collision.sha256,
                  collision.meshTriangleCounts(), true);
    });
    auto badOffset = semanticBytes;
    // Second prefix offset begins at byte 68.
    badOffset[68] = 2;
    expectFailure("H1SEM1 per-mesh cardinality", [&] {
      parseH1Sem1(badOffset, collision.sha256,
                  collision.meshTriangleCounts(), true);
    });
    auto badReserved = semanticBytes;
    setU32(badReserved, 28, 1);
    expectFailure("H1SEM1 reserved field", [&] {
      parseH1Sem1(badReserved, collision.sha256,
                  collision.meshTriangleCounts(), true);
    });
    auto badFinalOffset = semanticBytes;
    setU32(badFinalOffset, 72, 1);
    expectFailure("H1SEM1 final offset", [&] {
      parseH1Sem1(badFinalOffset, collision.sha256,
                  collision.meshTriangleCounts(), true);
    });
    expectFailure("H1SEM1 invalid zero", [&] {
      parseH1Sem1(semanticFixture(collision.sha256, 0, 8), collision.sha256,
                  collision.meshTriangleCounts(), false);
    });
    expectFailure("H1SEM1 production unknown", [&] {
      parseH1Sem1(semanticFixture(collision.sha256, 11, 8), collision.sha256,
                  collision.meshTriangleCounts(), true);
    });
    expectFailure("H1SEM1 production terrain", [&] {
      parseH1Sem1(semanticFixture(collision.sha256, 1, 8), collision.sha256,
                  collision.meshTriangleCounts(), true);
    });
    const H1Sem1Document diagnosticUnknown = parseH1Sem1(
        semanticFixture(collision.sha256, 11, 8), collision.sha256,
        collision.meshTriangleCounts(), false);
    require(diagnosticUnknown.semantics[0] == SemanticId::Unknown,
            "diagnostic H1SEM1 should retain unknown");
    auto truncatedSemantic = semanticBytes;
    truncatedSemantic.pop_back();
    expectFailure("truncated H1SEM1", [&] {
      parseH1Sem1(truncatedSemantic, collision.sha256,
                  collision.meshTriangleCounts(), true);
    });
    auto trailingSemantic = semanticBytes;
    trailingSemantic.push_back(0);
    expectFailure("trailing H1SEM1", [&] {
      parseH1Sem1(trailingSemantic, collision.sha256,
                  collision.meshTriangleCounts(), true);
    });

    HeightmapRgb heightmap(2, 2,
                           {16, 0, 0, 17, 0, 0,
                            18, 16, 0, 19, 31, 0});
    require(heightmap.decodePixel(0, 0) == 0.0f,
            "heightmap origin decode mismatch");
    require(heightmap.decodePixel(0, 1) == 16.5f,
            "heightmap green-channel decode mismatch");
    require(std::fabs(heightmap.sampleNearestWorld(4095.0f, -4095.0f) -
                      24.96875f) < 0.00001f,
            "heightmap world-axis mapping mismatch");
    require(heightmap.sampleNearestWorld(99999.0f, -99999.0f) == 0.0f,
            "heightmap edge clamp mismatch");

    std::cout << "forgelight nav source contract tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "forgelight nav source contract test failed: " << error.what()
              << '\n';
    return 1;
  }
}
