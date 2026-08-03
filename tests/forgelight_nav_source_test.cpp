#include "forgelight_nav_source.h"

#include "forgelight_heightmap_loader.h"
#include "forgelight_instance_index.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
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

std::vector<std::uint8_t>
h1cid1Fixture(const std::vector<std::uint32_t> &ids) {
  std::vector<std::uint8_t> bytes{'H', '1', 'C', 'I', 'D', '1', 0, 0};
  appendU32(bytes, 1);
  appendU32(bytes, static_cast<std::uint32_t>(ids.size()));
  for (std::uint32_t id : ids)
    appendU32(bytes, id);
  return bytes;
}

CollisionInstance makeInstance(float minX, float minZ, float maxX,
                               float maxZ) {
  CollisionInstance instance;
  instance.meshIndex = 0;
  instance.transform = {0, 0, 0,    0, 0,    0,    1,   1,
                        1, 1, minX, 0, minZ, maxX, 1,   maxZ};
  return instance;
}

bool containsExactly(std::vector<std::uint32_t> actual,
                     std::vector<std::uint32_t> expected) {
  std::sort(actual.begin(), actual.end());
  std::sort(expected.begin(), expected.end());
  return actual == expected;
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
    validateH1Sem1Compatibility(collision, semantics, true);
    H1Col2Document reviewedThinRoad = collision;
    reviewedThinRoad.meshes[0].kind = CollisionKind::Thin;
    validateH1Sem1Compatibility(reviewedThinRoad, semantics, true);
    H1Sem1Document unsafeKind = semantics;
    unsafeKind.semantics[1] = SemanticId::Exclude;
    expectFailure("solid H1COL2 kind with exclude semantic", [&] {
      validateH1Sem1Compatibility(collision, unsafeKind, true);
    });
    unsafeKind = semantics;
    unsafeKind.semantics[1] = SemanticId::DoorPanelDynamic;
    expectFailure("solid H1COL2 kind with door semantic", [&] {
      validateH1Sem1Compatibility(collision, unsafeKind, true);
    });

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

    const auto cid1Bytes = h1cid1Fixture({1001, 1002, 1003});
    const H1Cid1Document cid1 = parseH1Cid1(cid1Bytes, 3);
    require(cid1.version == 1 && cid1.instanceStableIds.size() == 3 &&
                cid1.instanceStableIds[0] == 1001 &&
                cid1.instanceStableIds[2] == 1003,
            "H1CID1 decode mismatch");

    expectFailure("H1CID1 count mismatch",
                  [&] { parseH1Cid1(cid1Bytes, 4); });
    auto badMagic = cid1Bytes;
    badMagic[0] = 'x';
    expectFailure("H1CID1 bad magic", [&] { parseH1Cid1(badMagic, 3); });
    auto badVersion = cid1Bytes;
    setU32(badVersion, 8, 2);
    expectFailure("H1CID1 bad version", [&] { parseH1Cid1(badVersion, 3); });
    auto truncatedCid1 = cid1Bytes;
    truncatedCid1.pop_back();
    expectFailure("truncated H1CID1",
                  [&] { parseH1Cid1(truncatedCid1, 3); });
    auto trailingCid1 = cid1Bytes;
    trailingCid1.push_back(0);
    expectFailure("trailing H1CID1", [&] { parseH1Cid1(trailingCid1, 3); });
    const auto duplicateCid1Bytes = h1cid1Fixture({7, 7});
    expectFailure("duplicate H1CID1 zone ID",
                  [&] { parseH1Cid1(duplicateCid1Bytes, 2); });
    const H1Cid1Document emptyCid1 = parseH1Cid1(h1cid1Fixture({}), 0);
    require(emptyCid1.instanceStableIds.empty(),
            "empty H1CID1 should decode to zero instances");

    // 2x2 truecolor PNG, RGB pixels (16,0,0) (17,0,0) / (18,16,0) (19,31,0),
    // matching the raw HeightmapRgb fixture above exactly so both loaders
    // are checked against the same expected decoded heights.
    const std::vector<std::uint8_t> heightmapPng{
        137, 80,  78,  71,  13,  10,  26,  10,  0,   0,   0,  13,
        73,  72,  68,  82,  0,   0,   0,   2,   0,   0,   0,  2,
        8,   2,   0,   0,   0,   253, 212, 154, 115, 0,   0,  0,
        22,  73,  68,  65,  84,  120, 156, 99,  20,  96,  96, 96,
        100, 96,  96,  97,  18,  96,  96,  228, 103, 0,   0,  1,
        143, 0,   57,  55,  128, 163, 10,  0,   0,   0,   0,  73,
        69,  78,  68,  174, 66,  96,  130};
    const std::filesystem::path tempPng =
        std::filesystem::temp_directory_path() /
        "forgelight_nav_source_test_heightmap.png";
    {
      std::ofstream out(tempPng, std::ios::binary);
      out.write(reinterpret_cast<const char *>(heightmapPng.data()),
                static_cast<std::streamsize>(heightmapPng.size()));
    }
    const HeightmapRgb decodedHeightmap = loadHeightmapImage(tempPng);
    std::filesystem::remove(tempPng);
    require(decodedHeightmap.width() == 2 && decodedHeightmap.height() == 2,
            "decoded heightmap PNG dimensions mismatch");
    require(decodedHeightmap.decodePixel(0, 0) == 0.0f,
            "decoded heightmap PNG origin decode mismatch");
    require(decodedHeightmap.decodePixel(0, 1) == 16.5f,
            "decoded heightmap PNG green-channel decode mismatch");
    require(std::fabs(decodedHeightmap.sampleNearestWorld(4095.0f, -4095.0f) -
                      24.96875f) < 0.00001f,
            "decoded heightmap PNG world-axis mapping mismatch");

    // Synthetic 40x40 world, 10-unit cells (4x4 grid), five instances:
    // A=0 in cell (0,0); B=1 and C=2 both touch cell (1,1), C spans four
    // cells; D=3 lies fully outside world bounds and must clamp to the
    // last cell rather than being dropped; E=4 lies fully below the world
    // origin and must clamp to cell (0,0).
    H1Col2Document synthetic;
    synthetic.meshes.resize(1);
    synthetic.instances = {
        makeInstance(1, 1, 2, 2),       // A: cell (0,0)
        makeInstance(12, 12, 13, 13),   // B: cell (1,1)
        makeInstance(18, 18, 22, 22),   // C: cells (1,1),(2,1),(1,2),(2,2)
        makeInstance(100, 100, 110, 110), // D: clamps to cell (3,3)
        makeInstance(-20, -20, -10, -10), // E: clamps to cell (0,0)
    };
    InstanceIndex index;
    const float synthBmin[2] = {0.0f, 0.0f};
    const float synthBmax[2] = {40.0f, 40.0f};
    index.build(synthetic, synthBmin, synthBmax, 10.0f);
    require(index.cellCountX() == 4 && index.cellCountZ() == 4,
            "InstanceIndex grid dimensions mismatch");

    {
      const float qmin[2] = {0.0f, 0.0f};
      const float qmax[2] = {9.0f, 9.0f};
      require(containsExactly(index.query(qmin, qmax), {0, 4}),
              "InstanceIndex cell (0,0) query mismatch");
    }
    {
      const float qmin[2] = {10.0f, 10.0f};
      const float qmax[2] = {19.0f, 19.0f};
      require(containsExactly(index.query(qmin, qmax), {1, 2}),
              "InstanceIndex cell (1,1) query mismatch");
    }
    {
      const float qmin[2] = {20.0f, 20.0f};
      const float qmax[2] = {25.0f, 25.0f};
      require(containsExactly(index.query(qmin, qmax), {2}),
              "InstanceIndex cell (2,2) query mismatch");
    }
    {
      const float qmin[2] = {35.0f, 35.0f};
      const float qmax[2] = {39.0f, 39.0f};
      require(containsExactly(index.query(qmin, qmax), {3}),
              "InstanceIndex out-of-bounds instance should clamp to last cell");
    }
    {
      const float qmin[2] = {30.0f, 0.0f};
      const float qmax[2] = {39.0f, 9.0f};
      require(containsExactly(index.query(qmin, qmax), {}),
              "InstanceIndex empty cell should return no instances");
    }
    {
      const float qmin[2] = {0.0f, 0.0f};
      const float qmax[2] = {40.0f, 40.0f};
      require(containsExactly(index.query(qmin, qmax), {0, 1, 2, 3, 4}),
              "InstanceIndex full-range query should return every instance");
    }

    std::cout << "forgelight nav source contract tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "forgelight nav source contract test failed: " << error.what()
              << '\n';
    return 1;
  }
}
