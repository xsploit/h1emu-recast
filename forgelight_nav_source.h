#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace h1emu::nav {

using Sha256Digest = std::array<std::uint8_t, 32>;

enum class CollisionKind : std::uint8_t {
  Walkable = 0,
  Solid = 1,
  Thin = 2,
  Door = 3,
};

enum class SemanticId : std::uint8_t {
  Invalid = 0,
  Terrain = 1,
  Road = 2,
  FloorExterior = 3,
  FloorInterior = 4,
  Stair = 5,
  Ramp = 6,
  Threshold = 7,
  ObstacleStatic = 8,
  DoorPanelDynamic = 9,
  Exclude = 10,
  Unknown = 11,
};

struct CollisionMesh {
  CollisionKind kind = CollisionKind::Walkable;
  std::vector<float> vertices;
  std::vector<std::uint32_t> indices;

  std::size_t triangleCount() const { return indices.size() / 3; }
};

struct CollisionInstance {
  std::uint32_t meshIndex = 0;
  // tx,ty,tz, qx,qy,qz,qw, sx,sy,sz, minX,minY,minZ,maxX,maxY,maxZ.
  std::array<float, 16> transform{};
};

struct H1Col2Document {
  std::uint32_t version = 0;
  Sha256Digest sha256{};
  std::vector<CollisionMesh> meshes;
  std::vector<CollisionInstance> instances;

  std::vector<std::uint32_t> meshTriangleCounts() const;
};

struct H1Sem1Document {
  std::uint32_t version = 0;
  std::uint32_t semanticSchemaVersion = 0;
  Sha256Digest collisionSha256{};
  std::vector<std::uint32_t> meshOffsets;
  std::vector<SemanticId> semantics;

  std::size_t meshCount() const { return meshOffsets.size() - 1; }
  std::size_t triangleCount() const { return semantics.size(); }
};

Sha256Digest sha256(const std::vector<std::uint8_t> &bytes);
std::string sha256Hex(const Sha256Digest &digest);
std::vector<std::uint8_t> readFileBytes(const std::filesystem::path &path);

H1Col2Document parseH1Col2(const std::vector<std::uint8_t> &bytes);
H1Col2Document loadH1Col2(const std::filesystem::path &path);

H1Sem1Document parseH1Sem1(
    const std::vector<std::uint8_t> &bytes,
    const Sha256Digest &expectedCollisionSha256,
    const std::vector<std::uint32_t> &expectedMeshTriangleCounts,
    bool strictProduction);
void validateH1Sem1Compatibility(const H1Col2Document &collision,
                                 const H1Sem1Document &semantics,
                                 bool strictProduction);
H1Sem1Document loadH1Sem1(
    const std::filesystem::path &path,
    const H1Col2Document &collision,
    bool strictProduction);

// Decoded RGB heightmap contract. PNG decoding is deliberately kept outside
// this value type so pixel decoding and world-axis behavior can be tested
// independently from the eventual image library.
class HeightmapRgb {
public:
  HeightmapRgb(std::uint32_t width, std::uint32_t height,
               std::vector<std::uint8_t> rgb);

  float decodePixel(std::uint32_t x, std::uint32_t y) const;
  float sampleNearestWorld(float worldX, float worldZ) const;
  std::uint32_t width() const { return width_; }
  std::uint32_t height() const { return height_; }

private:
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  std::vector<std::uint8_t> rgb_;
};

} // namespace h1emu::nav
