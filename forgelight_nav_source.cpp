#include "forgelight_nav_source.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace h1emu::nav {
namespace {

constexpr std::array<std::uint8_t, 8> H1COL2_MAGIC{
    'H', '1', 'C', 'O', 'L', '2', 0, 0};
constexpr std::array<std::uint8_t, 8> H1SEM1_MAGIC{
    'H', '1', 'S', 'E', 'M', '1', 0, 0};
constexpr std::uint32_t H1COL2_VERSION = 2;
constexpr std::uint32_t H1SEM1_VERSION = 1;
constexpr std::uint32_t H1SEM1_HEADER_BYTES = 64;
constexpr std::uint32_t H1SEM1_SCHEMA_VERSION = 1;
constexpr std::uint32_t MAX_MESHES = 1'000'000;
constexpr std::uint32_t MAX_INSTANCES = 10'000'000;

class Reader {
public:
  explicit Reader(const std::vector<std::uint8_t> &bytes) : bytes_(bytes) {}

  std::size_t offset() const { return offset_; }
  std::size_t remaining() const { return bytes_.size() - offset_; }

  void require(std::size_t count, const std::string &label) const {
    if (count > bytes_.size() - offset_)
      throw std::runtime_error("truncated " + label + " at byte " +
                               std::to_string(offset_));
  }

  std::uint8_t u8(const std::string &label) {
    require(1, label);
    return bytes_[offset_++];
  }

  std::uint32_t u32(const std::string &label) {
    require(4, label);
    const std::uint32_t value =
        std::uint32_t(bytes_[offset_]) |
        (std::uint32_t(bytes_[offset_ + 1]) << 8) |
        (std::uint32_t(bytes_[offset_ + 2]) << 16) |
        (std::uint32_t(bytes_[offset_ + 3]) << 24);
    offset_ += 4;
    return value;
  }

  float f32(const std::string &label) {
    const std::uint32_t bits = u32(label);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    if (!std::isfinite(value))
      throw std::runtime_error("non-finite " + label + " at byte " +
                               std::to_string(offset_ - 4));
    return value;
  }

  template <std::size_t N>
  std::array<std::uint8_t, N> bytes(const std::string &label) {
    require(N, label);
    std::array<std::uint8_t, N> output{};
    std::copy_n(bytes_.begin() + static_cast<std::ptrdiff_t>(offset_), N,
                output.begin());
    offset_ += N;
    return output;
  }

private:
  const std::vector<std::uint8_t> &bytes_;
  std::size_t offset_ = 0;
};

void requireMagic(const std::array<std::uint8_t, 8> &actual,
                  const std::array<std::uint8_t, 8> &expected,
                  const char *label) {
  if (actual != expected)
    throw std::runtime_error(std::string("invalid ") + label + " magic");
}

std::size_t checkedProduct(std::size_t left, std::size_t right,
                           const char *label) {
  if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left)
    throw std::runtime_error(std::string(label) + " size overflow");
  return left * right;
}

struct Sha256State {
  std::array<std::uint32_t, 8> state{
      0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
      0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
  std::array<std::uint8_t, 64> buffer{};
  std::uint64_t bitCount = 0;
  std::size_t buffered = 0;

  static std::uint32_t rotr(std::uint32_t value, unsigned int amount) {
    return (value >> amount) | (value << (32 - amount));
  }

  void transform(const std::uint8_t *block) {
    static constexpr std::uint32_t constants[64] = {
        0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u,
        0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
        0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
        0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
        0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
        0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
        0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u,
        0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
        0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
        0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
        0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u,
        0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
        0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u,
        0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
        0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
        0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};
    std::uint32_t words[64]{};
    for (int i = 0; i < 16; ++i) {
      words[i] = (std::uint32_t(block[i * 4]) << 24) |
                 (std::uint32_t(block[i * 4 + 1]) << 16) |
                 (std::uint32_t(block[i * 4 + 2]) << 8) |
                 std::uint32_t(block[i * 4 + 3]);
    }
    for (int i = 16; i < 64; ++i) {
      const std::uint32_t s0 = rotr(words[i - 15], 7) ^
                               rotr(words[i - 15], 18) ^
                               (words[i - 15] >> 3);
      const std::uint32_t s1 = rotr(words[i - 2], 17) ^
                               rotr(words[i - 2], 19) ^
                               (words[i - 2] >> 10);
      words[i] = words[i - 16] + s0 + words[i - 7] + s1;
    }
    std::uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    std::uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
    for (int i = 0; i < 64; ++i) {
      const std::uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const std::uint32_t choose = (e & f) ^ (~e & g);
      const std::uint32_t t1 = h + s1 + choose + constants[i] + words[i];
      const std::uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t t2 = s0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
    state[5] += f;
    state[6] += g;
    state[7] += h;
  }

  void update(const std::uint8_t *data, std::size_t size) {
    bitCount += std::uint64_t(size) * 8;
    while (size != 0) {
      const std::size_t take = std::min(size, buffer.size() - buffered);
      std::memcpy(buffer.data() + buffered, data, take);
      buffered += take;
      data += take;
      size -= take;
      if (buffered == buffer.size()) {
        transform(buffer.data());
        buffered = 0;
      }
    }
  }

  Sha256Digest finish() {
    const std::uint64_t originalBits = bitCount;
    const std::uint8_t marker = 0x80;
    update(&marker, 1);
    const std::uint8_t zero = 0;
    while (buffered != 56)
      update(&zero, 1);
    std::uint8_t length[8]{};
    for (int i = 0; i < 8; ++i)
      length[7 - i] = static_cast<std::uint8_t>(originalBits >> (i * 8));
    update(length, 8);
    Sha256Digest digest{};
    for (std::size_t i = 0; i < state.size(); ++i) {
      digest[i * 4] = static_cast<std::uint8_t>(state[i] >> 24);
      digest[i * 4 + 1] = static_cast<std::uint8_t>(state[i] >> 16);
      digest[i * 4 + 2] = static_cast<std::uint8_t>(state[i] >> 8);
      digest[i * 4 + 3] = static_cast<std::uint8_t>(state[i]);
    }
    return digest;
  }
};

bool isProductionForbidden(SemanticId id) {
  return id == SemanticId::Terrain || id == SemanticId::Unknown;
}

bool isCompatible(CollisionKind kind, SemanticId semantic,
                  bool strictProduction) {
  if (!strictProduction &&
      (semantic == SemanticId::Terrain || semantic == SemanticId::Unknown))
    return true;
  switch (kind) {
  case CollisionKind::Walkable:
    return (semantic >= SemanticId::Road &&
            semantic <= SemanticId::ObstacleStatic) ||
           semantic == SemanticId::Exclude;
  case CollisionKind::Solid:
    return semantic == SemanticId::ObstacleStatic;
  case CollisionKind::Thin:
    return semantic == SemanticId::ObstacleStatic ||
           semantic == SemanticId::Exclude;
  case CollisionKind::Door:
    return semantic == SemanticId::DoorPanelDynamic;
  }
  return false;
}

} // namespace

std::vector<std::uint32_t> H1Col2Document::meshTriangleCounts() const {
  std::vector<std::uint32_t> counts;
  counts.reserve(meshes.size());
  for (const CollisionMesh &mesh : meshes) {
    if (mesh.triangleCount() > std::numeric_limits<std::uint32_t>::max())
      throw std::runtime_error("H1COL2 mesh triangle count exceeds uint32");
    counts.push_back(static_cast<std::uint32_t>(mesh.triangleCount()));
  }
  return counts;
}

Sha256Digest sha256(const std::vector<std::uint8_t> &bytes) {
  Sha256State state;
  if (!bytes.empty())
    state.update(bytes.data(), bytes.size());
  return state.finish();
}

std::string sha256Hex(const Sha256Digest &digest) {
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (std::uint8_t byte : digest)
    out << std::setw(2) << static_cast<unsigned int>(byte);
  return out.str();
}

std::vector<std::uint8_t> readFileBytes(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input)
    throw std::runtime_error("cannot read " + path.string());
  const std::streamoff length = input.tellg();
  if (length < 0)
    throw std::runtime_error("cannot size " + path.string());
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(length));
  input.seekg(0);
  if (!bytes.empty() &&
      !input.read(reinterpret_cast<char *>(bytes.data()), length))
    throw std::runtime_error("short read from " + path.string());
  return bytes;
}

H1Col2Document parseH1Col2(const std::vector<std::uint8_t> &bytes) {
  Reader reader(bytes);
  requireMagic(reader.bytes<8>("H1COL2 magic"), H1COL2_MAGIC, "H1COL2");

  H1Col2Document output;
  output.sha256 = sha256(bytes);
  output.version = reader.u32("H1COL2 version");
  if (output.version != H1COL2_VERSION)
    throw std::runtime_error("unsupported H1COL2 version " +
                             std::to_string(output.version));
  const std::uint32_t meshCount = reader.u32("H1COL2 mesh count");
  const std::uint32_t instanceCount = reader.u32("H1COL2 instance count");
  if (meshCount > MAX_MESHES)
    throw std::runtime_error("H1COL2 mesh count exceeds safety limit");
  if (instanceCount > MAX_INSTANCES)
    throw std::runtime_error("H1COL2 instance count exceeds safety limit");

  output.meshes.reserve(meshCount);
  for (std::uint32_t meshIndex = 0; meshIndex < meshCount; ++meshIndex) {
    const std::uint8_t rawKind = reader.u8("H1COL2 mesh kind");
    if (rawKind > static_cast<std::uint8_t>(CollisionKind::Door))
      throw std::runtime_error("invalid H1COL2 kind at mesh " +
                               std::to_string(meshIndex));
    const std::uint32_t vertexCount = reader.u32("H1COL2 vertex count");
    const std::uint32_t indexCount = reader.u32("H1COL2 index count");
    if (vertexCount == 0 || indexCount == 0 || indexCount % 3 != 0)
      throw std::runtime_error("invalid H1COL2 geometry cardinality at mesh " +
                               std::to_string(meshIndex));
    reader.require(checkedProduct(checkedProduct(vertexCount, 3, "vertex"), 4,
                                  "vertex"),
                   "H1COL2 vertices");
    CollisionMesh mesh;
    mesh.kind = static_cast<CollisionKind>(rawKind);
    mesh.vertices.reserve(checkedProduct(vertexCount, 3, "vertex"));
    for (std::size_t valueIndex = 0;
         valueIndex < static_cast<std::size_t>(vertexCount) * 3; ++valueIndex)
      mesh.vertices.push_back(reader.f32("H1COL2 vertex"));

    reader.require(checkedProduct(indexCount, 4, "index"), "H1COL2 indices");
    mesh.indices.reserve(indexCount);
    for (std::uint32_t index = 0; index < indexCount; ++index) {
      const std::uint32_t vertexIndex = reader.u32("H1COL2 index");
      if (vertexIndex >= vertexCount)
        throw std::runtime_error("H1COL2 vertex index out of range at mesh " +
                                 std::to_string(meshIndex));
      mesh.indices.push_back(vertexIndex);
    }
    output.meshes.push_back(std::move(mesh));
  }

  std::vector<std::uint32_t> instanceMeshes;
  instanceMeshes.reserve(instanceCount);
  reader.require(checkedProduct(instanceCount, 4, "instance mesh"),
                 "H1COL2 instance mesh indices");
  for (std::uint32_t instanceIndex = 0; instanceIndex < instanceCount;
       ++instanceIndex) {
    const std::uint32_t meshIndex = reader.u32("H1COL2 instance mesh index");
    if (meshIndex >= meshCount)
      throw std::runtime_error("H1COL2 instance mesh index out of range at " +
                               std::to_string(instanceIndex));
    instanceMeshes.push_back(meshIndex);
  }

  reader.require(checkedProduct(checkedProduct(instanceCount, 16, "instance"),
                                4, "instance"),
                 "H1COL2 instance transforms");
  output.instances.reserve(instanceCount);
  for (std::uint32_t instanceIndex = 0; instanceIndex < instanceCount;
       ++instanceIndex) {
    CollisionInstance instance;
    instance.meshIndex = instanceMeshes[instanceIndex];
    for (std::size_t component = 0; component < instance.transform.size();
         ++component)
      instance.transform[component] = reader.f32("H1COL2 instance transform");
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (instance.transform[10 + axis] > instance.transform[13 + axis])
        throw std::runtime_error("H1COL2 inverted instance AABB at " +
                                 std::to_string(instanceIndex));
    }
    const float quaternionLengthSquared =
        instance.transform[3] * instance.transform[3] +
        instance.transform[4] * instance.transform[4] +
        instance.transform[5] * instance.transform[5] +
        instance.transform[6] * instance.transform[6];
    if (!(quaternionLengthSquared > 0.0f))
      throw std::runtime_error("H1COL2 zero-length instance quaternion at " +
                               std::to_string(instanceIndex));
    output.instances.push_back(instance);
  }

  if (reader.remaining() != 0)
    throw std::runtime_error("trailing H1COL2 data at byte " +
                             std::to_string(reader.offset()));
  return output;
}

H1Col2Document loadH1Col2(const std::filesystem::path &path) {
  return parseH1Col2(readFileBytes(path));
}

H1Sem1Document parseH1Sem1(
    const std::vector<std::uint8_t> &bytes,
    const Sha256Digest &expectedCollisionSha256,
    const std::vector<std::uint32_t> &expectedMeshTriangleCounts,
    bool strictProduction) {
  Reader reader(bytes);
  requireMagic(reader.bytes<8>("H1SEM1 magic"), H1SEM1_MAGIC, "H1SEM1");

  H1Sem1Document output;
  output.version = reader.u32("H1SEM1 version");
  const std::uint32_t headerBytes = reader.u32("H1SEM1 header size");
  output.semanticSchemaVersion = reader.u32("H1SEM1 semantic schema version");
  const std::uint32_t meshCount = reader.u32("H1SEM1 mesh count");
  const std::uint32_t triangleCount = reader.u32("H1SEM1 triangle count");
  const std::uint32_t reserved = reader.u32("H1SEM1 reserved field");
  output.collisionSha256 = reader.bytes<32>("H1SEM1 collision SHA256");

  if (output.version != H1SEM1_VERSION)
    throw std::runtime_error("unsupported H1SEM1 version " +
                             std::to_string(output.version));
  if (headerBytes != H1SEM1_HEADER_BYTES)
    throw std::runtime_error("invalid H1SEM1 header size");
  if (output.semanticSchemaVersion != H1SEM1_SCHEMA_VERSION)
    throw std::runtime_error("unsupported H1SEM1 semantic schema version");
  if (reserved != 0)
    throw std::runtime_error("H1SEM1 reserved field must be zero");
  if (meshCount != expectedMeshTriangleCounts.size())
    throw std::runtime_error("H1SEM1 mesh count mismatch");
  if (output.collisionSha256 != expectedCollisionSha256)
    throw std::runtime_error("H1SEM1 collision SHA256 mismatch");

  reader.require(checkedProduct(static_cast<std::size_t>(meshCount) + 1, 4,
                                "H1SEM1 offsets"),
                 "H1SEM1 mesh offsets");
  output.meshOffsets.reserve(static_cast<std::size_t>(meshCount) + 1);
  for (std::uint32_t offsetIndex = 0; offsetIndex <= meshCount; ++offsetIndex)
    output.meshOffsets.push_back(reader.u32("H1SEM1 mesh offset"));
  if (output.meshOffsets.front() != 0)
    throw std::runtime_error("H1SEM1 first mesh offset must be zero");
  for (std::uint32_t meshIndex = 0; meshIndex < meshCount; ++meshIndex) {
    const std::uint32_t start = output.meshOffsets[meshIndex];
    const std::uint32_t end = output.meshOffsets[meshIndex + 1];
    if (end < start)
      throw std::runtime_error("H1SEM1 mesh offsets are not monotonic");
    if (end - start != expectedMeshTriangleCounts[meshIndex])
      throw std::runtime_error("H1SEM1 mesh triangle count mismatch at mesh " +
                               std::to_string(meshIndex));
  }
  if (output.meshOffsets.back() != triangleCount)
    throw std::runtime_error("H1SEM1 final offset does not match triangle count");

  reader.require(triangleCount, "H1SEM1 triangle semantics");
  output.semantics.reserve(triangleCount);
  for (std::uint32_t triangleIndex = 0; triangleIndex < triangleCount;
       ++triangleIndex) {
    const std::uint8_t value = reader.u8("H1SEM1 semantic id");
    if (value < static_cast<std::uint8_t>(SemanticId::Terrain) ||
        value > static_cast<std::uint8_t>(SemanticId::Unknown))
      throw std::runtime_error("invalid H1SEM1 semantic id at triangle " +
                               std::to_string(triangleIndex));
    const SemanticId semantic = static_cast<SemanticId>(value);
    if (strictProduction && isProductionForbidden(semantic))
      throw std::runtime_error("production-forbidden H1SEM1 semantic id at triangle " +
                               std::to_string(triangleIndex));
    output.semantics.push_back(semantic);
  }
  if (reader.remaining() != 0)
    throw std::runtime_error("trailing H1SEM1 data at byte " +
                             std::to_string(reader.offset()));
  return output;
}

H1Sem1Document loadH1Sem1(const std::filesystem::path &path,
                          const H1Col2Document &collision,
                          bool strictProduction) {
  H1Sem1Document semantics =
      parseH1Sem1(readFileBytes(path), collision.sha256,
                  collision.meshTriangleCounts(), strictProduction);
  validateH1Sem1Compatibility(collision, semantics, strictProduction);
  return semantics;
}

void validateH1Sem1Compatibility(const H1Col2Document &collision,
                                 const H1Sem1Document &semantics,
                                 bool strictProduction) {
  if (collision.meshes.size() != semantics.meshCount())
    throw std::runtime_error("H1SEM1 compatibility mesh count mismatch");
  for (std::size_t meshIndex = 0; meshIndex < collision.meshes.size();
       ++meshIndex) {
    const std::uint32_t start = semantics.meshOffsets[meshIndex];
    const std::uint32_t end = semantics.meshOffsets[meshIndex + 1];
    for (std::uint32_t triangleIndex = start; triangleIndex < end;
         ++triangleIndex) {
      const SemanticId semantic = semantics.semantics[triangleIndex];
      if (!isCompatible(collision.meshes[meshIndex].kind, semantic,
                        strictProduction))
        throw std::runtime_error(
            "H1SEM1 semantic is incompatible with H1COL2 kind at mesh " +
            std::to_string(meshIndex) + ", triangle " +
            std::to_string(triangleIndex - start));
    }
  }
}

HeightmapRgb::HeightmapRgb(std::uint32_t width, std::uint32_t height,
                           std::vector<std::uint8_t> rgb)
    : width_(width), height_(height), rgb_(std::move(rgb)) {
  if (width_ == 0 || height_ == 0)
    throw std::runtime_error("heightmap dimensions must be nonzero");
  const std::size_t pixels = checkedProduct(width_, height_, "heightmap");
  const std::size_t expected = checkedProduct(pixels, 3, "heightmap RGB");
  if (rgb_.size() != expected)
    throw std::runtime_error("heightmap RGB byte count mismatch");
}

float HeightmapRgb::decodePixel(std::uint32_t x, std::uint32_t y) const {
  if (x >= width_ || y >= height_)
    throw std::out_of_range("heightmap pixel outside image");
  const std::size_t index =
      (static_cast<std::size_t>(y) * width_ + x) * 3;
  return (static_cast<float>(rgb_[index]) - 16.0f) * 8.0f +
         static_cast<float>(rgb_[index + 1]) / 32.0f;
}

float HeightmapRgb::sampleNearestWorld(float worldX, float worldZ) const {
  if (!std::isfinite(worldX) || !std::isfinite(worldZ))
    throw std::runtime_error("non-finite heightmap world coordinate");
  const long rawX = static_cast<long>(std::floor(worldZ + 4096.0f));
  const long rawY = static_cast<long>(std::floor(4096.0f - worldX));
  const long pixelX = std::clamp(rawX, 0L, static_cast<long>(width_) - 1);
  const long pixelY = std::clamp(rawY, 0L, static_cast<long>(height_) - 1);
  return decodePixel(static_cast<std::uint32_t>(pixelX),
                     static_cast<std::uint32_t>(pixelY));
}

} // namespace h1emu::nav
