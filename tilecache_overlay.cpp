#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "fastlz.h"
#include "nav_semantic.h"
#include "navigation_transitions.h"

namespace fs = std::filesystem;

namespace {

constexpr int TILECACHESET_MAGIC =
    'T' << 24 | 'S' << 16 | 'E' << 8 | 'T';
constexpr int TILECACHESET_VERSION = 1;
constexpr std::uint64_t DEFAULT_PART_BYTES = 25ULL * 1024ULL * 1024ULL;

struct TileCacheSetHeader {
  int magic;
  int version;
  int numTiles;
  dtNavMeshParams meshParams;
  dtTileCacheParams cacheParams;
};

struct TileCacheTileHeader {
  dtCompressedTileRef tileRef;
  int dataSize;
};

struct FastLZCompressor : dtTileCacheCompressor {
  int maxCompressedSize(const int bufferSize) override {
    return static_cast<int>(bufferSize * 1.05f) + 66;
  }
  dtStatus compress(const unsigned char *buffer, const int bufferSize,
                    unsigned char *compressed, const int,
                    int *compressedSize) override {
    *compressedSize = fastlz_compress(buffer, bufferSize, compressed);
    return DT_SUCCESS;
  }
  dtStatus decompress(const unsigned char *compressed,
                      const int compressedSize, unsigned char *buffer,
                      const int maxBufferSize, int *bufferSize) override {
    *bufferSize = fastlz_decompress(compressed, compressedSize, buffer,
                                    maxBufferSize);
    return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
  }
};

// TileCache reconstruction is designed around a resettable scratch arena.
// Using the default malloc/free allocator for 100k layers both distorts the
// timing measurement and exercises the Windows heap millions of times. This
// matches RecastDemo's production TileCache allocator contract.
struct LinearAllocator : dtTileCacheAlloc {
  unsigned char *buffer = nullptr;
  std::size_t capacity = 0;
  std::size_t top = 0;
  std::size_t high = 0;

  explicit LinearAllocator(std::size_t requestedCapacity)
      : capacity(requestedCapacity) {
    buffer = static_cast<unsigned char *>(
        dtAlloc(capacity, DT_ALLOC_PERM));
  }
  ~LinearAllocator() override { dtFree(buffer); }
  void reset() override {
    high = std::max(high, top);
    top = 0;
  }
  void *alloc(const std::size_t size) override {
    if (!buffer || top + size > capacity)
      return nullptr;
    unsigned char *memory = buffer + top;
    top += size;
    return memory;
  }
  void free(void *) override {}
};

static_assert(sizeof(TileCacheSetHeader) == 92,
              "TileCache set ABI changed; update the parser deliberately");
static_assert(sizeof(TileCacheTileHeader) == 8,
              "TileCache tile ABI changed; update the parser deliberately");
static_assert(sizeof(dtTileCacheLayerHeader) == 56,
              "TileCache layer ABI changed; update the parser deliberately");

struct Sha256 {
  std::array<std::uint32_t, 8> state{
      0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
      0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
  std::array<unsigned char, 64> buffer{};
  std::uint64_t bitCount = 0;
  std::size_t buffered = 0;

  static std::uint32_t rotr(std::uint32_t value, unsigned int amount) {
    return (value >> amount) | (value << (32 - amount));
  }

  void transform(const unsigned char *block) {
    static constexpr std::uint32_t k[64] = {
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
    std::uint32_t w[64]{};
    for (int i = 0; i < 16; ++i) {
      w[i] = (std::uint32_t(block[i * 4]) << 24) |
             (std::uint32_t(block[i * 4 + 1]) << 16) |
             (std::uint32_t(block[i * 4 + 2]) << 8) |
             std::uint32_t(block[i * 4 + 3]);
    }
    for (int i = 16; i < 64; ++i) {
      const std::uint32_t s0 =
          rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const std::uint32_t s1 =
          rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    std::uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
    for (int i = 0; i < 64; ++i) {
      const std::uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const std::uint32_t ch = (e & f) ^ (~e & g);
      const std::uint32_t temp1 = h + s1 + ch + k[i] + w[i];
      const std::uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = s0 + maj;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
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

  void update(const unsigned char *data, std::size_t size) {
    bitCount += std::uint64_t(size) * 8;
    while (size) {
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

  std::string finish() {
    const std::uint64_t originalBits = bitCount;
    const unsigned char marker = 0x80;
    update(&marker, 1);
    const unsigned char zero = 0;
    while (buffered != 56)
      update(&zero, 1);
    unsigned char length[8];
    for (int i = 0; i < 8; ++i)
      length[7 - i] = static_cast<unsigned char>(originalBits >> (i * 8));
    update(length, 8);
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (std::uint32_t value : state)
      out << std::setw(8) << value;
    return out.str();
  }
};

std::string sha256(const std::vector<unsigned char> &bytes) {
  Sha256 hash;
  hash.update(bytes.data(), bytes.size());
  return hash.finish();
}

struct FileDigest {
  std::string name;
  std::uint64_t bytes = 0;
  std::string sha;
};

struct LayerKey {
  int tx = 0;
  int ty = 0;
  int layer = 0;
  bool operator<(const LayerKey &other) const {
    return std::tie(ty, tx, layer) <
           std::tie(other.ty, other.tx, other.layer);
  }
  bool operator==(const LayerKey &other) const {
    return tx == other.tx && ty == other.ty && layer == other.layer;
  }
};

struct Record {
  LayerKey key;
  std::shared_ptr<std::vector<unsigned char>> data;
};

struct LoadedSet {
  TileCacheSetHeader header{};
  std::vector<Record> records;
  std::vector<FileDigest> files;
  std::string datasetSha;
};

struct Rect {
  int minTx = 0, minTy = 0, maxTx = 0, maxTy = 0;
  bool contains(int tx, int ty) const {
    return tx >= minTx && tx < maxTx && ty >= minTy && ty < maxTy;
  }
};

struct OverlayStats {
  std::size_t preservedBaseLayers = 0;
  std::size_t removedBaseLayers = 0;
  std::size_t insertedOverlayLayers = 0;
  std::size_t ignoredHaloLayers = 0;
};

std::vector<unsigned char> readBytes(const fs::path &path) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input)
    throw std::runtime_error("cannot read " + path.string());
  const std::streamoff length = input.tellg();
  if (length < 0)
    throw std::runtime_error("cannot size " + path.string());
  std::vector<unsigned char> bytes(static_cast<std::size_t>(length));
  input.seekg(0);
  if (!bytes.empty() &&
      !input.read(reinterpret_cast<char *>(bytes.data()), length))
    throw std::runtime_error("short read from " + path.string());
  return bytes;
}

bool isPowerOfTwo(int value) {
  return value > 0 && (value & (value - 1)) == 0;
}

void requireFinite(float value, const char *name) {
  if (!std::isfinite(value))
    throw std::runtime_error(std::string("non-finite parameter: ") + name);
}

bool sameFloat(float a, float b) {
  std::uint32_t left = 0, right = 0;
  std::memcpy(&left, &a, sizeof(left));
  std::memcpy(&right, &b, sizeof(right));
  return left == right;
}

void validateSetHeader(const TileCacheSetHeader &header,
                       const std::string &label) {
  if (header.magic != TILECACHESET_MAGIC)
    throw std::runtime_error(label + ": wrong TileCache set magic");
  if (header.version != TILECACHESET_VERSION)
    throw std::runtime_error(label + ": wrong TileCache set version");
  if (header.numTiles < 0)
    throw std::runtime_error(label + ": negative tile count");
  const auto &mesh = header.meshParams;
  const auto &cache = header.cacheParams;
  for (int i = 0; i < 3; ++i) {
    requireFinite(mesh.orig[i], "meshParams.orig");
    requireFinite(cache.orig[i], "cacheParams.orig");
    if (!sameFloat(mesh.orig[i], cache.orig[i]))
      throw std::runtime_error(label +
                               ": mesh/cache origins do not match");
  }
  requireFinite(mesh.tileWidth, "meshParams.tileWidth");
  requireFinite(mesh.tileHeight, "meshParams.tileHeight");
  requireFinite(cache.cs, "cacheParams.cs");
  requireFinite(cache.ch, "cacheParams.ch");
  requireFinite(cache.walkableHeight, "cacheParams.walkableHeight");
  requireFinite(cache.walkableRadius, "cacheParams.walkableRadius");
  requireFinite(cache.walkableClimb, "cacheParams.walkableClimb");
  requireFinite(cache.maxSimplificationError,
                "cacheParams.maxSimplificationError");
  if (cache.cs <= 0 || cache.ch <= 0 || cache.width <= 0 ||
      cache.width > 255 || cache.height <= 0 || cache.height > 255 ||
      cache.walkableHeight <= 0 || cache.walkableRadius < 0 ||
      cache.walkableClimb < 0 || cache.maxSimplificationError < 0)
    throw std::runtime_error(label + ": invalid TileCache dimensions/settings");
  const float expectedWidth = cache.width * cache.cs;
  const float expectedHeight = cache.height * cache.cs;
  if (!sameFloat(mesh.tileWidth, expectedWidth) ||
      !sameFloat(mesh.tileHeight, expectedHeight))
    throw std::runtime_error(label +
                             ": mesh tile size does not match cache voxels");
  if (!isPowerOfTwo(mesh.maxTiles) || !isPowerOfTwo(mesh.maxPolys) ||
      !isPowerOfTwo(cache.maxTiles) || mesh.maxTiles <= 0 ||
      mesh.maxPolys <= 0 || cache.maxTiles < header.numTiles ||
      cache.maxObstacles < 0)
    throw std::runtime_error(label + ": invalid or unsafe capacities");
}

void validateLayer(const dtTileCacheLayerHeader &layer,
                   const TileCacheSetHeader &setHeader,
                   const std::string &label) {
  if (layer.magic != DT_TILECACHE_MAGIC)
    throw std::runtime_error(label + ": wrong layer payload magic");
  if (layer.version != DT_TILECACHE_VERSION)
    throw std::runtime_error(label + ": wrong layer payload version");
  if (layer.tx < 0 || layer.ty < 0 || layer.tlayer < 0)
    throw std::runtime_error(label + ": negative layer coordinate");
  const auto &params = setHeader.cacheParams;
  if (layer.width != params.width || layer.height != params.height ||
      layer.minx > layer.maxx || layer.miny > layer.maxy ||
      layer.maxx >= layer.width || layer.maxy >= layer.height ||
      layer.hmin > layer.hmax)
    throw std::runtime_error(label + ": invalid layer dimensions/ranges");
  const float tileWidth = params.width * params.cs;
  const float tileHeight = params.height * params.cs;
  const float expected[6] = {
      params.orig[0] + layer.tx * tileWidth,
      params.orig[1] + layer.hmin * params.ch,
      params.orig[2] + layer.ty * tileHeight,
      params.orig[0] + (layer.tx + 1) * tileWidth,
      params.orig[1] + layer.hmax * params.ch,
      params.orig[2] + (layer.ty + 1) * tileHeight};
  const float actual[6] = {layer.bmin[0], layer.bmin[1], layer.bmin[2],
                           layer.bmax[0], layer.bmax[1], layer.bmax[2]};
  // The production baker uses /fp:fast; at the far edge of the 8 km world its
  // multiply/add sequence can differ from the verifier by one float ULP.
  const float tolerance = std::max(0.001f, params.cs * 0.001f);
  for (int i = 0; i < 6; ++i) {
    if (!std::isfinite(actual[i]) ||
        std::fabs(actual[i] - expected[i]) > tolerance) {
      std::ostringstream message;
      message << label << ": layer bmin/bmax does not match tile formula at ("
              << layer.tx << ',' << layer.ty << ',' << layer.tlayer
              << ") component=" << i << " actual=" << std::setprecision(9)
              << actual[i] << " expected=" << expected[i];
      throw std::runtime_error(message.str());
    }
  }
}

void validateUniqueContiguous(const std::vector<Record> &records,
                              const std::string &label) {
  std::map<std::pair<int, int>, std::vector<int>> columns;
  std::set<LayerKey> keys;
  for (const Record &record : records) {
    if (!keys.insert(record.key).second)
      throw std::runtime_error(label + ": duplicate (tx,ty,tlayer)");
    columns[{record.key.tx, record.key.ty}].push_back(record.key.layer);
  }
  for (auto &[column, layers] : columns) {
    std::sort(layers.begin(), layers.end());
    for (std::size_t i = 0; i < layers.size(); ++i) {
      if (layers[i] != static_cast<int>(i)) {
        std::ostringstream message;
        message << label << ": non-contiguous layers at column ("
                << column.first << ',' << column.second << ')';
        throw std::runtime_error(message.str());
      }
    }
  }
}

std::vector<fs::path> enumerateParts(const fs::path &directory) {
  if (!fs::is_directory(directory))
    throw std::runtime_error("not a cache directory: " + directory.string());
  const std::regex pattern(R"(^z1_cache_([0-9]+)\.bin$)");
  std::map<int, fs::path> indexed;
  for (const auto &entry : fs::directory_iterator(directory)) {
    if (!entry.is_regular_file())
      continue;
    std::smatch match;
    const std::string name = entry.path().filename().string();
    if (!std::regex_match(name, match, pattern))
      continue;
    const int index = std::stoi(match[1].str());
    if (!indexed.emplace(index, entry.path()).second)
      throw std::runtime_error("duplicate cache part index");
  }
  if (indexed.empty() || indexed.begin()->first != 0)
    throw std::runtime_error("cache set has no z1_cache_0.bin");
  std::vector<fs::path> result;
  for (int expected = 0; expected <= indexed.rbegin()->first; ++expected) {
    auto found = indexed.find(expected);
    if (found == indexed.end())
      throw std::runtime_error("cache part sequence has a gap at index " +
                               std::to_string(expected));
    result.push_back(found->second);
  }
  return result;
}

LoadedSet loadSet(const fs::path &directory, const std::string &label) {
  const std::vector<fs::path> parts = enumerateParts(directory);
  LoadedSet result;
  Sha256 datasetHash;
  bool sawHeader = false;
  for (std::size_t partIndex = 0; partIndex < parts.size(); ++partIndex) {
    const std::vector<unsigned char> bytes = readBytes(parts[partIndex]);
    datasetHash.update(bytes.data(), bytes.size());
    result.files.push_back({parts[partIndex].filename().string(), bytes.size(),
                            sha256(bytes)});
    std::size_t offset = 0;
    if (partIndex == 0) {
      if (bytes.size() < sizeof(TileCacheSetHeader))
        throw std::runtime_error(label + ": truncated set header");
      std::memcpy(&result.header, bytes.data(), sizeof(result.header));
      validateSetHeader(result.header, label);
      offset = sizeof(TileCacheSetHeader);
      sawHeader = true;
    }
    while (offset < bytes.size()) {
      if (bytes.size() - offset < sizeof(TileCacheTileHeader))
        throw std::runtime_error(label + ": trailing bytes after last record");
      TileCacheTileHeader recordHeader{};
      std::memcpy(&recordHeader, bytes.data() + offset,
                  sizeof(recordHeader));
      offset += sizeof(recordHeader);
      const int minimumDataSize =
          static_cast<int>((sizeof(dtTileCacheLayerHeader) + 3) & ~3U);
      if (recordHeader.dataSize < minimumDataSize ||
          static_cast<std::size_t>(recordHeader.dataSize) >
              bytes.size() - offset)
        throw std::runtime_error(label + ": invalid/truncated record size");
      auto data = std::make_shared<std::vector<unsigned char>>(
          bytes.begin() + static_cast<std::ptrdiff_t>(offset),
          bytes.begin() +
              static_cast<std::ptrdiff_t>(offset + recordHeader.dataSize));
      dtTileCacheLayerHeader layer{};
      std::memcpy(&layer, data->data(), sizeof(layer));
      validateLayer(layer, result.header, label);
      result.records.push_back({{layer.tx, layer.ty, layer.tlayer}, data});
      offset += static_cast<std::size_t>(recordHeader.dataSize);
    }
  }
  if (!sawHeader || result.header.numTiles !=
                        static_cast<int>(result.records.size()))
    throw std::runtime_error(label + ": header tile count mismatch");
  validateUniqueContiguous(result.records, label);
  std::sort(result.records.begin(), result.records.end(),
            [](const Record &a, const Record &b) { return a.key < b.key; });
  result.datasetSha = datasetHash.finish();
  return result;
}

struct PolycountOffender {
  LayerKey key;
  int polygonCount = 0;
  float worldX = 0;
  float worldZ = 0;
};

struct MaterializationFailure {
  LayerKey key;
  std::string stage;
  dtStatus status = 0;
  float worldX = 0;
  float worldZ = 0;
};

struct PolycountStats {
  std::map<int, std::size_t> histogram;
  std::vector<int> counts;
  std::vector<PolycountOffender> offenders;
  std::vector<MaterializationFailure> failures;
  std::size_t totalLayers = 0;
  std::size_t columns = 0;
  int maxLayersPerColumn = 0;
  std::size_t over32 = 0;
  std::size_t over64 = 0;
  std::size_t over128 = 0;
  std::size_t zeroPolygons = 0;
  int p50 = 0;
  int p95 = 0;
  int p99 = 0;
  int maximum = 0;
  double elapsedSeconds = 0;
};

std::string statusText(dtStatus status) {
  std::ostringstream text;
  text << "0x" << std::hex << std::setw(8) << std::setfill('0')
       << static_cast<unsigned int>(status);
  return text.str();
}

struct HistogramMeshProcess : dtTileCacheMeshProcess {
  explicit HistogramMeshProcess(
      const std::vector<h1emu::nav::NavigationTransition> *transitions)
      : transitions(transitions) {}

  void process(dtNavMeshCreateParams *params, unsigned char *polyAreas,
               unsigned short *polyFlags) override {
    for (int i = 0; i < params->polyCount; ++i)
      polyFlags[i] = flagsForArea(polyAreas[i]);
    if (!transitions || transitions->empty())
      return;
    binding = h1emu::nav::buildNavigationTransitionBinding(
        *transitions, params->bmin, params->bmax, params->walkableClimb);
    params->offMeshConVerts =
        binding.vertices.empty() ? nullptr : binding.vertices.data();
    params->offMeshConRad =
        binding.radii.empty() ? nullptr : binding.radii.data();
    params->offMeshConDir =
        binding.directions.empty() ? nullptr : binding.directions.data();
    params->offMeshConAreas =
        binding.areas.empty() ? nullptr : binding.areas.data();
    params->offMeshConFlags =
        binding.flags.empty() ? nullptr : binding.flags.data();
    params->offMeshConUserID =
        binding.userIds.empty() ? nullptr : binding.userIds.data();
    params->offMeshConCount = static_cast<int>(binding.size());
  }

  const std::vector<h1emu::nav::NavigationTransition> *transitions;
  h1emu::nav::NavigationTransitionBinding binding;
};

struct LayerMeasurement {
  bool success = false;
  int polygons = 0;
  std::string stage;
  dtStatus status = 0;
};

LayerMeasurement measureLayer(
    const Record &record, const TileCacheSetHeader &setHeader,
    LinearAllocator &allocator, FastLZCompressor &compressor,
    HistogramMeshProcess &meshProcess) {
  allocator.reset();
  dtTileCacheLayer *layer = nullptr;
  dtStatus status = dtDecompressTileCacheLayer(
      &allocator, &compressor, record.data->data(),
      static_cast<int>(record.data->size()), &layer);
  if (dtStatusFailed(status))
    return {false, 0, "decompress", status};
  const int walkableClimbVoxels = static_cast<int>(
      setHeader.cacheParams.walkableClimb / setHeader.cacheParams.ch);
  status =
      dtBuildTileCacheRegions(&allocator, *layer, walkableClimbVoxels);
  if (dtStatusFailed(status))
    return {false, 0, "regions", status};
  dtTileCacheContourSet *contours =
      dtAllocTileCacheContourSet(&allocator);
  if (!contours)
    return {false, 0, "contours allocation",
            DT_FAILURE | DT_OUT_OF_MEMORY};
  status = dtBuildTileCacheContours(
      &allocator, *layer, walkableClimbVoxels,
      setHeader.cacheParams.maxSimplificationError, *contours);
  if (dtStatusFailed(status))
    return {false, 0, "contours", status};
  dtTileCachePolyMesh *mesh = dtAllocTileCachePolyMesh(&allocator);
  if (!mesh)
    return {false, 0, "polymesh allocation", DT_FAILURE | DT_OUT_OF_MEMORY};
  status = dtBuildTileCachePolyMesh(&allocator, *contours, *mesh);
  if (dtStatusFailed(status))
    return {false, 0, "polymesh", status};
  if (!mesh->npolys)
    return {true, 0, {}, DT_SUCCESS};

  dtNavMeshCreateParams params{};
  params.verts = mesh->verts;
  params.vertCount = mesh->nverts;
  params.polys = mesh->polys;
  params.polyAreas = mesh->areas;
  params.polyFlags = mesh->flags;
  params.polyCount = mesh->npolys;
  params.nvp = DT_VERTS_PER_POLYGON;
  params.walkableHeight = setHeader.cacheParams.walkableHeight;
  params.walkableRadius = setHeader.cacheParams.walkableRadius;
  params.walkableClimb = setHeader.cacheParams.walkableClimb;
  params.tileX = layer->header->tx;
  params.tileY = layer->header->ty;
  params.tileLayer = layer->header->tlayer;
  params.cs = setHeader.cacheParams.cs;
  params.ch = setHeader.cacheParams.ch;
  params.buildBvTree = false;
  dtVcopy(params.bmin, layer->header->bmin);
  dtVcopy(params.bmax, layer->header->bmax);
  meshProcess.process(&params, mesh->areas, mesh->flags);

  unsigned char *navData = nullptr;
  int navDataSize = 0;
  if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
    return {false, 0, "navmesh data", DT_FAILURE};
  const dtMeshHeader *header =
      reinterpret_cast<const dtMeshHeader *>(navData);
  const int polygons = header ? header->polyCount : 0;
  dtFree(navData);
  return {true, polygons, {}, DT_SUCCESS};
}

void writePolycountReport(const fs::path &path, const LoadedSet &set,
                          const PolycountStats &stats, int limit,
                          std::size_t transitionCount) {
  if (path.empty())
    return;
  if (!path.parent_path().empty())
    fs::create_directories(path.parent_path());
  std::ofstream report(path, std::ios::binary);
  if (!report)
    throw std::runtime_error("cannot write report " + path.string());
  report << "{\n  \"schema\":\"h1emu-tilecache-polycount-v1\",\n"
         << "  \"datasetSha256\":\"" << set.datasetSha << "\",\n"
         << "  \"layers\":" << stats.totalLayers << ",\n"
         << "  \"measuredLayers\":" << stats.counts.size() << ",\n"
         << "  \"columns\":" << stats.columns << ",\n"
         << "  \"maxLayersPerColumn\":" << stats.maxLayersPerColumn
         << ",\n  \"transitionCount\":" << transitionCount
         << ",\n  \"limit\":" << limit << ",\n"
         << "  \"overLimit\":" << stats.offenders.size()
         << ",\n  \"materializationFailures\":"
         << stats.failures.size() << ",\n  \"over32\":" << stats.over32
         << ",\n  \"over64\":" << stats.over64
         << ",\n  \"over128\":" << stats.over128
         << ",\n  \"zeroPolygons\":" << stats.zeroPolygons
         << ",\n  \"p50\":" << stats.p50
         << ",\n  \"p95\":" << stats.p95
         << ",\n  \"p99\":" << stats.p99
         << ",\n  \"maximum\":" << stats.maximum
         << ",\n  \"elapsedSeconds\":" << std::fixed
         << std::setprecision(6) << stats.elapsedSeconds
         << ",\n  \"histogram\":{";
  bool first = true;
  for (const auto &[polygons, count] : stats.histogram) {
    if (!first)
      report << ',';
    first = false;
    report << '\"' << polygons << "\":" << count;
  }
  report << "},\n  \"offenders\":[";
  for (std::size_t i = 0; i < stats.offenders.size(); ++i) {
    if (i)
      report << ',';
    const PolycountOffender &offender = stats.offenders[i];
    report << "{\"tx\":" << offender.key.tx << ",\"ty\":"
           << offender.key.ty << ",\"layer\":" << offender.key.layer
           << ",\"worldX\":" << offender.worldX << ",\"worldZ\":"
           << offender.worldZ << ",\"polygons\":"
           << offender.polygonCount << '}';
  }
  report << "],\n  \"failures\":[";
  for (std::size_t i = 0; i < stats.failures.size(); ++i) {
    if (i)
      report << ',';
    const MaterializationFailure &failure = stats.failures[i];
    report << "{\"tx\":" << failure.key.tx << ",\"ty\":"
           << failure.key.ty << ",\"layer\":" << failure.key.layer
           << ",\"worldX\":" << failure.worldX << ",\"worldZ\":"
           << failure.worldZ << ",\"stage\":\"" << failure.stage
           << "\",\"status\":\"" << statusText(failure.status)
           << "\"}";
  }
  report << "]\n}\n";
  if (!report)
    throw std::runtime_error("short write to report " + path.string());
}

bool runPolycountHistogram(
    const LoadedSet &set,
    const std::vector<h1emu::nav::NavigationTransition> &transitions,
    int limit, const fs::path &reportPath, int progressEvery,
    const std::optional<std::pair<int, int>> &columnFilter) {
  const auto started = std::chrono::steady_clock::now();
  PolycountStats stats;
  stats.counts.reserve(set.records.size());
  std::map<std::pair<int, int>, int> columnLayers;
  for (const Record &record : set.records)
    columnLayers[{record.key.tx, record.key.ty}]++;
  if (columnFilter) {
    const auto found = columnLayers.find(*columnFilter);
    if (found == columnLayers.end())
      throw std::runtime_error("requested histogram column is not in the set");
    const auto selected = *found;
    columnLayers.clear();
    columnLayers.insert(selected);
  }
  stats.columns = columnLayers.size();
  for (const auto &[column, layers] : columnLayers) {
    (void)column;
    stats.maxLayersPerColumn = std::max(stats.maxLayersPerColumn, layers);
    stats.totalLayers += static_cast<std::size_t>(layers);
  }

  LinearAllocator allocator(16ULL * 1024ULL * 1024ULL);
  FastLZCompressor compressor;
  HistogramMeshProcess meshProcess(&transitions);
  const float tileWidth = set.header.cacheParams.width *
                          set.header.cacheParams.cs;
  const float tileHeight = set.header.cacheParams.height *
                           set.header.cacheParams.cs;
  std::size_t processedLayers = 0;
  for (const Record &record : set.records) {
    if (columnFilter &&
        std::pair<int, int>{record.key.tx, record.key.ty} != *columnFilter)
      continue;
    const LayerMeasurement measurement = measureLayer(
        record, set.header, allocator, compressor, meshProcess);
    const float worldX = set.header.cacheParams.orig[0] +
                         (record.key.tx + 0.5f) * tileWidth;
    const float worldZ = set.header.cacheParams.orig[2] +
                         (record.key.ty + 0.5f) * tileHeight;
    if (!measurement.success) {
      stats.failures.push_back({record.key, measurement.stage,
                                measurement.status, worldX, worldZ});
      processedLayers++;
      if (processedLayers % static_cast<std::size_t>(progressEvery) == 0 ||
          processedLayers == stats.totalLayers)
        std::cout << "\rPolycount " << processedLayers << '/'
                  << stats.totalLayers << " layers" << std::flush;
      continue;
    }
    const int polygons = measurement.polygons;
    stats.counts.push_back(polygons);
    stats.histogram[polygons]++;
    if (polygons == 0)
      stats.zeroPolygons++;
    if (polygons > 32)
      stats.over32++;
    if (polygons > 64)
      stats.over64++;
    if (polygons > 128)
      stats.over128++;
    if (polygons > limit) {
      stats.offenders.push_back({record.key, polygons, worldX, worldZ});
    }
    processedLayers++;
    if (processedLayers % static_cast<std::size_t>(progressEvery) == 0 ||
        processedLayers == stats.totalLayers)
      std::cout << "\rPolycount " << processedLayers << '/'
                << stats.totalLayers << " layers"
                << std::flush;
  }
  std::cout << '\n';
  stats.elapsedSeconds = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - started)
                             .count();
  std::sort(stats.counts.begin(), stats.counts.end());
  const auto percentile = [&](double fraction) {
    const std::size_t index = static_cast<std::size_t>(
        std::ceil(fraction * static_cast<double>(stats.counts.size())));
    return stats.counts[std::min(std::max<std::size_t>(index, 1) - 1,
                                 stats.counts.size() - 1)];
  };
  if (!stats.counts.empty()) {
    stats.p50 = percentile(0.50);
    stats.p95 = percentile(0.95);
    stats.p99 = percentile(0.99);
    stats.maximum = stats.counts.back();
  }
  std::cout << "Polycount summary: layers=" << stats.totalLayers
            << " measured=" << stats.counts.size()
            << " materializationFailures=" << stats.failures.size()
            << " columns=" << stats.columns
            << " maxLayersPerColumn=" << stats.maxLayersPerColumn
            << " zero=" << stats.zeroPolygons
            << " p50=" << stats.p50 << " p95=" << stats.p95
            << " p99=" << stats.p99 << " max=" << stats.maximum
            << " limit=" << limit
            << " overLimit=" << stats.offenders.size()
            << " over32=" << stats.over32 << " over64=" << stats.over64
            << " over128=" << stats.over128 << " elapsed=" << std::fixed
            << std::setprecision(3) << stats.elapsedSeconds << "s\n";
  std::cout << "Polycount histogram:";
  for (const auto &[polygons, count] : stats.histogram)
    std::cout << ' ' << polygons << ':' << count;
  std::cout << '\n';
  for (std::size_t i = 0; i < std::min<std::size_t>(20, stats.offenders.size());
       ++i) {
    const PolycountOffender &offender = stats.offenders[i];
    std::cerr << "[ERROR] polycount exceeds limit (tx=" << offender.key.tx
              << " ty=" << offender.key.ty
              << " layer=" << offender.key.layer
              << " world=" << offender.worldX << ',' << offender.worldZ
              << " polygons=" << offender.polygonCount
              << " limit=" << limit << ")\n";
  }
  if (stats.offenders.size() > 20)
    std::cerr << "[ERROR] " << stats.offenders.size() - 20
              << " additional offender(s) recorded in the report.\n";
  for (std::size_t i = 0;
       i < std::min<std::size_t>(20, stats.failures.size()); ++i) {
    const MaterializationFailure &failure = stats.failures[i];
    std::cerr << "[ERROR] materialization failed (tx=" << failure.key.tx
              << " ty=" << failure.key.ty
              << " layer=" << failure.key.layer
              << " world=" << failure.worldX << ',' << failure.worldZ
              << " stage=" << failure.stage
              << " status=" << statusText(failure.status) << ")\n";
  }
  if (stats.failures.size() > 20)
    std::cerr << "[ERROR] " << stats.failures.size() - 20
              << " additional materialization failure(s) recorded in the "
                 "report.\n";
  writePolycountReport(reportPath, set, stats, limit, transitions.size());
  if (!stats.offenders.empty() || !stats.failures.empty()) {
    std::cerr << "Polycount histogram FAIL: " << stats.offenders.size()
              << " layer(s) exceed " << limit << " polygons and "
              << stats.failures.size() << " layer(s) do not materialize.\n";
    return false;
  }
  std::cout << "Polycount histogram PASS: all " << stats.totalLayers
            << " layer(s) fit <= " << limit << " polygons.\n";
  return true;
}

template <typename T>
void compareValue(const T &base, const T &overlay, const char *name) {
  if (base != overlay)
    throw std::runtime_error(std::string("parameter mismatch: ") + name);
}

void compareFloat(float base, float overlay, const char *name) {
  if (!sameFloat(base, overlay))
    throw std::runtime_error(std::string("parameter mismatch: ") + name);
}

void requireCompatible(const TileCacheSetHeader &base,
                       const TileCacheSetHeader &overlay) {
  for (int i = 0; i < 3; ++i) {
    compareFloat(base.meshParams.orig[i], overlay.meshParams.orig[i],
                 "meshParams.orig");
    compareFloat(base.cacheParams.orig[i], overlay.cacheParams.orig[i],
                 "cacheParams.orig");
  }
  compareFloat(base.meshParams.tileWidth, overlay.meshParams.tileWidth,
               "meshParams.tileWidth");
  compareFloat(base.meshParams.tileHeight, overlay.meshParams.tileHeight,
               "meshParams.tileHeight");
  compareValue(base.meshParams.maxTiles, overlay.meshParams.maxTiles,
               "meshParams.maxTiles");
  compareValue(base.meshParams.maxPolys, overlay.meshParams.maxPolys,
               "meshParams.maxPolys");
  compareFloat(base.cacheParams.cs, overlay.cacheParams.cs, "cacheParams.cs");
  compareFloat(base.cacheParams.ch, overlay.cacheParams.ch, "cacheParams.ch");
  compareValue(base.cacheParams.width, overlay.cacheParams.width,
               "cacheParams.width");
  compareValue(base.cacheParams.height, overlay.cacheParams.height,
               "cacheParams.height");
  compareFloat(base.cacheParams.walkableHeight,
               overlay.cacheParams.walkableHeight,
               "cacheParams.walkableHeight");
  compareFloat(base.cacheParams.walkableRadius,
               overlay.cacheParams.walkableRadius,
               "cacheParams.walkableRadius");
  compareFloat(base.cacheParams.walkableClimb,
               overlay.cacheParams.walkableClimb,
               "cacheParams.walkableClimb");
  compareFloat(base.cacheParams.maxSimplificationError,
               overlay.cacheParams.maxSimplificationError,
               "cacheParams.maxSimplificationError");
  compareValue(base.cacheParams.maxTiles, overlay.cacheParams.maxTiles,
               "cacheParams.maxTiles");
  compareValue(base.cacheParams.maxObstacles,
               overlay.cacheParams.maxObstacles,
               "cacheParams.maxObstacles");
}

std::vector<Record> makeOverlay(const LoadedSet &base,
                                const LoadedSet &overlay,
                                const Rect &replaceCoverage,
                                const Rect &overlayCoverage,
                                OverlayStats &stats) {
  if (replaceCoverage.minTx >= replaceCoverage.maxTx ||
      replaceCoverage.minTy >= replaceCoverage.maxTy ||
      overlayCoverage.minTx >= overlayCoverage.maxTx ||
      overlayCoverage.minTy >= overlayCoverage.maxTy)
    throw std::runtime_error("invalid replacement/overlay coverage");
  if (overlayCoverage.minTx > replaceCoverage.minTx - 1 ||
      overlayCoverage.minTy > replaceCoverage.minTy - 1 ||
      overlayCoverage.maxTx < replaceCoverage.maxTx + 1 ||
      overlayCoverage.maxTy < replaceCoverage.maxTy + 1)
    throw std::runtime_error(
        "overlay coverage must include at least a one-tile halo");
  requireCompatible(base.header, overlay.header);
  for (const Record &record : overlay.records) {
    if (!overlayCoverage.contains(record.key.tx, record.key.ty))
      throw std::runtime_error("overlay layer falls outside declared coverage");
  }
  std::vector<Record> result;
  result.reserve(base.records.size() + overlay.records.size());
  for (const Record &record : base.records) {
    if (replaceCoverage.contains(record.key.tx, record.key.ty))
      ++stats.removedBaseLayers;
    else {
      result.push_back(record);
      ++stats.preservedBaseLayers;
    }
  }
  for (const Record &record : overlay.records) {
    if (replaceCoverage.contains(record.key.tx, record.key.ty)) {
      result.push_back(record);
      ++stats.insertedOverlayLayers;
    } else {
      ++stats.ignoredHaloLayers;
    }
  }
  std::sort(result.begin(), result.end(),
            [](const Record &a, const Record &b) { return a.key < b.key; });
  validateUniqueContiguous(result, "merged output");
  if (result.size() > static_cast<std::size_t>(base.header.cacheParams.maxTiles))
    throw std::runtime_error("merged output exceeds retained full-cache capacity");
  return result;
}

unsigned int ilog2Pow2(unsigned int value) {
  unsigned int bits = 0;
  while ((1u << bits) < value)
    ++bits;
  return bits;
}

std::vector<FileDigest>
writeSetUnchecked(const fs::path &directory, TileCacheSetHeader header,
                  const std::vector<Record> &records,
                  std::uint64_t maxPartBytes) {
  if (fs::exists(directory))
    throw std::runtime_error("output directory already exists: " +
                             directory.string());
  fs::create_directories(directory);
  header.numTiles = static_cast<int>(records.size());
  const unsigned int tileBits =
      ilog2Pow2(static_cast<unsigned int>(header.cacheParams.maxTiles));
  if (tileBits >= 31)
    throw std::runtime_error("cache capacity cannot encode deterministic refs");
  std::vector<std::pair<std::size_t, std::size_t>> parts;
  std::size_t cursor = 0;
  bool first = true;
  if (records.empty())
    parts.push_back({0, 0});
  while (cursor < records.size()) {
    const std::uint64_t headerCost = first ? sizeof(header) : 0;
    if (maxPartBytes <= headerCost + sizeof(TileCacheTileHeader))
      throw std::runtime_error("max part bytes is too small");
    std::uint64_t budget = maxPartBytes - headerCost;
    const std::size_t start = cursor;
    while (cursor < records.size()) {
      const std::uint64_t cost = sizeof(TileCacheTileHeader) +
                                 records[cursor].data->size();
      if (cursor > start && cost > budget)
        break;
      if (cost > budget && cursor == start)
        throw std::runtime_error("one record exceeds max part bytes");
      budget -= cost;
      ++cursor;
    }
    parts.push_back({start, cursor - start});
    first = false;
  }

  std::vector<FileDigest> digests;
  for (std::size_t partIndex = 0; partIndex < parts.size(); ++partIndex) {
    const fs::path path =
        directory / ("z1_cache_" + std::to_string(partIndex) + ".bin");
    std::ofstream output(path, std::ios::binary);
    if (!output)
      throw std::runtime_error("cannot write " + path.string());
    Sha256 hash;
    std::uint64_t bytesWritten = 0;
    auto write = [&](const void *data, std::size_t size) {
      output.write(reinterpret_cast<const char *>(data),
                   static_cast<std::streamsize>(size));
      if (!output)
        throw std::runtime_error("short write to " + path.string());
      hash.update(reinterpret_cast<const unsigned char *>(data), size);
      bytesWritten += size;
    };
    if (partIndex == 0)
      write(&header, sizeof(header));
    const auto [start, count] = parts[partIndex];
    for (std::size_t i = 0; i < count; ++i) {
      const std::size_t recordIndex = start + i;
      TileCacheTileHeader recordHeader{};
      recordHeader.tileRef =
          (dtCompressedTileRef(1) << tileBits) |
          static_cast<dtCompressedTileRef>(recordIndex);
      recordHeader.dataSize =
          static_cast<int>(records[recordIndex].data->size());
      write(&recordHeader, sizeof(recordHeader));
      write(records[recordIndex].data->data(),
            records[recordIndex].data->size());
    }
    output.close();
    digests.push_back(
        {path.filename().string(), bytesWritten, hash.finish()});
  }
  return digests;
}

void verifyOutput(const LoadedSet &expectedBaseHeader,
                  const std::vector<Record> &expected,
                  const LoadedSet &actual) {
  requireCompatible(expectedBaseHeader.header, actual.header);
  if (expected.size() != actual.records.size())
    throw std::runtime_error("output self-verification count mismatch");
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (!(expected[i].key == actual.records[i].key) ||
        sha256(*expected[i].data) != sha256(*actual.records[i].data))
      throw std::runtime_error("output self-verification payload mismatch");
  }
}

std::string jsonRect(const Rect &rect) {
  std::ostringstream out;
  out << "[" << rect.minTx << ',' << rect.minTy << ',' << rect.maxTx << ','
      << rect.maxTy << ']';
  return out.str();
}

void writeReport(const fs::path &path, const LoadedSet &base,
                 const LoadedSet &overlay, const LoadedSet &output,
                 const Rect &replaceCoverage, const Rect &overlayCoverage,
                 const OverlayStats &stats) {
  if (!path.parent_path().empty())
    fs::create_directories(path.parent_path());
  std::ofstream report(path, std::ios::binary);
  if (!report)
    throw std::runtime_error("cannot write report " + path.string());
  auto fileList = [&](const LoadedSet &set) {
    report << "{\"numTiles\":" << set.records.size()
           << ",\"datasetSha256\":\"" << set.datasetSha
           << "\",\"parts\":[";
    for (std::size_t i = 0; i < set.files.size(); ++i) {
      if (i)
        report << ',';
      report << "{\"name\":\"" << set.files[i].name << "\",\"bytes\":"
             << set.files[i].bytes << ",\"sha256\":\"" << set.files[i].sha
             << "\"}";
    }
    report << "]}";
  };
  report << "{\n  \"schema\":\"h1emu-tilecache-overlay-v1\",\n"
         << "  \"replaceCoverage\":" << jsonRect(replaceCoverage) << ",\n"
         << "  \"overlayCoverage\":" << jsonRect(overlayCoverage) << ",\n"
         << "  \"stats\":{\"preservedBaseLayers\":"
         << stats.preservedBaseLayers << ",\"removedBaseLayers\":"
         << stats.removedBaseLayers << ",\"insertedOverlayLayers\":"
         << stats.insertedOverlayLayers << ",\"ignoredHaloLayers\":"
         << stats.ignoredHaloLayers << "},\n  \"base\":";
  fileList(base);
  report << ",\n  \"overlay\":";
  fileList(overlay);
  report << ",\n  \"output\":";
  fileList(output);
  report << "\n}\n";
}

struct CliOptions {
  fs::path baseDir, overlayDir, outputDir, reportPath;
  fs::path transitionsPath;
  Rect replaceCoverage, overlayCoverage;
  bool haveReplace = false, haveOverlayCoverage = false;
  bool polycountHistogram = false;
  int maxPolys = 32;
  int progressEvery = 250;
  std::optional<std::pair<int, int>> histogramColumn;
  std::uint64_t maxPartBytes = DEFAULT_PART_BYTES;
};

int parseInt(const char *text) {
  std::size_t used = 0;
  const long long value = std::stoll(text, &used, 10);
  if (used != std::strlen(text) || value < INT32_MIN || value > INT32_MAX)
    throw std::runtime_error(std::string("invalid integer: ") + text);
  return static_cast<int>(value);
}

CliOptions parseCli(int argc, char **argv) {
  CliOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> const char * {
      if (++i >= argc)
        throw std::runtime_error("missing value for " + arg);
      return argv[i];
    };
    if (arg == "--base-dir")
      options.baseDir = value();
    else if (arg == "--polycount-histogram")
      options.polycountHistogram = true;
    else if (arg == "--overlay-dir")
      options.overlayDir = value();
    else if (arg == "--output-dir")
      options.outputDir = value();
    else if (arg == "--report")
      options.reportPath = value();
    else if (arg == "--transitions")
      options.transitionsPath = value();
    else if (arg == "--max-polys")
      options.maxPolys = parseInt(value());
    else if (arg == "--progress-every")
      options.progressEvery = parseInt(value());
    else if (arg == "--column") {
      const int tx = parseInt(value());
      const int ty = parseInt(value());
      options.histogramColumn = std::pair<int, int>{tx, ty};
    }
    else if (arg == "--max-part-bytes") {
      const std::string text = value();
      std::size_t used = 0;
      options.maxPartBytes = std::stoull(text, &used, 10);
      if (used != text.size())
        throw std::runtime_error("invalid --max-part-bytes");
    } else if (arg == "--replace-coverage" ||
               arg == "--overlay-coverage") {
      Rect rect;
      rect.minTx = parseInt(value());
      rect.minTy = parseInt(value());
      rect.maxTx = parseInt(value());
      rect.maxTy = parseInt(value());
      if (arg == "--replace-coverage") {
        options.replaceCoverage = rect;
        options.haveReplace = true;
      } else {
        options.overlayCoverage = rect;
        options.haveOverlayCoverage = true;
      }
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }
  if (options.polycountHistogram) {
    if (options.baseDir.empty() || options.maxPolys < 0 ||
        options.progressEvery <= 0)
      throw std::runtime_error(
          "required: --polycount-histogram --base-dir <cache> "
          "[--transitions <json>] [--max-polys <count>] [--report <json>]");
    if (!options.overlayDir.empty() || !options.outputDir.empty() ||
        options.haveReplace || options.haveOverlayCoverage)
      throw std::runtime_error(
          "polycount mode cannot be combined with overlay output options");
    return options;
  }
  if (options.baseDir.empty() || options.overlayDir.empty() ||
      options.outputDir.empty() || !options.haveReplace ||
      !options.haveOverlayCoverage)
    throw std::runtime_error(
        "required: --base-dir --overlay-dir --output-dir "
        "--replace-coverage minTx minTy maxTx maxTy "
        "--overlay-coverage minTx minTy maxTx maxTy");
  if (options.reportPath.empty())
    options.reportPath = options.outputDir / "tilecache-overlay-report.json";
  return options;
}

std::shared_ptr<std::vector<unsigned char>>
makeTestLayer(const TileCacheSetHeader &set, int tx, int ty, int layer,
              unsigned char marker) {
  auto data = std::make_shared<std::vector<unsigned char>>(
      sizeof(dtTileCacheLayerHeader) + 4, marker);
  dtTileCacheLayerHeader header{};
  header.magic = DT_TILECACHE_MAGIC;
  header.version = DT_TILECACHE_VERSION;
  header.tx = tx;
  header.ty = ty;
  header.tlayer = layer;
  header.width = static_cast<unsigned char>(set.cacheParams.width);
  header.height = static_cast<unsigned char>(set.cacheParams.height);
  header.minx = header.miny = 0;
  header.maxx = header.width - 1;
  header.maxy = header.height - 1;
  header.hmin = 2;
  header.hmax = 4;
  header.bmin[0] = set.cacheParams.orig[0] + tx * header.width *
                                                    set.cacheParams.cs;
  header.bmin[1] = set.cacheParams.orig[1] + header.hmin * set.cacheParams.ch;
  header.bmin[2] = set.cacheParams.orig[2] + ty * header.height *
                                                    set.cacheParams.cs;
  header.bmax[0] = header.bmin[0] + header.width * set.cacheParams.cs;
  header.bmax[1] = set.cacheParams.orig[1] + header.hmax * set.cacheParams.ch;
  header.bmax[2] = header.bmin[2] + header.height * set.cacheParams.cs;
  std::memcpy(data->data(), &header, sizeof(header));
  return data;
}

TileCacheSetHeader testHeader() {
  TileCacheSetHeader header{};
  header.magic = TILECACHESET_MAGIC;
  header.version = TILECACHESET_VERSION;
  header.meshParams.orig[0] = header.cacheParams.orig[0] = 0;
  header.meshParams.orig[1] = header.cacheParams.orig[1] = -10;
  header.meshParams.orig[2] = header.cacheParams.orig[2] = 0;
  header.cacheParams.cs = 1;
  header.cacheParams.ch = 0.5f;
  header.cacheParams.width = header.cacheParams.height = 16;
  header.meshParams.tileWidth = header.meshParams.tileHeight = 16;
  header.meshParams.maxTiles = 32;
  header.meshParams.maxPolys = 131072;
  header.cacheParams.walkableHeight = 2;
  header.cacheParams.walkableRadius = 0.5f;
  header.cacheParams.walkableClimb = 0.5f;
  header.cacheParams.maxSimplificationError = 1;
  header.cacheParams.maxTiles = 256;
  header.cacheParams.maxObstacles = 64;
  return header;
}

void expectFailure(const std::string &name, const std::string &needle,
                   const std::function<void()> &operation) {
  try {
    operation();
  } catch (const std::exception &error) {
    if (std::string(error.what()).find(needle) != std::string::npos)
      return;
    throw std::runtime_error(name + " failed with wrong diagnostic: " +
                             error.what());
  }
  throw std::runtime_error(name + " unexpectedly succeeded");
}

int selfTest() {
  const fs::path root = fs::temp_directory_path() /
                        ("h1emu-overlay-test-" +
                         std::to_string(std::chrono::high_resolution_clock::now()
                                            .time_since_epoch()
                                            .count()));
  try {
    const std::vector<unsigned char> abc = {'a', 'b', 'c'};
    if (sha256(abc) !=
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad")
      throw std::runtime_error("SHA-256 known-vector test failed");
    const TileCacheSetHeader header = testHeader();
    std::vector<Record> baseRecords;
    for (int tx : {0, 1, 2, 4})
      baseRecords.push_back(
          {{tx, 1, 0}, makeTestLayer(header, tx, 1, 0,
                                     static_cast<unsigned char>(tx + 1))});
    std::vector<Record> overlayRecords = {
        {{0, 0, 0}, makeTestLayer(header, 0, 0, 0, 10)},
        {{1, 1, 0}, makeTestLayer(header, 1, 1, 0, 11)},
        {{3, 1, 0}, makeTestLayer(header, 3, 1, 0, 12)}};
    writeSetUnchecked(root / "base", header, baseRecords, 256);
    writeSetUnchecked(root / "overlay", header, overlayRecords, 256);
    const LoadedSet base = loadSet(root / "base", "base");
    const LoadedSet overlay = loadSet(root / "overlay", "overlay");
    OverlayStats stats;
    const Rect replace{1, 1, 3, 2};
    const Rect halo{0, 0, 4, 3};
    const std::vector<Record> merged =
        makeOverlay(base, overlay, replace, halo, stats);
    if (merged.size() != 3 || merged[0].key.tx != 0 ||
        merged[1].key.tx != 1 || merged[2].key.tx != 4 ||
        stats.removedBaseLayers != 2 || stats.ignoredHaloLayers != 2)
      throw std::runtime_error("whole-column/halo overlay semantics failed");
    writeSetUnchecked(root / "out-a", base.header, merged, 256);
    writeSetUnchecked(root / "out-b", base.header, merged, 256);
    const LoadedSet outA = loadSet(root / "out-a", "out-a");
    const LoadedSet outB = loadSet(root / "out-b", "out-b");
    verifyOutput(base, merged, outA);
    if (outA.datasetSha != outB.datasetSha)
      throw std::runtime_error("deterministic output hash test failed");

    TileCacheSetHeader mismatch = header;
    mismatch.cacheParams.walkableClimb = 1.3f;
    std::vector<Record> mismatchRecords = {
        {{1, 1, 0}, makeTestLayer(mismatch, 1, 1, 0, 13)}};
    writeSetUnchecked(root / "mismatch", mismatch, mismatchRecords, 256);
    const LoadedSet badParams = loadSet(root / "mismatch", "mismatch");
    expectFailure("parameter mismatch", "cacheParams.walkableClimb", [&] {
      OverlayStats ignored;
      makeOverlay(base, badParams, replace, halo, ignored);
    });

    std::vector<Record> duplicate = {
        {{1, 1, 0}, makeTestLayer(header, 1, 1, 0, 1)},
        {{1, 1, 0}, makeTestLayer(header, 1, 1, 0, 2)}};
    writeSetUnchecked(root / "duplicate", header, duplicate, 256);
    expectFailure("duplicate", "duplicate (tx,ty,tlayer)", [&] {
      (void)loadSet(root / "duplicate", "duplicate");
    });

    std::vector<Record> layerGap = {
        {{1, 1, 1}, makeTestLayer(header, 1, 1, 1, 3)}};
    writeSetUnchecked(root / "layer-gap", header, layerGap, 256);
    expectFailure("layer gap", "non-contiguous layers", [&] {
      (void)loadSet(root / "layer-gap", "layer-gap");
    });

    auto badBoundsLayer = makeTestLayer(header, 1, 1, 0, 4);
    dtTileCacheLayerHeader badBounds{};
    std::memcpy(&badBounds, badBoundsLayer->data(), sizeof(badBounds));
    badBounds.bmin[0] += 1.0f;
    std::memcpy(badBoundsLayer->data(), &badBounds, sizeof(badBounds));
    std::vector<Record> badBoundsRecords = {{{1, 1, 0}, badBoundsLayer}};
    writeSetUnchecked(root / "bad-bounds", header, badBoundsRecords, 256);
    expectFailure("layer bounds", "bmin/bmax does not match tile formula", [&] {
      (void)loadSet(root / "bad-bounds", "bad-bounds");
    });

    TileCacheSetHeader wrongMagic = header;
    wrongMagic.magic = 0;
    writeSetUnchecked(root / "wrong-magic", wrongMagic, overlayRecords, 256);
    expectFailure("schema", "wrong TileCache set magic", [&] {
      (void)loadSet(root / "wrong-magic", "schema");
    });

    fs::copy(root / "overlay", root / "trailing",
             fs::copy_options::recursive);
    std::ofstream trailing(root / "trailing" / "z1_cache_1.bin",
                           std::ios::binary);
    trailing.put('\x7f');
    trailing.close();
    expectFailure("trailing bytes", "trailing bytes", [&] {
      (void)loadSet(root / "trailing", "trailing");
    });
    fs::remove_all(root);
    std::cout << "tilecache-overlay self-test PASS\n";
    return 0;
  } catch (...) {
    fs::remove_all(root);
    throw;
  }
}

} // namespace

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test")
      return selfTest();
    const CliOptions options = parseCli(argc, argv);
    if (options.polycountHistogram) {
      const LoadedSet set = loadSet(options.baseDir, "polycount input");
      std::vector<h1emu::nav::NavigationTransition> transitions;
      if (!options.transitionsPath.empty())
        transitions = h1emu::nav::loadNavigationTransitions(
            options.transitionsPath.string());
      return runPolycountHistogram(set, transitions, options.maxPolys,
                                   options.reportPath,
                                   options.progressEvery,
                                   options.histogramColumn)
                 ? 0
                 : 1;
    }
    const LoadedSet base = loadSet(options.baseDir, "base");
    const LoadedSet overlay = loadSet(options.overlayDir, "overlay");
    OverlayStats stats;
    const std::vector<Record> merged =
        makeOverlay(base, overlay, options.replaceCoverage,
                    options.overlayCoverage, stats);
    try {
      writeSetUnchecked(options.outputDir, base.header, merged,
                        options.maxPartBytes);
      const LoadedSet output = loadSet(options.outputDir, "output");
      verifyOutput(base, merged, output);
      writeReport(options.reportPath, base, overlay, output,
                  options.replaceCoverage, options.overlayCoverage, stats);
      std::cout << "overlay PASS: " << base.records.size() << " base + "
                << stats.insertedOverlayLayers << " replacement - "
                << stats.removedBaseLayers << " removed = "
                << output.records.size() << " layers; sha256="
                << output.datasetSha << '\n';
    } catch (...) {
      if (fs::exists(options.outputDir))
        fs::remove_all(options.outputDir);
      throw;
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "tilecache-overlay: ERROR: " << error.what() << '\n';
    return 1;
  }
}
