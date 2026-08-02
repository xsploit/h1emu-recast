#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#ifdef USE_OPENMP
#include <omp.h>
#endif

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "Recast.h"
#include "fastlz.h"

struct TriGrid {
  std::vector<std::vector<int>> cells;
  int nx = 0, nz = 0;
  float orig[2] = {};
  float cs = 0;

  void build(const float *verts, const int *tris, int ntris, const float *bmin,
             const float *bmax, float cellSize) {
    orig[0] = bmin[0];
    orig[1] = bmin[2];
    cs = cellSize;
    nx = (int)ceilf((bmax[0] - bmin[0]) / cs) + 1;
    nz = (int)ceilf((bmax[2] - bmin[2]) / cs) + 1;
    cells.assign((size_t)nx * nz, {});
    for (int i = 0; i < ntris; i++) {
      const float *v0 = &verts[tris[i * 3 + 0] * 3];
      const float *v1 = &verts[tris[i * 3 + 1] * 3];
      const float *v2 = &verts[tris[i * 3 + 2] * 3];
      float mn[2] = {std::min({v0[0], v1[0], v2[0]}),
                     std::min({v0[2], v1[2], v2[2]})};
      float mx[2] = {std::max({v0[0], v1[0], v2[0]}),
                     std::max({v0[2], v1[2], v2[2]})};
      int x0 = std::max(0, (int)floorf((mn[0] - orig[0]) / cs));
      int z0 = std::max(0, (int)floorf((mn[1] - orig[1]) / cs));
      int x1 = std::min(nx - 1, (int)floorf((mx[0] - orig[0]) / cs));
      int z1 = std::min(nz - 1, (int)floorf((mx[1] - orig[1]) / cs));
      for (int z = z0; z <= z1; z++)
        for (int x = x0; x <= x1; x++)
          cells[(size_t)z * nx + x].push_back(i);
    }
  }

  std::vector<int> query(const float qmin[2], const float qmax[2]) const {
    int x0 = std::max(0, (int)floorf((qmin[0] - orig[0]) / cs));
    int z0 = std::max(0, (int)floorf((qmin[1] - orig[1]) / cs));
    int x1 = std::min(nx - 1, (int)floorf((qmax[0] - orig[0]) / cs));
    int z1 = std::min(nz - 1, (int)floorf((qmax[1] - orig[1]) / cs));
    std::vector<int> out;
    for (int z = z0; z <= z1; z++)
      for (int x = x0; x <= x1; x++)
        for (int tri : cells[(size_t)z * nx + x])
          out.push_back(tri);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
  }
};

// Config
static float CELL_SIZE = 1.2f;
static float CELL_HEIGHT = 0.1f;
static float AGENT_HEIGHT = 2.0f;
static float AGENT_RADIUS = 0.5f;
static float AGENT_MAX_CLIMB = 1.3f;
static float AGENT_MAX_SLOPE = 60.0f;
static float REGION_MIN_SIZE = 16.0f;  // voxels
static float REGION_MERGE_SIZE = 3.0f; // voxels
static float EDGE_MAX_LEN = 12.0f;     // world units
static float EDGE_MAX_ERROR = 1.0f;    // voxels
static const int VERTS_PER_POLY = 6;
static float DETAIL_SAMPLE_DIST = 6.0f;    // world units
static float DETAIL_SAMPLE_MAX_ERR = 2.0f; // voxel heights
static int TILE_SIZE = 64;

static const char *SEMANTIC_CONTRACT = "h1emu-nav-semantics-v1";

enum class NavSemantic : unsigned char {
  Untagged,
  OrdinaryMaterial,
  Terrain,
  Road,
  FloorExterior,
  FloorInterior,
  Stair,
  Ramp,
  Threshold,
  ObstacleStatic,
  DoorPanelDynamic,
  Exclude,
  Unknown,
  Invalid
};

enum NavArea : unsigned char {
  NAV_AREA_TERRAIN = 1,
  NAV_AREA_ROAD = 2,
  NAV_AREA_FLOOR_EXTERIOR = 3,
  NAV_AREA_FLOOR_INTERIOR = 4,
  NAV_AREA_STAIR = 5,
  NAV_AREA_RAMP = 6,
  NAV_AREA_THRESHOLD = 7
};

enum NavFlag : unsigned short {
  NAV_FLAG_WALK = 0x01,
  NAV_FLAG_INDOOR = 0x02,
  NAV_FLAG_TRANSITION = 0x04,
  NAV_FLAG_DOOR = 0x08
};

struct SemanticSpec {
  NavSemantic semantic;
  const char *material;
  unsigned char area;
  unsigned short flags;
  bool rasterizeNull;
  bool exclude;
};

static const SemanticSpec SEMANTIC_SPECS[] = {
    {NavSemantic::Terrain, "nav_terrain", NAV_AREA_TERRAIN, NAV_FLAG_WALK,
     false, false},
    {NavSemantic::Road, "nav_road", NAV_AREA_ROAD, NAV_FLAG_WALK, false,
     false},
    {NavSemantic::FloorExterior, "nav_floor_exterior",
     NAV_AREA_FLOOR_EXTERIOR, NAV_FLAG_WALK, false, false},
    {NavSemantic::FloorInterior, "nav_floor_interior",
     NAV_AREA_FLOOR_INTERIOR, NAV_FLAG_WALK | NAV_FLAG_INDOOR, false, false},
    {NavSemantic::Stair, "nav_stair", NAV_AREA_STAIR,
     NAV_FLAG_WALK | NAV_FLAG_TRANSITION, false, false},
    {NavSemantic::Ramp, "nav_ramp", NAV_AREA_RAMP,
     NAV_FLAG_WALK | NAV_FLAG_TRANSITION, false, false},
    {NavSemantic::Threshold, "nav_threshold", NAV_AREA_THRESHOLD,
     NAV_FLAG_WALK | NAV_FLAG_TRANSITION | NAV_FLAG_DOOR, false, false},
    {NavSemantic::ObstacleStatic, "nav_obstacle_static", RC_NULL_AREA, 0,
     true, false},
    {NavSemantic::DoorPanelDynamic, "nav_door_panel_dynamic", RC_NULL_AREA, 0,
     false, true},
    {NavSemantic::Exclude, "nav_exclude", RC_NULL_AREA, 0, false, true},
    {NavSemantic::Unknown, "nav_unknown", RC_NULL_AREA, 0, true, false},
};

static const SemanticSpec *semanticSpec(NavSemantic semantic) {
  for (const SemanticSpec &spec : SEMANTIC_SPECS)
    if (spec.semantic == semantic)
      return &spec;
  return nullptr;
}

static NavSemantic parseSemanticMaterial(const std::string &material) {
  for (const SemanticSpec &spec : SEMANTIC_SPECS)
    if (material == spec.material)
      return spec.semantic;
  if (material.rfind("nav_", 0) == 0)
    return NavSemantic::Invalid;
  return NavSemantic::OrdinaryMaterial;
}

static unsigned short flagsForArea(unsigned char area) {
  switch (area) {
  case NAV_AREA_TERRAIN:
  case NAV_AREA_ROAD:
  case NAV_AREA_FLOOR_EXTERIOR:
    return NAV_FLAG_WALK;
  case NAV_AREA_FLOOR_INTERIOR:
    return NAV_FLAG_WALK | NAV_FLAG_INDOOR;
  case NAV_AREA_STAIR:
  case NAV_AREA_RAMP:
    return NAV_FLAG_WALK | NAV_FLAG_TRANSITION;
  case NAV_AREA_THRESHOLD:
    return NAV_FLAG_WALK | NAV_FLAG_TRANSITION | NAV_FLAG_DOOR;
  default:
    return 0;
  }
}

struct BuildBounds {
  bool enabled = false;
  float minX = 0.0f;
  float minZ = 0.0f;
  float maxX = 0.0f;
  float maxZ = 0.0f;
};

struct BuildOptions {
  BuildBounds bounds;
  bool legacyObjectFallback = false;
  bool dynamicDoorObstacles = false;
  bool validateSemanticsOnly = false;
  bool verifyBakedSemantics = false;
  bool requireAllSemantics = false;
  std::string semanticReportPath;
};

static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader {
  int magic;
  int version;
  int numTiles;
  dtNavMeshParams params;
};

struct NavMeshTileHeader {
  dtTileRef tileRef;
  int dataSize;
};

static const int TILECACHESET_MAGIC = 'T' << 24 | 'S' << 16 | 'E' << 8 | 'T';
static const int TILECACHESET_VERSION = 1;
static const int EXPECTED_LAYERS_PER_TILE = 16;

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
    return (int)(bufferSize * 1.05f) + 66;
  }
  dtStatus compress(const unsigned char *buffer, const int bufferSize,
                    unsigned char *compressed, const int /*maxCompressedSize*/,
                    int *compressedSize) override {
    *compressedSize = fastlz_compress(buffer, bufferSize, compressed);
    return DT_SUCCESS;
  }
  dtStatus decompress(const unsigned char *compressed, const int compressedSize,
                      unsigned char *buffer, const int maxBufferSize,
                      int *bufferSize) override {
    *bufferSize =
        fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
    return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
  }
};

struct NullMeshProcess : dtTileCacheMeshProcess {
  void process(dtNavMeshCreateParams *params, unsigned char *polyAreas,
               unsigned short *polyFlags) override {
    for (int i = 0; i < params->polyCount; ++i)
      polyFlags[i] = flagsForArea(polyAreas[i]);
  }
};

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

static double elapsed(TimePoint t0) {
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

static std::string fmtDuration(double s) {
  char buf[64];
  if (s < 60.0)
    snprintf(buf, sizeof(buf), "%.1fs", s);
  else if (s < 3600.0)
    snprintf(buf, sizeof(buf), "%dm %02ds", (int)s / 60, (int)s % 60);
  else
    snprintf(buf, sizeof(buf), "%dh %dm", (int)s / 3600, ((int)s % 3600) / 60);
  return buf;
}

static std::string fmtSize(long long bytes) {
  char buf[32];
  if (bytes < 1024)
    snprintf(buf, sizeof(buf), "%lld B", bytes);
  else if (bytes < 1024 * 1024)
    snprintf(buf, sizeof(buf), "%.1f KB", bytes / 1024.0);
  else
    snprintf(buf, sizeof(buf), "%.1f MB", bytes / (1024.0 * 1024.0));
  return buf;
}

static void printProgress(int done, int total, int built, long long totalBytes,
                          double elapsedSec) {
  const int barWidth = 38;
  float pct = total > 0 ? (float)done / (float)total : 0.0f;
  int filled = (int)(pct * barWidth);

  // ETA
  std::string eta = "---";
  if (done > 0 && done < total) {
    double secPerTile = elapsedSec / done;
    double remaining = secPerTile * (total - done);
    eta = fmtDuration(remaining);
  }

  char bar[64];
  int pos = 0;
  bar[pos++] = '[';
  for (int i = 0; i < barWidth; i++)
    bar[pos++] = (i < filled) ? '#' : '.';
  bar[pos++] = ']';
  bar[pos] = '\0';

  printf("\r%s %5.1f%%  %d/%d tiles  ETA %-8s  built: %d  size: %s   ", bar,
         pct * 100.0f, done, total, eta.c_str(), built,
         fmtSize(totalBytes).c_str());
  fflush(stdout);
}

class PrintContext : public rcContext {
  int tileX;
  int tileY;

protected:
  void doLog(const rcLogCategory category, const char *msg,
             const int /*len*/) override {
    if (category == RC_LOG_ERROR) {
      hadError = true;
      if (tileX >= 0)
        fprintf(stderr, "\n[ERROR] tile=(%d,%d) %s\n", tileX, tileY, msg);
      else
        fprintf(stderr, "\n[ERROR] %s\n", msg);
    } else if (category == RC_LOG_WARNING) {
      if (tileX >= 0)
        fprintf(stderr, "\n[WARN]  tile=(%d,%d) %s\n", tileX, tileY, msg);
      else
        fprintf(stderr, "\n[WARN]  %s\n", msg);
    }
  }

public:
  bool hadError = false;

  PrintContext(int x = -1, int y = -1) : tileX(x), tileY(y) {}
};

static unsigned int nextPow2(unsigned int v) {
  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  return ++v;
}
static unsigned int ilog2(unsigned int v) {
  unsigned int r = (v > 0xffff) << 4;
  v >>= r;
  unsigned int s = (v > 0xff) << 3;
  v >>= s;
  r |= s;
  s = (v > 0xf) << 2;
  v >>= s;
  r |= s;
  s = (v > 0x3) << 1;
  v >>= s;
  r |= s;
  return r | (v >> 1);
}

struct NonWalkableVolume {
  float bmin[3];
  float bmax[3];
};

struct Mesh {
  std::vector<float> verts; // x,y,z triples
  std::vector<int> tris;    // 0-based triangle indices
  // Parallel to tris (one entry per triangle). These triangles still
  // rasterize into the heightfield as solid geometry, but their upward faces
  // must never become walkable navmesh.
  std::vector<unsigned char> nonWalkableTris;
  // Parallel to tris (one entry per triangle). Semantic OBJ input assigns one
  // canonical nav_* material to every source face before triangulation.
  std::vector<NavSemantic> triangleSemantics;
  // Parallel to tris. Component-level semantic rules can override the default
  // material of the enclosing OBJ object without allowing an unrelated prop
  // object to lose collision at an overlapping position.
  std::vector<int> triangleObjects;
  std::vector<NonWalkableVolume> nonWalkableVolumes;
  std::map<std::string, long long> semanticHistogram;
  std::set<std::string> invalidSemanticMaterials;
  long long sourceTriangles = 0;
  long long excludedSemanticTriangles = 0;
  long long fallbackTriangles = 0;
  long long ordinaryMaterialTriangles = 0;
  bool semanticInput = false;
  bool legacyObjectFallback = false;
  bool dynamicDoorObstaclesAcknowledged = false;
  float bmin[3];
  float bmax[3];
};

static std::string lowerObjectName(const std::string &name) {
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return (char)std::tolower(c); });
  return lower;
}

static bool loadObj(const char *path, Mesh &out, bool legacyObjectFallback,
                    bool dynamicDoorObstacles) {
  printf("Loading %s ...\n", path);
  TimePoint t0 = Clock::now();

  FILE *f = fopen(path, "r");
  if (!f) {
    fprintf(stderr, "  Cannot open: %s\n", path);
    return false;
  }

  out.bmin[0] = out.bmin[1] = out.bmin[2] = 1e30f;
  out.bmax[0] = out.bmax[1] = out.bmax[2] = -1e30f;
  out.legacyObjectFallback = legacyObjectFallback;
  out.dynamicDoorObstaclesAcknowledged = dynamicDoorObstacles;

  char line[4096];
  std::string objectName;
  std::string materialName;
  NavSemantic currentSemantic = NavSemantic::Untagged;
  int currentObjectId = -1;
  float objectMin[3] = {1e30f, 1e30f, 1e30f};
  float objectMax[3] = {-1e30f, -1e30f, -1e30f};
  size_t objectTriStart = 0;
  int excludedHospitalDecorationObjects = 0;
  long long excludedHospitalDecorationTriangles = 0;
  int blockedStaticVehicleObjects = 0;
  long long blockedStaticVehicleTriangles = 0;
  float debugMinX = 0.0f, debugMinZ = 0.0f, debugMaxX = 0.0f,
        debugMaxZ = 0.0f;
  const char *debugBounds = getenv("H1EMU_NAV_DEBUG_OBJECT_BOUNDS");
  const bool debugObjects =
      debugBounds &&
      sscanf(debugBounds, "%f,%f,%f,%f", &debugMinX, &debugMinZ, &debugMaxX,
             &debugMaxZ) == 4;
  const auto finishObject = [&]() {
    if (objectName.empty() || objectMin[0] > objectMax[0])
      return;
    const std::string lower = lowerObjectName(objectName);
    const float spanX = objectMax[0] - objectMin[0];
    const float spanY = objectMax[1] - objectMin[1];
    const float spanZ = objectMax[2] - objectMin[2];
    // These thin meshes repeat across every hospital floor and are not
    // traversal surfaces. Rasterizing them as walkable creates hundreds of
    // artificial overlapping regions in a single tile column.
    const bool thinHospitalDecoration =
        lower.rfind("hospital_props_ceilinglight", 0) == 0 ||
        lower.rfind("hospital_props_paperdebris", 0) == 0;
    // Wrecked cars are authored world blockers, not traversal surfaces. Their
    // roofs are shallow enough to pass rcMarkWalkableTriangles and, with the
    // human climb profile, become connected islands that elevator NPCs onto
    // vehicles. Keep the full shell in the heightfield but force every face to
    // RC_NULL_AREA so it carves clearance without producing walkable polys.
    const bool staticVehicleObstacle =
        lower.rfind("common_props_wreckedcar", 0) == 0 ||
        lower.rfind("common_props_wreckedtruck", 0) == 0 ||
        lower.rfind("common_props_wreckedvan", 0) == 0;
    bool allFallbackTriangles = true;
    for (size_t tri = objectTriStart / 3; tri < out.triangleSemantics.size();
         ++tri) {
      const NavSemantic semantic = out.triangleSemantics[tri];
      if (semantic != NavSemantic::Untagged &&
          semantic != NavSemantic::OrdinaryMaterial) {
        allFallbackTriangles = false;
        break;
      }
    }
    // Historical object-name policy is intentionally opt-in and may only fill
    // untagged legacy geometry. It never overrides semantic source material.
    const bool applyLegacyObjectPolicy =
        legacyObjectFallback && allFallbackTriangles;
    const bool excluded = applyLegacyObjectPolicy && thinHospitalDecoration;
    if (excluded) {
      const size_t removed = (out.tris.size() - objectTriStart) / 3;
      out.tris.resize(objectTriStart);
      out.nonWalkableTris.resize(objectTriStart / 3);
      out.triangleSemantics.resize(objectTriStart / 3);
      out.triangleObjects.resize(objectTriStart / 3);
      excludedHospitalDecorationObjects++;
      excludedHospitalDecorationTriangles += (long long)removed;
    } else if (applyLegacyObjectPolicy && staticVehicleObstacle) {
      const size_t firstTriangle = objectTriStart / 3;
      const size_t triangleCount = out.tris.size() / 3 - firstTriangle;
      std::fill(out.nonWalkableTris.begin() + firstTriangle,
                out.nonWalkableTris.end(), static_cast<unsigned char>(1));
      blockedStaticVehicleObjects++;
      blockedStaticVehicleTriangles += (long long)triangleCount;
      NonWalkableVolume volume{};
      rcVcopy(volume.bmin, objectMin);
      rcVcopy(volume.bmax, objectMax);
      // Include the roof raster span and let normal navmesh erosion provide
      // the agent-radius clearance around the obstacle footprint.
      volume.bmax[1] += CELL_HEIGHT * 2.0f;
      out.nonWalkableVolumes.push_back(volume);
    }
    if (debugObjects && !excluded && objectMax[0] >= debugMinX &&
        objectMin[0] <= debugMaxX && objectMax[2] >= debugMinZ &&
        objectMin[2] <= debugMaxZ) {
      printf("[NAV-OBJECT] %s tris=%zu span=(%.2f,%.2f,%.2f) "
             "bounds=(%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f)\n",
             objectName.c_str(), (out.tris.size() - objectTriStart) / 3,
             spanX, spanY, spanZ, objectMin[0], objectMin[1], objectMin[2],
             objectMax[0], objectMax[1], objectMax[2]);
    }
  };
  while (fgets(line, sizeof(line), f)) {
    if (line[0] == 'o' && line[1] == ' ') {
      finishObject();
      ++currentObjectId;
      char *name = line + 2;
      name[strcspn(name, "\r\n")] = '\0';
      objectName = name;
      objectMin[0] = objectMin[1] = objectMin[2] = 1e30f;
      objectMax[0] = objectMax[1] = objectMax[2] = -1e30f;
      objectTriStart = out.tris.size();
    } else if (strncmp(line, "usemtl", 6) == 0 &&
               (line[6] == ' ' || line[6] == '\t')) {
      char *name = line + 7;
      while (*name == ' ' || *name == '\t')
        ++name;
      name[strcspn(name, "\r\n")] = '\0';
      materialName = name;
      currentSemantic = parseSemanticMaterial(materialName);
      if (currentSemantic == NavSemantic::Invalid)
        out.invalidSemanticMaterials.insert(materialName);
      else if (currentSemantic != NavSemantic::OrdinaryMaterial)
        out.semanticInput = true;
    } else if (line[0] == 'v' && line[1] == ' ') {
      float x, y, z;
      if (sscanf(line + 2, "%f %f %f", &x, &y, &z) == 3) {
        out.verts.push_back(x);
        out.verts.push_back(y);
        out.verts.push_back(z);
        out.bmin[0] = std::min(out.bmin[0], x);
        out.bmax[0] = std::max(out.bmax[0], x);
        out.bmin[1] = std::min(out.bmin[1], y);
        out.bmax[1] = std::max(out.bmax[1], y);
        out.bmin[2] = std::min(out.bmin[2], z);
        out.bmax[2] = std::max(out.bmax[2], z);
        objectMin[0] = std::min(objectMin[0], x);
        objectMax[0] = std::max(objectMax[0], x);
        objectMin[1] = std::min(objectMin[1], y);
        objectMax[1] = std::max(objectMax[1], y);
        objectMin[2] = std::min(objectMin[2], z);
        objectMax[2] = std::max(objectMax[2], z);
      }
    } else if (line[0] == 'f' && line[1] == ' ') {
      int idx[8];
      int n = 0;
      const char *p = line + 2;
      while (*p && n < 8) {
        while (*p == ' ' || *p == '\t')
          p++;
        if (!*p || *p == '\n' || *p == '\r')
          break;
        char *end;
        int vi = (int)strtol(p, &end, 10);
        if (end == p)
          break;
        int totalVerts = (int)(out.verts.size() / 3);
        idx[n++] = vi > 0 ? vi - 1 : totalVerts + vi;
        p = end;
        while (*p && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r')
          p++;
      }
      for (int i = 2; i < n; i++) {
        out.sourceTriangles++;
        const SemanticSpec *spec = semanticSpec(currentSemantic);
        if (spec)
          out.semanticHistogram[spec->material]++;
        else if (currentSemantic == NavSemantic::OrdinaryMaterial) {
          out.semanticHistogram["ordinary_material"]++;
          out.ordinaryMaterialTriangles++;
          out.fallbackTriangles++;
        } else {
          out.semanticHistogram["untagged"]++;
          out.fallbackTriangles++;
        }
        if (spec && spec->exclude) {
          out.excludedSemanticTriangles++;
          continue;
        }
        out.tris.push_back(idx[0]);
        out.tris.push_back(idx[i - 1]);
        out.tris.push_back(idx[i]);
        out.nonWalkableTris.push_back(0);
        out.triangleSemantics.push_back(currentSemantic);
        out.triangleObjects.push_back(currentObjectId);
      }
    }
  }
  finishObject();
  fclose(f);

  if (!out.invalidSemanticMaterials.empty()) {
    fprintf(stderr, "  Unknown semantic material(s):");
    for (const std::string &name : out.invalidSemanticMaterials)
      fprintf(stderr, " %s", name.c_str());
    fprintf(stderr, "\n");
    return false;
  }
  if (!legacyObjectFallback && !out.semanticInput) {
    fprintf(stderr,
            "  OBJ has no %s materials; use --legacy-object-fallback only "
            "for pre-semantic input\n",
            SEMANTIC_CONTRACT);
    return false;
  }
  if (!legacyObjectFallback && out.fallbackTriangles > 0) {
    fprintf(stderr,
            "  Semantic OBJ contains %lld untagged/ordinary-material "
            "triangle(s); every face must use a canonical nav_* material\n",
            out.fallbackTriangles);
    return false;
  }
  if (out.semanticHistogram.count("nav_door_panel_dynamic") &&
      !dynamicDoorObstacles) {
    fprintf(stderr,
            "  Semantic OBJ excludes nav_door_panel_dynamic geometry; pass "
            "--dynamic-door-obstacles to acknowledge that runtime door "
            "obstacles are required\n");
    return false;
  }

  if (out.verts.empty() || out.tris.empty()) {
    fprintf(stderr, "  No geometry found in %s\n", path);
    return false;
  }

  printf("  %d verts, %d tris  (%.2fs)\n", (int)(out.verts.size() / 3),
         (int)(out.tris.size() / 3), elapsed(t0));
  printf("  Semantics: contract=%s input=%s legacyFallback=%s source=%lld "
         "excluded=%lld kept=%zu\n",
         SEMANTIC_CONTRACT, out.semanticInput ? "true" : "false",
         legacyObjectFallback ? "true" : "false", out.sourceTriangles,
         out.excludedSemanticTriangles, out.triangleSemantics.size());
  for (const auto &[name, count] : out.semanticHistogram)
    printf("    %-24s %lld triangle(s)\n", name.c_str(), count);
  if (out.semanticHistogram.count("nav_unknown"))
    fprintf(stderr,
            "  [WARN] nav_unknown rasterizes non-walkable; classify these "
            "triangles before release\n");
  printf("  Excluded %d thin hospital decorations (%lld triangles)\n",
         excludedHospitalDecorationObjects,
         excludedHospitalDecorationTriangles);
  printf("  Marked %d static vehicle props non-walkable (%lld triangles)\n",
         blockedStaticVehicleObjects, blockedStaticVehicleTriangles);
  printf("  Bounds X [%.2f .. %.2f]  Y [%.2f .. %.2f]  Z [%.2f .. %.2f]\n",
         out.bmin[0], out.bmax[0], out.bmin[1], out.bmax[1], out.bmin[2],
         out.bmax[2]);
  return true;
}

static std::string jsonEscape(const std::string &value) {
  std::string out;
  for (unsigned char c : value) {
    switch (c) {
    case '\\': out += "\\\\"; break;
    case '"': out += "\\\""; break;
    case '\n': out += "\\n"; break;
    case '\r': out += "\\r"; break;
    case '\t': out += "\\t"; break;
    default:
      if (c < 0x20) {
        char escaped[7];
        snprintf(escaped, sizeof(escaped), "\\u%04x", c);
        out += escaped;
      } else {
        out += (char)c;
      }
    }
  }
  return out;
}

static std::string fnv1a64File(const char *path) {
  std::ifstream input(path, std::ios::binary);
  unsigned long long hash = 14695981039346656037ULL;
  char buffer[64 * 1024];
  while (input) {
    input.read(buffer, sizeof(buffer));
    const std::streamsize count = input.gcount();
    for (std::streamsize i = 0; i < count; ++i) {
      hash ^= (unsigned char)buffer[i];
      hash *= 1099511628211ULL;
    }
  }
  std::ostringstream result;
  result << std::hex << std::setw(16) << std::setfill('0') << hash;
  return result.str();
}

static bool writeSemanticReport(const char *inputPath, const std::string &path,
                                const Mesh &mesh) {
  const std::filesystem::path reportPath(path);
  if (!reportPath.parent_path().empty()) {
    std::error_code error;
    std::filesystem::create_directories(reportPath.parent_path(), error);
    if (error) {
      fprintf(stderr, "Cannot create semantic report directory: %s\n",
              error.message().c_str());
      return false;
    }
  }
  std::ofstream report(reportPath, std::ios::binary | std::ios::trunc);
  if (!report) {
    fprintf(stderr, "Cannot write semantic report %s\n", path.c_str());
    return false;
  }
  std::error_code sizeError;
  const auto inputBytes = std::filesystem::file_size(inputPath, sizeError);
  const std::string inputName =
      std::filesystem::path(inputPath).lexically_normal().filename().string();
  const std::string inputHash = fnv1a64File(inputPath);
  report << "{\n"
         << "  \"schemaVersion\": 1,\n"
         << "  \"semanticContract\": \"" << SEMANTIC_CONTRACT << "\",\n"
         << "  \"inputName\": \"" << jsonEscape(inputName) << "\",\n"
         << "  \"inputBytes\": " << (sizeError ? 0 : inputBytes) << ",\n"
         << "  \"inputIdentity\": \"fnv1a64:" << inputHash << "\",\n"
         << "  \"semanticInput\": " << (mesh.semanticInput ? "true" : "false")
         << ",\n"
         << "  \"legacyObjectFallback\": "
         << (mesh.legacyObjectFallback ? "true" : "false") << ",\n"
         << "  \"dynamicDoorObstaclesAcknowledged\": "
         << (mesh.dynamicDoorObstaclesAcknowledged ? "true" : "false")
         << ",\n"
         << "  \"sourceTriangles\": " << mesh.sourceTriangles << ",\n"
         << "  \"keptTriangles\": " << mesh.triangleSemantics.size() << ",\n"
         << "  \"excludedTriangles\": " << mesh.excludedSemanticTriangles
         << ",\n"
         << "  \"fallbackTriangles\": " << mesh.fallbackTriangles << ",\n"
         << "  \"ordinaryMaterialTriangles\": "
         << mesh.ordinaryMaterialTriangles << ",\n"
         << "  \"materials\": {\n";
  bool first = true;
  for (const auto &[name, count] : mesh.semanticHistogram) {
    if (!first)
      report << ",\n";
    first = false;
    report << "    \"" << jsonEscape(name) << "\": " << count;
  }
  report << "\n  },\n  \"taxonomy\": [\n";
  for (size_t i = 0; i < sizeof(SEMANTIC_SPECS) / sizeof(SEMANTIC_SPECS[0]);
       ++i) {
    const SemanticSpec &spec = SEMANTIC_SPECS[i];
    report << "    {\"material\": \"" << spec.material << "\", \"area\": "
           << (int)spec.area << ", \"flags\": " << spec.flags
           << ", \"rasterizeNull\": "
           << (spec.rasterizeNull ? "true" : "false")
           << ", \"exclude\": " << (spec.exclude ? "true" : "false") << "}";
    if (i + 1 != sizeof(SEMANTIC_SPECS) / sizeof(SEMANTIC_SPECS[0]))
      report << ',';
    report << '\n';
  }
  report << "  ],\n  \"warnings\": [";
  bool warning = false;
  if (mesh.semanticHistogram.count("nav_unknown")) {
    report << "\"nav_unknown triangles rasterize non-walkable\"";
    warning = true;
  }
  if (mesh.fallbackTriangles > 0) {
    if (warning)
      report << ", ";
    report << "\"legacy fallback triangles are not semantically classified\"";
  }
  report << "]\n}\n";
  printf("  Semantic provenance: %s\n", path.c_str());
  return true;
}

static bool validateSemanticContract(const Mesh &mesh, bool requireAll) {
  if (mesh.tris.size() / 3 != mesh.triangleSemantics.size() ||
      mesh.triangleSemantics.size() != mesh.nonWalkableTris.size()) {
    fprintf(stderr, "Semantic validation failed: triangle arrays are misaligned\n");
    return false;
  }
  for (const SemanticSpec &spec : SEMANTIC_SPECS) {
    if (spec.area != RC_NULL_AREA && flagsForArea(spec.area) != spec.flags) {
      fprintf(stderr,
              "Semantic validation failed: %s area=%u maps to flags=%u, "
              "expected=%u\n",
              spec.material, spec.area, flagsForArea(spec.area), spec.flags);
      return false;
    }
    if (requireAll && !mesh.semanticHistogram.count(spec.material)) {
      fprintf(stderr, "Semantic validation failed: fixture lacks %s\n",
              spec.material);
      return false;
    }
  }
  if (mesh.semanticInput && !mesh.legacyObjectFallback &&
      mesh.fallbackTriangles != 0) {
    fprintf(stderr,
            "Semantic validation failed: strict semantic input used fallback\n");
    return false;
  }
  printf("Semantic contract validation PASS (%s, %lld source triangles)\n",
         SEMANTIC_CONTRACT, mesh.sourceTriangles);
  return true;
}

struct RasterTriangles {
  std::vector<int> indices;
  std::vector<unsigned char> areas;
};

// This is the single semantic-to-Recast mapping used by both the direct
// navmesh and TileCache builders. Keeping slope marking and semantic overrides
// here prevents the two output paths from assigning different areas.
static RasterTriangles prepareRasterTriangles(rcContext *ctx, const Mesh &mesh,
                                               const std::vector<int> &triIds,
                                               float walkableSlopeAngle) {
  RasterTriangles raster;
  raster.indices.reserve(triIds.size() * 3);
  for (int triId : triIds) {
    raster.indices.push_back(mesh.tris[triId * 3 + 0]);
    raster.indices.push_back(mesh.tris[triId * 3 + 1]);
    raster.indices.push_back(mesh.tris[triId * 3 + 2]);
  }
  raster.areas.assign(triIds.size(), RC_NULL_AREA);
  rcMarkWalkableTriangles(ctx, walkableSlopeAngle, mesh.verts.data(),
                          (int)(mesh.verts.size() / 3), raster.indices.data(),
                          (int)triIds.size(), raster.areas.data());
  for (size_t i = 0; i < triIds.size(); ++i) {
    const int sourceTri = triIds[i];
    if (mesh.nonWalkableTris[sourceTri]) {
      raster.areas[i] = RC_NULL_AREA;
      continue;
    }
    const SemanticSpec *spec = semanticSpec(mesh.triangleSemantics[sourceTri]);
    if (spec && spec->rasterizeNull) {
      raster.areas[i] = RC_NULL_AREA;
      continue;
    }
    if (raster.areas[i] == RC_NULL_AREA)
      continue; // semantics never override the configured slope limit
    raster.areas[i] = spec ? spec->area : NAV_AREA_TERRAIN;
  }
  return raster;
}

static bool triangleOverlapsCellXZ(const float *a, const float *b,
                                   const float *c, float minX, float minZ,
                                   float maxX, float maxZ) {
  const float *vertices[3] = {a, b, c};
  const auto separatedOnAxis = [&](float axisX, float axisZ) {
    if (fabsf(axisX) + fabsf(axisZ) <= 1e-8f)
      return false;
    float triMin = 1e30f;
    float triMax = -1e30f;
    for (const float *vertex : vertices) {
      const float projection = vertex[0] * axisX + vertex[2] * axisZ;
      triMin = std::min(triMin, projection);
      triMax = std::max(triMax, projection);
    }
    const float boxCenterX = (minX + maxX) * 0.5f;
    const float boxCenterZ = (minZ + maxZ) * 0.5f;
    const float boxCenter = boxCenterX * axisX + boxCenterZ * axisZ;
    const float boxRadius =
        (maxX - minX) * 0.5f * fabsf(axisX) +
        (maxZ - minZ) * 0.5f * fabsf(axisZ);
    return triMax < boxCenter - boxRadius || triMin > boxCenter + boxRadius;
  };
  if (separatedOnAxis(1.0f, 0.0f) || separatedOnAxis(0.0f, 1.0f))
    return false;
  for (int edge = 0; edge < 3; ++edge) {
    const float *from = vertices[edge];
    const float *to = vertices[(edge + 1) % 3];
    if (separatedOnAxis(-(to[2] - from[2]), to[0] - from[0]))
      return false;
  }
  return true;
}

static bool trianglePlaneHeightXZ(const float *a, const float *b,
                                  const float *c, float x, float z,
                                  float &height) {
  const float denominator =
      (b[2] - c[2]) * (a[0] - c[0]) +
      (c[0] - b[0]) * (a[2] - c[2]);
  if (fabsf(denominator) <= 1e-8f)
    return false;
  const float wa = ((b[2] - c[2]) * (x - c[0]) +
                    (c[0] - b[0]) * (z - c[2])) /
                   denominator;
  const float wb = ((c[2] - a[2]) * (x - c[0]) +
                    (a[0] - c[0]) * (z - c[2])) /
                   denominator;
  height = wa * a[1] + wb * b[1] + (1.0f - wa - wb) * c[1];
  return true;
}

static void markTriangleSpans(const Mesh &mesh, int triId,
                              const rcConfig &cfg,
                              rcCompactHeightfield &chf,
                              std::vector<unsigned char> *protectedSpans,
                              bool collectProtection) {
  const float *a = &mesh.verts[mesh.tris[triId * 3 + 0] * 3];
  const float *b = &mesh.verts[mesh.tris[triId * 3 + 1] * 3];
  const float *c = &mesh.verts[mesh.tris[triId * 3 + 2] * 3];
  const float minX = std::min({a[0], b[0], c[0]});
  const float maxX = std::max({a[0], b[0], c[0]});
  const float minZ = std::min({a[2], b[2], c[2]});
  const float maxZ = std::max({a[2], b[2], c[2]});
  int firstX = (int)floorf((minX - chf.bmin[0]) / chf.cs);
  int lastX = (int)floorf((maxX - chf.bmin[0]) / chf.cs);
  int firstZ = (int)floorf((minZ - chf.bmin[2]) / chf.cs);
  int lastZ = (int)floorf((maxZ - chf.bmin[2]) / chf.cs);
  if (lastX < 0 || lastZ < 0 || firstX >= chf.width ||
      firstZ >= chf.height)
    return;
  firstX = std::max(0, firstX);
  lastX = std::min(chf.width - 1, lastX);
  firstZ = std::max(0, firstZ);
  lastZ = std::min(chf.height - 1, lastZ);

  const float quantizationTolerance =
      std::max(cfg.ch * 2.0f, cfg.ch * DETAIL_SAMPLE_MAX_ERR);
  for (int z = firstZ; z <= lastZ; ++z) {
    for (int x = firstX; x <= lastX; ++x) {
      const float sampleX = chf.bmin[0] + ((float)x + 0.5f) * chf.cs;
      const float sampleZ = chf.bmin[2] + ((float)z + 0.5f) * chf.cs;
      const float cellMinX = chf.bmin[0] + (float)x * chf.cs;
      const float cellMinZ = chf.bmin[2] + (float)z * chf.cs;
      if (!triangleOverlapsCellXZ(a, b, c, cellMinX, cellMinZ,
                                  cellMinX + chf.cs, cellMinZ + chf.cs))
        continue;
      float surfaceY = 0.0f;
      const bool hasPlaneHeight =
          trianglePlaneHeightXZ(a, b, c, sampleX, sampleZ, surfaceY);
      const float triangleMinY = std::min({a[1], b[1], c[1]});
      const float triangleMaxY = std::max({a[1], b[1], c[1]});
      const rcCompactCell &cell = chf.cells[x + z * chf.width];
      const int lastSpan = (int)(cell.index + cell.count);
      for (int spanIndex = (int)cell.index; spanIndex < lastSpan;
           ++spanIndex) {
        if (chf.areas[spanIndex] == RC_NULL_AREA)
          continue;
        const float spanY = chf.bmin[1] + chf.spans[spanIndex].y * chf.ch;
        const float clearance =
            hasPlaneHeight
                ? surfaceY - spanY
                : (triangleMinY >= spanY ? triangleMinY - spanY
                                         : (triangleMaxY >= spanY ? 0.0f
                                                                  : -1e30f));
        if (collectProtection) {
          if (fabsf(clearance) <= quantizationTolerance)
            (*protectedSpans)[spanIndex] = 1;
          continue;
        }
        // Preserve a more-specific walkable component on the same object when
        // the object's coarse obstacle surface is coplanar or underneath it.
        // An overhead surface still enforces normal headroom.
        if (protectedSpans && (*protectedSpans)[spanIndex] &&
            clearance <= quantizationTolerance)
          continue;
        // Only invalidate the same layer or a walkable span beneath this
        // obstacle without sufficient standing clearance. Distinct floors
        // above it are valid multi-storey topology.
        if (clearance >= -quantizationTolerance &&
            clearance <= AGENT_HEIGHT)
          chf.areas[spanIndex] = RC_NULL_AREA;
      }
    }
  }
}

static void applyNonWalkableCarves(rcContext *ctx, const Mesh &mesh,
                                   const std::vector<int> &triIds,
                                   const rcConfig &cfg,
                                   rcCompactHeightfield &chf) {
  for (const NonWalkableVolume &volume : mesh.nonWalkableVolumes) {
    if (volume.bmax[0] < cfg.bmin[0] || volume.bmin[0] > cfg.bmax[0] ||
        volume.bmax[2] < cfg.bmin[2] || volume.bmin[2] > cfg.bmax[2])
      continue;
    rcMarkBoxArea(ctx, volume.bmin, volume.bmax, RC_NULL_AREA, chf);
  }

  struct ObjectSemanticTriangles {
    std::vector<int> walkable;
    std::vector<int> obstacles;
  };
  std::map<int, ObjectSemanticTriangles> objects;
  for (int triId : triIds) {
    const SemanticSpec *spec = semanticSpec(mesh.triangleSemantics[triId]);
    if (!spec)
      continue;
    ObjectSemanticTriangles &object = objects[mesh.triangleObjects[triId]];
    if (spec->rasterizeNull)
      object.obstacles.push_back(triId);
    else if (spec->area != RC_NULL_AREA)
      object.walkable.push_back(triId);
  }
  for (const auto &[objectId, triangles] : objects) {
    (void)objectId;
    if (triangles.obstacles.empty())
      continue;
    std::vector<unsigned char> protectedSpans;
    std::vector<unsigned char> *protection = nullptr;
    if (!triangles.walkable.empty()) {
      protectedSpans.assign(chf.spanCount, 0);
      protection = &protectedSpans;
      for (int triId : triangles.walkable)
        markTriangleSpans(mesh, triId, cfg, chf, protection, true);
    }
    for (int triId : triangles.obstacles)
      markTriangleSpans(mesh, triId, cfg, chf, protection, false);
  }
}

static void debugCompactAreas(const char *stage,
                              const rcCompactHeightfield &chf) {
  if (!getenv("H1EMU_NAV_DEBUG_SEMANTIC_STAGES"))
    return;
  std::map<unsigned int, int> histogram;
  for (int i = 0; i < chf.spanCount; ++i)
    histogram[chf.areas[i]]++;
  printf("[SEMANTIC-STAGE] %s", stage);
  for (const auto &[area, count] : histogram)
    printf(" area%u=%d", area, count);
  printf("\n");
}

static bool validateSemanticRasterMapping(const Mesh &mesh) {
  std::vector<int> triIds(mesh.triangleSemantics.size());
  for (size_t i = 0; i < triIds.size(); ++i)
    triIds[i] = (int)i;
  PrintContext ctx;
  const RasterTriangles raster =
      prepareRasterTriangles(&ctx, mesh, triIds, AGENT_MAX_SLOPE);
  std::map<unsigned int, long long> areaHistogram;
  std::map<unsigned int, long long> flagHistogram;
  for (size_t i = 0; i < triIds.size(); ++i) {
    const unsigned char area = raster.areas[i];
    const unsigned short flags = flagsForArea(area);
    areaHistogram[area]++;
    flagHistogram[flags]++;
    const SemanticSpec *spec = semanticSpec(mesh.triangleSemantics[i]);
    if (mesh.nonWalkableTris[i] || (spec && spec->rasterizeNull)) {
      if (area != RC_NULL_AREA) {
        fprintf(stderr,
                "Semantic raster validation failed: triangle %zu must be "
                "non-walkable\n",
                i);
        return false;
      }
      continue;
    }
    const unsigned char expectedArea = spec ? spec->area : NAV_AREA_TERRAIN;
    if (area != expectedArea || flags != flagsForArea(expectedArea)) {
      fprintf(stderr,
              "Semantic raster validation failed: triangle %zu area=%u "
              "flags=%u expected area=%u flags=%u\n",
              i, area, flags, expectedArea, flagsForArea(expectedArea));
      return false;
    }
  }
  printf("Semantic raster areas:");
  for (const auto &[area, count] : areaHistogram)
    printf(" %u=%lld", area, count);
  printf("\nSemantic polygon flags:");
  for (const auto &[flags, count] : flagHistogram)
    printf(" 0x%02x=%lld", flags, count);
  printf("\nSemantic raster validation PASS (shared direct/tile-cache mapping)\n");
  return true;
}

static unsigned char *buildTile(rcContext *ctx, const Mesh &mesh,
                                const TriGrid &grid, const float *tileMin,
                                const float *tileMax, int tileX, int tileY,
                                int &outDataSize) {
  rcConfig cfg{};
  cfg.cs = CELL_SIZE;
  cfg.ch = CELL_HEIGHT;
  cfg.walkableSlopeAngle = AGENT_MAX_SLOPE;
  cfg.walkableHeight = (int)ceilf(AGENT_HEIGHT / cfg.ch);
  cfg.walkableClimb = (int)floorf(AGENT_MAX_CLIMB / cfg.ch);
  cfg.walkableRadius = (int)ceilf(AGENT_RADIUS / cfg.cs);
  cfg.maxEdgeLen = (int)(EDGE_MAX_LEN / cfg.cs);
  cfg.maxSimplificationError = EDGE_MAX_ERROR;
  // Canonical semantic components are intentional authored topology. Global
  // small-region pruning can otherwise delete short thresholds and individual
  // stair treads after they survived voxelization. Legacy geometry still uses
  // the configured noise filter; strict semantic input keeps every classified
  // region and relies on the classifier to reject decoration.
  cfg.minRegionArea = mesh.semanticInput ? 0 : (int)rcSqr(REGION_MIN_SIZE);
  cfg.mergeRegionArea = (int)rcSqr(REGION_MERGE_SIZE);
  cfg.maxVertsPerPoly = VERTS_PER_POLY;
  cfg.tileSize = TILE_SIZE;
  cfg.borderSize = cfg.walkableRadius + 3;
  cfg.width = cfg.tileSize + cfg.borderSize * 2;
  cfg.height = cfg.tileSize + cfg.borderSize * 2;
  cfg.detailSampleDist =
      DETAIL_SAMPLE_DIST < 0.9f ? 0.0f : cfg.cs * DETAIL_SAMPLE_DIST;
  cfg.detailSampleMaxError = cfg.ch * DETAIL_SAMPLE_MAX_ERR;

  rcVcopy(cfg.bmin, tileMin);
  rcVcopy(cfg.bmax, tileMax);
  cfg.bmin[0] -= (float)cfg.borderSize * cfg.cs;
  cfg.bmin[2] -= (float)cfg.borderSize * cfg.cs;
  cfg.bmax[0] += (float)cfg.borderSize * cfg.cs;
  cfg.bmax[2] += (float)cfg.borderSize * cfg.cs;

  rcHeightfield *hf = rcAllocHeightfield();
  if (!hf || !rcCreateHeightfield(ctx, *hf, cfg.width, cfg.height, cfg.bmin,
                                  cfg.bmax, cfg.cs, cfg.ch)) {
    rcFreeHeightField(hf);
    return nullptr;
  }

  // Query only triangles overlapping this tile
  float qmin[2] = {cfg.bmin[0], cfg.bmin[2]};
  float qmax[2] = {cfg.bmax[0], cfg.bmax[2]};
  const std::vector<int> triIds = grid.query(qmin, qmax);
  if (triIds.empty()) {
    rcFreeHeightField(hf);
    return nullptr;
  }

  const float *verts = mesh.verts.data();
  const int nVerts = (int)(mesh.verts.size() / 3);

  const int nTris = (int)triIds.size();
  const RasterTriangles raster =
      prepareRasterTriangles(ctx, mesh, triIds, cfg.walkableSlopeAngle);
  rcRasterizeTriangles(ctx, verts, nVerts, raster.indices.data(),
                       raster.areas.data(), nTris, *hf, cfg.walkableClimb);

  // The low-hanging filter deliberately copies a neighboring walkable area
  // onto a shallow RC_NULL_AREA span. That heuristic is useful for legacy,
  // unclassified collision, but it violates the semantic contract by turning
  // an explicitly tagged obstacle back into a traversable surface. Strict
  // semantic input already identifies stairs, ramps, and thresholds, so keep
  // its null spans authoritative and let ordinary climb connectivity handle
  // those tagged traversal surfaces.
  if (!mesh.semanticInput)
    rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *hf);
  rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
  rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *hf);

  rcCompactHeightfield *chf = rcAllocCompactHeightfield();
  if (!chf || !rcBuildCompactHeightfield(ctx, cfg.walkableHeight,
                                         cfg.walkableClimb, *hf, *chf)) {
    rcFreeHeightField(hf);
    rcFreeCompactHeightfield(chf);
    return nullptr;
  }
  rcFreeHeightField(hf);

  debugCompactAreas("direct-before-carves", *chf);
  applyNonWalkableCarves(ctx, mesh, triIds, cfg, *chf);
  debugCompactAreas("direct-after-carves", *chf);

  if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *chf)) {
    rcFreeCompactHeightfield(chf);
    return nullptr;
  }
  debugCompactAreas("direct-after-erosion", *chf);

  // Layer partitioning is designed for tiled, multi-storey worlds and cannot
  // produce the overlapping regions that make watershed fail in dense POIs.
  if (!rcBuildLayerRegions(ctx, *chf, cfg.borderSize, cfg.minRegionArea)) {
    rcFreeCompactHeightfield(chf);
    return nullptr;
  }

  rcContourSet *cset = rcAllocContourSet();
  if (!cset || !rcBuildContours(ctx, *chf, cfg.maxSimplificationError,
                                cfg.maxEdgeLen, *cset)) {
    rcFreeCompactHeightfield(chf);
    rcFreeContourSet(cset);
    return nullptr;
  }
  if (cset->nconts == 0) {
    rcFreeCompactHeightfield(chf);
    rcFreeContourSet(cset);
    return nullptr;
  }

  rcPolyMesh *pmesh = rcAllocPolyMesh();
  if (!pmesh || !rcBuildPolyMesh(ctx, *cset, cfg.maxVertsPerPoly, *pmesh)) {
    rcFreeCompactHeightfield(chf);
    rcFreeContourSet(cset);
    rcFreePolyMesh(pmesh);
    return nullptr;
  }

  rcPolyMeshDetail *dmesh = rcAllocPolyMeshDetail();
  if (!dmesh || !rcBuildPolyMeshDetail(ctx, *pmesh, *chf, cfg.detailSampleDist,
                                       cfg.detailSampleMaxError, *dmesh)) {
    rcFreeCompactHeightfield(chf);
    rcFreeContourSet(cset);
    rcFreePolyMesh(pmesh);
    rcFreePolyMeshDetail(dmesh);
    return nullptr;
  }

  rcFreeCompactHeightfield(chf);
  rcFreeContourSet(cset);

  if (pmesh->nverts >= 0xffff) {
    fprintf(stderr, "\n[WARN]  Tile (%d,%d): too many verts (%d), skipping\n",
            tileX, tileY, pmesh->nverts);
    rcFreePolyMesh(pmesh);
    rcFreePolyMeshDetail(dmesh);
    return nullptr;
  }

  for (int i = 0; i < pmesh->npolys; ++i)
    pmesh->flags[i] = flagsForArea(pmesh->areas[i]);

  dtNavMeshCreateParams params{};
  params.verts = pmesh->verts;
  params.vertCount = pmesh->nverts;
  params.polys = pmesh->polys;
  params.polyAreas = pmesh->areas;
  params.polyFlags = pmesh->flags;
  params.polyCount = pmesh->npolys;
  params.nvp = pmesh->nvp;
  params.detailMeshes = dmesh->meshes;
  params.detailVerts = dmesh->verts;
  params.detailVertsCount = dmesh->nverts;
  params.detailTris = dmesh->tris;
  params.detailTriCount = dmesh->ntris;
  params.walkableHeight = AGENT_HEIGHT;
  params.walkableRadius = AGENT_RADIUS;
  params.walkableClimb = AGENT_MAX_CLIMB;
  params.tileX = tileX;
  params.tileY = tileY;
  params.tileLayer = 0;
  rcVcopy(params.bmin, pmesh->bmin);
  rcVcopy(params.bmax, pmesh->bmax);
  params.cs = cfg.cs;
  params.ch = cfg.ch;
  params.buildBvTree = true;

  unsigned char *navData = nullptr;
  int navDataSize = 0;
  if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) {
    rcFreePolyMesh(pmesh);
    rcFreePolyMeshDetail(dmesh);
    return nullptr;
  }

  rcFreePolyMesh(pmesh);
  rcFreePolyMeshDetail(dmesh);

  outDataSize = navDataSize;
  return navData;
}

static std::vector<std::pair<unsigned char *, int>>
buildTileCacheLayers(rcContext *ctx, const Mesh &mesh, const TriGrid &grid,
                     const float *tileMin, const float *tileMax, int tileX,
                     int tileY) {
  std::vector<std::pair<unsigned char *, int>> result;

  rcConfig cfg{};
  cfg.cs = CELL_SIZE;
  cfg.ch = CELL_HEIGHT;
  cfg.walkableSlopeAngle = AGENT_MAX_SLOPE;
  cfg.walkableHeight = (int)ceilf(AGENT_HEIGHT / cfg.ch);
  cfg.walkableClimb = (int)floorf(AGENT_MAX_CLIMB / cfg.ch);
  cfg.walkableRadius = (int)ceilf(AGENT_RADIUS / cfg.cs);
  cfg.maxEdgeLen = (int)(EDGE_MAX_LEN / cfg.cs);
  cfg.maxSimplificationError = EDGE_MAX_ERROR;
  // Keep the direct and TileCache builders on the same semantic-region policy.
  cfg.minRegionArea = mesh.semanticInput ? 0 : (int)rcSqr(REGION_MIN_SIZE);
  cfg.mergeRegionArea = (int)rcSqr(REGION_MERGE_SIZE);
  cfg.maxVertsPerPoly = VERTS_PER_POLY;
  cfg.tileSize = TILE_SIZE;
  cfg.borderSize = cfg.walkableRadius + 3;
  cfg.width = cfg.tileSize + cfg.borderSize * 2;
  cfg.height = cfg.tileSize + cfg.borderSize * 2;
  cfg.detailSampleDist =
      DETAIL_SAMPLE_DIST < 0.9f ? 0.0f : cfg.cs * DETAIL_SAMPLE_DIST;
  cfg.detailSampleMaxError = cfg.ch * DETAIL_SAMPLE_MAX_ERR;

  rcVcopy(cfg.bmin, tileMin);
  rcVcopy(cfg.bmax, tileMax);
  cfg.bmin[0] -= (float)cfg.borderSize * cfg.cs;
  cfg.bmin[2] -= (float)cfg.borderSize * cfg.cs;
  cfg.bmax[0] += (float)cfg.borderSize * cfg.cs;
  cfg.bmax[2] += (float)cfg.borderSize * cfg.cs;

  rcHeightfield *hf = rcAllocHeightfield();
  if (!hf || !rcCreateHeightfield(ctx, *hf, cfg.width, cfg.height, cfg.bmin,
                                  cfg.bmax, cfg.cs, cfg.ch)) {
    rcFreeHeightField(hf);
    return result;
  }

  float qmin[2] = {cfg.bmin[0], cfg.bmin[2]};
  float qmax[2] = {cfg.bmax[0], cfg.bmax[2]};
  const std::vector<int> triIds = grid.query(qmin, qmax);
  if (triIds.empty()) {
    rcFreeHeightField(hf);
    return result;
  }

  const float *verts = mesh.verts.data();
  const int nVerts = (int)(mesh.verts.size() / 3);

  const int nTris = (int)triIds.size();
  const RasterTriangles raster =
      prepareRasterTriangles(ctx, mesh, triIds, cfg.walkableSlopeAngle);
  rcRasterizeTriangles(ctx, verts, nVerts, raster.indices.data(),
                       raster.areas.data(), nTris, *hf, cfg.walkableClimb);

  // Match the direct builder: semantic null spans are authoritative. Applying
  // the legacy low-obstacle promotion here would also make direct and
  // TileCache collision behavior diverge after materialization.
  if (!mesh.semanticInput)
    rcFilterLowHangingWalkableObstacles(ctx, cfg.walkableClimb, *hf);
  rcFilterLedgeSpans(ctx, cfg.walkableHeight, cfg.walkableClimb, *hf);
  rcFilterWalkableLowHeightSpans(ctx, cfg.walkableHeight, *hf);

  rcCompactHeightfield *chf = rcAllocCompactHeightfield();
  if (!chf || !rcBuildCompactHeightfield(ctx, cfg.walkableHeight,
                                         cfg.walkableClimb, *hf, *chf)) {
    rcFreeHeightField(hf);
    rcFreeCompactHeightfield(chf);
    return result;
  }
  rcFreeHeightField(hf);

  debugCompactAreas("tilecache-before-carves", *chf);
  applyNonWalkableCarves(ctx, mesh, triIds, cfg, *chf);
  debugCompactAreas("tilecache-after-carves", *chf);

  if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *chf)) {
    rcFreeCompactHeightfield(chf);
    return result;
  }
  debugCompactAreas("tilecache-after-erosion", *chf);

  rcHeightfieldLayerSet *lset = rcAllocHeightfieldLayerSet();
  if (!lset || !rcBuildHeightfieldLayers(ctx, *chf, cfg.borderSize,
                                         cfg.walkableHeight, *lset)) {
    rcFreeCompactHeightfield(chf);
    rcFreeHeightfieldLayerSet(lset);
    return result;
  }
  rcFreeCompactHeightfield(chf);

  FastLZCompressor comp;
  for (int i = 0; i < lset->nlayers; ++i) {
    const rcHeightfieldLayer *layer = &lset->layers[i];

    dtTileCacheLayerHeader header{};
    header.magic = DT_TILECACHE_MAGIC;
    header.version = DT_TILECACHE_VERSION;
    header.tx = tileX;
    header.ty = tileY;
    header.tlayer = i;
    dtVcopy(header.bmin, layer->bmin);
    dtVcopy(header.bmax, layer->bmax);
    header.width = (unsigned char)layer->width;
    header.height = (unsigned char)layer->height;
    header.minx = (unsigned char)layer->minx;
    header.maxx = (unsigned char)layer->maxx;
    header.miny = (unsigned char)layer->miny;
    header.maxy = (unsigned char)layer->maxy;
    header.hmin = (unsigned short)layer->hmin;
    header.hmax = (unsigned short)layer->hmax;

    unsigned char *data = nullptr;
    int dataSize = 0;
    if (dtStatusSucceed(dtBuildTileCacheLayer(&comp, &header, layer->heights,
                                              layer->areas, layer->cons, &data,
                                              &dataSize))) {
      result.push_back({data, dataSize});
    }
  }

  rcFreeHeightfieldLayerSet(lset);
  return result;
}

struct ObstacleProbe {
  float x, y, z;
};

struct PointXZ {
  float x, z;
};

static std::vector<PointXZ>
clipPolygonToAxis(const std::vector<PointXZ> &input, int axis, float boundary,
                  bool keepGreater) {
  std::vector<PointXZ> output;
  if (input.empty())
    return output;
  const auto coordinate = [axis](const PointXZ &point) {
    return axis == 0 ? point.x : point.z;
  };
  const auto inside = [&](const PointXZ &point) {
    return keepGreater ? coordinate(point) >= boundary
                       : coordinate(point) <= boundary;
  };
  PointXZ previous = input.back();
  bool previousInside = inside(previous);
  for (const PointXZ &current : input) {
    const bool currentInside = inside(current);
    if (previousInside != currentInside) {
      const float previousCoordinate = coordinate(previous);
      const float delta = coordinate(current) - previousCoordinate;
      if (fabsf(delta) > 1e-8f) {
        const float t = (boundary - previousCoordinate) / delta;
        output.push_back({previous.x + (current.x - previous.x) * t,
                          previous.z + (current.z - previous.z) * t});
      }
    }
    if (currentInside)
      output.push_back(current);
    previous = current;
    previousInside = currentInside;
  }
  return output;
}

static bool triangleOverlapsBoundsXZ(const float *a, const float *b,
                                     const float *c,
                                     const BuildBounds &bounds) {
  if (!bounds.enabled)
    return true;
  std::vector<PointXZ> clipped = {
      {a[0], a[2]}, {b[0], b[2]}, {c[0], c[2]}};
  clipped = clipPolygonToAxis(clipped, 0, bounds.minX, true);
  clipped = clipPolygonToAxis(clipped, 0, bounds.maxX, false);
  clipped = clipPolygonToAxis(clipped, 1, bounds.minZ, true);
  clipped = clipPolygonToAxis(clipped, 1, bounds.maxZ, false);
  if (clipped.size() < 3)
    return false;
  double twiceArea = 0.0;
  for (size_t i = 0; i < clipped.size(); ++i) {
    const PointXZ &from = clipped[i];
    const PointXZ &to = clipped[(i + 1) % clipped.size()];
    twiceArea += (double)from.x * to.z - (double)to.x * from.z;
  }
  // Boundary-only contact does not put rasterizable source area inside the
  // requested region and therefore cannot create a retained polygon there.
  return fabs(twiceArea) > 1e-8;
}

static bool triangleCouldBeWalkable(const float *a, const float *b,
                                    const float *c) {
  const float ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  const float ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
  const float normal[3] = {ab[1] * ac[2] - ab[2] * ac[1],
                           ab[2] * ac[0] - ab[0] * ac[2],
                           ab[0] * ac[1] - ab[1] * ac[0]};
  const float length = sqrtf(rcSqr(normal[0]) + rcSqr(normal[1]) +
                             rcSqr(normal[2]));
  if (length <= 1e-8f)
    return false;
  constexpr float PI = 3.14159265358979323846f;
  const float walkableThreshold = cosf(AGENT_MAX_SLOPE / 180.0f * PI);
  return normal[1] / length > walkableThreshold;
}

static std::map<unsigned int, long long>
collectRequiredSemanticAreas(const Mesh &mesh, const BuildBounds &bounds) {
  std::map<unsigned int, long long> required;
  for (size_t tri = 0; tri < mesh.triangleSemantics.size(); ++tri) {
    if (mesh.nonWalkableTris[tri])
      continue;
    const SemanticSpec *spec = semanticSpec(mesh.triangleSemantics[tri]);
    if (!spec || spec->area == RC_NULL_AREA)
      continue;
    const float *a = &mesh.verts[mesh.tris[tri * 3 + 0] * 3];
    const float *b = &mesh.verts[mesh.tris[tri * 3 + 1] * 3];
    const float *c = &mesh.verts[mesh.tris[tri * 3 + 2] * 3];
    if (!triangleCouldBeWalkable(a, b, c) ||
        !triangleOverlapsBoundsXZ(a, b, c, bounds))
      continue;
    required[spec->area]++;
  }
  return required;
}

static std::vector<ObstacleProbe> collectObstacleProbes(const Mesh &mesh) {
  std::vector<ObstacleProbe> probes;
  std::map<int, std::vector<int>> walkableByObject;
  for (size_t tri = 0; tri < mesh.triangleSemantics.size(); ++tri) {
    const SemanticSpec *spec = semanticSpec(mesh.triangleSemantics[tri]);
    if (spec && spec->area != RC_NULL_AREA)
      walkableByObject[mesh.triangleObjects[tri]].push_back((int)tri);
  }
  for (size_t tri = 0; tri < mesh.triangleSemantics.size(); ++tri) {
    if (mesh.triangleSemantics[tri] != NavSemantic::ObstacleStatic)
      continue;
    const float *a = &mesh.verts[mesh.tris[tri * 3 + 0] * 3];
    const float *b = &mesh.verts[mesh.tris[tri * 3 + 1] * 3];
    const float *c = &mesh.verts[mesh.tris[tri * 3 + 2] * 3];
    const float ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const float ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    const float normalY = ab[2] * ac[0] - ab[0] * ac[2];
    const float normalLength =
        sqrtf(rcSqr(ab[1] * ac[2] - ab[2] * ac[1]) + rcSqr(normalY) +
              rcSqr(ab[0] * ac[1] - ab[1] * ac[0]));
    if (normalLength <= 1e-5f || fabsf(normalY) / normalLength < 0.75f)
      continue; // probes target horizontal blocker footprint faces
    // Sub-cell paper scraps, trim, and similar decoration cannot own a stable
    // voxel center and therefore cannot be required to produce an individual
    // navigation hole. Larger props remain verified, and their final
    // agent-radius clearance is still provided by erosion.
    const float projectedArea = fabsf(normalY) * 0.5f;
    if (projectedArea < CELL_SIZE * CELL_SIZE * 0.25f)
      continue;
    const ObstacleProbe probe{(a[0] + b[0] + c[0]) / 3.0f,
                              (a[1] + b[1] + c[1]) / 3.0f,
                              (a[2] + b[2] + c[2]) / 3.0f};
    // Verification follows the same cell-level precedence as carving. A
    // component-specific walkable face on this OBJ object may intentionally
    // replace its coarse default obstacle face. Quantize to the raster cell
    // center because sub-cell conflicts cannot exist as separate nav spans.
    const float sampleX =
        mesh.bmin[0] +
        (floorf((probe.x - mesh.bmin[0]) / CELL_SIZE) + 0.5f) * CELL_SIZE;
    const float sampleZ =
        mesh.bmin[2] +
        (floorf((probe.z - mesh.bmin[2]) / CELL_SIZE) + 0.5f) * CELL_SIZE;
    bool overriddenBySpecificWalkable = false;
    const auto walkable = walkableByObject.find(mesh.triangleObjects[tri]);
    if (walkable != walkableByObject.end()) {
      const float tolerance =
          std::max(CELL_HEIGHT * 2.0f,
                   CELL_HEIGHT * DETAIL_SAMPLE_MAX_ERR);
      for (int walkableTri : walkable->second) {
        const float *wa =
            &mesh.verts[mesh.tris[walkableTri * 3 + 0] * 3];
        const float *wb =
            &mesh.verts[mesh.tris[walkableTri * 3 + 1] * 3];
        const float *wc =
            &mesh.verts[mesh.tris[walkableTri * 3 + 2] * 3];
        float walkableY = 0.0f;
        if (triangleOverlapsCellXZ(wa, wb, wc, sampleX - CELL_SIZE * 0.5f,
                                   sampleZ - CELL_SIZE * 0.5f,
                                   sampleX + CELL_SIZE * 0.5f,
                                   sampleZ + CELL_SIZE * 0.5f) &&
            trianglePlaneHeightXZ(wa, wb, wc, sampleX, sampleZ, walkableY) &&
            fabsf(walkableY - probe.y) <= tolerance) {
          overriddenBySpecificWalkable = true;
          break;
        }
      }
    }
    if (!overriddenBySpecificWalkable)
      probes.push_back(probe);
  }
  return probes;
}

static bool pointInPolyXZ(const float x, const float z, const dtMeshTile &tile,
                          const dtPoly &poly) {
  bool hasPositive = false;
  bool hasNegative = false;
  for (int edge = 0; edge < poly.vertCount; ++edge) {
    const float *a = &tile.verts[poly.verts[edge] * 3];
    const float *b = &tile.verts[poly.verts[(edge + 1) % poly.vertCount] * 3];
    const float cross = (b[0] - a[0]) * (z - a[2]) -
                        (b[2] - a[2]) * (x - a[0]);
    hasPositive = hasPositive || cross > 1e-4f;
    hasNegative = hasNegative || cross < -1e-4f;
    if (hasPositive && hasNegative)
      return false;
  }
  return true;
}

static bool inspectBakedSemantics(const char *label, const dtNavMesh &navMesh,
                                  const Mesh &mesh,
                                  const std::map<unsigned int, long long>
                                      &requiredSemanticAreas) {
  std::map<unsigned int, long long> areas;
  long long polygons = 0;
  for (int tileIndex = 0; tileIndex < navMesh.getMaxTiles(); ++tileIndex) {
    const dtMeshTile *tile = navMesh.getTile(tileIndex);
    if (!tile || !tile->header)
      continue;
    for (int polyIndex = 0; polyIndex < tile->header->polyCount; ++polyIndex) {
      const dtPoly &poly = tile->polys[polyIndex];
      if (poly.getType() != DT_POLYTYPE_GROUND)
        continue;
      const unsigned char area = poly.getArea();
      const unsigned short expectedFlags = flagsForArea(area);
      if (expectedFlags == 0 || poly.flags != expectedFlags) {
        fprintf(stderr,
                "%s semantic inspection failed: unexpected area=%u flags=%u "
                "expected=%u\n",
                label, area, poly.flags, expectedFlags);
        return false;
      }
      areas[area]++;
      polygons++;
    }
  }
  for (const auto &[area, sourceTriangles] : requiredSemanticAreas) {
    if (!areas.count(area)) {
      const SemanticSpec *requiredSpec = nullptr;
      for (const SemanticSpec &spec : SEMANTIC_SPECS)
        if (spec.area == area) {
          requiredSpec = &spec;
          break;
        }
      fprintf(stderr,
              "%s semantic inspection failed: no polygon retained area %u "
              "(%s, %lld in-bounds walkable source triangle(s))\n",
              label, area,
              requiredSpec ? requiredSpec->material : "unknown-semantic-area",
              sourceTriangles);
      return false;
    }
  }

  const std::vector<ObstacleProbe> probes = collectObstacleProbes(mesh);
  if (mesh.semanticHistogram.count("nav_obstacle_static") && probes.empty()) {
    fprintf(stderr,
            "%s semantic inspection failed: obstacle fixture has no horizontal "
            "footprint probes\n",
            label);
    return false;
  }
  for (const ObstacleProbe &probe : probes) {
    for (int tileIndex = 0; tileIndex < navMesh.getMaxTiles(); ++tileIndex) {
      const dtMeshTile *tile = navMesh.getTile(tileIndex);
      if (!tile || !tile->header)
        continue;
      for (int polyIndex = 0; polyIndex < tile->header->polyCount;
           ++polyIndex) {
        const dtPoly &poly = tile->polys[polyIndex];
        if (poly.getType() != DT_POLYTYPE_GROUND ||
            !pointInPolyXZ(probe.x, probe.z, *tile, poly))
          continue;
        float averageY = 0.0f;
        for (int vertex = 0; vertex < poly.vertCount; ++vertex)
          averageY += tile->verts[poly.verts[vertex] * 3 + 1];
        averageY /= poly.vertCount;
        // An obstacle surface invalidates a walkable span at the same height,
        // or below it without enough agent headroom. A polygon on a distinct
        // floor above the obstacle is valid multi-storey topology and must not
        // be rejected merely because the two layers are less than one agent
        // height apart. Allow only raster/detail quantization above the probe.
        const float verticalQuantizationTolerance =
            std::max(CELL_HEIGHT * 2.0f,
                     CELL_HEIGHT * DETAIL_SAMPLE_MAX_ERR);
        const float clearanceAboveWalkable = probe.y - averageY;
        if (clearanceAboveWalkable >= -verticalQuantizationTolerance &&
            clearanceAboveWalkable <= AGENT_HEIGHT) {
          fprintf(stderr,
                  "%s semantic inspection failed: walkable area %u covers "
                  "nav_obstacle_static probe (%.2f, %.2f, %.2f; "
                  "polygonY=%.2f clearance=%.2f)\n",
                  label, poly.getArea(), probe.x, probe.y, probe.z, averageY,
                  clearanceAboveWalkable);
          return false;
        }
      }
    }
  }

  printf("%s semantic polygons: total=%lld", label, polygons);
  for (const auto &[area, count] : areas)
    printf(" area%u=%lld", area, count);
  printf(" obstacleProbes=%zu PASS\n", probes.size());
  return true;
}

static void printUsage(const char *program) {
  fprintf(stderr,
          "Usage: %s <input.obj> [output.bin] [options]\n"
          "\n"
          "Options:\n"
          "  --profile legacy|human      Select legacy or fine human defaults\n"
          "  --cell-size <meters>        Horizontal voxel size\n"
          "  --cell-height <meters>      Vertical voxel size\n"
          "  --agent-height <meters>     Required headroom\n"
          "  --agent-radius <meters>     Clearance from obstacles\n"
          "  --agent-climb <meters>      Maximum step height\n"
          "  --agent-slope <degrees>     Maximum walkable slope\n"
          "  --tile-size <voxels>        Power of two from 16 through 128\n"
          "  --region-min <voxels>       Minimum region size\n"
          "  --region-merge <voxels>     Region merge size\n"
          "  --bounds <minX> <minZ> <maxX> <maxZ>\n"
          "                              Build only intersecting global tiles\n"
          "  --legacy-object-fallback   Enable pre-semantic object-name rules\n"
          "  --dynamic-door-obstacles  Acknowledge required runtime door blockers\n"
          "  --semantic-report <path>  Write deterministic semantic provenance\n"
          "  --validate-semantics-only Parse/report semantics without baking\n"
          "  --verify-baked-semantics  Inspect direct and TileCache polygons\n"
          "  --require-all-semantics    Require every canonical tag (fixtures)\n"
          "\n"
          "The human profile uses cs=.2, ch=.1, radius=.2, climb=1.3,\n"
          "slope=45, tile=128, region-min=8, and region-merge=20.\n",
          program);
}

static bool parseFloat(const char *text, float &value) {
  char *end = nullptr;
  value = strtof(text, &end);
  return end != text && *end == '\0' && std::isfinite(value);
}

static bool applyProfile(const char *name) {
  if (strcmp(name, "legacy") == 0) {
    CELL_SIZE = 1.2f;
    CELL_HEIGHT = 0.1f;
    AGENT_HEIGHT = 2.0f;
    AGENT_RADIUS = 0.5f;
    AGENT_MAX_CLIMB = 1.3f;
    AGENT_MAX_SLOPE = 60.0f;
    REGION_MIN_SIZE = 16.0f;
    REGION_MERGE_SIZE = 3.0f;
    TILE_SIZE = 64;
    return true;
  }
  if (strcmp(name, "human") == 0) {
    CELL_SIZE = 0.2f;
    CELL_HEIGHT = 0.1f;
    AGENT_HEIGHT = 2.0f;
    AGENT_RADIUS = 0.2f;
    AGENT_MAX_CLIMB = 1.3f;
    AGENT_MAX_SLOPE = 45.0f;
    REGION_MIN_SIZE = 8.0f;
    REGION_MERGE_SIZE = 20.0f;
    TILE_SIZE = 128;
    return true;
  }
  fprintf(stderr, "Unknown profile: %s\n", name);
  return false;
}

static bool parseOptions(int argc, char *argv[], int start,
                         BuildOptions &options) {
  for (int i = start; i < argc; ++i) {
    if (strcmp(argv[i], "--profile") == 0) {
      if (++i >= argc || !applyProfile(argv[i]))
        return false;
    }
  }

  for (int i = start; i < argc; ++i) {
    const char *arg = argv[i];
    if (strcmp(arg, "--profile") == 0) {
      ++i;
      continue;
    }
    if (strcmp(arg, "--help") == 0) {
      printUsage(argv[0]);
      return false;
    }
    if (strcmp(arg, "--bounds") == 0) {
      if (i + 4 >= argc || !parseFloat(argv[i + 1], options.bounds.minX) ||
          !parseFloat(argv[i + 2], options.bounds.minZ) ||
          !parseFloat(argv[i + 3], options.bounds.maxX) ||
          !parseFloat(argv[i + 4], options.bounds.maxZ)) {
        fprintf(stderr, "Invalid --bounds values\n");
        return false;
      }
      options.bounds.enabled = true;
      i += 4;
      continue;
    }
    if (strcmp(arg, "--legacy-object-fallback") == 0) {
      options.legacyObjectFallback = true;
      continue;
    }
    if (strcmp(arg, "--dynamic-door-obstacles") == 0) {
      options.dynamicDoorObstacles = true;
      continue;
    }
    if (strcmp(arg, "--validate-semantics-only") == 0) {
      options.validateSemanticsOnly = true;
      continue;
    }
    if (strcmp(arg, "--verify-baked-semantics") == 0) {
      options.verifyBakedSemantics = true;
      continue;
    }
    if (strcmp(arg, "--require-all-semantics") == 0) {
      options.requireAllSemantics = true;
      continue;
    }
    if (strcmp(arg, "--semantic-report") == 0) {
      if (++i >= argc) {
        fprintf(stderr, "Missing value for --semantic-report\n");
        return false;
      }
      options.semanticReportPath = argv[i];
      continue;
    }

    float *target = nullptr;
    if (strcmp(arg, "--cell-size") == 0)
      target = &CELL_SIZE;
    else if (strcmp(arg, "--cell-height") == 0)
      target = &CELL_HEIGHT;
    else if (strcmp(arg, "--agent-height") == 0)
      target = &AGENT_HEIGHT;
    else if (strcmp(arg, "--agent-radius") == 0)
      target = &AGENT_RADIUS;
    else if (strcmp(arg, "--agent-climb") == 0)
      target = &AGENT_MAX_CLIMB;
    else if (strcmp(arg, "--agent-slope") == 0)
      target = &AGENT_MAX_SLOPE;
    else if (strcmp(arg, "--region-min") == 0)
      target = &REGION_MIN_SIZE;
    else if (strcmp(arg, "--region-merge") == 0)
      target = &REGION_MERGE_SIZE;

    if (target) {
      if (++i >= argc || !parseFloat(argv[i], *target)) {
        fprintf(stderr, "Invalid value for %s\n", arg);
        return false;
      }
      continue;
    }
    if (strcmp(arg, "--tile-size") == 0) {
      float value = 0.0f;
      if (++i >= argc || !parseFloat(argv[i], value) ||
          value != floorf(value)) {
        fprintf(stderr, "Invalid value for --tile-size\n");
        return false;
      }
      TILE_SIZE = (int)value;
      continue;
    }

    fprintf(stderr, "Unknown option: %s\n", arg);
    return false;
  }

  if (CELL_SIZE <= 0.0f || CELL_HEIGHT <= 0.0f ||
      AGENT_HEIGHT <= 0.0f || AGENT_RADIUS < 0.0f ||
      AGENT_MAX_CLIMB < 0.0f || AGENT_MAX_SLOPE <= 0.0f ||
      AGENT_MAX_SLOPE >= 90.0f || REGION_MIN_SIZE <= 0.0f ||
      REGION_MERGE_SIZE <= 0.0f || TILE_SIZE < 16 || TILE_SIZE > 128 ||
      (TILE_SIZE & (TILE_SIZE - 1)) != 0) {
    fprintf(stderr, "Invalid build configuration\n");
    return false;
  }
  if (options.bounds.enabled &&
      (options.bounds.minX >= options.bounds.maxX ||
       options.bounds.minZ >= options.bounds.maxZ)) {
    fprintf(stderr, "Invalid build bounds\n");
    return false;
  }
  return true;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }

  const char *inputPath = argv[1];
  std::string outputPath;
  int optionStart = 2;
  if (argc >= 3 && strncmp(argv[2], "--", 2) != 0) {
    outputPath = argv[optionStart++];
  } else {
    outputPath = inputPath;
    auto dot = outputPath.rfind('.');
    if (dot != std::string::npos)
      outputPath = outputPath.substr(0, dot);
    outputPath += ".bin";
  }
  BuildOptions options;
  if (!parseOptions(argc, argv, optionStart, options))
    return 1;

  TimePoint tTotal = Clock::now();

  Mesh mesh;
  if (!loadObj(inputPath, mesh, options.legacyObjectFallback,
               options.dynamicDoorObstacles))
    return 1;
  if (options.semanticReportPath.empty())
    options.semanticReportPath = outputPath + ".semantics.json";
  if (!writeSemanticReport(inputPath, options.semanticReportPath, mesh) ||
      !validateSemanticContract(mesh, options.requireAllSemantics))
    return 1;
  if (options.validateSemanticsOnly) {
    if (!validateSemanticRasterMapping(mesh))
      return 1;
    return 0;
  }

  printf("Building spatial index...");
  fflush(stdout);
  TimePoint tIdx = Clock::now();
  PrintContext ctx;
  TriGrid grid;
  grid.build(mesh.verts.data(), mesh.tris.data(), (int)(mesh.tris.size() / 3),
             mesh.bmin, mesh.bmax, (float)TILE_SIZE * CELL_SIZE);
  printf(" done (%.2fs)\n", elapsed(tIdx));

  int gw = 0, gh = 0;
  rcCalcGridSize(mesh.bmin, mesh.bmax, CELL_SIZE, &gw, &gh);
  const int tw = (gw + TILE_SIZE - 1) / TILE_SIZE;
  const int th = (gh + TILE_SIZE - 1) / TILE_SIZE;
  const float tileWorldSize = (float)TILE_SIZE * CELL_SIZE;
  int firstTx = 0;
  int firstTy = 0;
  int lastTx = tw;
  int lastTy = th;
  if (options.bounds.enabled) {
    firstTx = std::clamp(
        (int)floorf((options.bounds.minX - mesh.bmin[0]) / tileWorldSize), 0,
        tw);
    firstTy = std::clamp(
        (int)floorf((options.bounds.minZ - mesh.bmin[2]) / tileWorldSize), 0,
        th);
    lastTx = std::clamp(
        (int)ceilf((options.bounds.maxX - mesh.bmin[0]) / tileWorldSize), 0,
        tw);
    lastTy = std::clamp(
        (int)ceilf((options.bounds.maxZ - mesh.bmin[2]) / tileWorldSize), 0,
        th);
  }
  const int totalTiles = (lastTx - firstTx) * (lastTy - firstTy);
  if (totalTiles <= 0) {
    fprintf(stderr, "Build bounds do not intersect the mesh\n");
    return 1;
  }

  const int tileBits =
      rcMin((int)ilog2(nextPow2((unsigned int)totalTiles)), 14);
  const int maxTiles = 1 << tileBits;
  const int maxPolysPerTile = 1 << (22 - tileBits);

  printf("Tile grid : %d x %d full; building [%d..%d) x [%d..%d) = %d tiles"
         "  (%.2f wu/tile)\n",
         tw, th, firstTx, lastTx, firstTy, lastTy, totalTiles,
         tileWorldSize);
  printf("Tile bits : %d  →  maxTiles=%d  maxPolys/tile=%d\n", tileBits,
         maxTiles, maxPolysPerTile);
  printf("Voxel     : cs=%.3f  ch=%.3f\n", CELL_SIZE, CELL_HEIGHT);
  printf("Agent     : height=%.2f  radius=%.2f  climb=%.2f  slope=%.1f°\n",
         AGENT_HEIGHT, AGENT_RADIUS, AGENT_MAX_CLIMB, AGENT_MAX_SLOPE);

  dtNavMeshParams nmParams{};
  rcVcopy(nmParams.orig, mesh.bmin);
  nmParams.tileWidth = (float)TILE_SIZE * CELL_SIZE;
  nmParams.tileHeight = (float)TILE_SIZE * CELL_SIZE;
  nmParams.maxTiles = maxTiles;
  nmParams.maxPolys = maxPolysPerTile;

  dtNavMesh *navMesh = dtAllocNavMesh();
  if (dtStatusFailed(navMesh->init(&nmParams))) {
    fprintf(stderr, "Failed to init dtNavMesh\n");
    dtFreeNavMesh(navMesh);
    return 1;
  }

  dtTileCacheParams tcParams{};
  rcVcopy(tcParams.orig, mesh.bmin);
  tcParams.cs = CELL_SIZE;
  tcParams.ch = CELL_HEIGHT;
  tcParams.width = TILE_SIZE;
  tcParams.height = TILE_SIZE;
  tcParams.walkableHeight = AGENT_HEIGHT;
  tcParams.walkableRadius = AGENT_RADIUS;
  tcParams.walkableClimb = AGENT_MAX_CLIMB;
  tcParams.maxSimplificationError = EDGE_MAX_ERROR;
  const int rawMaxTiles = totalTiles * EXPECTED_LAYERS_PER_TILE;
  const int tcTileBits = (int)ilog2(nextPow2((unsigned int)rawMaxTiles));
  const int cappedMaxTiles = 1 << tcTileBits;
  tcParams.maxTiles = cappedMaxTiles;
  tcParams.maxObstacles = 20000;

  FastLZCompressor tcComp;
  NullMeshProcess tcMeshProc;
  dtTileCacheAlloc tcAlloc;
  dtTileCache *tileCache = dtAllocTileCache();
  if (dtStatusFailed(
          tileCache->init(&tcParams, &tcAlloc, &tcComp, &tcMeshProc))) {
    fprintf(stderr, "Failed to init dtTileCache\n");
    dtFreeNavMesh(navMesh);
    dtFreeTileCache(tileCache);
    return 1;
  }

  printf("=== TileCache params ===\n");
  printf("cs: %f\n", tcParams.cs);
  printf("ch: %f\n", tcParams.ch);
  printf("width: %d\n", tcParams.width);
  printf("height: %d\n", tcParams.height);
  printf("tileBits: %d\n", tcTileBits);
  printf("cappedMaxTiles: %d\n", cappedMaxTiles);
  printf("maxTiles: %d\n", tcParams.maxTiles);
  printf("maxObstacles: %d\n", tcParams.maxObstacles);
  printf("walkableHeight: %f\n", tcParams.walkableHeight);
  printf("walkableRadius: %f\n", tcParams.walkableRadius);
  printf("walkableClimb: %f\n", tcParams.walkableClimb);

  // ── Build tiles ──────────────────────────────────────────────────────────
#ifdef USE_OPENMP
  printf("Building navmesh (parallel, %d threads)...\n", omp_get_max_threads());
#else
  printf("Building navmesh (single-threaded)...\n");
#endif
  int builtTiles = 0;
  int emptyTiles = 0;
  long long totalNavBytes = 0;
  int totalPolys = 0;
  int totalCacheLayers = 0;
  int maxLayersPerTile = 0;
  int failedRecastTiles = 0;
  TimePoint tBuild = Clock::now();

  std::vector<std::pair<int, int>> tileList;
  tileList.reserve(totalTiles);
  for (int ty = firstTy; ty < lastTy; ++ty)
    for (int tx = firstTx; tx < lastTx; ++tx)
      tileList.push_back({tx, ty});

  std::mutex navMeshMutex; // guards writes and progress counters
  int processed = 0;

  printProgress(0, totalTiles, 0, 0, 0.0);

#ifdef USE_OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
  for (int i = 0; i < totalTiles; ++i) {
    const auto [tx, ty] = tileList[i];

    float tmin[3], tmax[3];
    tmin[0] = mesh.bmin[0] + (float)tx * tileWorldSize;
    tmin[1] = mesh.bmin[1];
    tmin[2] = mesh.bmin[2] + (float)ty * tileWorldSize;
    tmax[0] = mesh.bmin[0] + (float)(tx + 1) * tileWorldSize;
    tmax[1] = mesh.bmax[1];
    tmax[2] = mesh.bmin[2] + (float)(ty + 1) * tileWorldSize;

    PrintContext tileCtx(tx, ty);
    int dataSize = 0;
    unsigned char *data =
        buildTile(&tileCtx, mesh, grid, tmin, tmax, tx, ty, dataSize);

    PrintContext cacheCtx(tx, ty);
    auto cacheLayers =
        buildTileCacheLayers(&cacheCtx, mesh, grid, tmin, tmax, tx, ty);

    {
      std::lock_guard<std::mutex> lock(navMeshMutex);
      if (!data) {
        emptyTiles++;
      } else {
        navMesh->removeTile(navMesh->getTileRefAt(tx, ty, 0), nullptr, nullptr);
        dtTileRef ref = 0;
        if (dtStatusFailed(
                navMesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, &ref))) {
          dtFree(data);
          emptyTiles++;
        } else {
          builtTiles++;
          totalNavBytes += dataSize;
          const dtMeshTile *tile = navMesh->getTileByRef(ref);
          if (tile && tile->header)
            totalPolys += tile->header->polyCount;
        }
      }
      int layersThisTile = (int)cacheLayers.size();
      if (tileCtx.hadError || cacheCtx.hadError)
        failedRecastTiles++;
      totalCacheLayers += layersThisTile;
      if (layersThisTile > maxLayersPerTile)
        maxLayersPerTile = layersThisTile;
      for (auto &[ldata, lsize] : cacheLayers) {
        dtStatus addStatus = tileCache->addTile(
            ldata, lsize, DT_COMPRESSEDTILE_FREE_DATA, nullptr);
        if (dtStatusFailed(addStatus)) {
          fprintf(stderr,
                  "\n[WARN]  tileCache->addTile failed (tx=%d ty=%d) — "
                  "maxTiles=%d may be too low\n",
                  tx, ty, tcParams.maxTiles);
          dtFree(ldata);
        }
      }
      processed++;
      printProgress(processed, totalTiles, builtTiles, totalNavBytes,
                    elapsed(tBuild));
    }
  }

  double buildSec = elapsed(tBuild);
  printf("\n"); // end progress bar line

  printf("Build done in %s\n", fmtDuration(buildSec).c_str());
  printf("  Tiles built  : %d / %d  (%d empty/water skipped)\n", builtTiles,
         totalTiles, emptyTiles);
  printf("  Total polys  : %d\n", totalPolys);
  printf("  Navmesh size : %s\n", fmtSize(totalNavBytes).c_str());
  if (builtTiles > 0)
    printf("  Avg/tile     : %.1f ms\n", (buildSec * 1000.0) / builtTiles);
  printf("  Cache layers : %d total  max/tile=%d  "
         "(EXPECTED_LAYERS_PER_TILE=%d)%s\n",
         totalCacheLayers, maxLayersPerTile, EXPECTED_LAYERS_PER_TILE,
         maxLayersPerTile > EXPECTED_LAYERS_PER_TILE
             ? "  *** LAYERS DROPPED ***"
             : "");
  if (failedRecastTiles > 0) {
    fprintf(stderr,
            "\n[ERROR] Navmesh build failed in %d tile(s); refusing to write "
            "a partial cache.\n",
            failedRecastTiles);
    dtFreeNavMesh(navMesh);
    dtFreeTileCache(tileCache);
    return 1;
  }

  if (options.verifyBakedSemantics) {
    const std::map<unsigned int, long long> requiredSemanticAreas =
        collectRequiredSemanticAreas(mesh, options.bounds);
    printf("Semantic verification source coverage:");
    if (requiredSemanticAreas.empty())
      printf(" none");
    for (const auto &[area, triangles] : requiredSemanticAreas)
      printf(" area%u=%lld", area, triangles);
    printf(" (%s)\n", options.bounds.enabled ? "requested bounds" : "full mesh");
    if (!inspectBakedSemantics("direct", *navMesh, mesh,
                               requiredSemanticAreas)) {
      dtFreeNavMesh(navMesh);
      dtFreeTileCache(tileCache);
      return 1;
    }
    dtNavMesh *cacheNavMesh = dtAllocNavMesh();
    dtNavMeshParams cacheInspectionParams = nmParams;
    const int cacheNavTileBits = std::min(
        (int)ilog2(nextPow2((unsigned int)std::max(totalCacheLayers, 1))),
        22 - 7);
    cacheInspectionParams.maxTiles = 1 << cacheNavTileBits;
    cacheInspectionParams.maxPolys = 1 << (22 - cacheNavTileBits);
    if (!cacheNavMesh ||
        dtStatusFailed(cacheNavMesh->init(&cacheInspectionParams))) {
      fprintf(stderr, "Cannot initialize TileCache semantic inspection mesh\n");
      dtFreeNavMesh(cacheNavMesh);
      dtFreeNavMesh(navMesh);
      dtFreeTileCache(tileCache);
      return 1;
    }
    bool cacheBuildOk = true;
    for (const auto &[tx, ty] : tileList) {
      const dtStatus buildStatus =
          tileCache->buildNavMeshTilesAt(tx, ty, cacheNavMesh);
      if (dtStatusFailed(buildStatus)) {
        fprintf(stderr,
                "TileCache semantic inspection failed to materialize "
                "(%d,%d), status=0x%08x\n",
                tx, ty, buildStatus);
        cacheBuildOk = false;
        break;
      }
    }
    if (!cacheBuildOk || !inspectBakedSemantics(
                             "tilecache", *cacheNavMesh, mesh,
                             requiredSemanticAreas)) {
      dtFreeNavMesh(cacheNavMesh);
      dtFreeNavMesh(navMesh);
      dtFreeTileCache(tileCache);
      return 1;
    }
    dtFreeNavMesh(cacheNavMesh);
  }

  // split into 25 MB parts
  static const size_t MAX_PART_BYTES = 25ULL * 1024 * 1024;

  const dtNavMesh *cnm = navMesh;

  std::vector<const dtMeshTile *> tiles;
  for (int i = 0; i < cnm->getMaxTiles(); ++i) {
    const dtMeshTile *tile = cnm->getTile(i);
    if (!tile || !tile->header || !tile->dataSize)
      continue;
    tiles.push_back(tile);
  }

  const std::filesystem::path outDir =
      std::filesystem::path(outputPath).parent_path();
  if (!outDir.empty()) {
    std::error_code createError;
    std::filesystem::create_directories(outDir, createError);
    if (createError) {
      fprintf(stderr, "\nCannot create output directory %s: %s\n",
              outDir.string().c_str(), createError.message().c_str());
      dtFreeNavMesh(navMesh);
      dtFreeTileCache(tileCache);
      return 1;
    }
  }

  struct PartRange {
    size_t start, count;
  };
  std::vector<PartRange> parts;
  {
    size_t idx = 0;
    bool firstPart = true;
    while (idx < tiles.size()) {
      size_t headerCost = firstPart ? sizeof(NavMeshSetHeader) : 0;
      size_t budget = MAX_PART_BYTES - headerCost;
      size_t start = idx;
      size_t count = 0;
      while (idx < tiles.size()) {
        size_t tileBytes =
            sizeof(NavMeshTileHeader) + (size_t)tiles[idx]->dataSize;
        if (count > 0 && tileBytes > budget)
          break;
        budget -= tileBytes;
        count++;
        idx++;
      }
      parts.push_back({start, count});
      firstPart = false;
    }
  }

  TimePoint tSave = Clock::now();

  for (size_t p = 0; p < parts.size(); p++) {
    const std::string partPath =
        (outDir / ("z1_" + std::to_string(p) + ".bin")).string();
    printf("Writing %s ...", partPath.c_str());
    fflush(stdout);

    FILE *f = fopen(partPath.c_str(), "wb");
    if (!f) {
      fprintf(stderr, "\nCannot write: %s\n", partPath.c_str());
      dtFreeNavMesh(navMesh);
      return 1;
    }

    if (p == 0) {
      NavMeshSetHeader header{};
      header.magic = NAVMESHSET_MAGIC;
      header.version = NAVMESHSET_VERSION;
      header.numTiles = (int)tiles.size();
      memcpy(&header.params, cnm->getParams(), sizeof(dtNavMeshParams));
      fwrite(&header, sizeof(header), 1, f);
    }

    for (size_t t = 0; t < parts[p].count; t++) {
      const dtMeshTile *tile = tiles[parts[p].start + t];
      NavMeshTileHeader tileHdr{};
      tileHdr.tileRef = cnm->getTileRef(tile);
      tileHdr.dataSize = tile->dataSize;
      fwrite(&tileHdr, sizeof(tileHdr), 1, f);
      fwrite(tile->data, tile->dataSize, 1, f);
    }
    fclose(f);
    printf(" done\n");
  }

  printf("Wrote %zu navmesh part(s) (%.2fs)\n", parts.size(), elapsed(tSave));

  // split into 25 MB parts
  std::vector<const dtCompressedTile *> cacheTiles;
  for (int i = 0; i < tileCache->getTileCount(); ++i) {
    const dtCompressedTile *tile = tileCache->getTile(i);
    if (!tile || !tile->header || !tile->dataSize)
      continue;
    cacheTiles.push_back(tile);
  }

  std::vector<PartRange> cacheParts;
  {
    size_t idx = 0;
    bool firstPart = true;
    while (idx < cacheTiles.size()) {
      size_t headerCost = firstPart ? sizeof(TileCacheSetHeader) : 0;
      size_t budget = MAX_PART_BYTES - headerCost;
      size_t start = idx;
      size_t count = 0;
      while (idx < cacheTiles.size()) {
        size_t tileBytes =
            sizeof(TileCacheTileHeader) + (size_t)cacheTiles[idx]->dataSize;
        if (count > 0 && tileBytes > budget)
          break;
        budget -= tileBytes;
        count++;
        idx++;
      }
      cacheParts.push_back({start, count});
      firstPart = false;
    }
  }

  TimePoint tCacheSave = Clock::now();
  for (size_t p = 0; p < cacheParts.size(); p++) {
    const std::string partPath =
        (outDir / ("z1_cache_" + std::to_string(p) + ".bin")).string();
    printf("Writing %s ...", partPath.c_str());
    fflush(stdout);

    FILE *f = fopen(partPath.c_str(), "wb");
    if (!f) {
      fprintf(stderr, "\nCannot write: %s\n", partPath.c_str());
      dtFreeNavMesh(navMesh);
      dtFreeTileCache(tileCache);
      return 1;
    }

    if (p == 0) {
      TileCacheSetHeader header{};
      header.magic = TILECACHESET_MAGIC;
      header.version = TILECACHESET_VERSION;
      header.numTiles = (int)cacheTiles.size();

      dtNavMeshParams tcMeshParams{};
      rcVcopy(tcMeshParams.orig, mesh.bmin);
      tcMeshParams.tileWidth = (float)TILE_SIZE * CELL_SIZE;
      tcMeshParams.tileHeight = (float)TILE_SIZE * CELL_SIZE;
      const int tcNavTileBits =
          std::min((int)ilog2(nextPow2((unsigned int)cacheTiles.size())), 22 - 7);
      tcMeshParams.maxTiles = 1 << tcNavTileBits;
      tcMeshParams.maxPolys = 1 << (22 - tcNavTileBits);

      memcpy(&header.meshParams, &tcMeshParams, sizeof(dtNavMeshParams));
      memcpy(&header.cacheParams, tileCache->getParams(),
             sizeof(dtTileCacheParams));
      fwrite(&header, sizeof(header), 1, f);
    }

    for (size_t t = 0; t < cacheParts[p].count; t++) {
      const dtCompressedTile *tile = cacheTiles[cacheParts[p].start + t];
      TileCacheTileHeader tileHdr{};
      tileHdr.tileRef = tileCache->getTileRef(tile);
      tileHdr.dataSize = tile->dataSize;
      fwrite(&tileHdr, sizeof(tileHdr), 1, f);
      fwrite(tile->data, tile->dataSize, 1, f);
    }
    fclose(f);
    printf(" done\n");
  }

  printf("Wrote %zu cache part(s) (%.2fs)\n", cacheParts.size(),
         elapsed(tCacheSave));
  printf("Total time: %s\n", fmtDuration(elapsed(tTotal)).c_str());

  dtFreeNavMesh(navMesh);
  dtFreeTileCache(tileCache);
  return 0;
}
