#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
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

struct BuildBounds {
  bool enabled = false;
  float minX = 0.0f;
  float minZ = 0.0f;
  float maxX = 0.0f;
  float maxZ = 0.0f;
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
  void process(dtNavMeshCreateParams *params, unsigned char * /*polyAreas*/,
               unsigned short *polyFlags) override {
    for (int i = 0; i < params->polyCount; ++i)
      polyFlags[i] = 1;
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
protected:
  void doLog(const rcLogCategory category, const char *msg,
             const int /*len*/) override {
    if (category == RC_LOG_ERROR)
      fprintf(stderr, "\n[ERROR] %s\n", msg);
    else if (category == RC_LOG_WARNING)
      fprintf(stderr, "\n[WARN]  %s\n", msg);
  }
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

struct Mesh {
  std::vector<float> verts; // x,y,z triples
  std::vector<int> tris;    // 0-based triangle indices
  float bmin[3];
  float bmax[3];
};

static bool loadObj(const char *path, Mesh &out) {
  printf("Loading %s ...\n", path);
  TimePoint t0 = Clock::now();

  FILE *f = fopen(path, "r");
  if (!f) {
    fprintf(stderr, "  Cannot open: %s\n", path);
    return false;
  }

  out.bmin[0] = out.bmin[1] = out.bmin[2] = 1e30f;
  out.bmax[0] = out.bmax[1] = out.bmax[2] = -1e30f;

  char line[4096];
  while (fgets(line, sizeof(line), f)) {
    if (line[0] == 'v' && line[1] == ' ') {
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
        out.tris.push_back(idx[0]);
        out.tris.push_back(idx[i - 1]);
        out.tris.push_back(idx[i]);
      }
    }
  }
  fclose(f);

  if (out.verts.empty() || out.tris.empty()) {
    fprintf(stderr, "  No geometry found in %s\n", path);
    return false;
  }

  printf("  %d verts, %d tris  (%.2fs)\n", (int)(out.verts.size() / 3),
         (int)(out.tris.size() / 3), elapsed(t0));
  printf("  Bounds X [%.2f .. %.2f]  Y [%.2f .. %.2f]  Z [%.2f .. %.2f]\n",
         out.bmin[0], out.bmax[0], out.bmin[1], out.bmax[1], out.bmin[2],
         out.bmax[2]);
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
  cfg.minRegionArea = (int)rcSqr(REGION_MIN_SIZE);
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

  std::vector<int> nodeTris;
  nodeTris.reserve(triIds.size() * 3);
  for (int i : triIds) {
    nodeTris.push_back(mesh.tris[i * 3 + 0]);
    nodeTris.push_back(mesh.tris[i * 3 + 1]);
    nodeTris.push_back(mesh.tris[i * 3 + 2]);
  }
  const int nTris = (int)triIds.size();
  std::vector<unsigned char> areas(nTris, 0);
  rcMarkWalkableTriangles(ctx, cfg.walkableSlopeAngle, verts, nVerts,
                          nodeTris.data(), nTris, areas.data());
  rcRasterizeTriangles(ctx, verts, nVerts, nodeTris.data(), areas.data(), nTris,
                       *hf, cfg.walkableClimb);

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

  if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *chf)) {
    rcFreeCompactHeightfield(chf);
    return nullptr;
  }

  // Watershed -> best quality
  if (!rcBuildDistanceField(ctx, *chf) ||
      !rcBuildRegions(ctx, *chf, cfg.borderSize, cfg.minRegionArea,
                      cfg.mergeRegionArea)) {
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
    pmesh->flags[i] = 1; // walkable

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
  cfg.minRegionArea = (int)rcSqr(REGION_MIN_SIZE);
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

  std::vector<int> nodeTris;
  nodeTris.reserve(triIds.size() * 3);
  for (int i : triIds) {
    nodeTris.push_back(mesh.tris[i * 3 + 0]);
    nodeTris.push_back(mesh.tris[i * 3 + 1]);
    nodeTris.push_back(mesh.tris[i * 3 + 2]);
  }
  const int nTris = (int)triIds.size();
  std::vector<unsigned char> areas(nTris, 0);
  rcMarkWalkableTriangles(ctx, cfg.walkableSlopeAngle, verts, nVerts,
                          nodeTris.data(), nTris, areas.data());
  rcRasterizeTriangles(ctx, verts, nVerts, nodeTris.data(), areas.data(), nTris,
                       *hf, cfg.walkableClimb);

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

  if (!rcErodeWalkableArea(ctx, cfg.walkableRadius, *chf)) {
    rcFreeCompactHeightfield(chf);
    return result;
  }

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
                         BuildBounds &bounds) {
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
      if (i + 4 >= argc || !parseFloat(argv[i + 1], bounds.minX) ||
          !parseFloat(argv[i + 2], bounds.minZ) ||
          !parseFloat(argv[i + 3], bounds.maxX) ||
          !parseFloat(argv[i + 4], bounds.maxZ)) {
        fprintf(stderr, "Invalid --bounds values\n");
        return false;
      }
      bounds.enabled = true;
      i += 4;
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
  if (bounds.enabled &&
      (bounds.minX >= bounds.maxX || bounds.minZ >= bounds.maxZ)) {
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
  BuildBounds bounds;
  if (!parseOptions(argc, argv, optionStart, bounds))
    return 1;

  TimePoint tTotal = Clock::now();

  Mesh mesh;
  if (!loadObj(inputPath, mesh))
    return 1;

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
  if (bounds.enabled) {
    firstTx = std::clamp(
        (int)floorf((bounds.minX - mesh.bmin[0]) / tileWorldSize), 0, tw);
    firstTy = std::clamp(
        (int)floorf((bounds.minZ - mesh.bmin[2]) / tileWorldSize), 0, th);
    lastTx = std::clamp(
        (int)ceilf((bounds.maxX - mesh.bmin[0]) / tileWorldSize), 0, tw);
    lastTy = std::clamp(
        (int)ceilf((bounds.maxZ - mesh.bmin[2]) / tileWorldSize), 0, th);
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

    PrintContext tileCtx;
    int dataSize = 0;
    unsigned char *data =
        buildTile(&tileCtx, mesh, grid, tmin, tmax, tx, ty, dataSize);

    PrintContext cacheCtx;
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
