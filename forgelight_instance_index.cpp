#include "forgelight_instance_index.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace h1emu::nav {

int InstanceIndex::clampedCellX(float worldX) const {
  const int raw = static_cast<int>(std::floor((worldX - origin_[0]) / cellSize_));
  return std::max(0, std::min(nx_ - 1, raw));
}

int InstanceIndex::clampedCellZ(float worldZ) const {
  const int raw = static_cast<int>(std::floor((worldZ - origin_[1]) / cellSize_));
  return std::max(0, std::min(nz_ - 1, raw));
}

void InstanceIndex::build(const H1Col2Document &collision,
                          const float worldBminXZ[2],
                          const float worldBmaxXZ[2], float cellSize) {
  if (!(cellSize > 0.0f))
    throw std::runtime_error("InstanceIndex cell size must be positive");
  if (!(worldBmaxXZ[0] > worldBminXZ[0]) ||
      !(worldBmaxXZ[1] > worldBminXZ[1]))
    throw std::runtime_error(
        "InstanceIndex world bounds must be non-degenerate");

  cellSize_ = cellSize;
  origin_[0] = worldBminXZ[0];
  origin_[1] = worldBminXZ[1];
  nx_ = std::max(
      1, static_cast<int>(
             std::ceil((worldBmaxXZ[0] - worldBminXZ[0]) / cellSize_)));
  nz_ = std::max(
      1, static_cast<int>(
             std::ceil((worldBmaxXZ[1] - worldBminXZ[1]) / cellSize_)));

  const std::size_t cellCount =
      static_cast<std::size_t>(nx_) * static_cast<std::size_t>(nz_);

  // instance.transform layout (forgelight_nav_source.h): tx,ty,tz, qx,qy,qz,
  // qw, sx,sy,sz, minX,minY,minZ,maxX,maxY,maxZ -- indices 10/12 and 13/15
  // are the instance's own stored world AABB min/max X and Z. No mesh
  // geometry needs to be touched to bucket an instance.
  const auto cellRange = [&](const CollisionInstance &instance, int &x0,
                            int &z0, int &x1, int &z1) {
    x0 = clampedCellX(instance.transform[10]);
    z0 = clampedCellZ(instance.transform[12]);
    x1 = clampedCellX(instance.transform[13]);
    z1 = clampedCellZ(instance.transform[15]);
  };

  // Pass 1: count memberships per cell, offset by one so an in-place prefix
  // sum below turns this directly into CSR row pointers (cellOffsets_[i] =
  // total memberships in cells [0..i)).
  std::vector<std::uint32_t> offsets(cellCount + 1, 0);
  for (const CollisionInstance &instance : collision.instances) {
    int x0, z0, x1, z1;
    cellRange(instance, x0, z0, x1, z1);
    for (int z = z0; z <= z1; ++z)
      for (int x = x0; x <= x1; ++x)
        offsets[static_cast<std::size_t>(z) * nx_ + x + 1]++;
  }
  for (std::size_t i = 1; i < offsets.size(); ++i)
    offsets[i] += offsets[i - 1];
  cellOffsets_ = offsets;

  // Pass 2: scatter instance indices into their cells using a per-cell write
  // cursor seeded from the row pointers (standard CSR construction).
  std::vector<std::uint32_t> cursor(cellOffsets_.begin(),
                                    cellOffsets_.end() - 1);
  cellInstances_.assign(cellOffsets_.back(), 0);
  for (std::uint32_t instanceIndex = 0;
       instanceIndex < collision.instances.size(); ++instanceIndex) {
    int x0, z0, x1, z1;
    cellRange(collision.instances[instanceIndex], x0, z0, x1, z1);
    for (int z = z0; z <= z1; ++z) {
      for (int x = x0; x <= x1; ++x) {
        const std::size_t cell = static_cast<std::size_t>(z) * nx_ + x;
        cellInstances_[cursor[cell]++] = instanceIndex;
      }
    }
  }
}

std::vector<std::uint32_t> InstanceIndex::query(const float qmin[2],
                                                const float qmax[2]) const {
  std::vector<std::uint32_t> out;
  if (nx_ <= 0 || nz_ <= 0)
    return out;
  const int x0 = clampedCellX(qmin[0]);
  const int z0 = clampedCellZ(qmin[1]);
  const int x1 = clampedCellX(qmax[0]);
  const int z1 = clampedCellZ(qmax[1]);
  for (int z = z0; z <= z1; ++z) {
    for (int x = x0; x <= x1; ++x) {
      const std::size_t cell = static_cast<std::size_t>(z) * nx_ + x;
      for (std::uint32_t i = cellOffsets_[cell]; i < cellOffsets_[cell + 1];
           ++i)
        out.push_back(cellInstances_[i]);
    }
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

} // namespace h1emu::nav
