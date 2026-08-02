#include "forgelight_heightmap_loader.h"

#include <stdexcept>
#include <vector>

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_STDIO
#include "third_party/stb_image.h"

#include <cstdio>

namespace h1emu::nav {
namespace {

// stb_image's file-path entry points assume a narrow/ANSI-compatible path
// on Windows; STBI_NO_STDIO plus our own fopen keeps this identical to
// readFileBytes' cross-platform behavior in forgelight_nav_source.cpp
// instead of relying on stb's internal fopen.
std::vector<std::uint8_t> readWholeFile(const std::filesystem::path &path) {
  FILE *file = std::fopen(path.string().c_str(), "rb");
  if (!file)
    throw std::runtime_error("cannot open heightmap image " + path.string());
  std::fseek(file, 0, SEEK_END);
  const long length = std::ftell(file);
  if (length < 0) {
    std::fclose(file);
    throw std::runtime_error("cannot size heightmap image " + path.string());
  }
  std::fseek(file, 0, SEEK_SET);
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(length));
  const std::size_t read =
      bytes.empty() ? 0
                    : std::fread(bytes.data(), 1, bytes.size(), file);
  std::fclose(file);
  if (read != bytes.size())
    throw std::runtime_error("short read from heightmap image " +
                             path.string());
  return bytes;
}

} // namespace

HeightmapRgb loadHeightmapImage(const std::filesystem::path &path) {
  const std::vector<std::uint8_t> encoded = readWholeFile(path);
  int width = 0;
  int height = 0;
  int sourceChannels = 0;
  // Force exactly 3 output channels: grayscale is expanded, alpha (if any)
  // is discarded. decodePixel/sampleNearestWorld (forgelight_nav_source.cpp)
  // only ever read the R and G bytes of each pixel, so requesting anything
  // other than a flat RGB layout here would silently misalign them.
  unsigned char *decoded = stbi_load_from_memory(
      encoded.data(), static_cast<int>(encoded.size()), &width, &height,
      &sourceChannels, 3);
  if (!decoded)
    throw std::runtime_error("failed to decode heightmap image " +
                             path.string() + ": " + stbi_failure_reason());
  std::vector<std::uint8_t> rgb(decoded, decoded + (std::size_t)width *
                                                       (std::size_t)height *
                                                       3);
  stbi_image_free(decoded);
  return HeightmapRgb(static_cast<std::uint32_t>(width),
                      static_cast<std::uint32_t>(height), std::move(rgb));
}

HeightmapRgb loadHeightmapRaw(const std::filesystem::path &path,
                              std::uint32_t width, std::uint32_t height) {
  return HeightmapRgb(width, height, readWholeFile(path));
}

} // namespace h1emu::nav
