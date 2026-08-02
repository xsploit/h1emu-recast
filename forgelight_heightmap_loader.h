#pragma once

#include <cstdint>
#include <filesystem>

#include "forgelight_nav_source.h"

namespace h1emu::nav {

// Decodes a heightmap image file into the decode-agnostic HeightmapRgb
// contract (forgelight_nav_source.h). Requires an RGB (or RGBA, alpha
// discarded) image; any other channel layout is rejected rather than
// silently reinterpreted.
HeightmapRgb loadHeightmapImage(const std::filesystem::path &path);

// Fallback for fixtures/tests that want to supply already-decoded raw RGB
// bytes without a PNG codec in the loop at all.
HeightmapRgb loadHeightmapRaw(const std::filesystem::path &path,
                              std::uint32_t width, std::uint32_t height);

} // namespace h1emu::nav
