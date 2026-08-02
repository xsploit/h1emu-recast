#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace h1emu::nav {

// Authored world-space graph edge. The JSON source order is authoritative:
// both the baker and the live server derive the Detour user id from it.
struct NavigationTransition {
  std::string name;
  std::array<float, 3> start{};
  std::array<float, 3> end{};
  float radius = 0.8f;
  bool bidirectional = true;
  std::size_t sourceIndex = 0;
};

// Structure-of-arrays storage matching dtNavMeshCreateParams. Keeping this
// independent of Detour makes parsing/ownership independently testable and
// lets each parallel direct-tile worker own its pointer backing storage.
struct NavigationTransitionBinding {
  std::vector<float> vertices;
  std::vector<float> radii;
  std::vector<unsigned char> directions;
  std::vector<unsigned char> areas;
  std::vector<unsigned short> flags;
  std::vector<unsigned int> userIds;

  std::size_t size() const { return radii.size(); }
};

// Reads the existing h1z1-pv-nav navigationTransitions.json schema. This is a
// deliberately strict, dependency-free parser: malformed JSON, duplicate or
// unknown fields, invalid coordinates, non-positive radii, one-way links, and
// duplicate names are rejected instead of being silently normalized.
std::vector<NavigationTransition>
loadNavigationTransitions(const std::string &path);

// Build Detour input arrays in canonical source order. When bounds are
// supplied, ownership is determined only by the start endpoint: X/Z are
// minimum-inclusive and maximum-exclusive, matching Detour tile ownership;
// Y admits the layer bounds plus yPadding and is finalized by Detour's tight
// polygon-height classification.
NavigationTransitionBinding buildNavigationTransitionBinding(
    const std::vector<NavigationTransition> &transitions,
    const float *boundsMin = nullptr, const float *boundsMax = nullptr,
    float yPadding = 0.0f);

constexpr unsigned int NAVIGATION_TRANSITION_USER_ID_BASE = 0x48000000u;

} // namespace h1emu::nav
