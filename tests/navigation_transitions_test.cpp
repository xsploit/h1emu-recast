#include "navigation_transitions.h"

#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace h1emu::nav;

namespace {

void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}

std::filesystem::path fixturePath() {
  return std::filesystem::temp_directory_path() /
         "h1emu-navigation-transitions-test.json";
}

void writeFixture(const std::string &contents) {
  std::ofstream output(fixturePath(), std::ios::binary | std::ios::trunc);
  if (!output)
    throw std::runtime_error("could not create transition fixture");
  output << contents;
  if (!output)
    throw std::runtime_error("could not write transition fixture");
}

void expectFailure(const std::string &name, const std::string &contents) {
  writeFixture(contents);
  try {
    (void)loadNavigationTransitions(fixturePath().string());
  } catch (const std::exception &) {
    return;
  }
  throw std::runtime_error(name + " unexpectedly succeeded");
}

bool equalBinding(const NavigationTransitionBinding &a,
                  const NavigationTransitionBinding &b) {
  return a.vertices == b.vertices && a.radii == b.radii &&
         a.directions == b.directions && a.areas == b.areas &&
         a.flags == b.flags && a.userIds == b.userIds;
}

} // namespace

int main() {
  try {
    writeFixture(R"json([
      {
        "name": "front steps",
        "kind": "stairs",
        "source": "manual-validated",
        "start": [0, 2, 0],
        "end": [4, 3.5, 0]
      },
      {
        "name": "interior stairs",
        "start": [10, 5, 20],
        "end": [14, 7, 20],
        "radius": 0.3,
        "bidirectional": true
      }
    ])json");

    const auto transitions =
        loadNavigationTransitions(fixturePath().string());
    require(transitions.size() == 2, "expected two transitions");
    require(transitions[0].name == "front steps" &&
                transitions[0].radius == 0.8f &&
                transitions[0].sourceIndex == 0,
            "defaults/source order mismatch");
    require(transitions[1].start[0] == 10.0f &&
                transitions[1].end[1] == 7.0f &&
                transitions[1].radius == 0.3f &&
                transitions[1].sourceIndex == 1,
            "explicit transition mismatch");

    const NavigationTransitionBinding all =
        buildNavigationTransitionBinding(transitions);
    require(all.size() == 2 && all.vertices.size() == 12,
            "full binding size mismatch");
    require(all.directions[0] == 1 && all.directions[1] == 1,
            "links must be bidirectional");
    require(all.areas[0] == 7 && all.flags[0] == 0x0d,
            "threshold area/flags mismatch");
    require(all.userIds[0] == NAVIGATION_TRANSITION_USER_ID_BASE &&
                all.userIds[1] == NAVIGATION_TRANSITION_USER_ID_BASE + 1,
            "stable user-id mapping mismatch");

    // Identical authored input must always produce identical Detour arrays.
    require(equalBinding(all, buildNavigationTransitionBinding(transitions)),
            "binding is not deterministic");

    // Start-point ownership is min-inclusive/max-exclusive in X/Z. Endpoint
    // placement is irrelevant: the second link ends outside this tile but is
    // still owned by it because its start is inside.
    const float firstMin[3] = {0, 0, 0};
    const float firstMax[3] = {10, 10, 20};
    const auto first = buildNavigationTransitionBinding(
        transitions, firstMin, firstMax, 0.0f);
    require(first.size() == 1 &&
                first.userIds[0] == NAVIGATION_TRANSITION_USER_ID_BASE,
            "first tile ownership mismatch");

    const float secondMin[3] = {10, 4, 20};
    const float secondMax[3] = {25, 4.5f, 25};
    require(buildNavigationTransitionBinding(transitions, secondMin,
                                             secondMax, 0.0f)
                .size() == 0,
            "vertical layer filter should reject link without padding");
    const auto second = buildNavigationTransitionBinding(
        transitions, secondMin, secondMax, 1.0f);
    require(second.size() == 1 &&
                second.userIds[0] == NAVIGATION_TRANSITION_USER_ID_BASE + 1,
            "boundary/layer ownership mismatch");

    expectFailure("unknown field",
                  R"([{"name":"a","start":[0,0,0],"end":[1,0,0],"wat":1}])");
    expectFailure("duplicate field",
                  R"([{"name":"a","name":"b","start":[0,0,0],"end":[1,0,0]}])");
    expectFailure("duplicate name",
                  R"([{"name":"a","start":[0,0,0],"end":[1,0,0]},{"name":"a","start":[2,0,0],"end":[3,0,0]}])");
    expectFailure("one-way link",
                  R"([{"name":"a","start":[0,0,0],"end":[1,0,0],"bidirectional":false}])");
    expectFailure("zero radius",
                  R"([{"name":"a","start":[0,0,0],"end":[1,0,0],"radius":0}])");
    expectFailure("extra coordinate",
                  R"([{"name":"a","start":[0,0,0,1],"end":[1,0,0]}])");
    expectFailure("trailing content", "[] nope");

    std::error_code ignored;
    std::filesystem::remove(fixturePath(), ignored);
    std::cout << "navigation transition contract PASS\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "navigation transition contract FAIL: " << error.what()
              << "\n";
    return 1;
  }
}
