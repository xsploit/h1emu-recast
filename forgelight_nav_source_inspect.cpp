#include "forgelight_nav_source.h"

#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace h1emu::nav;

int main(int argc, char **argv) {
  if (argc < 2 || argc > 4) {
    std::cerr << "Usage: forgelight-nav-source-inspect <z1_collision.bin> "
                 "[z1_collision.h1sem] [--strict-production]\n";
    return 2;
  }
  try {
    bool strictProduction = false;
    std::string semanticPath;
    for (int index = 2; index < argc; ++index) {
      const std::string argument = argv[index];
      if (argument == "--strict-production")
        strictProduction = true;
      else if (semanticPath.empty())
        semanticPath = argument;
      else
        throw std::runtime_error("unexpected argument " + argument);
    }

    const H1Col2Document collision = loadH1Col2(argv[1]);
    std::array<std::uint64_t, 4> kindCounts{};
    std::uint64_t triangleCount = 0;
    for (const CollisionMesh &mesh : collision.meshes) {
      ++kindCounts[static_cast<std::uint8_t>(mesh.kind)];
      triangleCount += mesh.triangleCount();
    }

    std::cout << "{\"collision\":{\"version\":" << collision.version
              << ",\"sha256\":\"" << sha256Hex(collision.sha256)
              << "\",\"meshCount\":" << collision.meshes.size()
              << ",\"instanceCount\":" << collision.instances.size()
              << ",\"triangleCount\":" << triangleCount
              << ",\"kindHistogram\":[" << kindCounts[0] << ','
              << kindCounts[1] << ',' << kindCounts[2] << ',' << kindCounts[3]
              << "]}";

    if (!semanticPath.empty()) {
      const H1Sem1Document semantics =
          loadH1Sem1(semanticPath, collision, strictProduction);
      std::array<std::uint64_t, 12> histogram{};
      for (SemanticId id : semantics.semantics)
        ++histogram[static_cast<std::uint8_t>(id)];
      std::cout << ",\"semantics\":{\"version\":" << semantics.version
                << ",\"schemaVersion\":" << semantics.semanticSchemaVersion
                << ",\"meshCount\":" << semantics.meshCount()
                << ",\"triangleCount\":" << semantics.triangleCount()
                << ",\"strictProduction\":"
                << (strictProduction ? "true" : "false")
                << ",\"histogram\":[";
      for (std::size_t index = 0; index < histogram.size(); ++index) {
        if (index != 0)
          std::cout << ',';
        std::cout << histogram[index];
      }
      std::cout << "]}";
    }
    std::cout << "}\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Forgelight source validation failed: " << error.what()
              << '\n';
    return 1;
  }
}
