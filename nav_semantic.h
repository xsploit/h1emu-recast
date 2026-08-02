#pragma once

#include <string>

#include "Recast.h"

// Shared nav-semantic vocabulary. Relocated verbatim from main.cpp so that
// non-OBJ GeometrySource implementations (e.g. forgelight_geometry_source)
// can classify triangles into the same NavSemantic/NavArea/NavFlag space
// that prepareRasterTriangles/applyNonWalkableCarves consume. No logic here
// differs from the prior main.cpp inline definitions.

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
