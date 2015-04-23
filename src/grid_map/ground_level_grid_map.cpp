#include <vigir_terrain_classifier/grid_map/ground_level_grid_map.h>

namespace vigir_terrain_classifier
{
GroundLevelGridMap::GroundLevelGridMap(const std::string& frame_id, double resolution, double min_expansion_size, double update_weight)
  : GridMap(frame_id, resolution, min_expansion_size)
{
}

GroundLevelGridMap::~GroundLevelGridMap()
{
}

//void GroundLevelGridMap::fromMsg(const GroundLevelGridMapMsg& msg)
//{
//}

//void GroundLevelGridMap::toMsg(GroundLevelGridMapMsg& msg) const
//{
//}
}
