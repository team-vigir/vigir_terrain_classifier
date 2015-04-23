#include <vigir_terrain_classifier/grid_map/height_grid_map.h>

namespace vigir_terrain_classifier
{
HeightGridMap::HeightGridMap(const std::string& frame_id, double resolution, double min_expansion_size, double update_weight)
  : GridMap(frame_id, resolution, min_expansion_size)
  , update_weight(update_weight)
  , height_scale(0.0)
  , inv_height_scale(0.0)
{
}

HeightGridMap::HeightGridMap(const HeightGridMapMsg& msg, double min_expansion_size, double update_weight)
  : GridMap(msg.grid_map, min_expansion_size)
  , update_weight(update_weight)
{
  height_scale = msg.height_scale;
  inv_height_scale = 1.0/height_scale;
}

HeightGridMap::~HeightGridMap()
{
}

void HeightGridMap::clear()
{
  GridMap::clear();
  height_scale = 0.0;
  inv_height_scale = 0.0;
}

double HeightGridMap::getUpdateWeight() const
{
  return update_weight;
}

void HeightGridMap::fromMsg(const HeightGridMapMsg& msg)
{
  GridMap::fromMsg(msg.grid_map);
  height_scale = msg.height_scale;
  inv_height_scale = 1.0/height_scale;
}

void HeightGridMap::toMsg(HeightGridMapMsg& msg) const
{
  msg.header.frame_id = grid_map->header.frame_id;
  msg.header.stamp = ros::Time::now();
  GridMap::toMsg(msg.grid_map);
  msg.height_scale = height_scale;
}

void HeightGridMap::resize(const geometry_msgs::Vector3& min, const geometry_msgs::Vector3& max)
{
  GridMap::resize(min, max);

  // check if rescale is needed
  if (min.z >= this->min.z && max.z <= this->max.z)
    return;

  // update boundary in z
  double new_min_z = std::min(this->min.z, min.z);
  double new_max_z = std::max(this->max.z, max.z);

  rescale(new_min_z, new_max_z);

  grid_map->info.origin.position.z = this->min.z;
}

double HeightGridMap::rescale(nav_msgs::OccupancyGrid& map, double old_min_z, double old_max_z, double new_min_z, double new_max_z)
{
  double old_height_scale = (old_max_z-old_min_z) / (std::numeric_limits<int8_t>::max()-std::numeric_limits<int8_t>::min()+1);
  double new_inv_height_scale = (std::numeric_limits<int8_t>::max()-std::numeric_limits<int8_t>::min()+1) / (new_max_z-new_min_z);

  for (nav_msgs::OccupancyGrid::_data_type::iterator itr = map.data.begin(); itr != map.data.end(); itr++)
  {
    int8_t& val = *itr;
    if (val == std::numeric_limits<int8_t>::min())
      continue;
    val = heightToMap(heightToWorld(val, old_min_z, old_height_scale), new_min_z, new_inv_height_scale);
  }

  return 1.0/new_inv_height_scale;
}

void HeightGridMap::rescale(double min_z, double max_z)
{
  if (min_z >= max_z)
    return;

  if (height_scale != 0.0)
    height_scale = rescale(*grid_map, this->min.z, this->max.z, min_z, max_z);
  else
    height_scale = (max_z-min_z) / static_cast<double>(std::numeric_limits<int8_t>::max()-std::numeric_limits<int8_t>::min()+1);

  inv_height_scale = 1.0/height_scale;
  this->min.z = min_z;
  this->max.z = max_z;
}

int8_t HeightGridMap::heightToMap(double height, double min_z, double max_z, int8_t min_val, int8_t max_val)
{
  assert(height >= min_z && height <= max_z);
  double inv_height_scale = static_cast<double>(max_val-min_val)/(max_z-min_z);
  return heightToMap(height, min_z, inv_height_scale, min_val);
}

int8_t HeightGridMap::heightToMap(double height, double min_z, double inv_height_scale, int8_t min_val)
{
  assert(height >= min_z);
  return static_cast<int8_t>(round((height-min_z)*inv_height_scale)) + min_val;
}

double HeightGridMap::heightToWorld(int8_t height, double min_z, double max_z, int8_t min_val, int8_t max_val)
{
  double height_scale = (max_z-min_z)/static_cast<double>(max_val-min_val);
  return heightToWorld(height, min_z, height_scale, min_val);
}

double HeightGridMap::heightToWorld(int8_t height, double min_z, double height_scale, int8_t min_val)
{
  return static_cast<double>(height - min_val) * height_scale + min_z;
}

void HeightGridMap::setHeight(double x, double y, double height)
{
  int idx = 0;
  if (getGridMapIndex(x, y, idx))
    grid_map->data.at(idx) = heightToMap(height, min.z, inv_height_scale);
}

void HeightGridMap::setHeight(int map_x, int map_y, double height)
{
  int idx = 0;
  if (getGridMapIndex(map_x, map_y, idx))
    grid_map->data.at(idx) = heightToMap(height, min.z, inv_height_scale);
}

void HeightGridMap::updateHeight(double x, double y, double height)
{
  int idx = 0;
  if (getGridMapIndex(x, y, idx))
  {
    int8_t h = heightToMap(height, min.z, inv_height_scale);
    grid_map->data.at(idx) += update_weight*static_cast<double>(h-grid_map->data.at(idx));
  }
}

void HeightGridMap::updateHeight(int map_x, int map_y, double height)
{
  int idx = 0;
  if (getGridMapIndex(map_x, map_y, idx))
  {
    int8_t h = heightToMap(height, min.z, inv_height_scale);
    grid_map->data.at(idx) += update_weight*static_cast<double>(h-grid_map->data.at(idx));
  }
}

bool HeightGridMap::getHeight(double x, double y, double& height) const
{
  int idx = 0;
  if (getGridMapIndex(x, y, idx))
  {
    const int8_t& h = grid_map->data.at(idx);
    if (h != std::numeric_limits<int8_t>::max()-std::numeric_limits<int8_t>::min())
    {
      height = heightToWorld(h, min.z, height_scale);
      return true;
    }
  }

  return false;
}

bool HeightGridMap::getHeight(int map_x, int map_y, double& height) const
{
  int idx = 0;
  if (getGridMapIndex(map_x, map_y, idx))
  {
    const int8_t& h = grid_map->data.at(idx);
    if (h != std::numeric_limits<int8_t>::max()-std::numeric_limits<int8_t>::min())
    {
      height = heightToWorld(h, min.z, height_scale);
      return true;
    }
  }

  return false;
}
}
