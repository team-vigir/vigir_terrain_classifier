#include <vigir_terrain_classifier/generator/grid_map_generator.h>

namespace vigir_terrain_classifier
{
GridMapGenerator::GridMapGenerator(GridMap::Ptr grid_map)
{
  this->grid_map = grid_map;
}

GridMapGenerator::~GridMapGenerator()
{
}

void GridMapGenerator::reset()
{
  grid_map->clear();
}

void GridMapGenerator::setGridMap(GridMap::Ptr grid_map)
{
  this->grid_map = grid_map;
}

GridMap::ConstPtr GridMapGenerator::getGridMap() const
{
  return this->grid_map;
}

GridMap::Ptr GridMapGenerator::getGridMap()
{
  return this->grid_map;
}

bool GridMapGenerator::isMapAvailable() const
{
  return !grid_map->empty();
}

void GridMapGenerator::update(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  if (cloud->empty())
    return;

  grid_map->resize(cloud);

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator itr = cloud->begin(); itr != cloud->end(); itr++)
    update(*itr);

  grid_map->map()->header.stamp = ros::Time::now();
  grid_map->map()->header.seq++;
}

void GridMapGenerator::update(const sensor_msgs::PointCloud2& cloud)
{
  std::string cloud_frame_id = vigir_footstep_planning::strip_const(cloud.header.frame_id, '/');
  if (grid_map->map()->header.frame_id != cloud_frame_id)
  {
    ROS_ERROR_THROTTLE(5.0, "[GridMap] update: Frame of input point ('%s') cloud mismatch! Should be '%s'.", cloud_frame_id.c_str(), grid_map->map()->header.frame_id.c_str());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud, *point_cloud);
  this->update(point_cloud);
}
}
