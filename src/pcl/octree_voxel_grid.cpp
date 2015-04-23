#include <vigir_terrain_classifier/pcl/octree_voxel_grid.h>

namespace vigir_terrain_classifier
{
template<>
void OctreeCentroidContainer<pcl::PointNormal>::addPoint(const pcl::PointNormal& new_point, double update_weight)
{
  if (!updated_once)
  {
    update_weight = 1.0;
    updated_once = true;
  }

  point.x += update_weight * (new_point.x - point.x);
  point.y += update_weight * (new_point.y - point.y);
  point.z += update_weight * (new_point.z - point.z);

  point.normal_x += update_weight * (new_point.normal_x - point.normal_x);
  point.normal_y += update_weight * (new_point.normal_y - point.normal_y);
  point.normal_z += update_weight * (new_point.normal_z - point.normal_z);
}

template<>
void OctreeCentroidContainer<pcl::PointNormal>::reset()
{
  point.x = point.y = point.z = 0;
  point.normal_x = point.normal_y = point.normal_z = 0;
  updated_once = false;
}
}
