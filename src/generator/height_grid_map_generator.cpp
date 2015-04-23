#include <vigir_terrain_classifier/generator/height_grid_map_generator.h>

namespace vigir_terrain_classifier
{
HeightGridMapGenerator::HeightGridMapGenerator(HeightGridMap::Ptr grid_map)
  : GridMapGenerator(grid_map)
{
  this->grid_map = boost::dynamic_pointer_cast<HeightGridMap>(GridMapGenerator::grid_map);

  input_octree.reset(new OctreeVoxelGrid<pcl::PointXYZ>(grid_map->map()->info.resolution, grid_map->getUpdateWeight()));
}

HeightGridMapGenerator::~HeightGridMapGenerator()
{
}

void HeightGridMapGenerator::reset()
{
  GridMapGenerator::reset();
}

void HeightGridMapGenerator::setGridMap(GridMap::Ptr grid_map)
{
  GridMapGenerator::setGridMap(grid_map);
  this->grid_map = boost::dynamic_pointer_cast<HeightGridMap>(GridMapGenerator::grid_map);
}

HeightGridMap::ConstPtr HeightGridMapGenerator::getGridMap() const
{
  return this->grid_map;
}

HeightGridMap::Ptr HeightGridMapGenerator::getGridMap()
{
  return this->grid_map;
}

void HeightGridMapGenerator::setExternalInputOctree(const OctreeVoxelGrid<pcl::PointXYZ>::ConstPtr& octree)
{
  external_input_octree = octree;
}

void HeightGridMapGenerator::update(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  if (cloud->empty())
    return;

  GridMapGenerator::update(cloud);

  // update own octree if no external is available
  if (!external_input_octree && input_octree)
    input_octree->insertPointCloud(cloud);

//  const OctreeVoxelGrid<pcl::PointXYZ>::ConstPtr& octree = external_input_octree ? external_input_octree : input_octree;

//  pcl::PointCloud<pcl::PointXYZ>::VectorType points;
//  octree->getVoxelCentroids(points);

//  // fill missing data
//  pcl::KdTreeFLANN<pcl::PointNormal> tree;
//  tree.setInputCloud(cloud_points_with_normals);
//  pcl::PointNormal current;
//  current.z = ground_z;

//  unsigned int k = 100;
//  std::vector<int> pointIdxNKNSearch;
//  std::vector<float> pointNKNSquaredDistance;
//  pcl::PointNormal result;

//  pointIdxNKNSearch.resize(k);
//  pointNKNSquaredDistance.resize(k);

//  int8_t ground_level_height = (int8_t)floor((ground_z-min.z) * height_scale_inv)-127;

//  for (size_t i = 0; i < height_grid_map.data.size(); i++)
//  {
//    int8_t& height = height_grid_map.data.at(i);
//    if (height == -128)
//    {
//      if (params.gg_reconstruct)
//      {
//        getGridMapCoords(height_grid_map, i, current.x, current.y);

//        // find k nearest neighbour and use their z
//        if (tree.nearestKSearch(current, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
//        {
//          double z = 0.0;
//          float total_dist = 0.0;

//          for (size_t n = 0; n < k; n++)
//            total_dist += pointNKNSquaredDistance[n];

//          for (size_t n = 0; n < k; n++)
//          {
//            result = cloud_points_with_normals->points[pointIdxNKNSearch[n]];
//            z += result.z * (1-pointNKNSquaredDistance[n]/total_dist);
//          }
//          z /= k;

//          height = (int8_t)floor((z-min.z) * height_scale_inv)-127;

//          result.x = current.x;
//          result.y = current.y;
//          result.z = z;
//          cloud_points_with_normals->push_back(result);
//        }
//      }
//      else
//        height = ground_level_height;
//    }
//  }

  grid_map->map()->header.stamp = ros::Time::now();
}

void HeightGridMapGenerator::update(const pcl::PointXYZ& point)
{
  // adding height information from point
  grid_map->updateHeight(point.x, point.y, point.z);
}
}
