#include <vigir_terrain_classifier/terrain_model.h>

namespace vigir_terrain_classifier
{
TerrainModel::TerrainModel(const std::string& frame_id, double resolution, HeightGridMap::Ptr height_grid_map)
  : frame_id(frame_id)
  , resolution(resolution)
  , height_grid_map(height_grid_map)
{
  point_cloud_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>());
  points_with_normals_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointNormal>());
}

TerrainModel::TerrainModel(const TerrainModelMsg& terrain_model)
{
  point_cloud_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>());
  points_with_normals_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointNormal>());

  fromMsg(terrain_model);
}

TerrainModel::~TerrainModel()
{
}

void TerrainModel::reset()
{
  point_cloud_with_normals.reset(new pcl::PointCloud<pcl::PointNormal>());
  points_with_normals_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointNormal>());

  if (ground_level_grid_map)
    ground_level_grid_map->clear();
  if (height_grid_map)
    height_grid_map->clear();
}

void TerrainModel::fromMsg(const TerrainModelMsg& terrain_model)
{
  frame_id = terrain_model.header.frame_id;
  resolution = terrain_model.resolution;

  pcl::fromROSMsg(terrain_model.points_with_normals, *point_cloud_with_normals);

  //ground_level_grid_map->fromMsg(terrain_model.ground_level_map);

  if (height_grid_map)
    height_grid_map->fromMsg(terrain_model.height_grid_map);
  else
    height_grid_map.reset(new HeightGridMap(terrain_model.height_grid_map));

  updateSearchTree();
}

void TerrainModel::toMsg(TerrainModelMsg& terrain_model) const
{
  terrain_model.header.frame_id = frame_id;
  terrain_model.header.stamp = ros::Time::now();
  terrain_model.resolution = resolution;

  pcl::toROSMsg(*point_cloud_with_normals, terrain_model.points_with_normals);

  //ground_level_grid_map->toMsg(terrain_model.ground_level_map);

  if (height_grid_map)
    height_grid_map->toMsg(terrain_model.height_grid_map);
}

double TerrainModel::getResolution() const
{
  return resolution;
}

void TerrainModel::updateSearchTree()
{
  // generate kd-tree
  points_with_normals_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointNormal>());
  points_with_normals_kdtree->setInputCloud(point_cloud_with_normals);
}

bool TerrainModel::hasTerrainModel() const
{
  return !height_grid_map->empty() && !point_cloud_with_normals->empty();
}

const pcl::PointCloud<pcl::PointNormal>::ConstPtr TerrainModel::getPointCloudWithNormals() const
{
  return point_cloud_with_normals;
}

pcl::PointCloud<pcl::PointNormal>::Ptr TerrainModel::getPointCloudWithNormals()
{
  return point_cloud_with_normals;
}

const HeightGridMap::ConstPtr TerrainModel::getHeightGridMap() const
{
  return height_grid_map;
}

HeightGridMap::Ptr TerrainModel::getHeightGridMap()
{
  return height_grid_map;
}

bool TerrainModel::getPointWithNormal(const pcl::PointNormal& p_search, pcl::PointNormal& p_result) const
{
  if (!points_with_normals_kdtree || points_with_normals_kdtree->getInputCloud()->size() == 0)
    return false;

  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;

//  if (points_with_normals_kdtree->nearestKSearch(p_search, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
//  {
//    const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud = points_with_normals_kdtree->getInputCloud();

//    if (pointNKNSquaredDistance[0] > resolution*resolution*2.0)
//      return false;

//    p_result = p_search;
//    p_result.normal_x = cloud->points[pointIdxNKNSearch[0]].normal_x;
//    p_result.normal_y = cloud->points[pointIdxNKNSearch[0]].normal_y;
//    p_result.normal_z = cloud->points[pointIdxNKNSearch[0]].normal_z;
//    return true;
//  }

  double search_radius = resolution*3.0;
  double search_radius_sq = search_radius*search_radius;
  if (points_with_normals_kdtree->radiusSearch(p_search, search_radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
    // estimate weighted mean normal
    const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud = points_with_normals_kdtree->getInputCloud();

    double normal_x = 0.0;
    double normal_y = 0.0;
    double normal_z = 0.0;

    double total_weights = 0.0;

    for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
    {
      double weight = search_radius_sq - pointNKNSquaredDistance[i];
      total_weights += weight;

      normal_x += cloud->points[pointIdxNKNSearch[i]].normal_x * weight;
      normal_y += cloud->points[pointIdxNKNSearch[i]].normal_y * weight;
      normal_z += cloud->points[pointIdxNKNSearch[i]].normal_z * weight;
    }

    p_result = p_search;
    p_result.normal_x = normal_x / total_weights;
    p_result.normal_y = normal_y / total_weights;
    p_result.normal_z = normal_z / total_weights;

    return true;
  }

  return false;
}

bool TerrainModel::getHeight(double x, double y, double& height) const
{
  if (!height_grid_map)
    return false;

  return height_grid_map->getHeight(x, y, height);
}

bool TerrainModel::update3DData(geometry_msgs::Pose& p) const
{
  bool result = true;

  // get z
  if (!getHeight(p.position.x, p.position.y, p.position.z))
  {
    //ROS_WARN_THROTTLE(1.0, "No height data found at %f/%f", p.position.x, p.position.y);
    result = false;
  }

  // get roll and pitch
  pcl::PointNormal p_n;
  vigir_footstep_planning::copyPosition(p.position, p_n);

  if (!getPointWithNormal(p_n, p_n))
  {
    //ROS_WARN_THROTTLE(1.0, "No normal data found at %f/%f", p.position.x, p.position.y);
    result = false;
  }
  else
  {
    geometry_msgs::Vector3 n;
    n.x = p_n.normal_x;
    n.y = p_n.normal_y;
    n.z = p_n.normal_z;

    vigir_footstep_planning::normalToQuaternion(n, tf::getYaw(p.orientation), p.orientation);
  }

  return result;
}
}
