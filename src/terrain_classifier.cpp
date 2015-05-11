#include <vigir_terrain_classifier/terrain_classifier.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>

//#include <pcl/surface/mls_omp.h>
#include <pcl/surface/mls.h>

#include <pcl/surface/gp3.h>

#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <vigir_terrain_classifier/pcl/octree_voxel_grid.h>
#include <vigir_terrain_classifier/pcl/point_cloud_filter.h>

#include <vigir_footstep_planning_lib/helper.h>



namespace vigir_terrain_classifier
{
TerrainClassifier::TerrainClassifier(ros::NodeHandle& nh, const TerrainClassifierParams& params)
{
  setParams(params);

  // start service clients
  generate_feet_pose_client = nh.serviceClient<vigir_footstep_planning_msgs::GenerateFeetPoseService>("/vigir/footstep_planning/generate_feet_pose");
}

TerrainClassifier::TerrainClassifier(ros::NodeHandle& nh)
{
  setParams(TerrainClassifierParams(nh));

  // start service clients
  generate_feet_pose_client = nh.serviceClient<vigir_footstep_planning_msgs::GenerateFeetPoseService>("/vigir/footstep_planning/generate_feet_pose");
}

TerrainClassifier::~TerrainClassifier()
{
}

void TerrainClassifier::reset()
{
  // clear old data
  cloud_processed.reset(new pcl::PointCloud<pcl::PointXYZ>());

  input_octree.reset(new OctreeVoxelGrid<pcl::PointXYZ>(params.aggregation_res, params.update_weight));
  normals_octree.reset(new OctreeVoxelGrid<pcl::PointNormal>(params.normals_res, params.update_weight));

  // init terrain model
  HeightGridMap::Ptr height_grid_map(new HeightGridMap(params.world_frame_id, params.hgm_res, params.hgm_min_expansion_size, params.hgm_update_weight));
  terrain_model.reset(new TerrainModel(params.world_frame_id, params.hgm_res, height_grid_map));

  // init ground level grid map generator

  // init height grid map generator
  height_grid_map_generator.reset(new HeightGridMapGenerator(terrain_model->getHeightGridMap()));
  height_grid_map_generator->setExternalInputOctree(input_octree);

  setDataOutdated();
}

void TerrainClassifier::setParams(const TerrainClassifierParams& params)
{
  this->params = params;

  if (params.filter_mask & FILTER_PASS_THROUGH_BOX)       // cut out area of interest
    ROS_INFO("Using FILTER_PASS_THROUGH_BOX");
  if (params.filter_mask & FILTER_PASS_THROUGH_ELLIPSE)   // cut out area of interest
    ROS_INFO("Using FILTER_PASS_THROUGH_ELLIPSE");
  if (params.filter_mask & FILTER_STATISTICAL_OUTLIER)    // remove outliers
    ROS_INFO("Using FILTER_STATISTICAL_OUTLIER");
  if (params.filter_mask & FILTER_VOXEL_GRID)             // summarize data
    ROS_INFO("Using FILTER_VOXEL_GRID");
  if (params.filter_mask & FILTER_MLS_SMOOTH)             // smooth data
    ROS_INFO("Using FILTER_MLS_SMOOTH");

  reset();
}

void TerrainClassifier::filterPointCloudData(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  // get currend position on ground
  geometry_msgs::Pose pose;
  determineCurrentPose(pose);

  if (params.filter_mask & FILTER_PASS_THROUGH_BOX)       // cut out area of interest
    filterPassThroughBox<pcl::PointXYZ>(cloud, params.pt_field_name, pose.position.z + params.pt_min, pose.position.z + params.pt_max);
  if (params.filter_mask & FILTER_PASS_THROUGH_ELLIPSE)   // cut out area of interest
    filterPassThroughEllipse<pcl::PointXYZ>(cloud, pcl::PointXYZ(pose.position.x, pose.position.y, pose.position.z), tf::getYaw(pose.orientation), params.aggregation_rx, params.aggregation_ry);
}

void TerrainClassifier::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  // reset aggregated data
  input_octree->deleteTree();
  normals_octree->deleteTree();
  height_grid_map_generator->reset();

  insertPointCloud(cloud);
}

void TerrainClassifier::insertPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  // update oct-tree
  input_octree->insertPointCloud(cloud);

  // mark recent result as outdated
  setDataOutdated();
}

bool TerrainClassifier::hasTerrainModel() const
{
  return terrain_model->hasTerrainModel();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr TerrainClassifier::getInputCloud() const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
  input_octree->getPointCloud(*pc);
  return pc;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr& TerrainClassifier::getCloudProcessed() const
{
  return cloud_processed;
}

const pcl::PointCloud<pcl::PointNormal>::ConstPtr TerrainClassifier::getPointsWithsNormals() const
{
  if (terrain_model->getPointCloudWithNormals()->empty())
    ROS_WARN("getPointsWithsNormals was called before normals were computed!");
  return terrain_model->getPointCloudWithNormals();
}

const pcl::PointCloud<pcl::PointXYZI>::Ptr& TerrainClassifier::getGradients() const
{
  if (!cloud_gradients || cloud_gradients->empty())
    ROS_WARN("getGradient was called before gradients were computed!");
  return cloud_gradients;
}

const pcl::PointCloud<pcl::PointXYZI>::Ptr& TerrainClassifier::getEdges() const
{
  if (!cloud_edges || cloud_edges->empty())
    ROS_WARN("getEdges was called before edges were detected!");
  return cloud_edges;
}

void TerrainClassifier::getCloudProcessedLowRes(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const
{
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

  if (terrain_model->getPointCloudWithNormals()->empty())
    ROS_WARN("getCloudProcessedLow was called before normals were computed!");
  else
  {
    double half_low_res = 0.5*params.low_res;
    cloud->resize(terrain_model->getPointCloudWithNormals()->size());
    for (size_t i = 0; i < cloud->size(); i++)
    {
      cloud->at(i).x = terrain_model->getPointCloudWithNormals()->at(i).x;
      cloud->at(i).y = terrain_model->getPointCloudWithNormals()->at(i).y;
      cloud->at(i).z = terrain_model->getPointCloudWithNormals()->at(i).z - half_low_res; // reduce height so it matches vis with boxes
    }

    filterVoxelGrid<pcl::PointXYZ>(cloud, params.low_res, params.low_res, 0.01);
  }
}

void TerrainClassifier::getTerrainModel(TerrainModelMsg& model) const
{
  terrain_model->toMsg(model);
}

const nav_msgs::OccupancyGrid::Ptr& TerrainClassifier::getGroundLevelGridMap() const
{
  if (!ground_level_grid_map)
    ROS_WARN("getGroundLevelGridMap was called before gradients were computed!");
  return ground_level_grid_map;
}

nav_msgs::OccupancyGrid::Ptr TerrainClassifier::getHeightGridMap(double& height_grid_map_scale) const
{
  HeightGridMap::Ptr height_grid_map = terrain_model->getHeightGridMap();
  HeightGridMapMsg msg;
  height_grid_map->toMsg(msg);
  nav_msgs::OccupancyGrid::Ptr map(new nav_msgs::OccupancyGrid(msg.grid_map));
  height_grid_map_scale = msg.height_scale;
  return map;
}

nav_msgs::OccupancyGrid::Ptr TerrainClassifier::getHeightGridMapRescaled(int8_t min_val, int8_t max_val) const
{
  if (max_val <= min_val)
    return nav_msgs::OccupancyGrid::Ptr();

  HeightGridMap::Ptr height_grid_map = terrain_model->getHeightGridMap();

  double height_grid_map_scale;
  nav_msgs::OccupancyGrid::Ptr map = getHeightGridMap(height_grid_map_scale);

  const geometry_msgs::Vector3& min = height_grid_map->getMin();
  const geometry_msgs::Vector3& max = height_grid_map->getMax();

  // rescale
  for (nav_msgs::OccupancyGrid::_data_type::iterator itr = map->data.begin(); itr != map->data.end(); itr++)
  {
    int8_t& val = *itr;
    if (val == GRID_MAP_EMPTY_VAL)
    {
      val = min_val;
      continue;
    }
    val = HeightGridMap::heightToMap(HeightGridMap::heightToWorld(val, min.z, height_grid_map_scale), min.z, max.z, min_val, max_val);
  }

  return map;
}

const pcl::PolygonMesh::Ptr& TerrainClassifier::getMeshSurface() const
{
  if (!mesh_surface || mesh_surface->polygons.empty())
    ROS_WARN("getSurfaceMesh was called before surface is reconstructed!");
  return mesh_surface;
}

bool TerrainClassifier::computeNormals(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr update)
{
  // normals are up-to-date -> do nothing
  if (!cloud_normals_outdated)
    return true;

  pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_with_normals = terrain_model->getPointCloudWithNormals();

  // determine search points
  pcl::PointCloud<pcl::PointXYZ>::Ptr search_points;
  if (update)
  {
    if (update->empty())
      return false;
    search_points.reset(new pcl::PointCloud<pcl::PointXYZ>(*update));
  }
  else
    search_points = getInputCloud();

  // reduce search points to a minimum
  if (params.filter_mask & FILTER_VOXEL_GRID)
    filterVoxelGrid<pcl::PointXYZ>(search_points, params.vg_lx, params.vg_ly, params.vg_lz);

  std::vector<float> k_sqr_distances;
  input_octree->radiusSearch(*search_points, 1.2*params.ne_radius, *cloud_processed, k_sqr_distances);

  // preprocessing of search surface
  if (params.filter_mask & FILTER_MLS_SMOOTH)             // smooth data
    filterMlsSmooth<pcl::PointXYZ>(cloud_processed, params.ms_radius);

  if (cloud_processed->empty())
    return false;

  // init normals data structure
  pcl::PointCloud<pcl::Normal> cloud_normals;

  // compute normals
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne(params.threads);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setInputCloud(search_points);
  ne.setSearchSurface(cloud_processed);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(params.ne_radius);
  //ne.setKSearch(20);
  ne.compute(cloud_normals);

  // concat resulting normals with their positions
  pcl::PointCloud<pcl::PointNormal>::Ptr new_points_with_normals(new pcl::PointCloud<pcl::PointNormal>());
  pcl::concatenateFields(*search_points, cloud_normals, *new_points_with_normals);

  // postprocessing
  if (params.filter_mask & FILTER_STATISTICAL_OUTLIER)    // remove outliers
    filterStatisticalOutlier<pcl::PointNormal>(new_points_with_normals, params.so_k, params.so_radius);

  // filter and some further postprocessing
  std::vector<int> indices;
  for (size_t i = 0; i < new_points_with_normals->size(); i++)
  {
    pcl::PointNormal& n = new_points_with_normals->at(i);
    if (!pcl_isfinite(n.normal_z))
      continue;

    indices.push_back((int)i);

    // flip all other normals in one direction
    if (n.normal_z < 0.0)
    {
      n.normal_x = -n.normal_x;
      n.normal_y = -n.normal_y;
      n.normal_z = -n.normal_z;
    }
  }

  // apply filtering
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_filtered(new pcl::PointCloud<pcl::PointNormal>());
  copyPointCloud(*new_points_with_normals, indices, *normals_filtered);

  // update oct-tree
  normals_octree->insertPointCloud(normals_filtered);
  normals_octree->getPointCloud(*point_cloud_with_normals);

  cloud_normals_outdated = false;
  return true;
}

bool TerrainClassifier::computeGradients(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr update)
{
  // gradients are up-to-date -> do nothing
  if (!cloud_gradients_outdated && cloud_gradients)
    return true;

  if (!computeNormals(update))
  {
    ROS_ERROR("Can't generate gradients!");
    return false;
  }

  const pcl::PointCloud<pcl::PointNormal>::ConstPtr point_cloud_with_normals = terrain_model->getPointCloudWithNormals();

  // init gradient data structure
  cloud_gradients.reset(new pcl::PointCloud<pcl::PointXYZI>());
  cloud_gradients->resize(point_cloud_with_normals->size());

  for (unsigned int i = 0; i < point_cloud_with_normals->size(); i++)
  {
    const pcl::PointNormal& pn = point_cloud_with_normals->at(i);
    pcl::PointXYZI& pi = cloud_gradients->at(i);

    pi.x = pn.x;
    pi.y = pn.y;
    pi.z = pn.z;

    pi.intensity = sqrt(pn.normal_x*pn.normal_x + pn.normal_y*pn.normal_y); // = sqrt(1 - n.normal_z*n.normal_z)
    pi.intensity = pi.intensity < params.ge_thresh ? 1.0 : 0.0;
  }

// do voxelizing only after gradient estimation?
//  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
//  pcl::VoxelGrid<pcl::PointXYZI> vox;
//  vox.setInputCloud(cloud_gradient);
//  vox.setLeafSize(0.01f, 0.01f, 10.0f);
//  vox.filter(*cloud_filtered);
//  cloud_gradient = cloud_filtered;

  cloud_gradients_outdated = false;
  return true;
}

bool TerrainClassifier::detectEdges()
{
  // edges are up-to-date -> do nothing
  if (!cloud_edges_outdated && cloud_edges)
    return true;

  if (!computeNormals())
  {
    ROS_ERROR("Can't run edge detection!");
    return false;
  }

  const pcl::PointCloud<pcl::PointNormal>::ConstPtr point_cloud_with_normals = terrain_model->getPointCloudWithNormals();

  // init edge data structure
  cloud_edges.reset(new pcl::PointCloud<pcl::PointXYZI>());
  cloud_edges->resize(point_cloud_with_normals->size());

  // project all data to plane
  pcl::PointCloud<pcl::PointXY>::Ptr points(new pcl::PointCloud<pcl::PointXY>());
  points->resize(point_cloud_with_normals->size());
  for (unsigned int i = 0; i < point_cloud_with_normals->size(); i++)
  {
    const pcl::PointNormal& n = point_cloud_with_normals->at(i);
    pcl::PointXY& p = points->at(i);
    p.x = n.x;
    p.y = n.y;
  }

  pcl::KdTreeFLANN<pcl::PointXY> tree;
  tree.setInputCloud(points);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_edges(new pcl::PointCloud<pcl::PointXYZI>());
  tmp_edges->resize(point_cloud_with_normals->size());

  // run edge detection
  for (size_t i = 0; i < points->size(); i++)
  {
    const pcl::PointNormal& current = point_cloud_with_normals->at(i);
    pcl::PointXYZI& result = tmp_edges->at(i);

    result.x = current.x;
    result.y = current.y;
    result.z = current.z;
    result.intensity = 0.0;

    if (tree.radiusSearch(i, params.ed_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      // determine squared mean error
      double sq_sum_e = 0.0;

      for (size_t j = 0; j < pointIdxRadiusSearch.size (); j++)
      {
        if (pointIdxRadiusSearch[j] == (int)i)
          continue;

        const pcl::PointNormal& neigh = point_cloud_with_normals->at(pointIdxRadiusSearch[j]);

        // determine diff of normals
        double diff_nx = (neigh.normal_x - current.normal_x);
        double diff_ny = (neigh.normal_y - current.normal_y);
        double sq_err_nx = diff_nx*diff_nx;
        double sq_err_ny = diff_ny*diff_ny;

        // determine diff in height
        double diff_z = (neigh.z - current.z)*5.0; // scale up diff (weight)
        double sq_err_z = diff_z*diff_z;

        // compute support in direction of normal
        double neigh_norm = sqrt((current.normal_x*current.normal_x + current.normal_y*current.normal_y) *
                                 ((neigh.x-current.x)*(neigh.x-current.x) + (neigh.y-current.y)*(neigh.y-current.y)));
        double dot_scale = std::abs(current.normal_x * std::abs(neigh.x-current.x) + current.normal_y * std::abs(neigh.y-current.y)) / neigh_norm;

        // compute support in distance
        double dist_scale = (params.ed_radius-sqrt(pointRadiusSquaredDistance[j]))/params.ed_radius;

        sq_sum_e += (sq_err_nx + sq_err_ny + sq_err_z) * dot_scale * dist_scale;
      }

      double sq_mean_e = sq_sum_e/pointIdxRadiusSearch.size();

      // check for edge
      if (sq_mean_e > params.ed_max_std*params.ed_max_std)
        result.intensity = sq_mean_e;
    }
  }

  // run non-maximum suppression
  for (size_t i = 0; i < points->size(); i++)
  {
    const pcl::PointNormal& current = point_cloud_with_normals->at(i);
    const pcl::PointXYZI& edge = tmp_edges->at(i);
    pcl::PointXYZI& result = cloud_edges->at(i);

    result.x = edge.x;
    result.y = edge.y;
    result.z = edge.z;
    result.intensity = 1.0; // = no edge

    if (edge.intensity == 0.0)
      continue;

    if (tree.radiusSearch(i, params.ed_non_max_supp_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      bool local_max = true;

      // determine local max
      for (size_t j = 0; j < pointIdxRadiusSearch.size (); j++)
      {
        if (pointIdxRadiusSearch[j] == (int)i)
          continue;

        const pcl::PointXYZI& neigh = tmp_edges->at(pointIdxRadiusSearch[j]);

        /// @TODO: Handle 0/0/1 normals
        double neigh_norm = sqrt((current.normal_x*current.normal_x + current.normal_y*current.normal_y) *
                                 ((neigh.x-current.x)*(neigh.x-current.x) + (neigh.y-current.y)*(neigh.y-current.y)));
        double dot_scale = std::abs(current.normal_x * std::abs(neigh.x-current.x) + current.normal_y * std::abs(neigh.y-current.y)) / neigh_norm;

        // check for local maximum
        if (edge.intensity < neigh.intensity * dot_scale)
        {
          local_max = false;
          break;
        }
      }

      // check for edge
      if (local_max)
        result.intensity = 0.0;
    }
  }

  cloud_edges_outdated = false;
  return true;
}

bool TerrainClassifier::generateGroundLevelGridmap()
{

  // generate new grid map only if needed
  if (!ground_level_grid_map_outdated && ground_level_grid_map)
    return true;

  if ((!cloud_gradients || !computeGradients()) && (!cloud_edges || !detectEdges()))
  {
    ROS_ERROR("Can't generate grid map!");
    return false;
  }

  // determine min and max coordinatesss
  double min_x, max_x;
  double min_y, max_y;

  min_x = min_y = FLT_MAX;
  max_x = max_y = FLT_MIN;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  if (cloud_gradients)
    cloud = cloud_gradients;
  else if (cloud_edges)
    cloud = cloud_edges;

  for (size_t i = 0; i < cloud->size(); i++)
  {
    const pcl::PointXYZI& p = cloud->at(i);
    min_x = std::min(min_x, (double)p.x);
    min_y = std::min(min_y, (double)p.y);
    max_x = std::max(max_x, (double)p.x);
    max_y = std::max(max_y, (double)p.y);
  }

  // setting up grid map
  ground_level_grid_map.reset(new nav_msgs::OccupancyGrid());

  ground_level_grid_map->info.resolution = params.gg_res;
  ground_level_grid_map->info.width = ceil((max_x-min_x) * 1/params.gg_res + 1);
  ground_level_grid_map->info.height = ceil((max_y-min_y) * 1/params.gg_res + 1);

  ground_level_grid_map->info.origin.position.x = min_x;
  ground_level_grid_map->info.origin.position.y = min_y;
  ground_level_grid_map->info.origin.position.z = 0.0;

  ground_level_grid_map->data.clear();
  ground_level_grid_map->data.resize(ground_level_grid_map->info.width * ground_level_grid_map->info.height, -1);

  // THIS IS A HACK, REMOVED IT SOON AS POSSIBLE
//  bool ignore_near = false;
//  double robot_x = 0.0;
//  double robot_y = 0.0;
//  if (lower_body_state)
//  {
//    robot_x = lower_body_state->pelvis_pose.position.x;
//    robot_y = lower_body_state->pelvis_pose.position.y;
//    ignore_near = true;
//  }

  // add data from gradients point cloud
  if (cloud_gradients)
  {
    ROS_INFO("...adding gradients");
    for (size_t i = 0; i < cloud_gradients->size(); i++)
    {
      const pcl::PointXYZI& p = cloud_gradients->at(i);

//      if (ignore_near)
//      {
//        if ((p.x-robot_x)*(p.x-robot_x) + (p.y-robot_y)*(p.y-robot_y) < 0.4*0.4)
//          continue;
//      }

      int idx = 0;
      if (GridMap::getGridMapIndex(*ground_level_grid_map, p.x, p.y, idx))
        ground_level_grid_map->data.at(idx) = std::max(ground_level_grid_map->data.at(idx), (int8_t)floor((1.0-p.intensity) * 100));
    }
  }

  // add data from edge point cloud
  if (cloud_edges)
  {
    ROS_INFO("...adding edges");
    ROS_ASSERT(cloud_gradients->size() == cloud_edges->size());

    for (size_t i = 0; i < cloud_edges->size(); i++)
    {
      const pcl::PointXYZI& p = cloud_edges->at(i);

//      if (ignore_near)
//      {
//        if ((p.x-robot_x)*(p.x-robot_x) + (p.y-robot_y)*(p.y-robot_y) < 0.4*0.4)
//          continue;
//      }

      int idx = 0;
      if (GridMap::getGridMapIndex(*ground_level_grid_map, p.x, p.y, idx))
        ground_level_grid_map->data.at(idx) = std::max(ground_level_grid_map->data.at(idx), (int8_t)floor((1.0-p.intensity) * 100));
    }
  }

  ground_level_grid_map->header.stamp = ros::Time::now();
  ground_level_grid_map->header.frame_id = params.world_frame_id;

  ground_level_grid_map_outdated = false;
  return true;
}

bool TerrainClassifier::generateHeightGridmap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr update)
{
  if (update)
  {
    if (update->empty())
      return false;
    height_grid_map_generator->update(update);
  }
  else if (height_grid_grid_map_outdated)
    height_grid_map_generator->update(getInputCloud());

  height_grid_grid_map_outdated = false;
  return true;
}

bool TerrainClassifier::computeSurfaceMesh()
{
  // mesh is up-to-date -> do nothing
  if (!mesh_surface_outdated && mesh_surface)
    return true;

  if (!computeNormals())
  {
    ROS_ERROR("Can't run surface reconstruction!");
    return false;
  }

  const pcl::PointCloud<pcl::PointNormal>::ConstPtr point_cloud_with_normals = terrain_model->getPointCloudWithNormals();

  // init mesh data structure
  mesh_surface.reset(new pcl::PolygonMesh());

  // Create search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud(point_cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(0.10);

  // Set typical values for the parameters
  gp3.setMu(6.0);
  gp3.setMaximumNearestNeighbors(50);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud(point_cloud_with_normals);
  gp3.setSearchMethod(tree);
  gp3.reconstruct(*mesh_surface);

  // Additional vertex information
//  std::vector<int> parts = gp3.getPartIDs();
//  std::vector<int> states = gp3.getPointStates();

  mesh_surface_outdated = false;
  return true;
}

void TerrainClassifier::setDataOutdated()
{
  cloud_normals_outdated = true;
  cloud_gradients_outdated = true;
  cloud_edges_outdated = true;
  ground_level_grid_map_outdated = true;
  height_grid_grid_map_outdated = true;
  mesh_surface_outdated = true;
}

bool TerrainClassifier::determineCurrentPose(geometry_msgs::Pose& pose)
{
  //pose.position.x = 0.0; pose.position.y = 0.0; pose.position.z = 0.0;
  //pose.orientation = tf::createQuaternionMsgFromYaw(-1.95536);
  //return true; /// TODO: TEST

  // get current feet pose
  std_msgs::Header header;
  header.frame_id = params.world_frame_id;
  header.stamp = ros::Time::now();
  vigir_footstep_planning_msgs::Feet feet_pose;
  vigir_footstep_planning_msgs::ErrorStatus status = vigir_footstep_planning::determineStartFeetPose(feet_pose, generate_feet_pose_client, header);

  // get position
  if (!vigir_footstep_planning::hasError(status))
  {
    pose.position.x = 0.5*(feet_pose.left.pose.position.x + feet_pose.right.pose.position.x);
    pose.position.y = 0.5*(feet_pose.left.pose.position.y + feet_pose.right.pose.position.y);
    pose.position.z = 0.5*(feet_pose.left.pose.position.z + feet_pose.right.pose.position.z);
    pose.orientation = tf::createQuaternionMsgFromYaw(0.5*(tf::getYaw(feet_pose.left.pose.orientation) + tf::getYaw(feet_pose.right.pose.orientation)));
    return true;
  }
  else
  {
    ROS_WARN_THROTTLE(10.0, "determineCurrentPosition: No state estimation of feet available. Defaulting to origin!");
    pose = geometry_msgs::Pose();
    pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    return false;
  }
}

double TerrainClassifier::determineCurrentGroundHeight()
{
  geometry_msgs::Pose pose;
  if (determineCurrentPose(pose))
    return pose.position.z;

  return 0.0;
}
}
