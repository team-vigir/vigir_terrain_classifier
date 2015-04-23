#include <vigir_terrain_classifier/terrain_classifier_vis.h>

namespace vigir_terrain_classifier
{
void showNormals(const TerrainClassifier::ConstPtr& terrain_classifier, pcl::visualization::PCLVisualizer& viewer, const std::string& name, int viewport)
{
  pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_points_with_normals = terrain_classifier->getPointsWithsNormals();

  if (!cloud_points_with_normals || cloud_points_with_normals->empty())
  {
    ROS_WARN("showNormals was called before normals were computed!");
    return;
  }

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> random_color_handler(cloud_points_with_normals, 0.0, 200.0, 200.0);
  viewer.addPointCloud(cloud_points_with_normals, random_color_handler, name + std::string("_cloud"), viewport);
  viewer.addPointCloudNormals<pcl::PointNormal>(cloud_points_with_normals, 30, 0.1, name + std::string("_normals"), viewport);
}

void showGradients(const TerrainClassifier::ConstPtr& terrain_classifier, pcl::visualization::PCLVisualizer& viewer, const std::string& name, int viewport)
{
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_gradients = terrain_classifier->getGradients();

  if (!cloud_gradients || cloud_gradients->empty())
  {
    ROS_WARN("showGradients was called before normals were computed!");
    return;
  }

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_gradients, "intensity");
  viewer.addPointCloud<pcl::PointXYZI>(cloud_gradients, intensity_distribution, name + std::string("_cloud"), viewport);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name + std::string("_gradient"), viewport);
}

void showEdges(const TerrainClassifier::ConstPtr& terrain_classifier, pcl::visualization::PCLVisualizer& viewer, const std::string& name, int viewport)
{
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_edges = terrain_classifier->getEdges();

  if (!cloud_edges || cloud_edges->empty())
  {
    ROS_WARN("showEdges was called before edges were detected!");
    return;
  }

  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud_edges, "intensity");
  viewer.addPointCloud<pcl::PointXYZI>(cloud_edges, intensity_distribution, name + std::string("_cloud"), viewport);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name + std::string("_edges"), viewport);
}
}
