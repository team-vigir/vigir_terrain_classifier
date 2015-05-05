#include <vigir_terrain_classifier/terrain_classifier_node.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vigir_footstep_planning_lib/helper.h>
#include <vigir_footstep_planning_lib/math.h>

#include <vigir_terrain_classifier/pcl/point_cloud_filter.h>



namespace vigir_terrain_classifier
{
TerrainClassifierNode::TerrainClassifierNode(ros::NodeHandle& nh)
  : terrain_classifier(nh)
{
  double publish_rate;

  // load parameters
  nh.param("min_aggregation_size", (int&) min_aggregation_size, 10000);
  nh.param("publish_rate", publish_rate, 1.0);
  nh.param("compute_update_skips", (int&) compute_update_skips, 0);
  compute_update_skips_counter = compute_update_skips; // allows to do first compution without waiting

  // subscribe topics
  reset_terrain_model_sub = nh.subscribe<std_msgs::Empty>("reset", 1, &TerrainClassifierNode::reset, this);
  sys_command_sub = nh.subscribe("/syscommand", 1, &TerrainClassifierNode::sysCommandCallback, this);
  set_point_cloud_sub = nh.subscribe("set_point_cloud", 1, &TerrainClassifierNode::setPointCloud, this);
  point_cloud_update_sub = nh.subscribe("point_cloud_update", 1, &TerrainClassifierNode::insertPointCloud, this);

  // publish topics
  cloud_input_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_input", 1, true);
  cloud_points_processed_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_processed", 1);
  cloud_points_processed_low_res_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_processed_low_res", 1);
  cloud_normals_pub = nh.advertise<geometry_msgs::PoseArray>("cloud_normals", 1, true);
  cloud_gradients_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_gradients", 1);
  ground_level_grid_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("ground_level_grid_map", 1, true);
  height_grid_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("height_grid_map", 1, true);
  mesh_surface_pub = nh.advertise<pcl_msgs::PolygonMesh>("mesh_surface", 1, true);
  terrain_model_pub = nh.advertise<vigir_terrain_classifier::TerrainModelMsg>("/terrain_model", 1, true);

  // start service clients
  generate_feet_pose_client = nh.serviceClient<vigir_footstep_planning_msgs::GenerateFeetPoseService>("/vigir/footstep_planning/generate_feet_pose");

  // start own services
  generate_terrain_model_srv = nh.advertiseService("get_terrain_model", &TerrainClassifierNode::terrainModelService, this);

  publish_timer = nh.createTimer(ros::Duration(1.0/publish_rate), &TerrainClassifierNode::publishTerrainModel, this);
}

TerrainClassifierNode::~TerrainClassifierNode()
{
}

void TerrainClassifierNode::loadTestPointCloud()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  std::string path = "/home/alex/vigir/catkin_ws/src/vigir_footstep_planning/vigir_terrain_classifier/pointclouds/";

  //path += "things_on_ground_1_09_25.pcd";
  //path += "things_on_ground_2_09_25.pcd";
  //path += "things_on_ground_3.pcd";
  //path += "things_on_ground_4.pcd";
  //path += "things_on_ground_5.pcd";
  //path += "traps_ground.pcd";
  //path += "ramp.pcd";
  path += "ramp2.pcd";
  //path += "rough_terrain_2_4.pcd";
  //path += "rough_terrain_2_on_top.pcd";
  //path += "zick_zack.pcd";
  //path += "new.pcd";
  // add filtered point cloud to classifier


  ROS_INFO("Load point cloud from %s...", path.c_str());
  pcl::io::loadPCDFile(path, *point_cloud);
  ROS_INFO("Done!");

  sensor_msgs::PointCloud2 point_cloud_msg;
  pcl::toROSMsg(*point_cloud, point_cloud_msg);
  point_cloud_msg.header.frame_id = terrain_classifier.getFrameId();
  point_cloud_msg.header.stamp = ros::Time::now();

  ROS_INFO("Generating terrain model...");
  setPointCloud(point_cloud_msg);
  ROS_INFO("Done!");
}

bool TerrainClassifierNode::terrainModelService(TerrainModelService::Request& /*req*/, TerrainModelService::Response& resp)
{
  // generate terrain model
  if (!generateTerrainModel())
    return false;

  terrain_classifier.getTerrainModel(resp.terrain_model);
  return true;
}

void TerrainClassifierNode::reset(const std_msgs::Empty::ConstPtr& /*empty*/)
{
  terrain_classifier.reset();
}

void TerrainClassifierNode::sysCommandCallback(const std_msgs::String::ConstPtr& command)
{
  if (command->data == "reset")
    terrain_classifier.reset();
}

void TerrainClassifierNode::setPointCloud(const sensor_msgs::PointCloud2& point_cloud_msg)
{
  std::string world_frame_id = vigir_footstep_planning::strip_const(terrain_classifier.getFrameId(), '/');
  std::string cloud_frame_id = vigir_footstep_planning::strip_const(point_cloud_msg.header.frame_id, '/');
  if (world_frame_id != cloud_frame_id)
  {
    ROS_ERROR_THROTTLE(5.0, "[TerrainClassifierNode] setPointCloud: Frame of input point ('%s') cloud mismatch! Should be '%s'.", cloud_frame_id.c_str(), world_frame_id.c_str());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(point_cloud_msg, *point_cloud);

  // filter input
  filterVoxelGrid<pcl::PointXYZ>(point_cloud, 0.02, 0.02, 2.0);

  // filtering of input point cloud
  terrain_classifier.filterPointCloudData(point_cloud);

  terrain_classifier.setPointCloud(point_cloud);
  generateTerrainModel();

  publishDebugData();
  publishTerrainModel();

  //ROS_INFO("Saved");
  //pcl::io::savePCDFile("/home/alex/vigir/catkin_ws/vigir_footstep_planning/vigir_terrain_classifier/pointclouds/new.pcd", cloud_input);
}

void TerrainClassifierNode::insertPointCloud(const sensor_msgs::PointCloud2& point_cloud_msg)
{
  std::string world_frame_id = vigir_footstep_planning::strip_const(terrain_classifier.getFrameId(), '/');
  std::string cloud_frame_id = vigir_footstep_planning::strip_const(point_cloud_msg.header.frame_id, '/');
  if (world_frame_id != cloud_frame_id)
  {
    ROS_ERROR_THROTTLE(5.0, "[TerrainClassifierNode] insertPointCloud: Frame of input point ('%s') cloud mismatch! Should be '%s'.", cloud_frame_id.c_str(), world_frame_id.c_str());
    return;
  }

  if (point_cloud_msg.data.empty())
    return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(point_cloud_msg, *point_cloud);

  // filtering of input point cloud
  terrain_classifier.filterPointCloudData(point_cloud);

  terrain_classifier.insertPointCloud(point_cloud);

  if (terrain_classifier.getInputCloud()->size() <= min_aggregation_size)
  {
    if (cloud_input_pub.getNumSubscribers() > 0)
    {
      sensor_msgs::PointCloud2 cloud_point_msg;
      pcl::toROSMsg(*(terrain_classifier.getInputCloud()), cloud_point_msg);
      cloud_point_msg.header.stamp = ros::Time::now();
      cloud_point_msg.header.frame_id = terrain_classifier.getFrameId();
      cloud_input_pub.publish(cloud_point_msg);
    }

    return;
  }

  if (compute_update_skips_counter++ >= compute_update_skips)
  {
    terrain_classifier.computeNormals(point_cloud);
    compute_update_skips_counter = 0;
  }

  terrain_classifier.generateHeightGridmap(point_cloud);

  publishDebugData();

  //ROS_INFO("Saved");
  //pcl::io::savePCDFile("/home/alex/vigir/catkin_ws/vigir_footstep_planning/vigir_terrain_classifier/pointclouds/new.pcd", cloud_input);
}

bool TerrainClassifierNode::generateTerrainModel()
{
  // generate normals
  ROS_INFO("Generate normals...");
  if (!terrain_classifier.computeNormals())
    return false;

  // generate gradients
  ROS_INFO("Generate gradients...");
  if (!terrain_classifier.computeGradients())
    return false;

  // detect edges
  ROS_INFO("Detect edges...");
  if (!terrain_classifier.detectEdges())
    return false;

  // generate height grid map
  ROS_INFO("Generate height grid map...");
  if (!terrain_classifier.generateHeightGridmap())
    return false;

  // generate ground level grid map
  ROS_INFO("Generate ground level grid map...");
  if (!terrain_classifier.generateGroundLevelGridmap())
    return false;

  return true;
}

void TerrainClassifierNode::publishTerrainModel() const
{
  if (terrain_model_pub.getNumSubscribers() > 0 && terrain_classifier.hasTerrainModel())
  {
    vigir_terrain_classifier::TerrainModelMsg model;
    terrain_classifier.getTerrainModel(model);
    terrain_model_pub.publish(model);
  }

  // publish ground level grid map
  if ((ground_level_grid_map_pub.getNumSubscribers() > 0) && (terrain_classifier.getGroundLevelGridMap()))
  {
    ground_level_grid_map_pub.publish(terrain_classifier.getGroundLevelGridMap());
  }

  // publish mesh surface
  if ((mesh_surface_pub.getNumSubscribers() > 0) && (terrain_classifier.getMeshSurface()))
  {
    pcl_msgs::PolygonMesh mesh_msg;
    pcl_conversions::fromPCL(*(terrain_classifier.getMeshSurface()), mesh_msg);
    mesh_msg.header.stamp = ros::Time::now();
    mesh_msg.header.frame_id = terrain_classifier.getFrameId();
    mesh_surface_pub.publish(mesh_msg);
  }
}

void TerrainClassifierNode::publishTerrainModel(const ros::TimerEvent& /*publish_timer*/) const
{
  publishTerrainModel();
}

void TerrainClassifierNode::publishDebugData() const
{
  sensor_msgs::PointCloud2 cloud_point_msg;

  if (cloud_input_pub.getNumSubscribers() > 0)
  {
    pcl::toROSMsg(*(terrain_classifier.getInputCloud()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier.getFrameId();
    cloud_input_pub.publish(cloud_point_msg);
  }

  if (cloud_points_processed_pub.getNumSubscribers() > 0)
  {
    pcl::toROSMsg(*(terrain_classifier.getCloudProcessed()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier.getFrameId();
    cloud_points_processed_pub.publish(cloud_point_msg);
  }

  if (cloud_points_processed_low_res_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    terrain_classifier.getCloudProcessedLowRes(cloud);
    pcl::toROSMsg(*cloud, cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier.getFrameId();
    cloud_points_processed_low_res_pub.publish(cloud_point_msg);
  }

  if (cloud_normals_pub.getNumSubscribers() > 0)
  {
    // convert normals to PoseArray and publish them
    const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_points_with_normals = terrain_classifier.getPointsWithsNormals();

    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = terrain_classifier.getFrameId();
    pose_array.header.stamp = ros::Time::now();

    geometry_msgs::Pose pose_msg;
    for (size_t i = 0; i < cloud_points_with_normals->size(); i++)
    {
      const pcl::PointNormal& p_n = cloud_points_with_normals->at(i);

      pose_msg.position.x = p_n.x;
      pose_msg.position.y = p_n.y;
      pose_msg.position.z = p_n.z;

      geometry_msgs::Vector3 n;
      n.x = p_n.normal_x;
      n.y = p_n.normal_y;
      n.z = p_n.normal_z;

      double r, p;
      vigir_footstep_planning::normalToRP(n, 0.0, r, p);
      p -= M_PI_2;
      pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, 0.0);

      pose_array.poses.push_back(pose_msg);
    }

    cloud_normals_pub.publish(pose_array);
  }

  if (cloud_gradients_pub.getNumSubscribers() > 0)
  {
    pcl::toROSMsg(*(terrain_classifier.getGradients()), cloud_point_msg);
    cloud_point_msg.header.stamp = ros::Time::now();
    cloud_point_msg.header.frame_id = terrain_classifier.getFrameId();
    cloud_gradients_pub.publish(cloud_point_msg);
  }

  // publish height grid map
  if (height_grid_map_pub.getNumSubscribers() > 0)
    height_grid_map_pub.publish(terrain_classifier.getHeightGridMapRescaled());
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vigir_terrain_classifier");

  ros::NodeHandle nh;
  vigir_terrain_classifier::TerrainClassifierNode terrain_classifier_node(nh);

  for (int i = 1; i < argc; ++i)
  {
    if (std::string(argv[i]) == "-loadTestCloud")
      terrain_classifier_node.loadTestPointCloud();
  }

  ros::spin();

  return 0;
}
