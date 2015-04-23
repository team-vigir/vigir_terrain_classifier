#include <vigir_terrain_classifier/point_cloud_portioner_node.h>

namespace vigir_terrain_classifier
{
PointCloudPortionerNode::PointCloudPortionerNode()
{
  ros::NodeHandle nh;

  double publish_rate;

  nh.param("point_cloud_portioner/world_frame_id", world_frame_id, std::string("/world"));
  nh.param("point_cloud_portioner/publish_rate", publish_rate, 30.0);
  nh.param("point_cloud_portioner/points_per_update", (int&) points_per_update, 500);
  nh.param("point_cloud_portioner/play_in_loop", play_in_loop, true);

  // publish topics
  point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_update", 1);

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

  loadPointCloud(path);

  publish_timer = nh.createTimer(ros::Duration(1.0/publish_rate), &PointCloudPortionerNode::update, this);
}

PointCloudPortionerNode::~PointCloudPortionerNode()
{
}

void PointCloudPortionerNode::loadPointCloud(const std::string& path)
{
//  vigir_terrain_classifier::TerrainClassifierParams params;
//  terrain_classifier->getParams(params);
//  params.ed_radius = 0.05;
//  params.ed_max_std = 0.02;
//  params.ed_non_max_supp_radius = 0.05;
//  params.gg_res = 0.02;
//  terrain_classifier->setParams(params);

  ROS_INFO("Load point cloud from %s...", path.c_str());
  pcl::io::loadPCDFile(path, point_cloud);
  ROS_INFO("Done!");

  current_point_index = 0;
}

void PointCloudPortionerNode::update(const ros::TimerEvent& /*timer*/)
{
  pcl::PointCloud<pcl::PointXYZ> point_cloud_portion;
  sensor_msgs::PointCloud2 point_cloud_msg;

  if (point_cloud.size() <= current_point_index)
  {
    if (play_in_loop)
      current_point_index = (current_point_index+points_per_update) % point_cloud.size();
    else
      return;
  }

  int offset = current_point_index;
  for (unsigned int i = 0; i < points_per_update; i++)
  {
    if (point_cloud.size() <= offset+i)
    {
      if (play_in_loop)
        offset = -i;
      else
        break;
    }

    point_cloud_portion.push_back(point_cloud[offset+i]);
  }

  current_point_index += points_per_update;

  if (point_cloud_pub.getNumSubscribers() > 0)
  {
    pcl::toROSMsg(point_cloud_portion, point_cloud_msg);
    point_cloud_msg.header.stamp = ros::Time::now();
    point_cloud_msg.header.frame_id = world_frame_id;
    point_cloud_pub.publish(point_cloud_msg);
  }

}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vigir_point_cloud_portioner");

  vigir_terrain_classifier::PointCloudPortionerNode vigir_point_cloud_portioner;

  ros::spin();

  return 0;
}
