//=================================================================================================
// Copyright (c) 2017, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_TERRAIN_CLASSIFIER_NODE_H__
#define VIGIR_TERRAIN_CLASSIFIER_NODE_H__

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>

#include <vigir_terrain_classifier/TerrainModelRequest.h>
#include <vigir_terrain_classifier/TerrainModelService.h>
#include <vigir_terrain_classifier/TerrainModelMsg.h>

#include <vigir_terrain_classifier/terrain_classifier.h>



namespace vigir_terrain_classifier
{
class TerrainClassifierNode
{
public:
  TerrainClassifierNode(ros::NodeHandle& nh);
  virtual ~TerrainClassifierNode();

  void loadTestPointCloud(const std::string& path);

protected:
  bool terrainModelService(TerrainModelService::Request& req, TerrainModelService::Response& resp);

  void reset(const std_msgs::Empty::ConstPtr& empty);

  void sysCommandCallback(const std_msgs::String::ConstPtr& command);

  void setPointCloud(const sensor_msgs::PointCloud2& point_cloud_msg);
  void insertPointCloud(const sensor_msgs::PointCloud2& point_cloud_msg);

  bool generateTerrainModel();

  void publishTerrainModel() const;
  void publishTerrainModel(const ros::TimerEvent& publish_timer) const;
  void publishDebugData() const;

  // subscribers
  ros::Subscriber reset_terrain_model_sub;
  ros::Subscriber sys_command_sub;
  ros::Subscriber set_point_cloud_sub;
  ros::Subscriber point_cloud_update_sub;
  ros::Subscriber generate_terrain_model_sub;

  // publisher
  ros::Publisher cloud_input_pub;
  ros::Publisher cloud_points_processed_pub;
  ros::Publisher cloud_points_processed_low_res_pub;
  ros::Publisher cloud_normals_pub;
  ros::Publisher cloud_gradients_pub;
  ros::Publisher ground_level_grid_map_pub;
  ros::Publisher height_grid_map_pub;
  ros::Publisher mesh_surface_pub;
  ros::Publisher terrain_model_pub;

  // service clients
  ros::ServiceClient generate_feet_pose_client;

  // service servers
  ros::ServiceServer generate_terrain_model_srv;

  ros::Timer publish_timer;

  TerrainClassifier terrain_classifier;

  // parameters
  unsigned int compute_update_skips; // number of updates not triggering( re)computation (but inserted to overall data)
  unsigned int min_aggregation_size;

  unsigned int compute_update_skips_counter;
};
}

#endif
