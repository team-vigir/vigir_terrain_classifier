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

#ifndef VIGIR_TERRAIN_CLASSIFIER_TERRAIN_MODEL_H__
#define VIGIR_TERRAIN_CLASSIFIER_TERRAIN_MODEL_H__

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <nav_msgs/OccupancyGrid.h>

#include <vigir_footstep_planning_lib/helper.h>
#include <vigir_footstep_planning_lib/math.h>

#include <vigir_terrain_classifier/grid_map/ground_level_grid_map.h>
#include <vigir_terrain_classifier/grid_map/height_grid_map.h>

#include <vigir_terrain_classifier/TerrainModelMsg.h>



namespace vigir_terrain_classifier
{
class TerrainModel
{
public:
  TerrainModel(const std::string& frame_id, double resolution, HeightGridMap::Ptr height_grid_map);
  TerrainModel(const TerrainModelMsg& terrain_model);
  virtual ~TerrainModel();

  void reset();

  void fromMsg(const TerrainModelMsg& terrain_model);
  void toMsg(TerrainModelMsg& terrain_model) const;

  double getResolution() const;

  void updateSearchTree();

  bool hasTerrainModel() const;

  const pcl::PointCloud<pcl::PointNormal>::ConstPtr getPointCloudWithNormals() const;
  pcl::PointCloud<pcl::PointNormal>::Ptr getPointCloudWithNormals();

  const HeightGridMap::ConstPtr getHeightGridMap() const;
  HeightGridMap::Ptr getHeightGridMap();

  bool getPointWithNormal(const pcl::PointNormal& p_search, pcl::PointNormal& p_result) const;
  bool getHeight(double x, double y, double& height) const;
  bool update3DData(geometry_msgs::Pose& p) const;

  // typedefs
  typedef boost::shared_ptr<TerrainModel> Ptr;
  typedef boost::shared_ptr<const TerrainModel> ConstPtr;

protected:
  // parameters
  std::string frame_id;
  double resolution;

  // point cloud with normals
  pcl::PointCloud<pcl::PointNormal>::Ptr point_cloud_with_normals;

  // ground level grid map
  GroundLevelGridMap::Ptr ground_level_grid_map;

  // height grid map
  HeightGridMap::Ptr height_grid_map;

  // kd-tree for searching normals of specific point
  pcl::KdTreeFLANN<pcl::PointNormal>::Ptr points_with_normals_kdtree;
};
}

#endif
