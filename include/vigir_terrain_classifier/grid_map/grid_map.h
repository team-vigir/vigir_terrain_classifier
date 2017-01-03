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

#ifndef VIGIR_TERRAIN_CLASSIFIER_GRID_MAP_H__
#define VIGIR_TERRAIN_CLASSIFIER_GRID_MAP_H__

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <vigir_footstep_planning_lib/helper.h>
#include <vigir_footstep_planning_lib/math.h>

#include <vigir_terrain_classifier/pcl/octree_voxel_grid.h>

#define GRID_MAP_EMPTY_VAL std::numeric_limits<int8_t>::min()
#define GRID_MAP_MIN_VAL (std::numeric_limits<int8_t>::min()+1)
#define GRID_MAP_VALUE_RANGE (std::numeric_limits<int8_t>::max()-std::numeric_limits<int8_t>::min())



namespace vigir_terrain_classifier
{
class GridMap
{
public:
  GridMap(const std::string& frame_id, double resolution, double min_expansion_size = 1.0);
  GridMap(const nav_msgs::OccupancyGrid& map, double min_expansion_size = 1.0);
  virtual ~GridMap();

  virtual void clear();

  bool empty() const;

  int8_t& at(int idx);

  void fromMsg(const nav_msgs::OccupancyGrid& map);
  void toMsg(nav_msgs::OccupancyGrid& map) const;

  const nav_msgs::OccupancyGrid::Ptr& map() const;
  nav_msgs::OccupancyGrid::Ptr& map();

  const geometry_msgs::Vector3& getMin() const;
  const geometry_msgs::Vector3& getMax() const;

  virtual void resize(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
  virtual void resize(const geometry_msgs::Vector3& min, const geometry_msgs::Vector3& max);

  static bool getGridMapCoords(const nav_msgs::OccupancyGrid& map, double x, double y, int& map_x, int& map_y);
  inline bool getGridMapCoords(double x, double y, int& map_x, int& map_y) const { return getGridMapCoords(*grid_map, x, y, map_x, map_y); }

  static bool getGridMapCoords(const nav_msgs::OccupancyGrid& map, int idx, int& map_x, int& map_y);
  inline bool getGridMapCoords(int idx, int& map_x, int& map_y) const { return getGridMapCoords(*grid_map, idx, map_x, map_y); }

  static bool getGridMapIndex(const nav_msgs::OccupancyGrid& map, double x, double y, int& idx);
  inline bool getGridMapIndex(double x, double y, int& idx) const { return getGridMapIndex(*grid_map, x, y, idx); }

  static bool getGridMapIndex(const nav_msgs::OccupancyGrid& map, int map_x, int map_y, int& idx);
  inline bool getGridMapIndex(int map_x, int map_y, int& idx) const { return getGridMapIndex(*grid_map, map_x, map_y, idx); }

  static bool getGridMapWorldCoords(const nav_msgs::OccupancyGrid& map, int map_x, int map_y, double& x, double& y);
  inline bool getGridMapWorldCoords(int map_x, int map_y, double& x, double& y) const { return getGridMapWorldCoords(*grid_map, map_x, map_y, x, y); }

  static bool getGridMapWorldCoords(const nav_msgs::OccupancyGrid& map, int idx, double& x, double& y);
  inline bool getGridMapWorldCoords(int idx, double& x, double& y) const {return getGridMapWorldCoords(*grid_map, idx, x, y); }

  // typedefs
  typedef boost::shared_ptr<GridMap> Ptr;
  typedef boost::shared_ptr<const GridMap> ConstPtr;

protected:
  void getPointCloudBoundary(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, geometry_msgs::Vector3& min, geometry_msgs::Vector3& max) const;

  nav_msgs::OccupancyGrid::Ptr grid_map;

  double min_expansion_size;
  geometry_msgs::Vector3 min;
  geometry_msgs::Vector3 max;
};
}

#endif
