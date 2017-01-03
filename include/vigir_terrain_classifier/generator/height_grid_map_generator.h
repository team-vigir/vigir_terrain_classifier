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

#ifndef VIGIR_TERRAIN_CLASSIFIER_HEIGHT_GRID_MAP_GENERATOR_H__
#define VIGIR_TERRAIN_CLASSIFIER_HEIGHT_GRID_MAP_GENERATOR_H__

#include <ros/ros.h>

#include <vigir_terrain_classifier/pcl/octree_voxel_grid.h>
#include <vigir_terrain_classifier/generator/grid_map_generator.h>
#include <vigir_terrain_classifier/grid_map/height_grid_map.h>
#include <vigir_terrain_classifier/HeightGridMapMsg.h>



namespace vigir_terrain_classifier
{
class HeightGridMapGenerator
  : public GridMapGenerator
{
public:
  HeightGridMapGenerator(HeightGridMap::Ptr grid_map);
  virtual ~HeightGridMapGenerator();

  void reset();

  void setGridMap(GridMap::Ptr grid_map);
  HeightGridMap::ConstPtr getGridMap() const;
  HeightGridMap::Ptr getGridMap();

  void setExternalInputOctree(const OctreeVoxelGrid<pcl::PointXYZ>::ConstPtr& octree);

  void update(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

  // typedefs
  typedef boost::shared_ptr<HeightGridMapGenerator> Ptr;
  typedef boost::shared_ptr<const HeightGridMapGenerator> ConstPtr;

protected:
  void update(const pcl::PointXYZ& point);

  HeightGridMap::Ptr grid_map; // Pointer to downcasted grid map pointing to same object as GridMapGenerator::grid_map

  OctreeVoxelGrid<pcl::PointXYZ>::Ptr input_octree;
  OctreeVoxelGrid<pcl::PointXYZ>::ConstPtr external_input_octree;
};
}

#endif
