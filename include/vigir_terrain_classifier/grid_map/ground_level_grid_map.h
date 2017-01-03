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

#ifndef VIGIR_TERRAIN_CLASSIFIER_GROUND_LEVEL_GRID_MAP_H__
#define VIGIR_TERRAIN_CLASSIFIER_GROUND_LEVEL_GRID_MAP_H__

#include <ros/ros.h>

#include <vigir_terrain_classifier/grid_map/grid_map.h>



namespace vigir_terrain_classifier
{
class GroundLevelGridMap
  : public GridMap
{
public:
  GroundLevelGridMap(const std::string& frame_id, double resolution, double min_expansion_size = 0.5, double update_weight = 1.0);
  virtual ~GroundLevelGridMap();

//  void fromMsg(const GroundLevelGridMapMsg& msg);
//  void toMsg(GroundLevelGridMapMsg& msg) const;

  // typedefs
  typedef boost::shared_ptr<GroundLevelGridMap> Ptr;
  typedef boost::shared_ptr<const GroundLevelGridMap> ConstPtr;
};
}

#endif
