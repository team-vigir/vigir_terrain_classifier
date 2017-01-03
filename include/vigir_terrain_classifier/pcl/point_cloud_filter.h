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

#ifndef VIGIR_TERRAIN_CLASSIFIER_POINT_CLOUD_FILTER_H__
#define VIGIR_TERRAIN_CLASSIFIER_POINT_CLOUD_FILTER_H__

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>



namespace vigir_terrain_classifier
{
template <typename PointT> void filterPassThroughBox(typename pcl::PointCloud<PointT>::Ptr& cloud, const std::string& field_name, double min, double max)
{
  if (!cloud)
  {
    ROS_ERROR("filterPassThrough was called but no point cloud was available");
    return;
  }

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(field_name);
  pass.setFilterLimits(min, max);
  pass.filter(*cloud_filtered);
  cloud = cloud_filtered;
}

template <typename PointT> void filterPassThroughEllipse(typename pcl::PointCloud<PointT>::Ptr& cloud, const PointT& center, double orientation, double rx, double ry)
{
  if (!cloud)
  {
    ROS_ERROR("filterPassThroughRadius was called but no point cloud was available");
    return;
  }

  double cos_o = cos(orientation);
  double sin_o = sin(orientation);

  for (typename pcl::PointCloud<PointT>::iterator itr = cloud->begin(); itr != cloud->end();)
  {
    double dx = itr->x - center.x;
    double dy = itr->y - center.y;

    double a = (cos_o*dx + sin_o*dy)/rx;
    a = a*a;

    double b = (sin_o*dx - cos_o*dy)/ry;
    b = b*b;

    if (a+b > 1)
      itr = cloud->erase(itr);
    else
      itr++;
  }
}

template <typename PointT> void filterVoxelGrid(typename pcl::PointCloud<PointT>::Ptr& cloud, float lx, float ly, float lz)
{
  if (!cloud)
  {
    ROS_ERROR("filterVoxelGrid was called but no point cloud was available");
    return;
  }

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
  pcl::VoxelGrid<PointT> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(lx, ly, lz);
  vox.filter(*cloud_filtered);
  cloud = cloud_filtered;
}

template <typename PointT> void filterMlsSmooth(typename pcl::PointCloud<PointT>::Ptr& cloud, double radius)
{
  if (!cloud)
  {
    ROS_ERROR("filterSmooth was called but no point cloud was available");
    return;
  }

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  pcl::MovingLeastSquares<PointT, PointT> mls;
  mls.setInputCloud(cloud);
  mls.setSearchMethod(tree);
  //mls.setComputeNormals(false);
  //mls.setPolynomialFit(false);
  mls.setPolynomialOrder(1);
  mls.setSearchRadius(radius);
  //  mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
  //  mls.setUpsamplingRadius(0.025);
  //  mls.setUpsamplingStepSize(0.01);
  //  mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::RANDOM_UNIFORM_DENSITY);
  //  mls.setPointDensity(radius*5000);
  //  mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION);
  //  mls.setDilationVoxelSize(0.01);
  //  mls.setDilationIterations(1);
  mls.process(*cloud_filtered);
  cloud = cloud_filtered;
}

template <typename PointT> void filterStatisticalOutlier(typename pcl::PointCloud<PointT>::Ptr& cloud, int k, double radius, bool set_negative = false)
{
  if (!cloud)
  {
    ROS_ERROR("filterStatisticalOutlier was called but no point cloud was available");
    return;
  }

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setNegative(set_negative);
  sor.setMeanK(k);
  sor.setStddevMulThresh(radius);
  sor.filter(*cloud_filtered);
  cloud = cloud_filtered;
}
}

#endif
