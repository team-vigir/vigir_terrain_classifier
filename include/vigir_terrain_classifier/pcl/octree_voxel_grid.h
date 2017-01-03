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

#ifndef VIGIR_TERRAIN_CLASSIFIER_OCTREE_VOXEL_GRID_FILTER_H__
#define VIGIR_TERRAIN_CLASSIFIER_OCTREE_VOXEL_GRID_FILTER_H__

#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>



namespace vigir_terrain_classifier
{
template<class PointT>
class OctreeCentroidContainer
  : public pcl::octree::OctreeContainerBase
{
public:
  OctreeCentroidContainer()
  {
    this->reset();
  }

  virtual ~OctreeCentroidContainer()
  {
  }

  OctreeCentroidContainer<PointT>* deepCopy() const
  {
    return(new OctreeCentroidContainer(*this));
  }

  void addPoint(const PointT& new_point, double update_weight = 1.0)
  {
    if (!updated_once)
    {
      update_weight = 1.0;
      updated_once = true;
    }

    point.x += update_weight * (new_point.x - point.x);
    point.y += update_weight * (new_point.y - point.y);
    point.z += update_weight * (new_point.z - point.z);
  }

  void getCentroid(PointT& centroid) const
  {
    centroid = point;
  }

  void reset()
  {
    point.x = point.y = point.z = 0;
    updated_once = false;
  }

protected:
  PointT point;
  bool updated_once;
};

// Specialization for PointNormal where the normal is averaged
template<>
void OctreeCentroidContainer<pcl::PointNormal>::addPoint(const pcl::PointNormal& new_point, double update_weight);

template<>
void OctreeCentroidContainer<pcl::PointNormal>::reset();



template<class PointT>
class OctreeVoxelGrid
  : public pcl::octree::OctreePointCloudVoxelCentroid<PointT, OctreeCentroidContainer<PointT> >
{
public:
  typedef pcl::octree::OctreePointCloudVoxelCentroid<PointT, OctreeCentroidContainer<PointT> >OctreeT;
  typedef OctreeCentroidContainer<PointT> LeafContainerT;
  typedef typename OctreeT::LeafNode LeafNode;
  typedef typename OctreeT::BranchNode BranchNode;

  OctreeVoxelGrid(double resolution, double update_weight)
    : pcl::octree::OctreePointCloudVoxelCentroid<PointT, OctreeCentroidContainer<PointT> >(resolution)
    , update_weight(update_weight)
  {
  }

  virtual ~OctreeVoxelGrid()
  {
  }

  void addPointIdx(const int pointIdx)
  {
    pcl::octree::OctreeKey key;

    assert(pointIdx < static_cast<int>(this->input_->points.size()));

    const PointT& point = this->input_->points[pointIdx];

    // make sure bounding box is big enough
    this->adoptBoundingBoxToPoint(point);

    // generate key
    this->genOctreeKeyforPoint(point, key);

    // add point to octree at key
    LeafContainerT* container = this->createLeaf(key);
    container->addPoint(point, update_weight);
  }

  void insertPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr& cloud)
  {
    this->setInputCloud(cloud);
    this->addPointsFromInputCloud();
  }

  void getPointCloud(pcl::PointCloud<PointT>& cloud)
  {
    cloud.clear();
    cloud.width = this->getVoxelCentroids(cloud.points);
    cloud.height = 1;
    cloud.is_dense = false;
  }

  int radiusSearch(const PointT &p_q, const double radius, pcl::PointCloud<PointT>& cloud, std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const
  {
    assert(isFinite(p_q) && "Invalid(NaN, Inf) point coordinates given to nearestKSearch!");
    pcl::octree::OctreeKey key;
    key.x = key.y = key.z = 0;

    cloud.clear();
    k_sqr_distances.clear();

    this->getNeighborsWithinRadiusRecursive(p_q, radius*radius, this->root_node_, key, 1, cloud, k_sqr_distances, max_nn);

    return static_cast<int>(cloud.size());
  }

  int radiusSearch(const pcl::PointCloud<PointT>& pc_q, const double radius, pcl::PointCloud<PointT>& cloud, std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const
  {
    cloud.clear();

    pcl::PointCloud<PointT> pc;
    for (typename pcl::PointCloud<PointT>::const_iterator itr = pc_q.begin(); itr != pc_q.end(); itr++)
    {
      radiusSearch(*itr, radius, pc, k_sqr_distances, max_nn);
      cloud.insert(cloud.end(), pc.begin(), pc.end());
    }

    return static_cast<int>(cloud.size());
  }

  // typedefs
  typedef boost::shared_ptr<OctreeVoxelGrid> Ptr;
  typedef boost::shared_ptr<const OctreeVoxelGrid> ConstPtr;

protected:
  void getNeighborsWithinRadiusRecursive(const PointT & point, const double radiusSquared, const BranchNode* node, const pcl::octree::OctreeKey& key,
                                         unsigned int tree_depth, pcl::PointCloud<PointT>& cloud, std::vector<float>& k_sqr_distances, unsigned int max_nn) const
  {
    // child iterator
    unsigned char child_idx;

    // get spatial voxel information
    double voxel_squared_diameter = this->getVoxelSquaredDiameter(tree_depth);

    // iterate over all children
    for (child_idx = 0; child_idx < 8; child_idx++)
    {
      if (!this->branchHasChild(*node, child_idx))
        continue;

      const pcl::octree::OctreeNode* child_node;
      child_node = this->getBranchChildPtr(*node, child_idx);

      pcl::octree::OctreeKey new_key;
      PointT voxel_center;
      float squared_dist;

      // generate new key for current branch voxel
      new_key.x = (key.x << 1) + (!!(child_idx &(1 << 2)));
      new_key.y = (key.y << 1) + (!!(child_idx &(1 << 1)));
      new_key.z = (key.z << 1) + (!!(child_idx &(1 << 0)));

      // generate voxel center point for voxel at key
      this->genVoxelCenterFromOctreeKey(new_key, tree_depth, voxel_center);

      // calculate distance to search point
      squared_dist = pointSquaredDist(static_cast<const PointT&>(voxel_center), point);

      // if distance is smaller than search radius
      if (squared_dist + this->epsilon_ <= voxel_squared_diameter / 4.0 + radiusSquared + sqrt(voxel_squared_diameter * radiusSquared))
      {
        if (tree_depth < this->octree_depth_)
        {
          // we have not reached maximum tree depth
          getNeighborsWithinRadiusRecursive(point, radiusSquared, static_cast<const BranchNode*>(child_node), new_key, tree_depth + 1, cloud, k_sqr_distances, max_nn);
          if (max_nn != 0 && cloud.size() == static_cast<unsigned int>(max_nn))
            return;
        }
        else
        {
          // we reached leaf node level
          const LeafNode* child_leaf = static_cast<const LeafNode*>(child_node);

          PointT candidate_point;
          child_leaf->getContainer().getCentroid(candidate_point);

          // calculate point distance to search point
          squared_dist = pointSquaredDist(candidate_point, point);

            // check if a match is found
          if (squared_dist < radiusSquared)
          {
            // add point to result vector
            cloud.push_back(candidate_point);
            k_sqr_distances.push_back(squared_dist);

            if (max_nn != 0 && cloud.size() == static_cast<unsigned int>(max_nn))
              return;
          }
        }
      }
    }
  }

  float pointSquaredDist(const PointT& point_a, const PointT& point_b) const
  {
    return (point_a.getVector3fMap() - point_b.getVector3fMap()).squaredNorm();
  }

  double update_weight;
};
}

#endif
