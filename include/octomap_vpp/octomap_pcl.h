#ifndef OCTOMAP_PCL_H
#define OCTOMAP_PCL_H

#include <functional>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <octomap/octomap_types.h>
#include <octomap/OcTreeKey.h>
#include <octomap/Pointcloud.h>

namespace octomap_vpp
{

template<typename PointT>
inline static PointT octomapPointToPcl(const octomap::point3d &p)
{
  PointT p_out;
  p_out.x = p.x();
  p_out.y = p.y();
  p_out.z = p.z();
  return p_out;
}

template<typename PointT>
inline static octomap::point3d pclPointToOctomap(const PointT &p)
{
  return octomap::point3d(p.x, p.y, p.z);
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr octomapPointcloudToPcl(const octomap::Pointcloud &pc)
{
  typename pcl::PointCloud<PointT>::Ptr pc_out(new pcl::PointCloud<PointT>);
  pc_out->reserve(pc.size());
  for (const octomap::point3d &p : pc)
  {
    pc_out->push_back(octomapPointToPcl<PointT>(p));
  }
  return pc_out;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr octomapPointCollectionToPcl(const octomap::point3d_collection &pc)
{
  typename pcl::PointCloud<PointT>::Ptr pc_out(new pcl::PointCloud<PointT>);
  pc_out->reserve(pc.size());
  for (const octomap::point3d &p : pc)
  {
    pc_out->push_back(octomapPointToPcl<PointT>(p));
  }
  return pc_out;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr octomapPointListToPcl(const octomap::point3d_list &pc)
{
  typename pcl::PointCloud<PointT>::Ptr pc_out(new pcl::PointCloud<PointT>);
  pc_out->reserve(pc.size());
  for (const octomap::point3d &p : pc)
  {
    pc_out->push_back(octomapPointToPcl<PointT>(p));
  }
  return pc_out;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr octomapKeysetToPcl(const octomap::KeySet &keys, std::function<octomap::point3d(const octomap::OcTreeKey&)> keyToCoord)
{
  typename pcl::PointCloud<PointT>::Ptr pc_out(new pcl::PointCloud<PointT>);
  pc_out->reserve(keys.size());
  for (const octomap::OcTreeKey &key : keys)
  {
    pc_out->push_back(octomapPointToPcl<PointT>(keyToCoord(key)));
  }
  return pc_out;
}

template<typename PointT>
octomap::Pointcloud pclPointcloudToOctomap(const pcl::PointCloud<PointT> &pc)
{
  octomap::Pointcloud pc_out;
  pc_out.reserve(pc.size());
  for (const PointT &p : pc)
  {
    pc_out.push_back(pclPointToOctomap(p));
  }
  return pc_out;
}

template<typename TREE, typename PointT>
typename pcl::PointCloud<PointT>::Ptr octomapToPcl(const TREE &tree, bool (*isOcc)(const TREE &tree, const typename TREE::NodeType*))
{
  typename pcl::PointCloud<PointT>::Ptr pc_out(new pcl::PointCloud<PointT>);
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; it++)
  {
    if (isOcc(tree, &(*it)))
    {
      pc_out->push_back(octomapPointToPcl<PointT>(it.getCoordinate()));
    }
  }
  return pc_out;
}

template<typename TREE, typename PointT>
void pclToOctomap(TREE &tree, const pcl::PointCloud<PointT> &pc, void (*updateNode)(TREE &tree, const octomap::point3d &p))
{
  for (const PointT &p : pc)
  {
    updateNode(tree, pclPointToOctomap<PointT>(p));
  }
}

template<typename TREE, typename PointT>
void pclIndicesToOctomap(TREE &tree, const pcl::PointCloud<PointT> &pc, const pcl::PointIndices &inds, void (*updateNode)(TREE &tree, const octomap::point3d &p))
{
  for (const int &i : inds.indices)
  {
    const PointT &p = pc.at(i);
    updateNode(tree, pclPointToOctomap<PointT>(p));
  }
}

} // namespace octomap_vpp

#endif // OCTOMAP_PCL_H
