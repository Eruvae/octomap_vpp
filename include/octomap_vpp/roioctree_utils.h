#ifndef ROIOCTREE_UTILS_H
#define ROIOCTREE_UTILS_H

#include <octomap/octomap_utils.h>
#include <octomap/Pointcloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Transform.h>
#include <unordered_set>
#include <vector>
#include "octomap_transforms.h"

namespace octomap_vpp
{

enum class NodeProperty {
    OCCUPANCY, ROI
};

enum class NodeState {
    UNKNOWN, FREE_NONROI, OCCUPIED_ROI
};

enum Neighborhood {
  NB_6 = 6, NB_18 = 18, NB_26 = 26
};

const int nbLut [26][3] = {
  {-1, 0, 0},
  {0, -1, 0},
  {0, 0, -1},
  {0, 0, 1},
  {0, 1, 0},
  {1, 0, 0},
  {-1, -1, 0},
  {-1, 0, -1},
  {-1, 0, 1},
  {-1, 1, 0},
  {0, -1, -1},
  {0, -1, 1},
  {0, 1, -1},
  {0, 1, 1},
  {1, -1, 0},
  {1, 0, -1},
  {1, 0, 1},
  {1, 1, 0},
  {-1, -1, -1},
  {-1, -1, 1},
  {-1, 1, -1},
  {-1, 1, 1},
  {1, -1, -1},
  {1, -1, 1},
  {1, 1, -1},
  {1, 1, 1}
};

const int nbCoordDiff [26] = {1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3};

// Old separate LUTs, currently kept for backwards compatibility

const int nb6Lut [6][3] = {
  {-1, 0, 0},
  {0, -1, 0},
  {0, 0, -1},
  {0, 0, 1},
  {0, 1, 0},
  {1, 0, 0}
};

const int nb18Lut [18][3] = {
  {-1, -1, 0},
  {-1, 0, -1},
  {-1, 0, 0},
  {-1, 0, 1},
  {-1, 1, 0},
  {0, -1, -1},
  {0, -1, 0},
  {0, -1, 1},
  {0, 0, -1},
  {0, 0, 1},
  {0, 1, -1},
  {0, 1, 0},
  {0, 1, 1},
  {1, -1, 0},
  {1, 0, -1},
  {1, 0, 0},
  {1, 0, 1},
  {1, 1, 0}
};

const int nb26Lut [26][3] = {
  {-1, -1, -1},
  {-1, -1, 0},
  {-1, -1, 1},
  {-1, 0, -1},
  {-1, 0, 0},
  {-1, 0, 1},
  {-1, 1, -1},
  {-1, 1, 0},
  {-1, 1, 1},
  {0, -1, -1},
  {0, -1, 0},
  {0, -1, 1},
  {0, 0, -1},
  {0, 0, 1},
  {0, 1, -1},
  {0, 1, 0},
  {0, 1, 1},
  {1, -1, -1},
  {1, -1, 0},
  {1, -1, 1},
  {1, 0, -1},
  {1, 0, 0},
  {1, 0, 1},
  {1, 1, -1},
  {1, 1, 0},
  {1, 1, 1}
};

inline double probabilityToEntropy(double p)
{
  return -(p*log2(p) + (1-p)*log2(1-p));
}

inline double logOddsToEntropy(float logOdds)
{
  return probabilityToEntropy(octomap::probability(logOdds));
}

static void pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::unordered_set<size_t> &indices,
                                   octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud, octomap::Pointcloud &fullCloud)
{
   inlierCloud.reserve(indices.size());
   outlierCloud.reserve(cloud.data.size() / cloud.point_step - indices.size());
   fullCloud.reserve(cloud.data.size() / cloud.point_step);

   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
   sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

   for (size_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i)
   {
     octomap::point3d p(*iter_x, *iter_y, *iter_z);
     fullCloud.push_back(p);
     // Check if the point is valid
     if (std::isfinite (*iter_x) && std::isfinite (*iter_y) && std::isfinite (*iter_z))
     {
       if (indices.find(i) != indices.end())
         inlierCloud.push_back(p);
       else
         outlierCloud.push_back(p);
     }

   }
}

static void pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::unordered_set<size_t> &indices, const geometry_msgs::Transform &transform,
                                   octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud, octomap::Pointcloud &fullCloud)
{
   inlierCloud.reserve(indices.size());
   outlierCloud.reserve(cloud.data.size() / cloud.point_step - indices.size());
   fullCloud.reserve(cloud.data.size() / cloud.point_step);

   octomap::pose6d t = transformToOctomath(transform);

   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
   sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

   for (size_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i)
   {
     octomap::point3d p(*iter_x, *iter_y, *iter_z);
     octomap::point3d p_tf = t.transform(p);
     fullCloud.push_back(p_tf);
     // Check if the point is valid
     if (std::isfinite (*iter_x) && std::isfinite (*iter_y) && std::isfinite (*iter_z))
     {
       if (indices.find(i) != indices.end())
         inlierCloud.push_back(p_tf);
       else
         outlierCloud.push_back(p_tf);
     }

   }
}

// indices must be ordered!
static void pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::vector<int> &indices,
                                   octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud, octomap::Pointcloud &fullCloud)
{
  inlierCloud.reserve(indices.size());
  outlierCloud.reserve(cloud.data.size() / cloud.point_step - indices.size());
  fullCloud.reserve(cloud.data.size() / cloud.point_step);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  std::vector<int>::const_iterator it = indices.begin();
  for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i)
  {
    // Check if the point is valid
    if (std::isfinite (*iter_x) && std::isfinite (*iter_y) && std::isfinite (*iter_z))
    {
      octomap::point3d p(*iter_x, *iter_y, *iter_z);
      fullCloud.push_back(p);
      if (it != indices.end() && i == *it)
      {
        inlierCloud.push_back(p);
        it++;
      }
      else
      {
        outlierCloud.push_back(p);
      }
    }
  }
}

static void pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::vector<int> &indices, const geometry_msgs::Transform &transform,
                                   octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud, octomap::Pointcloud &fullCloud)
{
  inlierCloud.reserve(indices.size());
  outlierCloud.reserve(cloud.data.size() / cloud.point_step - indices.size());
  fullCloud.reserve(cloud.data.size() / cloud.point_step);

  octomap::pose6d t = transformToOctomath(transform);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  std::vector<int>::const_iterator it = indices.begin();
  for (int i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i)
  {
    // Check if the point is valid
    if (std::isfinite (*iter_x) && std::isfinite (*iter_y) && std::isfinite (*iter_z))
    {
      octomap::point3d p(*iter_x, *iter_y, *iter_z);
      octomap::point3d p_tf = t.transform(p);
      fullCloud.push_back(p_tf);
      if (it != indices.end() && i == *it)
      {
        inlierCloud.push_back(p_tf);
        it++;
      }
      else
      {
        outlierCloud.push_back(p_tf);
      }
    }
  }
}

}

#endif // ROIOCTREE_UTILS_H
