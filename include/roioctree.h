#ifndef ROIOCTREE_H
#define ROIOCTREE_H

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>

class RoiOcTreeNode : public octomap::OcTreeNode
{
public:
  RoiOcTreeNode() : OcTreeNode() {}

protected:
  float roiLogOdd;
};

class RoiOcTree : public octomap::OccupancyOcTreeBase <RegionOcTreeNode>
{
public:
  RoiOcTree(double resolution) : octomap::OccupancyOcTreeBase <RegionOcTreeNode>(resolution)
  {
  }

  void insertRegionScan(const octomap::Pointcloud &regionPoints, const octomap::Pointcloud &offRegionPoints)
  {
    std::unordered_set<RoiOcTreeNode*> regionNodes;
    std::unordered_set<RoiOcTreeNode*> offRegionNodes;
    for (size_t i = 0; i < regionPoints.size(); i++)
    {
      const octomap::point3d &p = scan[i];
      RoiOcTreeNode *node = this->search(p);

      if (node != NULL)
        regionNodes.insert(node);
    }

    for (size_t i = 0; i < offRegionPoints.size(); i++)
    {
      const octomap::point3d &p = scan[i];
      RoiOcTreeNode *node = this->search(p);

      if (node != NULL)
        offRegionNodes.insert(node);
    }

    for (RoiOcTreeNode *node : regionNodes)
    {

    }

    for (RoiOcTreeNode *node : offRegionNodes)
    {

    }
  }

protected:
};

#endif // ROIOCTREE_H
