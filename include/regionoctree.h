#ifndef REGIONOCTREE_H
#define REGIONOCTREE_H

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>


class RegionOcTreeNode : public octomap::OcTreeNode
{
public:
  class Region
  {
  public:
    Region(const std::string &name, int priority) : name(name), priority(priority) {}
  private:
    std::string name;
    int priority;
  };

  RegionOcTreeNode() : OcTreeNode() {}

protected:
  std::vector<float> regionValues;
};

class RegionOcTree : public octomap::OccupancyOcTreeBase <RegionOcTreeNode>
{
public:
  RegionOcTree(double resolution) : octomap::OccupancyOcTreeBase <RegionOcTreeNode>(resolution)
  {
  }

  bool initializeRegions(const std::vector<std::string> &names, const std::vector<int> priorities)
  {
    if (names.size() != priorities.size())
      return false;

    for (size_t i = 0; i < names.size(); i++)
    {
      regions.push_back(RegionOcTreeNode::Region(names[i], priorities[i]));
      regionNameMap.insert(std::make_pair(names[i], i));
    }
  }

  void insertRegionScan(const octomap::Pointcloud &scan, const std::string &region)
  {
    std::unordered_set<RegionOcTreeNode*> nodesToUpdate;
    for (size_t i = 0; i < scan.size(); i++)
    {
      const octomap::point3d &p = scan[i];
      RegionOcTreeNode *node = this->search(p);

      if (node != NULL)
        nodesToUpdate.insert(node);
    }

    for (RegionOcTreeNode *node : nodesToUpdate)
    {

    }
  }

protected:
  std::vector<RegionOcTreeNode::Region> regions;
  std::unordered_map<std::string, size_t> regionNameMap;
};

#endif // REGIONOCTREE_H
