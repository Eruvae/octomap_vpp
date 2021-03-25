#ifndef NEAREST_REGION_OCTREE_H
#define NEAREST_REGION_OCTREE_H

#include <octomap/OcTreeBase.h>
#include <octomap/OcTreeDataNode.h>
#include "CountingOcTree.h"
#include <memory>
#include <unordered_map>

namespace octomap_vpp
{

struct RegionInfo
{
  RegionInfo() : nearestRegionId(0), distance(0.f) {}
  RegionInfo(unsigned int id, float d) : nearestRegionId(id), distance(d) {}

  unsigned int nearestRegionId;
  float distance;

  bool operator== (const RegionInfo &rhs) const
  {
    return nearestRegionId == rhs.nearestRegionId && distance == rhs.distance;
  }

  bool operator!= (const RegionInfo &rhs) const
  {
    return nearestRegionId != rhs.nearestRegionId || distance != rhs.distance;
  }
};

class NearestRegionOcTreeNode : public octomap::OcTreeDataNode<RegionInfo>
{
public:
  RegionInfo getMeanChildRegInf() const;

  /// update this node's value according to its children's mean value
  inline void updateValChildren()
  {
    this->setValue(this->getMeanChildRegInf());
  }
};

class NearestRegionOcTree : public octomap::OcTreeBase <NearestRegionOcTreeNode> {
private:
  double max_dist;

public:
  NearestRegionOcTree(double resolution, double max_dist = std::numeric_limits<double>::quiet_NaN());
  virtual NearestRegionOcTree* create() const {return new NearestRegionOcTree(resolution); }
  virtual std::string getTreeType() const {return "NearestRegionOcTree";}

  static std::shared_ptr<NearestRegionOcTree> createFromCountringOctree(const CountingOcTree &in_tree, double max_dist);

  virtual NearestRegionOcTreeNode* updateNode(const octomap::point3d& p, const RegionInfo &regInf);
  virtual NearestRegionOcTreeNode* updateNode(const octomap::OcTreeKey& k, const RegionInfo &regInf);

  double computeMaxDist()
  {
    max_dist = 0;
    for (auto it = begin_leafs(), end = end_leafs(); it != end; it++)
    {
      RegionInfo regInf = it->getValue();
      if (regInf.distance > max_dist)
        max_dist = regInf.distance;
    }
    return max_dist;
  }

  double getMaxDist()
  {
    if (std::isnan(max_dist))
      return computeMaxDist();

    return max_dist;
  }


protected:
  /**
   * Static member object which ensures that this OcTree's prototype
   * ends up in the classIDMapping only once. You need this as a
   * static member in any derived octree class in order to read .ot
   * files through the AbstractOcTree factory. You should also call
   * ensureLinking() once from the constructor.
   */
  class StaticMemberInitializer{
     public:
       StaticMemberInitializer() {
         NearestRegionOcTree* tree = new NearestRegionOcTree(0.1);
         //uintptr_t *vtable = *(uintptr_t**)tree;
         //auto p = vtable[3];
         tree->clearKeyRays();
         AbstractOcTree::registerTreeType(tree);
       }

       /**
       * Dummy function to ensure that MSVC does not drop the
       * StaticMemberInitializer, causing this tree failing to register.
       * Needs to be called from the constructor of this octree.
       */
       void ensureLinking() {}
  };
  /// static member to ensure static initialization (only once)
  static StaticMemberInitializer nearestRegionOcTreeMemberInit;
};

}

#endif // NEAREST_REGION_OCTREE_H
