#ifndef WORKSPACEOCTREE_H
#define WORKSPACEOCTREE_H

#include <octomap/OcTreeNode.h>
#include <octomap/OcTreeBase.h>

#include "CountingOcTree.h"
#include "roioctree_utils.h"

namespace octomap_vpp
{

/*struct WorkspaceData
{
  std::vector<std::vector<double>> joint_configs;
  bool operator ==(const WorkspaceData &other) const
  {
    return joint_configs == other.joint_configs;
  }
};*/

class WorkspaceNode : public octomap::OcTreeDataNode<float>
{
public:
  /**
   * @return mean of all children's value (including unknown, counted as 0)
   */
  double getMeanChildValue() const;

  /// update this node's value according to its children's mean value
  inline void updateValChildren()
  {
    this->setValue(this->getMeanChildValue());
  }
};

class WorkspaceOcTree : public octomap::OcTreeBase<WorkspaceNode>
{
public:
  WorkspaceOcTree(double res);
  WorkspaceOcTree(const CountingOcTree &count_tree);
  virtual WorkspaceOcTree* create() const {return new WorkspaceOcTree(resolution); }
  virtual std::string getTreeType() const {return "WorkspaceOcTree";}

  WorkspaceNode* updateNode(const octomap::point3d& p, float value, bool updateInnerNodes = true);
  WorkspaceNode* updateNode(const octomap::OcTreeKey& k, float value, bool updateInnerNodes = true);

  void updateInnerVals();
  void updateInnerValsRecurs(WorkspaceNode* node, unsigned int depth);

  inline float getReachability(const octomap::OcTreeKey &key, unsigned int depth = 0) const
  {
    WorkspaceNode *node = search(key, depth);
    return node ? node->getValue() : 0.f;
  }

  inline float getReachability(const octomap::point3d &point, unsigned int depth = 0) const
  {
    WorkspaceNode *node = search(point, depth);
    return node ? node->getValue() : 0.f;
  }

  inline octomap::point3d computeGradient(const octomap::OcTreeKey &key, unsigned int depth = 0) const
  {
    float rb = getReachability(key, depth);
    octomap::point3d gradient;
    for (size_t i = 0; i < 26; i++)
    {
      octomap::OcTreeKey nKey(key[0] + nb26Lut[i][0], key[1] + nb26Lut[i][1], key[2] + nb26Lut[i][2]);
      octomap::point3d dirVec(nb26Lut[i][0], nb26Lut[i][1], nb26Lut[i][2]);
      //dirVec.normalize();
      float nrb = getReachability(nKey, depth);
      float diff = nrb - rb;
      gradient += dirVec * diff;
    }
    gradient /= 26;
    return gradient;
  }

  inline octomap::point3d computeGradient(const octomap::point3d &point, unsigned int depth = 0) const
  {
    return computeGradient(depth == 0 ? coordToKey(point) : coordToKey(point, depth), depth);
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
         WorkspaceOcTree* tree = new WorkspaceOcTree(0.1);
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
  static StaticMemberInitializer workspaceOcTreeMemberInit;
};

}

#endif // WORKSPACEOCTREE_H
