#ifndef INFLATEDROIOCTREE_H
#define INFLATEDROIOCTREE_H

#include <octomap/OcTreeDataNode.h>
#include <octomap/OcTreeBase.h>

namespace octomap_vpp
{

class InflatedRoiOcTreeNode : public octomap::OcTreeDataNode<float>
{
public:
  float getMeanChildValues() const
  {
      float mean = 0;
      uint8_t c = 0;
      if (children !=NULL){
        for (unsigned int i=0; i<8; i++) {
          if (children[i] != NULL) {
            mean += static_cast<InflatedRoiOcTreeNode*>(children[i])->getValue();
            ++c;
          }
        }
      }

      if (c > 0)
        mean /= c;

      return mean;
  }

  /// update this node's value according to its children's mean value
  inline void updateValChildren()
  {
    this->setValue(this->getMeanChildValues());
  }
};

class InflatedRoiOcTree : public octomap::OcTreeBase<InflatedRoiOcTreeNode>
{
public:
  InflatedRoiOcTree(double resolution, double influence_radius = 0.5, double max_roi_val = 1.0);

  virtual InflatedRoiOcTree* create() const {return new InflatedRoiOcTree(resolution); }

  virtual std::string getTreeType() const {return "InflatedRoiOcTree";}

  InflatedRoiOcTreeNode* updateNodeVal(const octomap::OcTreeKey& key, float val, bool lazy_eval = false, bool increase_only = false)
  {
   bool createdRoot = false;
   if (this->root == NULL){
     this->root = new InflatedRoiOcTreeNode();
     this->tree_size++;
     createdRoot = true;
   }

   return updateNodeValRecurs(this->root, createdRoot, key, 0, val, lazy_eval, increase_only);
 }

  InflatedRoiOcTreeNode* updateNodeValRecurs(InflatedRoiOcTreeNode* node, bool node_just_created, const octomap::OcTreeKey& key, unsigned int depth, const float& val, bool lazy_eval, bool increase_only)
  {
    bool created_node = false;

    assert(node);

    // follow down to last level
    if (depth < this->tree_depth) {
      unsigned int pos = computeChildIdx(key, this->tree_depth -1 - depth);
      if (!this->nodeChildExists(node, pos)) {
        // child does not exist, but maybe it's a pruned node?
        if (!this->nodeHasChildren(node) && !node_just_created ) {
          // current node does not have children AND it is not a new node
          // -> expand pruned node
          this->expandNode(node);
        }
        else {
          // not a pruned node, create requested child
          this->createNodeChild(node, pos);
          created_node = true;
        }
      }

      if (lazy_eval)
        return updateNodeValRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, val, lazy_eval, increase_only);
      else {
        InflatedRoiOcTreeNode* retval = updateNodeValRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, val, lazy_eval, increase_only);
        // prune node if possible, otherwise set own probability
        // note: combining both did not lead to a speedup!
        if (this->pruneNode(node)){
          // return pointer to current parent (pruned), the just updated node no longer exists
          retval = node;
        } else{
          node->updateValChildren();
        }

        return retval;
      }

    }

    // at last level, update node, end of recursion
    else
    {
      if (!increase_only || val > node->getValue())
        node->setValue(val);

      return node;
    }
  }

  void updateInnerVals()
  {
      if (this->root)
        this->updateInnerValsRecurs(this->root, 0);
  }

  void updateInnerValsRecurs(InflatedRoiOcTreeNode* node, unsigned int depth)
  {
    assert(node);

    // only recurse and update for inner nodes:
    if (this->nodeHasChildren(node)){
      // return early for last level:
      if (depth < this->tree_depth){
        for (unsigned int i=0; i<8; i++) {
          if (this->nodeChildExists(node, i)) {
            updateInnerValsRecurs(this->getNodeChild(node, i), depth+1);
          }
        }
      }
      node->updateValChildren();
    }
  }

  float getInfluenceRadius() {return influence_radius;}
  float getMaxRoiVal() {return max_roi_val;}

  void setInfluenceRadius(float radius) {influence_radius = radius;}
  void setMaxRoiVal(float val) {max_roi_val = val;}

protected:
  double influence_radius;
  double max_roi_val;

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
      InflatedRoiOcTree* tree = new InflatedRoiOcTree(0.1);
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

  /// to ensure static initialization (only once)
  static StaticMemberInitializer ocTreeMemberInit;
};

}

#endif // INFLATEDROIOCTREE_H
