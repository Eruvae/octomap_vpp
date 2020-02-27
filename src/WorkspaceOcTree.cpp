#include "octomap_vpp/WorkspaceOcTree.h"
#include <cmath>

namespace octomap_vpp
{

double WorkspaceNode::getMeanChildValue() const
{
    double mean = 0;
    if (children !=NULL){
      for (unsigned int i=0; i<8; i++) {
        if (children[i] != NULL) {
          mean += static_cast<WorkspaceNode*>(children[i])->getValue();
        }
      }
    }
    mean /= 8;
    return mean;
}

WorkspaceOcTree::WorkspaceOcTree(double res) : octomap::OcTreeBase<WorkspaceNode>(res)
{
  workspaceOcTreeMemberInit.ensureLinking();
}

WorkspaceOcTree::WorkspaceOcTree(const CountingOcTree &count_tree) : octomap::OcTreeBase<WorkspaceNode>(count_tree.getResolution())
{
  double mean, variance;
  count_tree.computeStatistics(mean, variance);
  double stdev = std::sqrt(variance);
  float minCnt = std::max(0.0, mean - stdev);
  float maxCnt = mean + stdev;
  float range = maxCnt - minCnt;
  for(auto it = count_tree.begin_leafs(), end=count_tree.end_leafs(); it != end; ++it)
  {
    unsigned int count = it->getCount();
    float val = std::max(0.f, std::min(1.f, (count - minCnt) / range));
    updateNode(it.getKey(), val, false);
  }
  updateInnerVals();
}

WorkspaceNode* WorkspaceOcTree::updateNode(const octomap::point3d& p, float value, bool updateInnerNodes)
{
  octomap::OcTreeKey key;
  if (!coordToKeyChecked(p, key)) return NULL;
  return updateNode(key, value, updateInnerNodes);
}

WorkspaceNode* WorkspaceOcTree::updateNode(const octomap::OcTreeKey& k, float value, bool updateInnerNodes)
{
  if (root == NULL) {
    root = new WorkspaceNode();
    tree_size++;
  }
  WorkspaceNode* curNode (root);

  std::vector<WorkspaceNode*> node_stack;

  // follow or construct nodes down to last level...
  for (int i=(tree_depth-1); i>=0; i--) {

    if (updateInnerNodes)
      node_stack.push_back(curNode);

    unsigned int pos = computeChildIdx(k, i);

    // requested node does not exist
    if (!nodeChildExists(curNode, pos)) {
      createNodeChild(curNode, pos);
    }
    // descent tree
    curNode = getNodeChild(curNode, pos);
  }

  curNode->setValue(value);

  if (updateInnerNodes)
  {
    for (size_t i = node_stack.size() - 1; i >= 0; i--)
      node_stack[i]->updateValChildren();
  }

  return curNode;
}

void WorkspaceOcTree::updateInnerVals()
{
    if (this->root)
      this->updateInnerValsRecurs(this->root, 0);
}

void WorkspaceOcTree::updateInnerValsRecurs(WorkspaceNode* node, unsigned int depth)
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

WorkspaceOcTree::StaticMemberInitializer WorkspaceOcTree::workspaceOcTreeMemberInit;

}
