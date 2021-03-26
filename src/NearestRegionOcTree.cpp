#include "octomap_vpp/NearestRegionOcTree.h"
#include <boost/heap/fibonacci_heap.hpp>
#include "octomap_vpp/roioctree_utils.h"

namespace octomap_vpp
{

RegionInfo NearestRegionOcTreeNode::getMeanChildRegInf() const
{
  std::unordered_map<unsigned int, RegionInfo> regmap;
  if (children != NULL){
    for (unsigned int i=0; i<8; i++) {
      if (children[i] != NULL) {
        RegionInfo regInf = static_cast<NearestRegionOcTreeNode*>(children[i])->getValue();
        auto it = regmap.find(regInf.nearestRegionId);
        if (it == regmap.end())
        {
          regmap[regInf.nearestRegionId] = RegionInfo(1, regInf.distance);
        }
        else
        {
          it->second.nearestRegionId++; // nearestRegionId used as occurrence counter
          it->second.distance += regInf.distance; // store sum of all child distances
        }
      }
    }
  }
  unsigned int maxRegId = 0;
  unsigned int maxRegCount = 0;
  float maxDistSum = 0.f;
  for (const auto &pair : regmap)
  {
    if (pair.second.nearestRegionId > maxRegCount)
    {
      maxRegId = pair.first;
      maxRegCount = pair.second.nearestRegionId;
      maxDistSum = pair.second.distance;
    }
  }

  if (maxRegCount == 0) // no children, return own value
    return getValue();

  return RegionInfo(maxRegId, maxDistSum / maxRegCount);
}

NearestRegionOcTree::NearestRegionOcTree(double in_resolution, double max_dist)
 : octomap::OcTreeBase<NearestRegionOcTreeNode>(in_resolution), max_dist(max_dist)
{
  nearestRegionOcTreeMemberInit.ensureLinking();
}

std::shared_ptr<NearestRegionOcTree> NearestRegionOcTree::createFromCountringOctree(const CountingOcTree &in_tree, double max_dist)
{
  struct KeyWithRegInf
  {
    KeyWithRegInf(const octomap::OcTreeKey &key, const RegionInfo &regInf) : key(key), regInf(regInf) {}
    KeyWithRegInf(const octomap::OcTreeKey &key, unsigned int regId, float regDist) : key(key), regInf(regId, regDist) {}

    octomap::OcTreeKey key;
    RegionInfo regInf;

    bool operator< (const KeyWithRegInf &rhs) const
    {
      return regInf.distance > rhs.regInf.distance; // sort by inverse distance
    }
  };

  typedef boost::heap::fibonacci_heap<KeyWithRegInf> ValueHeap;
  typedef octomap::unordered_ns::unordered_map<octomap::OcTreeKey, ValueHeap::handle_type, octomap::OcTreeKey::KeyHash> KeyHandleMap;

  ValueHeap nodeVals;
  KeyHandleMap openKeys;
  octomap::KeySet processedKeys;

  double resolution = in_tree.getResolution();
  std::shared_ptr<NearestRegionOcTree> out_tree(new NearestRegionOcTree(resolution, max_dist));

  for (auto it = in_tree.begin_leafs(), end = in_tree.end_leafs(); it != end; it++)
  {
    ValueHeap::handle_type handle = nodeVals.push(KeyWithRegInf(it.getKey(), it->getCount(), 0));
    openKeys[it.getKey()] = handle;
  }

  while(!nodeVals.empty())
  {
    KeyWithRegInf minDistEl = nodeVals.top();
    octomap::OcTreeKey curKey = minDistEl.key;
    nodeVals.pop();
    openKeys.erase(curKey);
    out_tree->updateNode(curKey, minDistEl.regInf);
    processedKeys.insert(curKey);

    for (int i = 0; i < NB_26; i++)
    {
      int coordDiff = nbCoordDiff[i];
      RegionInfo newNeighbourRegInf = minDistEl.regInf;
      if (coordDiff == 1) newNeighbourRegInf.distance += resolution;
      else if (coordDiff == 2) newNeighbourRegInf.distance += resolution * sqrt(2);
      else /* coordDiff == 3*/ newNeighbourRegInf.distance += resolution * sqrt(3);

      if (newNeighbourRegInf.distance > max_dist) continue; // new value is out of influence radius

      octomap::OcTreeKey neighbourKey(curKey[0] + nbLut[i][0], curKey[1] + nbLut[i][1], curKey[2] + nbLut[i][2]);

      if (processedKeys.find(neighbourKey) != processedKeys.end()) continue; // already processed keys can be ignored

      auto it = openKeys.find(neighbourKey);
      if (it != openKeys.end()) // key already known, check if update needed
      {
        ValueHeap::handle_type handle = it->second;
        if (newNeighbourRegInf.distance < (*handle).regInf.distance) // only update if new value would be higher
        {
          (*handle).regInf = newNeighbourRegInf;
          nodeVals.increase(handle);
        }
      }
      else // otherwise, enter new key to map
      {
        ValueHeap::handle_type handle = nodeVals.push(KeyWithRegInf(neighbourKey, newNeighbourRegInf));
        openKeys[neighbourKey] = handle;
      }
    }
  }
  return out_tree;
}

NearestRegionOcTreeNode* NearestRegionOcTree::updateNode(const octomap::point3d& p, const RegionInfo &regInf)
{
  octomap::OcTreeKey key;
  if (!coordToKeyChecked(p, key)) return NULL;
  return updateNode(key, regInf);
}

NearestRegionOcTreeNode *NearestRegionOcTree::updateNode(const octomap::OcTreeKey& k, const RegionInfo &regInf)
{
  if (root == NULL)
  {
    root = new NearestRegionOcTreeNode();
    tree_size++;
  }

  std::vector<NearestRegionOcTreeNode*> nodes;
  nodes.reserve(tree_depth);
  NearestRegionOcTreeNode* curNode (root);
  nodes.push_back(curNode);

  // follow or construct nodes down to last level...
  for (int i=(tree_depth-1); i>=0; i--)
  {
    unsigned int pos = computeChildIdx(k, i);

    // requested node does not exist
    if (!nodeChildExists(curNode, pos))
    {
      curNode = createNodeChild(curNode, pos);
    }
    else // descent tree
    {
      curNode = getNodeChild(curNode, pos);
    }
    nodes.push_back(curNode);
  }

  if (regInf != curNode->getValue())
  {
    curNode->setValue(regInf);
    // traverse back to update parent nodes accordingly
    for (int i = nodes.size()-2; i >= 0; i--)
    {
      nodes[i]->updateValChildren();
    }
  }

  return curNode;
}

NearestRegionOcTree::StaticMemberInitializer NearestRegionOcTree::nearestRegionOcTreeMemberInit;

} // namespace octomap_vpp
