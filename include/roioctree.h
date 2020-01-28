#ifndef ROIOCTREE_H
#define ROIOCTREE_H

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include "octomap/octomap_utils.h"
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <ros/ros.h>

#include "inflatedroioctree.h"

class RoiOcTreeNode : public octomap::OcTreeNode
{
public:
  RoiOcTreeNode() : OcTreeNode(), roiValue(0) {}

  RoiOcTreeNode(const RoiOcTreeNode& rhs) : OcTreeNode(rhs), roiValue(rhs.roiValue) {}

  bool operator==(const RoiOcTreeNode& rhs) const{
    return (rhs.value == value && rhs.roiValue == roiValue);
  }

  void copyData(const RoiOcTreeNode& from){
    OcTreeNode::copyData(from);
    roiValue = from.roiValue;
  }

  std::ostream& writeData(std::ostream &s) const
  {
    s.write((const char*) &value, sizeof(value)); // occupancy
    s.write((const char*) &roiValue, sizeof(roiValue)); // roi value

    return s;
  }

  std::istream& readData(std::istream &s)
  {
    s.read((char*) &value, sizeof(value)); // occupancy
    s.read((char*) &roiValue, sizeof(roiValue)); // roi value

    return s;
  }

  // -- node roi value  ----------------------------

  /// \return priority probability of node
  inline double getRoiProb() const
  {
    return octomap::probability(roiValue);
  }

  /// \return log odds representation of occupancy probability of node
  inline float getRoiLogOdds() const
  {
    return roiValue;
  }

  /// sets log odds occupancy of node
  inline void setRoiLogOdds(float l)
  {
    roiValue = l;
  }

  /**
   * @return mean of all children's occupancy probabilities, in log odds
   */
  double getMeanChildRoiLogOdds() const;

  /**
   * @return maximum of children's occupancy probabilities, in log odds
   */
  float getMaxChildRoiLogOdds() const;

  /// update this node's occupancy according to its children's maximum occupancy
  inline void updateRoiValChildren()
  {
    this->setRoiLogOdds(this->getMeanChildRoiLogOdds());  // conservative
  }

  /// adds p to the node's logOdds value (with no boundary / threshold checking!)
  void addRoiValue(const float& p);

protected:
  float roiValue;
};

class RoiOcTree : public octomap::OccupancyOcTreeBase <RoiOcTreeNode>
{
public:
  RoiOcTree(double resolution);

  virtual RoiOcTree* create() const {return new RoiOcTree(resolution); }

  virtual std::string getTreeType() const {return "RoiOcTree";}

  void insertRegionScan(const octomap::Pointcloud &regionPoints, const octomap::Pointcloud &offRegionPoints)
  {
    //std::unordered_set<RoiOcTreeNode*> regionNodes;
    //std::unordered_set<RoiOcTreeNode*> offRegionNodes;
    octomap::KeySet regionNodes;
    octomap::KeySet offRegionNodes;
    for (size_t i = 0; i < regionPoints.size(); i++)
    {
      const octomap::point3d &p = regionPoints[i];
      //RoiOcTreeNode *node = this->search(p);
      //if (node != NULL)
      //  regionNodes.insert(node);

      octomap::OcTreeKey key;
      if (coordToKeyChecked(p, key))
        regionNodes.insert(key);
    }

    for (size_t i = 0; i < offRegionPoints.size(); i++)
    {
      const octomap::point3d &p = offRegionPoints[i];
      //RoiOcTreeNode *node = this->search(p);
      //if (node != NULL)
      //  offRegionNodes.insert(node);

      octomap::OcTreeKey key;
      if (coordToKeyChecked(p, key))
        offRegionNodes.insert(key);
    }

    ROS_INFO_STREAM("Key set sizes: " << regionNodes.size() << ", " << offRegionNodes.size());
    ROS_INFO_STREAM("Hit/miss prob: " << this->prob_miss_log << ", " << this->prob_hit_log);

    for (const octomap::OcTreeKey &key : regionNodes)
    {
      updateNodeRoi(key, true, false);
    }

    for (const octomap::OcTreeKey &key : offRegionNodes)
    {
      updateNodeRoi(key, false, false);
    }
  }

  RoiOcTreeNode* updateNodeRoi(const octomap::OcTreeKey& key, float log_odds_update, bool lazy_eval) {
     // early abort (no change will happen).
     // may cause an overhead in some configuration, but more often helps
     RoiOcTreeNode* leaf = this->search(key);
     // no change: node already at threshold
     if (leaf
         && ((log_odds_update >= 0 && leaf->getRoiLogOdds() >= this->clamping_thres_max)
         || ( log_odds_update <= 0 && leaf->getRoiLogOdds() <= this->clamping_thres_min)))
     {
       return leaf;
     }

     bool createdRoot = false;
     if (this->root == NULL){
       //return leaf; // don't create new nodes
       this->root = new RoiOcTreeNode();
       this->tree_size++;
       createdRoot = true;
     }

     return updateNodeRoiRecurs(this->root, createdRoot, key, 0, log_odds_update, lazy_eval);
   }

  RoiOcTreeNode* updateNodeRoi(const octomap::OcTreeKey& key, bool isRoi, bool lazy_eval)
  {
    float logOdds = this->prob_miss_log;
    if (isRoi)
      logOdds = this->prob_hit_log;

    return updateNodeRoi(key, logOdds, lazy_eval);
  }

  RoiOcTreeNode* updateNodeRoiRecurs(RoiOcTreeNode* node, bool node_just_created, const octomap::OcTreeKey& key, unsigned int depth, const float& log_odds_update, bool lazy_eval)
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
          return updateNodeRoiRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, log_odds_update, lazy_eval);
        else {
          RoiOcTreeNode* retval = updateNodeRoiRecurs(this->getNodeChild(node, pos), created_node, key, depth+1, log_odds_update, lazy_eval);
          // prune node if possible, otherwise set own probability
          // note: combining both did not lead to a speedup!
          if (this->pruneNode(node)){
            // return pointer to current parent (pruned), the just updated node no longer exists
            retval = node;
          } else{
            node->updateRoiValChildren();
          }

          return retval;
        }
      }

      // at last level, update node, end of recursion
      else {
        //if (use_change_detection) {
        bool roiBefore = this->isNodeROI(node);
        updateNodeRoiLogOdds(node, log_odds_update);
        bool roiAfter = this->isNodeROI(node);

        if (roiAfter && !roiBefore) // roi added
        {
          octomap::KeySet::iterator it = deleted_rois.find(key);
          if (it != deleted_rois.end())
          {
            deleted_rois.erase(it);
          }
          added_rois.insert(key);
          roi_keys.insert(key);
        }
        else if (!roiAfter && roiBefore) // roi deleted
        {
          octomap::KeySet::iterator it = added_rois.find(key);
          if (it != added_rois.end())
          {
            added_rois.erase(it);
          }
          deleted_rois.insert(key);
          it = roi_keys.find(key);
          if (it != roi_keys.end())
          {
            roi_keys.erase(it);
          }
        }

        /*if (node_just_created){  // new node
          changed_keys.insert(std::pair<octomap::OcTreeKey,bool>(key, true));
        } else if (roiBefore != this->isNodeROI(node)) {  // occupancy changed, track it
          octomap::KeyBoolMap::iterator it = changed_keys.find(key);
          if (it == changed_keys.end())
            changed_keys.insert(std::pair<octomap::OcTreeKey,bool>(key, false));
          else if (it->second == false)
            changed_keys.erase(it);
        }
        } else {
          //float lo_before = node->getRoiLogOdds();
          updateNodeRoiLogOdds(node, log_odds_update);
          //if (log_odds_update > 0)
          //  ROS_INFO_STREAM("Updated LOs from " << lo_before << " to " << node->getRoiLogOdds() << " (Prob: " << node->getRoiProb() << "CV: " << log_odds_update << ")");
        }*/
        return node;
      }
    }

  inline octomap::KeySet getRoiKeys()
  {
    /*std::vector<octomap::OcTreeKey> res;
    for (auto it = this->begin_leafs(), end = this->end_leafs(); it != end; it++)
    {
      if(isNodeROI(*it))
        res.push_back(it.getKey());
    }
    return res;*/
    return roi_keys;
  }

  inline size_t getRoiSize() {return roi_keys.size();}
  inline size_t getAddedRoiSize() {return added_rois.size();}
  inline size_t getDeletedRoiSize() {return deleted_rois.size();}

  inline std::shared_ptr<InflatedRoiOcTree> getInflatedRois() {return inflated_rois;}

  std::shared_ptr<InflatedRoiOcTree> computeInflatedRois()
  {
    //std::vector<octomap::OcTreeKey> roiKeys = getRoiKeys();
    //if (roi_keys.size() == 0)
    //  return NULL;

    if (added_rois.empty() && deleted_rois.empty()) // no changes, return existing tree
      return inflated_rois;

    bool full_construct = false;
    if (inflated_rois == nullptr || !deleted_rois.empty())
    {
      inflated_rois.reset(new InflatedRoiOcTree(this->resolution));
      full_construct = true;
    }

    //InflatedRoiOcTree *inflatedTree = new InflatedRoiOcTree(this->resolution);

    typedef std::unordered_map<octomap::OcTreeKey, float, octomap::OcTreeKey::KeyHash> KeyFloatMap;
    typedef std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> KeySet;

    /*struct cmp_fkey_pair {
      bool operator() (const std::pair<float, octomap::OcTreeKey>& lhs, const std::pair<float, octomap::OcTreeKey>& rhs) const
      {return lhs.first > rhs.first;}
    };
    typedef std::set<std::pair<float, octomap::OcTreeKey>, cmp_fkey_pair> SortedFloatKeySet;*/

    KeyFloatMap keyMap;
    //SortedFloatKeySet floatKeySet;
    KeySet processedKeys;

    float maxVal = inflated_rois->getMaxRoiVal();
    float stepReduction = maxVal / inflated_rois->getInfluenceRadius() * inflated_rois->getResolution() ;
    ROS_INFO_STREAM("Max Val: " << maxVal << "; Step reduction: " << stepReduction);
    if (full_construct)
    {
      for (const octomap::OcTreeKey &key : roi_keys)
      {
        keyMap[key] = maxVal;
        //floatKeySet.insert(std::make_pair(maxVal, key));
      }
    }
    else // only use added rois for update
    {
      for (const octomap::OcTreeKey &key : added_rois)
      {
        keyMap[key] = maxVal;
        //floatKeySet.insert(std::make_pair(maxVal, key));
      }
    }
    while(!keyMap.empty())
    {
      KeyFloatMap::iterator maxIt = std::max_element
      (
          keyMap.begin(), keyMap.end(),
          [] (const KeyFloatMap::value_type &p1, const KeyFloatMap::value_type &p2) {
              return p1.second < p2.second;
          }
      );
      octomap::OcTreeKey curKey = maxIt->first;
      float curVal = maxIt->second;
      keyMap.erase(maxIt);
      inflated_rois->updateNodeVal(curKey, curVal, true, !full_construct);
      processedKeys.insert(curKey);
      /*SortedFloatKeySet::iterator maxIt = floatKeySet.begin();
      octomap::OcTreeKey curKey = maxIt->second;
      float curVal = maxIt->first;
      floatKeySet.erase(maxIt);
      keyMap.erase(curKey);
      inflated_rois->updateNodeVal(curKey, curVal, true, !full_construct);
      processedKeys.insert(curKey);*/

      for (int i = -1; i <= 1; i++)
      {
        for (int j = -1; j <= 1; j++)
        {
          for (int k = -1; k <= 1; k++)
          {
            int coordDiff = std::abs(i) + std::abs(j) + std::abs(k);
            float newNeighbourVal = 0;
            if (coordDiff == 0) continue; // only process neighbours, not the node itself
            else if (coordDiff == 1) newNeighbourVal = curVal - stepReduction;
            else if (coordDiff == 2) newNeighbourVal = curVal - stepReduction * sqrt(2);
            else /* coordDiff == 3*/ newNeighbourVal = curVal - stepReduction * sqrt(3);
            if (newNeighbourVal <= 0) continue; // new value is out of influence radius

            octomap::OcTreeKey neighbourKey(curKey[0] + i, curKey[1] + j, curKey[2] + k);
            if (processedKeys.find(neighbourKey) != processedKeys.end()) continue; // already processed keys can be ignored

            KeyFloatMap::iterator it = keyMap.find(neighbourKey);
            if (it != keyMap.end()) // key already known, check if update needed
            {
              if (newNeighbourVal > it->second) // only update if new value would be higher
              {
                //SortedFloatKeySet::iterator set_it = floatKeySet.find(std::make_pair(it->second, it->first));
                //floatKeySet.erase(set_it);
                it->second = newNeighbourVal;
                //floatKeySet.insert(std::make_pair(it->second, it->first));
              }
            }
            else // otherwise, enter new key to map
            {
              keyMap[neighbourKey] = newNeighbourVal;
              //floatKeySet.insert(std::make_pair(newNeighbourVal, neighbourKey));
            }
          }
        }
      }
    }
    inflated_rois->updateInnerVals();
    added_rois.clear();
    deleted_rois.clear();
    return inflated_rois;
  }

  void updateNodeRoiLogOdds(RoiOcTreeNode* node, const float& update) const;

  /// queries whether a node is occupied according to the tree's parameter for "occupancy"
  inline bool isNodeROI(const RoiOcTreeNode* node) const{
    return (node->getRoiLogOdds() >= this->roi_prob_thres_log);
  }

  /// queries whether a node is occupied according to the tree's parameter for "occupancy"
  inline bool isNodeROI(const RoiOcTreeNode& node) const{
    return (node.getRoiLogOdds() >= this->roi_prob_thres_log);
  }


protected:
  float roi_prob_thres_log;
  std::shared_ptr<InflatedRoiOcTree> inflated_rois;

  octomap::KeySet added_rois;
  octomap::KeySet deleted_rois;
  octomap::KeySet roi_keys;

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
      RoiOcTree* tree = new RoiOcTree(0.1);
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

#endif // ROIOCTREE_H
