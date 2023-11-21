#pragma once

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include "octomap/octomap_utils.h"
#include <vector>
#include <string>
#include <octomap/OcTreeKey.h>
#include <queue>
#include <memory>

#include "roioctree_utils.h"

namespace octomap_vpp
{

class SemanticOcTreeNode : public octomap::OcTreeNode
{
  friend class SemanticOcTree;

public:
  SemanticOcTreeNode() : OcTreeNode() {}

  SemanticOcTreeNode(const SemanticOcTreeNode& rhs) : OcTreeNode(rhs), classProbs(rhs.classProbs) {}

  bool operator==(const SemanticOcTreeNode& rhs) const{
    return (rhs.value == value && rhs.classProbs == classProbs);
  }

  void copyData(const SemanticOcTreeNode& from){
    OcTreeNode::copyData(from);
    classProbs = from.classProbs;
  }

  std::ostream& writeData(std::ostream &s) const;

  std::istream& readData(std::istream &s);

  // -- node roi value  ----------------------------

  /// \return class probability of node
  inline double getClassProb(uint8_t class_id) const
  {
    auto it = classProbs.find(class_id);
    if (it != classProbs.end())
      return octomap::probability(it->second);
    else
      return 0;
  }

  /// \return log odds representation of class probability of node
  inline float getClassLogOdds(uint8_t class_id) const
  {
    auto it = classProbs.find(class_id);
    if (it != classProbs.end())
      return it->second;
    else
      return -std::numeric_limits<float>::infinity();
  }

  /// sets class log odds of node
  inline void setClassLogOdds(uint8_t class_id, float log_odds)
  {
    classProbs[class_id] = log_odds;
  }

  /// update this node's class probabilities according to its children's values
  inline void updateClassValuesFromChildren()
  {
    if (children == nullptr)
      return;

    for (unsigned int i=0; i<8; i++)
    {
      if (children[i] == nullptr)
        continue;
      
      SemanticOcTreeNode *child = static_cast<SemanticOcTreeNode*>(children[i]);
      for (auto it = child->classProbs.begin(); it != child->classProbs.end(); it++)
      {
        auto it_this = classProbs.find(it->first);
        if (it_this != classProbs.end()) // use max of child and this
          it_this->second = std::max(it_this->second, it->second);
        else
          classProbs[it->first] = it->second;
      }
    }
  }

  /// adds to the node's class logOdds value (with no boundary / threshold checking!)
  void addClassLogOdds(uint8_t class_id, float log_odds)
  {
    classProbs[class_id] += log_odds;
  }

protected:
  uint8_t num_classes;
  std::unordered_map<uint8_t, float> classProbs;
};

class SemanticOcTree : public octomap::OccupancyOcTreeBase <SemanticOcTreeNode>
{
public:
  SemanticOcTree(double resolution);

  virtual SemanticOcTree* create() const {return new SemanticOcTree(resolution); }

  virtual std::string getTreeType() const {return "SemanticOcTree";}

  // override default to read class labels
  std::istream& readData(std::istream &s) override;

  // override default to write class labels
  std::ostream& writeData(std::ostream &s) const override;

  //void insertRegionScan(const octomap::Pointcloud &regionPoints, const octomap::Pointcloud &offRegionPoints);

  SemanticOcTreeNode* updateNodeClass(const octomap::OcTreeKey& key, uint8_t class_id, float log_odds_update, bool lazy_eval);

  SemanticOcTreeNode* updateNodeClass(const octomap::OcTreeKey& key, uint8_t class_id, bool lazy_eval);

  SemanticOcTreeNode* updateNodeClassRecurs(SemanticOcTreeNode* node, bool node_just_created, const octomap::OcTreeKey& key, unsigned int depth, uint8_t class_id, float log_odds_update, bool lazy_eval);

  void updateNodeClassLogOdds(SemanticOcTreeNode* node, const float& update) const;

  /// queries the node's most likely class
  inline int getNodeClass(const SemanticOcTreeNode *node) const{
    int class_id = -1;
    float max_prob = -std::numeric_limits<float>::infinity();
    for (auto it = node->classProbs.begin(); it != node->classProbs.end(); it++)
    {
      if (it->second > max_prob)
      {
        max_prob = it->second;
        class_id = static_cast<int>(it->first);
      }
    }
    return class_id;
  }

  /// queries the node's most likely class
  inline int getNodeClass(const SemanticOcTreeNode &node) const{
    int class_id = -1;
    float max_prob = -std::numeric_limits<float>::infinity();
    for (auto it = node.classProbs.begin(); it != node.classProbs.end(); it++)
    {
      if (it->second > max_prob)
      {
        max_prob = it->second;
        class_id = static_cast<int>(it->first);
      }
    }
    return class_id;
  }

  inline float keyToLogOdds(const octomap::OcTreeKey &key)
  {
    SemanticOcTreeNode *node = search(key);
    float logOdds = 0;
    if (node != NULL)
      logOdds = node->getLogOdds();
    return logOdds;
  }

  inline double keyToProbability(const octomap::OcTreeKey &key)
  {
    SemanticOcTreeNode *node = search(key);
    double p = 0;
    if (node != NULL)
      p = node->getOccupancy();
    return p;
  }


protected:
  std::vector<std::string> class_names;

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
      SemanticOcTree* tree = new SemanticOcTree(0.1);
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

} // namespace octomap_vpp