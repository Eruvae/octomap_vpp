#include "octomap_vpp/SemanticOcTree.h"

namespace octomap_vpp
{

std::ostream& SemanticOcTreeNode::writeData(std::ostream& s) const
{
  uint8_t num_classes = static_cast<uint8_t>(classProbs.size());
  s.write((const char*)&value, sizeof(value));              // occupancy
  s.write((const char*)&num_classes, sizeof(num_classes));  // size of classProbs
  for (auto it = classProbs.begin(); it != classProbs.end(); it++)
  {
    s.write((const char*)&it->first, sizeof(it->first));    // class id
    s.write((const char*)&it->second, sizeof(it->second));  // class prob
  }
  return s;
}

std::istream& SemanticOcTreeNode::readData(std::istream& s)
{
  uint8_t num_classes;
  s.read((char*)&value, sizeof(value));              // occupancy
  s.read((char*)&num_classes, sizeof(num_classes));  // size of classProbs
  for (size_t i = 0; i < static_cast<size_t>(num_classes); i++)
  {
    uint8_t class_id;
    float class_prob;
    s.read((char*)&class_id, sizeof(class_id));      // class id
    s.read((char*)&class_prob, sizeof(class_prob));  // class prob
    classProbs[class_id] = class_prob;
  }
  return s;
}

SemanticOcTree::SemanticOcTree(double resolution) : octomap::OccupancyOcTreeBase<SemanticOcTreeNode>(resolution)
{
  ocTreeMemberInit.ensureLinking();
}

// override default to read class labels
std::istream& SemanticOcTree::readData(std::istream& s)
{
  if (!s.good())
  {
    OCTOMAP_WARNING_STR(__FILE__ << ":" << __LINE__ << "Warning: Input filestream not \"good\"");
  }

  this->tree_size = 0;
  size_changed = true;

  // tree needs to be newly created or cleared externally
  if (root)
  {
    OCTOMAP_ERROR_STR("Trying to read into an existing tree.");
    return s;
  }

  // read semantic data
  uint8_t num_classes;
  s.read((char*)&num_classes, sizeof(num_classes));
  for (size_t i = 0; i < static_cast<size_t>(num_classes); i++)
  {
    uint8_t label_size;
    s.read((char*)&label_size, sizeof(label_size));
    std::string label;
    label.resize(label_size);
    s.read((char*)label.data(), label_size);
    class_names.push_back(label);
  }

  // read nodes
  root = new SemanticOcTreeNode();
  readNodesRecurs(root, s);

  tree_size = calcNumNodes();  // compute number of nodes
  return s;
}

// override default to write class labels
std::ostream& SemanticOcTree::writeData(std::ostream& s) const
{
  if (root)
  {
    uint8_t num_classes = static_cast<uint8_t>(class_names.size());
    // write semantic data
    s.write((const char*)&num_classes, sizeof(num_classes));
    for (auto it = class_names.begin(); it != class_names.end(); it++)
    {
      uint8_t label_size = static_cast<uint8_t>(it->size());
      s.write((const char*)&label_size, sizeof(label_size));
      s.write(it->data(), label_size);
    }
    // write nodes
    writeNodesRecurs(root, s);
  }
  return s;
}

SemanticOcTreeNode* setNodeClass(const octomap::OcTreeKey &key, uint8_t class_id, bool lazy_eval = false);
  SemanticOcTreeNode* setNodeClass(const octomap::point3d &value, uint8_t class_id, bool lazy_eval = false);
  SemanticOcTreeNode* setNodeClass(double x, double y, double z, uint8_t class_id, bool lazy_eval = false);

SemanticOcTreeNode* SemanticOcTree::setNodeClass(const octomap::OcTreeKey &key, uint8_t class_id, bool lazy_eval)
{
  bool createdRoot = false;
  if (this->root == NULL){
    this->root = new SemanticOcTreeNode();
    this->tree_size++;
    createdRoot = true;
  }

  return updateNodeClassRecurs(this->root, createdRoot, key, 0, class_id, this->prob_hit_log, lazy_eval);
}

SemanticOcTreeNode* SemanticOcTree::setNodeClass(const octomap::point3d &value, uint8_t class_id, bool lazy_eval)
{
  octomap::OcTreeKey key;
  if (!this->coordToKeyChecked(value, key))
    return NULL;

  return setNodeClass(key, class_id, lazy_eval);
}

SemanticOcTreeNode* SemanticOcTree::setNodeClass(double x, double y, double z, uint8_t class_id, bool lazy_eval)
{
  octomap::OcTreeKey key;
  if (!this->coordToKeyChecked(x, y, z, key))
    return NULL;

  return setNodeClass(key, class_id, lazy_eval);
}

SemanticOcTreeNode* SemanticOcTree::updateNodeClass(const octomap::OcTreeKey& key, uint8_t class_id,
                                                    float log_odds_update, bool lazy_eval)
{
  // early abort (no change will happen).
  // may cause an overhead in some configuration, but more often helps
  SemanticOcTreeNode* leaf = this->search(key);
  // no change: node already at threshold
  if (leaf && ((log_odds_update >= 0 && leaf->getClassLogOdds(class_id) >= this->clamping_thres_max) ||
               (log_odds_update <= 0 && leaf->getClassLogOdds(class_id) <= this->clamping_thres_min)))
  {
    return leaf;
  }

  bool createdRoot = false;
  if (this->root == nullptr)
  {
    // return leaf; // don't create new nodes
    this->root = new SemanticOcTreeNode();
    this->tree_size++;
    createdRoot = true;
  }

  return updateNodeClassRecurs(this->root, createdRoot, key, 0, class_id, log_odds_update, lazy_eval);
}

SemanticOcTreeNode* SemanticOcTree::updateNodeClass(const octomap::OcTreeKey& key, uint8_t class_id, bool lazy_eval)
{
  return updateNodeClass(key, class_id, this->prob_hit_log, lazy_eval);
}

SemanticOcTreeNode* SemanticOcTree::updateNodeClassRecurs(SemanticOcTreeNode* node, bool node_just_created,
                                                          const octomap::OcTreeKey& key, unsigned int depth,
                                                          uint8_t class_id, float log_odds_update, bool lazy_eval)
{
  bool created_node = false;

  assert(node);

  // follow down to last level
  if (depth < this->tree_depth)
  {
    unsigned int pos = computeChildIdx(key, this->tree_depth - 1 - depth);
    if (!this->nodeChildExists(node, pos))
    {
      // child does not exist, but maybe it's a pruned node?
      if (!this->nodeHasChildren(node) && !node_just_created)
      {
        // current node does not have children AND it is not a new node
        // -> expand pruned node
        this->expandNode(node);
      }
      else
      {
        // not a pruned node, create requested child
        this->createNodeChild(node, pos);
        created_node = true;
      }
    }

    if (lazy_eval)
      return updateNodeClassRecurs(this->getNodeChild(node, pos), created_node, key, depth + 1, class_id, log_odds_update,
                                 lazy_eval);
    else
    {
      SemanticOcTreeNode* retval =
          updateNodeClassRecurs(this->getNodeChild(node, pos), created_node, key, depth + 1, class_id, log_odds_update, lazy_eval);
      // prune node if possible, otherwise set own probability
      // note: combining both did not lead to a speedup!
      if (this->pruneNode(node))
      {
        // return pointer to current parent (pruned), the just updated node no longer exists
        retval = node;
      }
      else
      {
        node->updateClassValuesFromChildren();
      }

      return retval;
    }
  }

  // at last level, update node, end of recursion
  else
  {
    // TODO
    //updateNodeClassLogOdds(node, log_odds_update);
    return node;
  }
}

void SemanticOcTree::updateNodeClassLogOdds(SemanticOcTreeNode* node, const float& update) const
{
}

SemanticOcTree::StaticMemberInitializer SemanticOcTree::ocTreeMemberInit;

}  // namespace octomap_vpp