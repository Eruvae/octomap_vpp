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
#include <memory>
//#include <ros/ros.h>

#include "InflatedRoiOcTree.h"
#include "roioctree_utils.h"

namespace octomap_vpp
{

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

  void insertRegionScan(const octomap::Pointcloud &regionPoints, const octomap::Pointcloud &offRegionPoints);

  RoiOcTreeNode* updateNodeRoi(const octomap::OcTreeKey& key, float log_odds_update, bool lazy_eval);

  RoiOcTreeNode* updateNodeRoi(const octomap::OcTreeKey& key, bool isRoi, bool lazy_eval);

  RoiOcTreeNode* updateNodeRoiRecurs(RoiOcTreeNode* node, bool node_just_created, const octomap::OcTreeKey& key, unsigned int depth, const float& log_odds_update, bool lazy_eval);

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

  std::vector<octomap::point3d> getClusterCenters(size_t min_cluster_size = 10)
  {
    std::vector<std::vector<octomap::OcTreeKey>> clusters = computeClusters(min_cluster_size);
    std::vector<octomap::point3d> clusterCenters;
    for (auto &cluster : clusters)
    {
      octomap::point3d sum;
      for (auto &key : cluster)
      {
        sum += keyToCoord(key);
      }
      sum /= cluster.size();
      clusterCenters.push_back(sum);
    }
    return clusterCenters;
  }

  std::pair<std::vector<octomap::point3d>, std::vector<octomap::point3d>> getClusterCentersWithVolume(size_t min_cluster_size = 10)
  {
    std::vector<std::vector<octomap::OcTreeKey>> clusters = computeClusters(min_cluster_size);
    std::pair<std::vector<octomap::point3d>, std::vector<octomap::point3d>> clusterCenters;
    const float FMIN = -std::numeric_limits<float>::infinity();
    const float FMAX = std::numeric_limits<float>::infinity();
    for (auto &cluster : clusters)
    {
      octomap::point3d sum, min(FMAX, FMAX, FMAX), max(FMIN, FMIN, FMIN);
      for (auto &key : cluster)
      {
        octomap::point3d coord = keyToCoord(key);
        sum += coord;
        if (coord.x() < min.x()) min.x() = coord.x();
        if (coord.y() < min.y()) min.y() = coord.y();
        if (coord.z() < min.z()) min.z() = coord.z();
        if (coord.x() > max.x()) max.x() = coord.x();
        if (coord.y() > max.y()) max.y() = coord.y();
        if (coord.z() > max.z()) max.z() = coord.z();
      }
      sum /= cluster.size();
      //double vol = (max.x() - min.x()) * (max.y() - min.y()) * (max.z() - min.z());
      clusterCenters.first.push_back(sum);
      clusterCenters.second.push_back(max - min);
    }
    return clusterCenters;
  }

  std::vector<std::vector<octomap::OcTreeKey>> computeClusters(size_t min_cluster_size = 10)
  {
    std::vector<std::vector<octomap::OcTreeKey>> clusters;
    if (roi_keys.size() == 0) return clusters;
    octomap::KeySet roisToProcess = roi_keys;
    while (!roisToProcess.empty())
    {
      std::vector<octomap::OcTreeKey> currentCluster;
      auto it = roisToProcess.begin();
      std::deque<octomap::OcTreeKey> nbsToProcess;
      nbsToProcess.push_back(*it);
      roisToProcess.erase(it);
      while (!nbsToProcess.empty())
      {
        octomap::OcTreeKey curKey = nbsToProcess.front();
        nbsToProcess.pop_front();
        currentCluster.push_back(curKey);
        for (size_t i = 0; i < 18; i++)
        {
          //octomap::OcTreeKey neighbourKey(curKey);
          //neighbourKey[i/2] += i%2 ? 1 : -1;
          octomap::OcTreeKey neighbourKey(curKey[0] + nb18Lut[i][0], curKey[1] + nb18Lut[i][1], curKey[2] + nb18Lut[i][2]);
          it = roisToProcess.find(neighbourKey);
          if (it != roisToProcess.end())
          {
            nbsToProcess.push_back(*it);
            roisToProcess.erase(it);
          }
        }
      }
      if (currentCluster.size() >= min_cluster_size)
        clusters.push_back(currentCluster);
    }
    return clusters;
  }

  double getDiscoveredRatio(const octomap::point3d &min, const octomap::point3d &max)
  {
    size_t known_cnt = 0;
    octomap::OcTreeKey minKey, maxKey;
    if (!this->coordToKeyChecked(min, minKey) || !this->coordToKeyChecked(max, maxKey))
      return -1;

    size_t total_cnt = (maxKey[0] - minKey[0]) * (maxKey[1] - minKey[1]) * (maxKey[2] - minKey[2]);
    for (leaf_bbx_iterator it = this->begin_leafs_bbx(minKey, maxKey), end = this->end_leafs_bbx(); it != end; it++)
    {
      if (it->getLogOdds() != 0) // known; could be replaced with range
        known_cnt++;
    }
    return (double)known_cnt / (double)total_cnt;
  }

  inline size_t getRoiSize() {return roi_keys.size();}
  inline size_t getAddedRoiSize() {return added_rois.size();}
  inline size_t getDeletedRoiSize() {return deleted_rois.size();}

  inline std::shared_ptr<InflatedRoiOcTree> getInflatedRois() {return inflated_rois;}

  std::shared_ptr<InflatedRoiOcTree> computeInflatedRois();

  void updateNodeRoiLogOdds(RoiOcTreeNode* node, const float& update) const;

  /// queries whether a node is occupied according to the tree's parameter for "occupancy"
  inline bool isNodeROI(const RoiOcTreeNode* node) const{
    return (node->getRoiLogOdds() >= this->roi_prob_thres_log);
  }

  /// queries whether a node is occupied according to the tree's parameter for "occupancy"
  inline bool isNodeROI(const RoiOcTreeNode& node) const{
    return (node.getRoiLogOdds() >= this->roi_prob_thres_log);
  }

  inline float keyToLogOdds(const octomap::OcTreeKey &key)
  {
    RoiOcTreeNode *node = search(key);
    float logOdds = 0;
    if (node != NULL)
      logOdds = node->getLogOdds();
    return logOdds;
  }

  inline double keyToProbability(const octomap::OcTreeKey &key)
  {
    RoiOcTreeNode *node = search(key);
    double p = 0;
    if (node != NULL)
      p = node->getOccupancy();
    return p;
  }

  inline double keyToRoiVal(const octomap::OcTreeKey &key) // computeInflatedRois must be called first
  {
    if (inflated_rois == nullptr)
      return 0;

    bool rescale_key = this->resolution != inflated_rois->getResolution();
    InflatedRoiOcTreeNode *node = inflated_rois->search(rescale_key ? inflated_rois->coordToKey(this->keyToCoord(key)) : key);
    double roi_val = 0.0;
    if (node != NULL)
      roi_val = node->getValue() / inflated_rois->getMaxRoiVal();
    return roi_val;
  }

  double computeCellEntropy(const octomap::OcTreeKey &key)
  {
    RoiOcTreeNode *node = search(key);
    double p = 0.5; // defaults to 0.5 if cell not known
    if (node != NULL)
      p = node->getOccupancy();

    return probabilityToEntropy(p);
  }

  double computeExpectedInformationGain(const octomap::OcTreeKey &key)
  {
    RoiOcTreeNode *node = search(key);
    float logOdds = 0;
    if (node != NULL)
      logOdds = node->getLogOdds();

    double p = octomap::probability(logOdds);
    double ent_cur = probabilityToEntropy(p);
    double ent_hit = logOddsToEntropy(logOdds + prob_hit_log);
    double ent_miss = logOddsToEntropy(logOdds + prob_miss_log);

    return p * std::abs(ent_hit - ent_cur) + (1-p) * std::abs(ent_miss - ent_cur);
  }

  double computeExpectedInformationGain(float logOdds)
  {
    double p = octomap::probability(logOdds);
    double ent_cur = probabilityToEntropy(p);
    double ent_hit = logOddsToEntropy(logOdds + prob_hit_log);
    double ent_miss = logOddsToEntropy(logOdds + prob_miss_log);

    return p * std::abs(ent_hit - ent_cur) + (1-p) * std::abs(ent_miss - ent_cur);
  }

  double computeExpectedRayInformationGain(const octomap::KeyRay &ray)
  {
    double expected_gain = 0;
    //double curProb = 1;
    for (const octomap::OcTreeKey &key : ray)
    {
      float logOdds = keyToLogOdds(key);
      expected_gain += /*curProb * */computeExpectedInformationGain(logOdds);
      //curProb *= 1 - octomap::probability(logOdds);
      //if (curProb < 0.05) // stop checking cells if probability of being hit is low
      //  break;
      if (logOdds > octomap::logodds(0.9)) // stop if cell is likely occupied
        break;
    }
    return expected_gain;
  }

  double computeWeightedExpectedRayInformationGain(const octomap::KeyRay &ray)
  {
    double expected_gain = 0;
    //double curProb = 1;
    for (const octomap::OcTreeKey &key : ray)
    {
      float logOdds = keyToLogOdds(key);
      double roi_val = keyToRoiVal(key);
      double gain = computeExpectedInformationGain(logOdds);
      const double ROI_WEIGHT = 0.5;
      double weightedGain = ROI_WEIGHT * roi_val * gain + (1 - ROI_WEIGHT) * gain;
      expected_gain += /*curProb * */weightedGain;
      //curProb *= 1 - octomap::probability(logOdds);
      //if (curProb < 0.05) // stop checking cells if probability of being hit is low
      //  break;
      if (logOdds > octomap::logodds(0.9)) // stop if cell is likely occupied
        break;
    }
    return expected_gain;
  }

  double computeWeightedCellEntropy(const octomap::OcTreeKey &key) // computeInflatedRois must be called first
  {
    if (inflated_rois == nullptr)
      return 0;

    double entropy = computeCellEntropy(key);
    InflatedRoiOcTreeNode *node = inflated_rois->search(key);
    double roi_val = 0.0;
    if (node != NULL)
      roi_val = node->getValue() / inflated_rois->getMaxRoiVal();

    const double ROI_WEIGHT = 0.5;
    double weightedEntropy = ROI_WEIGHT * roi_val * entropy + (1 - ROI_WEIGHT) * entropy;
    return weightedEntropy;
  }

  double computeViewpointValue(const octomap::pose6d &viewpoint, const double &hfov, size_t x_steps, size_t y_steps, const double &maxRange, bool use_roi_weighting = true)
  {
    double f_rec =  2 * tan(hfov / 2) / (double)x_steps;
    double cx = (double)x_steps / 2.0;
    double cy = (double)y_steps / 2.0;
    double value = 0;
    for (size_t i = 0; i < x_steps; i++)
    {
      for(size_t j = 0; j < y_steps; j++)
      {
        double x = (i + 0.5 - cx) * f_rec;
        double y = (j + 0.5 - cy) * f_rec;
        octomap::point3d dir(x, y, 1.0);
        octomap::point3d end = dir * maxRange;
        end = viewpoint.transform(end);
        octomap::KeyRay ray;
        computeRayKeys(viewpoint.trans(), end, ray);
        double gain = use_roi_weighting ? computeWeightedExpectedRayInformationGain(ray) : computeExpectedRayInformationGain(ray);
        //ROS_INFO_STREAM("Ray (" << i << ", " << j << ") from " << viewpoint.trans() << " to " << end << ": " << ray.size() << " Keys, Gain: " << gain);
        value += gain;
      }
    }
    return value;
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

}

#endif // ROIOCTREE_H
