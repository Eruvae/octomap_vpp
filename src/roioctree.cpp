#include "roioctree.h"

double RoiOcTreeNode::getMeanChildRoiLogOdds() const
{
    double mean = 0;
    uint8_t c = 0;
    if (children !=NULL){
      for (unsigned int i=0; i<8; i++) {
        if (children[i] != NULL) {
          mean += static_cast<RoiOcTreeNode*>(children[i])->getRoiProb(); // TODO check if works generally
          ++c;
        }
      }
    }

    if (c > 0)
      mean /= (double) c;

    return log(mean/(1-mean));
}

float RoiOcTreeNode::getMaxChildRoiLogOdds() const
{
    float max = -std::numeric_limits<float>::max();

    if (children !=NULL){
      for (unsigned int i=0; i<8; i++) {
        if (children[i] != NULL) {
          float l = static_cast<RoiOcTreeNode*>(children[i])->getRoiLogOdds(); // TODO check if works generally
          if (l > max)
            max = l;
        }
      }
    }
    return max;
}

void RoiOcTreeNode::addRoiValue(const float& logOdds)
{
  roiValue += logOdds;
}

RoiOcTree::RoiOcTree(double resolution) : octomap::OccupancyOcTreeBase <RoiOcTreeNode>(resolution), roi_prob_thres_log(1)
{
  ocTreeMemberInit.ensureLinking();
}

RoiOcTree::StaticMemberInitializer RoiOcTree::ocTreeMemberInit;

void RoiOcTree::updateNodeRoiLogOdds(RoiOcTreeNode* node, const float& update) const
{
  node->addRoiValue(update);
  if (node->getRoiLogOdds() < this->clamping_thres_min) {
    node->setRoiLogOdds(this->clamping_thres_min);
    return;
  }
  if (node->getRoiLogOdds() > this->clamping_thres_max) {
    node->setRoiLogOdds(this->clamping_thres_max);
  }
}
