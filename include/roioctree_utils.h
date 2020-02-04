#ifndef ROIOCTREE_UTILS_H
#define ROIOCTREE_UTILS_H

#include <octomap/octomap_utils.h>
#include <geometry_msgs/PoseStamped.h>

inline double probabilityToEntropy(double p)
{
  return -(p*log2(p) + (1-p)*log2(1-p));
}

inline double logOddsToEntropy(float logOdds)
{
  return probabilityToEntropy(octomap::probability(logOdds));
}

#endif // ROIOCTREE_UTILS_H
