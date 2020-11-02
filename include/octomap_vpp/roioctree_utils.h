#ifndef ROIOCTREE_UTILS_H
#define ROIOCTREE_UTILS_H

#include <octomap/octomap_utils.h>
//#include <geometry_msgs/PoseStamped.h>

namespace octomap_vpp
{

enum Neighborhood {
  NB_6 = 6, NB_18 = 18, NB_26 = 26
};

const int nbLut [26][3] = {
  {-1, 0, 0},
  {0, -1, 0},
  {0, 0, -1},
  {0, 0, 1},
  {0, 1, 0},
  {1, 0, 0},
  {-1, -1, 0},
  {-1, 0, -1},
  {-1, 0, 1},
  {-1, 1, 0},
  {0, -1, -1},
  {0, -1, 1},
  {0, 1, -1},
  {0, 1, 1},
  {1, -1, 0},
  {1, 0, -1},
  {1, 0, 1},
  {1, 1, 0},
  {-1, -1, -1},
  {-1, -1, 1},
  {-1, 1, -1},
  {-1, 1, 1},
  {1, -1, -1},
  {1, -1, 1},
  {1, 1, -1},
  {1, 1, 1}
};

// Old separate LUTs, currently kept for backwards compatibility

const int nb6Lut [6][3] = {
  {-1, 0, 0},
  {0, -1, 0},
  {0, 0, -1},
  {0, 0, 1},
  {0, 1, 0},
  {1, 0, 0}
};

const int nb18Lut [18][3] = {
  {-1, -1, 0},
  {-1, 0, -1},
  {-1, 0, 0},
  {-1, 0, 1},
  {-1, 1, 0},
  {0, -1, -1},
  {0, -1, 0},
  {0, -1, 1},
  {0, 0, -1},
  {0, 0, 1},
  {0, 1, -1},
  {0, 1, 0},
  {0, 1, 1},
  {1, -1, 0},
  {1, 0, -1},
  {1, 0, 0},
  {1, 0, 1},
  {1, 1, 0}
};

const int nb26Lut [26][3] = {
  {-1, -1, -1},
  {-1, -1, 0},
  {-1, -1, 1},
  {-1, 0, -1},
  {-1, 0, 0},
  {-1, 0, 1},
  {-1, 1, -1},
  {-1, 1, 0},
  {-1, 1, 1},
  {0, -1, -1},
  {0, -1, 0},
  {0, -1, 1},
  {0, 0, -1},
  {0, 0, 1},
  {0, 1, -1},
  {0, 1, 0},
  {0, 1, 1},
  {1, -1, -1},
  {1, -1, 0},
  {1, -1, 1},
  {1, 0, -1},
  {1, 0, 0},
  {1, 0, 1},
  {1, 1, -1},
  {1, 1, 0},
  {1, 1, 1}
};

inline double probabilityToEntropy(double p)
{
  return -(p*log2(p) + (1-p)*log2(1-p));
}

inline double logOddsToEntropy(float logOdds)
{
  return probabilityToEntropy(octomap::probability(logOdds));
}

}

#endif // ROIOCTREE_UTILS_H
