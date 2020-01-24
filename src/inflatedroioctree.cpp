#include "inflatedroioctree.h"

InflatedRoiOcTree::InflatedRoiOcTree(double resolution) : octomap::OcTreeBase <InflatedRoiOcTreeNode>(resolution), max_roi_val(10.0f), influence_radius(0.1f)
{
  ocTreeMemberInit.ensureLinking();
}

InflatedRoiOcTree::StaticMemberInitializer InflatedRoiOcTree::ocTreeMemberInit;
