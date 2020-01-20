#include "roi_octree_display.h"

#include "rviz/properties/enum_property.h"
#include "rviz/properties/status_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"

#include "octomap_msgs/conversions.h"

namespace roioctree_rviz_plugin
{

static const std::size_t max_octree_depth_ = sizeof(unsigned short) * 8;

enum OctreeVoxelRenderMode
{
  OCTOMAP_FREE_VOXELS = 1,
  OCTOMAP_OCCUPIED_VOXELS = 2,
  OCTOMAP_ROI_VOXELS = 4
};

enum OctreeVoxelColorMode
{
  OCTOMAP_Z_AXIS_COLOR,
  OCTOMAP_PROBABLILTY_COLOR,
  OCTOMAP_ROI_COLOR
};

using rviz::StatusProperty;
using rviz::PointCloud;

RoiOcTreeDisplay::RoiOcTreeDisplay() : octomap_rviz_plugin::OccupancyGridDisplay()
{
  octree_render_property_->addOption( "ROI Voxels",  OCTOMAP_ROI_VOXELS);
  octree_coloring_property_->clearOptions();
  octree_coloring_property_->addOption( "Z-Axis",  OCTOMAP_Z_AXIS_COLOR );
  octree_coloring_property_->addOption( "Cell Probability",  OCTOMAP_PROBABLILTY_COLOR );
  octree_coloring_property_->addOption( "ROI Probability",  OCTOMAP_ROI_COLOR );
}

void RoiOcTreeDisplay::incomingMessageCallback(const octomap_msgs::OctomapConstPtr& msg)
{
  ++messages_received_;
  setStatus(StatusProperty::Ok, "Messages", QString::number(messages_received_) + " octomap messages received");
  setStatusStd(StatusProperty::Ok, "Type", msg->id.c_str());
  if(!checkType(msg->id)){
    setStatusStd(StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type.");
    return;
  }

  ROS_DEBUG("Received OctomapBinary message (size: %d bytes)", (int)msg->data.size());

  header_ = msg->header;
  if (!updateFromTF()) {
      std::stringstream ss;
      ss << "Failed to transform from frame [" << header_.frame_id << "] to frame ["
          << context_->getFrameManager()->getFixedFrame() << "]";
      setStatusStd(StatusProperty::Error, "Message", ss.str());
      return;
  }

  // creating octree
  RoiOcTree* octomap = NULL;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree){
    octomap = dynamic_cast<RoiOcTree*>(tree);
    if(!octomap){
      setStatusStd(StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type.");
    }
  }
  else
  {
    setStatusStd(StatusProperty::Error, "Message", "Failed to deserialize octree message.");
    return;
  }


  tree_depth_property_->setMax(octomap->getTreeDepth());

  // get dimensions of octree
  double minX, minY, minZ, maxX, maxY, maxZ;
  octomap->getMetricMin(minX, minY, minZ);
  octomap->getMetricMax(maxX, maxY, maxZ);

  // reset rviz pointcloud classes
  for (std::size_t i = 0; i < max_octree_depth_; ++i)
  {
    point_buf_[i].clear();
    box_size_[i] = octomap->getNodeSize(i + 1);
  }

  size_t pointCount = 0;
  {
    // traverse all leafs in the tree:
    unsigned int treeDepth = std::min<unsigned int>(tree_depth_property_->getInt(), octomap->getTreeDepth());
    double maxHeight = std::min<double>(max_height_property_->getFloat(), maxZ);
    double minHeight = std::max<double>(min_height_property_->getFloat(), minZ);
    int stepSize = 1 << (octomap->getTreeDepth() - treeDepth); // for pruning of occluded voxels
    for (typename RoiOcTree::iterator it = octomap->begin(treeDepth), end = octomap->end(); it != end; ++it)
    {
        if(it.getZ() <= maxHeight && it.getZ() >= minHeight)
        {
          int render_mode_mask = octree_render_property_->getOptionInt();

          bool display_voxel = false;

          // the left part evaluates to 1 for free voxels and 2 for occupied voxels
          int cur_node_mode_mask = ((int)octomap->isNodeOccupied(*it) + 1) | ((int)octomap->isNodeROI(*it) << 2);
          if (cur_node_mode_mask & render_mode_mask)
          {
            // check if current voxel has neighbors on all sides -> no need to be displayed
            bool allNeighborsFound = true;

            octomap::OcTreeKey key;
            octomap::OcTreeKey nKey = it.getKey();

            // determine indices of potentially neighboring voxels for depths < maximum tree depth
            // +/-1 at maximum depth, +2^(depth_difference-1) and -2^(depth_difference-1)-1 on other depths
            int diffBase = (it.getDepth() < octomap->getTreeDepth()) ? 1 << (octomap->getTreeDepth() - it.getDepth() - 1) : 1;
            int diff[2] = {-((it.getDepth() == octomap->getTreeDepth()) ? diffBase : diffBase + 1), diffBase};

            // cells with adjacent faces can occlude a voxel, iterate over the cases x,y,z (idxCase) and +/- (diff)
            for (unsigned int idxCase = 0; idxCase < 3; ++idxCase)
            {
              int idx_0 = idxCase % 3;
              int idx_1 = (idxCase + 1) % 3;
              int idx_2 = (idxCase + 2) % 3;

              for (int i = 0; allNeighborsFound && i < 2; ++i)
              {
                key[idx_0] = nKey[idx_0] + diff[i];
                // if rendering is restricted to treeDepth < maximum tree depth inner nodes with distance stepSize can already occlude a voxel
                for (key[idx_1] = nKey[idx_1] + diff[0] + 1; allNeighborsFound && key[idx_1] < nKey[idx_1] + diff[1]; key[idx_1] += stepSize)
                {
                  for (key[idx_2] = nKey[idx_2] + diff[0] + 1; allNeighborsFound && key[idx_2] < nKey[idx_2] + diff[1]; key[idx_2] += stepSize)
                  {
                    typename RoiOcTree::NodeType* node = octomap->search(key, treeDepth);

                    // the left part evaluates to 1 for free voxels and 2 for occupied voxels
                    if (!(node && ((((int)octomap->isNodeOccupied(node)) + 1) & render_mode_mask)))
                    {
                      // we do not have a neighbor => break!
                      allNeighborsFound = false;
                    }
                  }
                }
              }
            }

            display_voxel |= !allNeighborsFound;
          }


          if (display_voxel)
          {
            PointCloud::Point newPoint;

            newPoint.position.x = it.getX();
            newPoint.position.y = it.getY();
            newPoint.position.z = it.getZ();



            setVoxelColor(newPoint, *it, minZ, maxZ);
            // push to point vectors
            unsigned int depth = it.getDepth();
            point_buf_[depth - 1].push_back(newPoint);

            ++pointCount;
          }
        }
    }
  }

  if (pointCount)
  {
    boost::mutex::scoped_lock lock(mutex_);

    new_points_received_ = true;

    for (size_t i = 0; i < max_octree_depth_; ++i)
      new_points_[i].swap(point_buf_[i]);

  }
  delete octomap;
}

void RoiOcTreeDisplay::setVoxelColor(rviz::PointCloud::Point& newPoint, typename RoiOcTree::NodeType& node, double minZ, double maxZ)
{
    float cell_probability;
    OctreeVoxelColorMode octree_color_mode = static_cast<OctreeVoxelColorMode>(octree_coloring_property_->getOptionInt());
    switch (octree_color_mode)
    {
      case OCTOMAP_Z_AXIS_COLOR:
        setColor(newPoint.position.z, minZ, maxZ, color_factor_, newPoint);
        break;
      case OCTOMAP_PROBABLILTY_COLOR:
        cell_probability = node.getOccupancy();
        newPoint.setColor((1.0f-cell_probability), cell_probability, 0.0);
        break;
      case OCTOMAP_ROI_COLOR:
        cell_probability = node.getRoiProb();
        newPoint.setColor((1.0f-cell_probability), cell_probability, 0.0);
        break;
      default:
        break;
    }
}

bool RoiOcTreeDisplay::checkType(std::string type_id)
{
  if(type_id == "RoiOcTree") return true;
  else return false;
}

}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( roioctree_rviz_plugin::RoiOcTreeDisplay, rviz::Display);
