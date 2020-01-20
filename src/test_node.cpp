#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <instance_segmentation_msgs/Detections.h>

#include "roioctree.h"

RoiOcTree testTree(0.05);
tf2_ros::Buffer tfBuffer(ros::Duration(30));
ros::Publisher octomapPub;

const double OCCUPANCY_THRESH = 0.7;
const double FREE_THRESH = 0.3;

const std::string MAP_FRAME = "world";
const std::string PC_TOPIC = "/camera/depth/points";

void publishMap()
{
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = MAP_FRAME;
  map_msg.header.stamp = ros::Time::now();
  if (octomap_msgs::fullMapToMsg(testTree, map_msg))
    octomapPub.publish(map_msg);
  //if (octomap_msgs::binaryMapToMsg(testTree, map_msg))
  //    octomapPub.publish(map_msg);
}

void registerNewScan(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
  ros::Time cbStartTime = ros::Time::now();
  geometry_msgs::TransformStamped pcFrameTf;

  try
  {
    pcFrameTf = tfBuffer.lookupTransform(MAP_FRAME, pc_msg->header.frame_id, pc_msg->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
    return;
  }
  ROS_INFO_STREAM("Transform for time " << pc_msg->header.stamp << " successful");

  const geometry_msgs::Vector3 &pcfOrig = pcFrameTf.transform.translation;
  octomap::point3d scan_orig(pcfOrig.x, pcfOrig.y, pcfOrig.z);

  ros::Time tfTime = ros::Time::now();

  sensor_msgs::PointCloud2 pc_glob;
  tf2::doTransform(*pc_msg, pc_glob, pcFrameTf);

  ros::Time doTFTime = ros::Time::now();

  octomap::Pointcloud pc;
  octomap::pointCloud2ToOctomap(pc_glob, pc);

  ros::Time toOctoTime = ros::Time::now();

  testTree.insertPointCloud(pc, scan_orig);

  ros::Time insertTime = ros::Time::now();

  ROS_INFO_STREAM("Timings - TF: " << tfTime - cbStartTime << "; doTF: " << doTFTime - tfTime << "; toOct: " << toOctoTime - doTFTime << "; insert: " << insertTime - toOctoTime);
}

void pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, std::unordered_set<size_t> indices,  octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud)
{
   inlierCloud.reserve(indices.size());
   outlierCloud.reserve(cloud.data.size() / cloud.point_step - indices.size());

   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
   sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

   for (size_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i){
     // Check if the point is invalid
     if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z))
     {
       if (indices.find(i) != indices.end())
         inlierCloud.push_back(*iter_x, *iter_y, *iter_z);
       else
         outlierCloud.push_back(*iter_x, *iter_y, *iter_z);
     }
   }
 }

void registerRoi(const sensor_msgs::PointCloud2ConstPtr &pc_msg, const instance_segmentation_msgs::DetectionsConstPtr &dets_msg)
{
  ROS_INFO_STREAM("Register ROI called, " << dets_msg->detections.size() << " detections");
  std::unordered_set<size_t> inlier_indices;
  for (const auto &det : dets_msg->detections)
  {
    if (det.class_name != "capsicum") // not region of interest
      continue;

    for (int y = det.box.y1; y < det.box.y2; y++)
    {
      for (int x = det.box.x1; x < det.box.x2; x++)
      {
        if (det.mask.mask[(y - det.box.y1) * det.mask.width + (x - det.box.x1)])
        {
          inlier_indices.insert(y * pc_msg->width + x);
        }
      }
    }
  }

  octomap::Pointcloud inlierCloud, outlierCloud;
  pointCloud2ToOctomapByIndices(*pc_msg, inlier_indices, inlierCloud, outlierCloud);

  testTree.insertRegionScan(inlierCloud, outlierCloud);
}

void testRayTrace(const octomap::point3d &orig, const octomap::point3d &end)
{
  octomap::KeyRay ray;
  testTree.computeRayKeys(orig, end, ray);

  for (const octomap::OcTreeKey &key : ray)
  {
    octomap::OcTreeNode *node = testTree.search(key);
    if (node != NULL)
    {
      double occ = node->getOccupancy();
    }

  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  tf2_ros::TransformListener tfListener(tfBuffer);

  octomapPub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> depthCloudSub(nh, PC_TOPIC, 1);
  //tf2_ros::MessageFilter<sensor_msgs::PointCloud2> tfCloudFilter(depthCloudSub, tfBuffer, MAP_FRAME, 1, nh);
  //message_filters::Cache<sensor_msgs::PointCloud2> cloudCache(tfCloudFilter, 1);

  depthCloudSub.registerCallback(registerNewScan);
  //tfCloudFilter.registerCallback(registerNewScan);
  //cloudCache.registerCallback(registerNewScan);

  message_filters::Subscriber<instance_segmentation_msgs::Detections> detectionsSub(nh, "/mask_rcnn/detections", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, instance_segmentation_msgs::Detections> DetsSyncPolicy;
  message_filters::Synchronizer<DetsSyncPolicy> syncDets(DetsSyncPolicy(50), depthCloudSub, detectionsSub);

  syncDets.registerCallback(registerRoi);

  for (ros::Rate rate(1); ros::ok(); rate.sleep())
  {
    publishMap();
  }
}
