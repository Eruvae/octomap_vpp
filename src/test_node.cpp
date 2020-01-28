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
//#include <moveit_msgs/PlanningScene.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include <instance_segmentation_msgs/Detections.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>

#include <boost/thread/mutex.hpp>

#include "roioctree.h"

RoiOcTree testTree(0.02);
tf2_ros::Buffer tfBuffer(ros::Duration(30));
ros::Publisher octomapPub;
ros::Publisher inflatedOctomapPub;
ros::Publisher pcGlobalPub;
//ros::Publisher planningScenePub;

boost::mutex tree_mtx;

const double OCCUPANCY_THRESH = 0.7;
const double FREE_THRESH = 0.3;

const std::string MAP_FRAME = "world";
const std::string PC_TOPIC = "/camera/depth/points";
const std::string PC_GLOBAL = "/points_global";

/*void publishOctomapToPlanningScene(const octomap_msgs::Octomap &map_msg)
{
  moveit_msgs::PlanningScene scene;
  scene.world.octomap.header = map_msg.header;
  scene.world.octomap.octomap = map_msg;
  scene.world.octomap.octomap.id = "OcTree";
  scene.is_diff = true;
  planningScenePub.publish(scene);
}*/

void publishMap()
{
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = MAP_FRAME;
  map_msg.header.stamp = ros::Time::now();
  tree_mtx.lock();
  bool msg_generated = octomap_msgs::fullMapToMsg(testTree, map_msg);
  tree_mtx.unlock();
  if (msg_generated)
  {
    octomapPub.publish(map_msg);
    //publishOctomapToPlanningScene(map_msg);
  }

  //if (octomap_msgs::binaryMapToMsg(testTree, map_msg))
  //    octomapPub.publish(map_msg);

  tree_mtx.lock();
  ros::Time inflationStart = ros::Time::now();
  std::shared_ptr<InflatedRoiOcTree> inflatedTree = testTree.computeInflatedRois();
  ros::Time inflationDone = ros::Time::now();
  tree_mtx.unlock();

  ROS_INFO_STREAM("Time for inflation: " << inflationDone - inflationStart);
  if (inflatedTree != nullptr)
  {
    octomap_msgs::Octomap inflated_map;
    inflated_map.header.frame_id = MAP_FRAME;
    inflated_map.header.stamp = ros::Time::now();
    if (octomap_msgs::fullMapToMsg(*inflatedTree, inflated_map))
    {
      inflatedOctomapPub.publish(inflated_map);
    }
  }
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

  //pcGlobalPub.publish(pc_glob);

  ros::Time doTFTime = ros::Time::now();

  octomap::Pointcloud pc;
  octomap::pointCloud2ToOctomap(pc_glob, pc);

  ros::Time toOctoTime = ros::Time::now();

  tree_mtx.lock();
  testTree.insertPointCloud(pc, scan_orig);
  tree_mtx.unlock();

  ros::Time insertTime = ros::Time::now();

  ROS_INFO_STREAM("Timings - TF: " << tfTime - cbStartTime << "; doTF: " << doTFTime - tfTime << "; toOct: " << toOctoTime - doTFTime << "; insert: " << insertTime - toOctoTime);
}

void pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::unordered_set<size_t> &indices,  octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud)
{
   inlierCloud.reserve(indices.size());
   outlierCloud.reserve(cloud.data.size() / cloud.point_step - indices.size());

   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
   sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

   size_t invalid_points = 0;
   for (size_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i){
     // Check if the point is invalid
     if (std::isfinite (*iter_x) && std::isfinite (*iter_y) && std::isfinite (*iter_z))
     {
       if (indices.find(i) != indices.end())
         inlierCloud.push_back(*iter_x, *iter_y, *iter_z);
       else
         outlierCloud.push_back(*iter_x, *iter_y, *iter_z);
     }
     else
       invalid_points++;
   }
   ROS_INFO_STREAM("Number of invalid points: " << invalid_points);
}

// indices must be ordered!
void pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::vector<uint32_t> &indices,  octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud)
{
   inlierCloud.reserve(indices.size());
   outlierCloud.reserve(cloud.data.size() / cloud.point_step - indices.size());

   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
   sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

   size_t invalid_points = 0;
   std::vector<uint32_t>::const_iterator it = indices.begin();
   for (uint32_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i){
     // Check if the point is invalid
     if (std::isfinite (*iter_x) && std::isfinite (*iter_y) && std::isfinite (*iter_z))
     {
       if (i == *it)
       {
         inlierCloud.push_back(*iter_x, *iter_y, *iter_z);
         it++;
       }
       else
       {
         outlierCloud.push_back(*iter_x, *iter_y, *iter_z);
       }
     }
     else
       invalid_points++;
   }
   ROS_INFO_STREAM("Number of invalid points: " << invalid_points);
}

void registerRoiPCL(const pointcloud_roi_msgs::PointcloudWithRoi &roi)
{
  octomap::Pointcloud inlierCloud, outlierCloud;
  pointCloud2ToOctomapByIndices(roi.cloud, roi.roi_indices, inlierCloud, outlierCloud);
  ROS_INFO_STREAM("Cloud sizes: " << inlierCloud.size() << ",  " << outlierCloud.size());

  tree_mtx.lock();
  testTree.insertRegionScan(inlierCloud, outlierCloud);
  tree_mtx.unlock();

  //std::vector<octomap::OcTreeKey> roi_keys = testTree.getRoiKeys();
  ROS_INFO_STREAM("Found " << testTree.getRoiSize() << " ROI keys (" << testTree.getAddedRoiSize() << " added, " << testTree.getDeletedRoiSize() << " removed)");
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

  ROS_INFO_STREAM("Number of inliers: " << inlier_indices.size());

  octomap::Pointcloud inlierCloud, outlierCloud;
  pointCloud2ToOctomapByIndices(*pc_msg, inlier_indices, inlierCloud, outlierCloud);
  ROS_INFO_STREAM("Cloud sizes: " << inlierCloud.size() << ",  " << outlierCloud.size());

  tree_mtx.lock();
  testTree.insertRegionScan(inlierCloud, outlierCloud);
  tree_mtx.unlock();
}

inline float keyToLogOdds(const octomap::OcTreeKey &key)
{
  RoiOcTreeNode *node = testTree.search(key);
  float logOdds = 0;
  if (node != NULL)
    logOdds = node->getLogOdds();
  return logOdds;
}

inline double keyToProbability(const octomap::OcTreeKey &key)
{
  RoiOcTreeNode *node = testTree.search(key);
  double p = 0;
  if (node != NULL)
    p = node->getOccupancy();
  return p;
}

inline double keyToRoiVal(const octomap::OcTreeKey &key)
{
  auto inflatedRois = testTree.getInflatedRois(); // must call computeInflatedRois first
  InflatedRoiOcTreeNode *node = inflatedRois->search(key);
  double roi_val = 0.0;
  if (node != NULL)
    roi_val = node->getValue() / inflatedRois->getMaxRoiVal();
  return roi_val;
}

inline double probabilityToEntropy(double p)
{
  return -(p*log(p) + (1-p)*log(1-p));
}

inline double logOddsToEntropy(float logOdds)
{
  return probabilityToEntropy(octomap::probability(logOdds));
}

double computeCellEntropy(const octomap::OcTreeKey &key)
{
  RoiOcTreeNode *node = testTree.search(key);
  double p = 0.5; // defaults to 0.5 if cell not known
  if (node != NULL)
    p = node->getOccupancy();

  return probabilityToEntropy(p);
}

double computeExpectedInformationGain(const octomap::OcTreeKey &key)
{
  RoiOcTreeNode *node = testTree.search(key);
  float hitLog = testTree.getProbHitLog();
  float missLog = testTree.getProbMissLog();
  float logOdds = 0;
  if (node != NULL)
    logOdds = node->getLogOdds();

  double p = octomap::probability(logOdds);
  double ent_cur = probabilityToEntropy(p);
  double ent_hit = logOddsToEntropy(logOdds + hitLog);
  double ent_miss = logOddsToEntropy(logOdds + missLog);

  return p * abs(ent_hit - ent_cur) + (1-p) * abs(ent_miss - ent_cur);
}

double computeExpectedInformationGain(float logOdds)
{
  float hitLog = testTree.getProbHitLog();
  float missLog = testTree.getProbMissLog();
  double p = octomap::probability(logOdds);
  double ent_cur = probabilityToEntropy(p);
  double ent_hit = logOddsToEntropy(logOdds + hitLog);
  double ent_miss = logOddsToEntropy(logOdds + missLog);

  return p * abs(ent_hit - ent_cur) + (1-p) * abs(ent_miss - ent_cur);
}

double computeExpectedRayInformationGain(const octomap::KeyRay &ray)
{
  double expected_gain = 0;
  double curProb = 1;
  for (const octomap::OcTreeKey &key : ray)
  {
    float logOdds = keyToLogOdds(key);
    expected_gain += curProb * computeExpectedInformationGain(logOdds);
    curProb *= octomap::probability(logOdds);
  }
  return expected_gain;
}

double computeWeightedExpectedRayInformationGain(const octomap::KeyRay &ray)
{
  double expected_gain = 0;
  double curProb = 1;
  for (const octomap::OcTreeKey &key : ray)
  {
    float logOdds = keyToLogOdds(key);
    double roi_val = keyToRoiVal(key);
    double gain = computeExpectedInformationGain(logOdds);
    const double ROI_WEIGHT = 0.5;
    double weightedGain = ROI_WEIGHT * roi_val * gain + (1 - ROI_WEIGHT) * gain;
    expected_gain += curProb * weightedGain;
    curProb *= octomap::probability(logOdds);
  }
  return expected_gain;
}

double computeWeightedCellEntropy(const octomap::OcTreeKey &key)
{
  double entropy = computeCellEntropy(key);
  auto inflatedRois = testTree.getInflatedRois(); // must call computeInflatedRois first
  InflatedRoiOcTreeNode *node = inflatedRois->search(key);
  double roi_val = 0.0;
  if (node != NULL)
    roi_val = node->getValue() / inflatedRois->getMaxRoiVal();

  const double ROI_WEIGHT = 0.5;
  double weightedEntropy = ROI_WEIGHT * roi_val * entropy + (1 - ROI_WEIGHT) * entropy;
  return weightedEntropy;
}

/*double computeViewpointValue(const octomap::pose6d &viewpoint, const image_geometry::PinholeCameraModel &model, const double &minRange, const double &maxRange, size_t xRes, size_t yRes)
{
  //cv::Point2d uv;
  return 0;
}*/

// make sure x_steps / y_steps equals camera aspect ratio; hfov in rad
double computeViewpointValue(const octomap::pose6d &viewpoint, const double &hfov, size_t x_steps, size_t y_steps, const double &maxRange)
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
      testTree.computeRayKeys(viewpoint.trans(), end, ray);
      value += computeWeightedExpectedRayInformationGain(ray);
    }
  }
  return value;
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
  inflatedOctomapPub = nh.advertise<octomap_msgs::Octomap>("inflated_octomap", 1);
  //pcGlobalPub = nh.advertise<sensor_msgs::PointCloud2>(PC_GLOBAL, 1);
  //planningScenePub = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> depthCloudSub(nh, PC_TOPIC, 1);
  //tf2_ros::MessageFilter<sensor_msgs::PointCloud2> tfCloudFilter(depthCloudSub, tfBuffer, MAP_FRAME, 1, nh);
  //message_filters::Cache<sensor_msgs::PointCloud2> cloudCache(tfCloudFilter, 1);

  depthCloudSub.registerCallback(registerNewScan);
  //tfCloudFilter.registerCallback(registerNewScan);
  //cloudCache.registerCallback(registerNewScan);

  //message_filters::Subscriber<sensor_msgs::PointCloud2> pcGlobalSub(nh, PC_GLOBAL, 1);
  //message_filters::Subscriber<instance_segmentation_msgs::Detections> detectionsSub(nh, "/mask_rcnn/detections", 1);

  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, instance_segmentation_msgs::Detections> DetsSyncPolicy;
  //message_filters::Synchronizer<DetsSyncPolicy> syncDets(DetsSyncPolicy(50), pcGlobalSub, detectionsSub);

  //syncDets.registerCallback(registerRoi);

  ros::Subscriber roiSub = nh.subscribe("/pointcloud_roi", 1, registerRoiPCL);

  for (ros::Rate rate(1); ros::ok(); rate.sleep())
  {
    publishMap();
  }
}
