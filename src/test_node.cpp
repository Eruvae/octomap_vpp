#include <ros/ros.h>
#include <message_filters/subscriber.h>
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

octomap::OcTree testTree(0.05);
tf2_ros::Buffer tfBuffer(ros::Duration(20));
ros::Publisher octomapPub;

const double OCCUPANCY_THRESH = 0.7;
const double FREE_THRESH = 0.3;

void registerNewScan(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
  sensor_msgs::PointCloud2 pc_glob;
  octomap::Pointcloud pc;
  octomap::pointCloud2ToOctomap(*pc_msg, pc);

  geometry_msgs::TransformStamped pcFrameTf;

  try {
    pcFrameTf = tfBuffer.lookupTransform("map", pc_msg->header.frame_id, pc_msg->header.stamp);
  } catch (const tf2::TransformException &e) {
  }

  const geometry_msgs::Vector3 &pcfOrig = pcFrameTf.transform.translation;
  octomap::point3d scan_orig(pcfOrig.x, pcfOrig.y, pcfOrig.z);
  tf2::doTransform(*pc_msg, pc_glob, pcFrameTf);

  testTree.insertPointCloud(pc, scan_orig);
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

void publishMap()
{
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = "map";
  map_msg.header.stamp = ros::Time::now();
  if (octomap_msgs::fullMapToMsg(testTree, map_msg))
    octomapPub.publish(map_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tfListener(tfBuffer);

  octomapPub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> depthCloudSub(nh, "/depth_registered/points", 20);
  tf2_ros::MessageFilter<sensor_msgs::PointCloud2> tfCloudFilter(depthCloudSub, tfBuffer, "map", 20, nh);

  tfCloudFilter.registerCallback(registerNewScan);

  ros::spin();
}
