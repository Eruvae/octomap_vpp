#ifndef OCTOMAP_TRANSFORMS_H
#define OCTOMAP_TRANSFORMS_H

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <octomap/octomap_types.h>
#include <octomap/Pointcloud.h>

namespace octomap_vpp
{

inline static octomath::Vector3 vectorToOctomath(const geometry_msgs::Vector3 &vec)
{
  return octomath::Vector3(vec.x, vec.y, vec.z);
}

inline static octomap::point3d pointToOctomath(const geometry_msgs::Point &p)
{
  return octomap::point3d(p.x, p.y, p.z);
}

inline static octomath::Quaternion quaternionToOctomath(const geometry_msgs::Quaternion &quat)
{
  return octomath::Quaternion(quat.w, quat.x, quat.y, quat.z);
}

inline static octomath::Pose6D transformToOctomath(const geometry_msgs::Transform &transform)
{
  return octomath::Pose6D(vectorToOctomath(transform.translation), quaternionToOctomath(transform.rotation));
}

inline static octomath::Pose6D poseToOctomath(const geometry_msgs::Pose &pose)
{
  return octomath::Pose6D(pointToOctomath(pose.position), quaternionToOctomath(pose.orientation));
}

inline static geometry_msgs::Vector3 octomathToVector(const octomath::Vector3 &vec)
{
  geometry_msgs::Vector3 v;
  v.x = static_cast<double>(vec.x());
  v.y = static_cast<double>(vec.y());
  v.z = static_cast<double>(vec.z());
  return v;
}

inline static geometry_msgs::Point octomathToPoint(const octomap::point3d &p)
{
  geometry_msgs::Point pm;
  pm.x = static_cast<double>(p.x());
  pm.y = static_cast<double>(p.y());
  pm.z = static_cast<double>(p.z());
  return pm;
}

inline static geometry_msgs::Quaternion octomathToQuaternion(const octomath::Quaternion &quat)
{
  geometry_msgs::Quaternion q;
  q.x = static_cast<double>(quat.x());
  q.y = static_cast<double>(quat.y());
  q.z = static_cast<double>(quat.z());
  q.w = static_cast<double>(quat.u());
  return q;
}

inline static geometry_msgs::Transform octomathToTransform(const octomath::Pose6D &transform)
{
  geometry_msgs::Transform tf;
  tf.translation = octomathToVector(transform.trans());
  tf.rotation = octomathToQuaternion(transform.rot());
  return tf;
}

inline static geometry_msgs::Pose octomathToPose(const octomath::Pose6D &pose)
{
  geometry_msgs::Pose p;
  p.position = octomathToPoint(pose.trans());
  p.orientation = octomathToQuaternion(pose.rot());
  return p;
}

} // namespace octomap_vpp

namespace tf2
{

template <> inline void doTransform(const octomap::point3d& t_in, octomap::point3d& t_out, const geometry_msgs::TransformStamped& transform)
{
  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Vector3 v_in(t_in.x(), t_in.y(), t_in.z());
  tf2::Vector3 v_out = t * v_in;
  t_out = octomap::point3d(v_out.x(), v_out.y(), v_out.z());
}

template <> inline void doTransform(const octomap::pose6d& t_in, octomap::pose6d& t_out, const geometry_msgs::TransformStamped& transform)
{
  geometry_msgs::Pose p_in = octomap_vpp::octomathToPose(t_in);
  geometry_msgs::Pose p_out;
  doTransform(p_in, p_out, transform);
  t_out = octomap_vpp::poseToOctomath(p_out);
}

} // namespace tf2

#endif // OCTOMAP_TRANSFORMS_H
