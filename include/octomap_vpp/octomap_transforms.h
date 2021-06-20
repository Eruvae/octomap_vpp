#ifndef OCTOMAP_TRANSFORMS_H
#define OCTOMAP_TRANSFORMS_H

#include <tf2/convert.h>
#include <tf2/utils.h>
#include <octomap/octomap_types.h>
#include <octomap/Pointcloud.h>

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

} // namespace tf2

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

} // namespace octomap_vpp

#endif // OCTOMAP_TRANSFORMS_H
