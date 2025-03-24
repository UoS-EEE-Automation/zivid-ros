#include <Zivid/Calibration/Pose.h>
#include <Zivid/Matrix.h>
#include <Zivid/Point.h>

#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <zivid_camera/utility.hpp>

namespace zivid_camera
{
namespace
{
Zivid::Matrix4x4 transposeMatrix(const Zivid::Matrix4x4 & value)
{
  Zivid::Matrix4x4 transposed;
  for (size_t i = 0; i < Zivid::Matrix4x4::rows; ++i) {
    for (size_t j = 0; j < Zivid::Matrix4x4::cols; ++j) {
      transposed(i, j) = value(j, i);
    }
  }
  return transposed;
}
}  // namespace

Zivid::Calibration::Pose toZividPose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Transform tf2Transform;
  tf2::fromMsg(pose, tf2Transform);
  std::array<tf2Scalar, 16> tf2_transform = {};
  tf2Transform.getOpenGLMatrix(tf2_transform.data());

  auto transform = transposeMatrix(Zivid::Matrix4x4{tf2_transform.begin(), tf2_transform.end()});
  transform.at(0, 3) = rosLengthToZivid(tf2Transform.getOrigin().x());
  transform.at(1, 3) = rosLengthToZivid(tf2Transform.getOrigin().y());
  transform.at(2, 3) = rosLengthToZivid(tf2Transform.getOrigin().z());
  return Zivid::Calibration::Pose{transform};
}

Zivid::PointXYZ toZividPoint(const geometry_msgs::msg::Point & point)
{
  return Zivid::PointXYZ{
    rosLengthToZivid(point.x),
    rosLengthToZivid(point.y),
    rosLengthToZivid(point.z),
  };
}

geometry_msgs::msg::Transform toGeometryMsgTransform(const Zivid::Matrix4x4 & transform)
{
  const tf2::Matrix3x3 rotation{transform(0, 0), transform(0, 1), transform(0, 2),
                                transform(1, 0), transform(1, 1), transform(1, 2),
                                transform(2, 0), transform(2, 1), transform(2, 2)};
  tf2::Quaternion q;
  rotation.getRotation(q);

  geometry_msgs::msg::Transform result;
  result.translation.x = zividLengthToRos(transform(0, 3));
  result.translation.y = zividLengthToRos(transform(1, 3));
  result.translation.z = zividLengthToRos(transform(2, 3));
  result.rotation.x = q.x();
  result.rotation.y = q.y();
  result.rotation.z = q.z();
  result.rotation.w = q.w();
  return result;
}

}  // namespace zivid_camera
