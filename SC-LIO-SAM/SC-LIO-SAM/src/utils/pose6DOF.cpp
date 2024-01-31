
#include "utils/pose6DOF.h"

Pose6DOF::Pose6DOF() {
  setIdentity();
}

Pose6DOF::Pose6DOF(const Eigen::Matrix4d& T, ros::Time stamp) {
  setIdentity();
  time_stamp = stamp;
  this->fromEigenMatrix(T);
  cov = Eigen::MatrixXd::Identity(6, 6);
}

Pose6DOF::Pose6DOF(
    const Eigen::Matrix4d& T, const std::string& tgt_frame, const std::string& src_frame, tf::TransformListener* tf_listener,
    ros::Time stamp) {
  setIdentity();
  time_stamp = stamp;
  this->fromEigenMatrixInFixedFrame(T, tgt_frame, src_frame, tf_listener);
  cov = Eigen::MatrixXd::Identity(6, 6);
}

Pose6DOF::Pose6DOF(const geometry_msgs::Pose& pose_msg, ros::Time stamp) {
  setIdentity();
  time_stamp = stamp;
  this->fromROSPose(pose_msg);
  cov = Eigen::MatrixXd::Identity(6, 6);
}

Pose6DOF::Pose6DOF(
    const geometry_msgs::Pose& pose_msg, const std::string& tgt_frame, const std::string& src_frame, tf::TransformListener* tf_listener,
    ros::Time stamp) {
  setIdentity();
  time_stamp = stamp;
  this->fromROSPoseInFixedFrame(pose_msg, tgt_frame, src_frame, tf_listener);
  cov = Eigen::MatrixXd::Identity(6, 6);
}

Pose6DOF::Pose6DOF(const geometry_msgs::PoseWithCovariance& pose_msg, ros::Time stamp) {
  setIdentity();
  time_stamp = stamp;
  this->fromROSPoseWithCovariance(pose_msg);
}

Pose6DOF::Pose6DOF(const tf::Transform& transform, ros::Time stamp) {
  setIdentity();
  time_stamp = stamp;
  this->fromTFTransform(transform);
  cov = Eigen::MatrixXd::Identity(6, 6);
}

Pose6DOF::Pose6DOF(const geometry_msgs::Transform& transform, ros::Time stamp) {
  setIdentity();
  time_stamp = stamp;
  this->fromROSTransform(transform);
  cov = Eigen::MatrixXd::Identity(6, 6);
}

double Pose6DOF::norm() const {
  return pos.norm();
}

Pose6DOF Pose6DOF::inverse() const {
  return inverse(*this);
}

Pose6DOF Pose6DOF::compose(const Pose6DOF& p2) {
  Pose6DOF p3 = compose(*this, p2);
  pos = p3.pos;
  rot = p3.rot;
  return *this;
}

Pose6DOF Pose6DOF::subtract(const Pose6DOF& p2) {
  Pose6DOF p3 = subtract(*this, p2);
  pos = p3.pos;
  rot = p3.rot;
  return *this;
}

Pose6DOF Pose6DOF::setIdentity() {
  time_stamp = ros::Time(0);
  pos = Eigen::Vector3d(0, 0, 0);
  rot = Eigen::Quaterniond(1, 0, 0, 0);
  cov = Eigen::MatrixXd::Identity(6, 6);
  return *this;
}

double Pose6DOF::distanceEuclidean(const Pose6DOF p2) const {
  return distanceEuclidean(*this, p2);
}

double Pose6DOF::distanceEuclidean(const Pose6DOF& p1, const Pose6DOF& p2) {
  return subtract(p1, p2).pos.norm();
}

Pose6DOF Pose6DOF::compose(const Pose6DOF& p1, const Pose6DOF& p2) {
  Pose6DOF p3;
  p3.time_stamp = p2.time_stamp;
  p3.pos = p1.pos + p1.rot.toRotationMatrix() * p2.pos;
  p3.rot = p1.rot * p2.rot;
  p3.rot.normalize();
  return p3;
}

Pose6DOF Pose6DOF::subtract(const Pose6DOF& p1, const Pose6DOF& p2) {
  Pose6DOF p3;
  p3.time_stamp = p1.time_stamp;
  p3.pos = p1.rot.inverse() * (p2.pos - p1.pos);
  p3.rot = p2.rot * p1.rot.inverse();
  p3.rot.normalize();
  p3.cov = p1.cov;
  return p3;
}

Pose6DOF Pose6DOF::inverse(const Pose6DOF& pose) {
  Pose6DOF inv;
  inv.pos = (pose.rot.inverse() * pose.pos) * -1.0;
  inv.rot = pose.rot.inverse();
  return inv;
}

Pose6DOF Pose6DOF::getIdentity() {
  Pose6DOF pose;
  pose.time_stamp = ros::Time().now();
  pose.pos = Eigen::Vector3d(0, 0, 0);
  pose.rot = Eigen::Quaterniond(1, 0, 0, 0);
  pose.cov = Eigen::MatrixXd::Identity(6, 6);
  return pose;
}

std::string Pose6DOF::toStringQuat(const std::string& indent) const {
  std::string output;
  output += indent + "Time stamp = " + std::to_string(time_stamp.toSec()) + "\n";
  output += indent + "Position = (" + std::to_string(pos(0)) + ", " + std::to_string(pos(1)) + ", " + std::to_string(pos(2)) +
            ") with Norm = " + std::to_string(pos.norm()) + "\n";
  output += indent + "Rotation (q) = (" + std::to_string(rot.w()) + ", " + std::to_string(rot.x()) + ", " + std::to_string(rot.y()) + ", " +
            std::to_string(rot.z()) + ") with Norm = " + std::to_string(rot.norm()) + "\n";

  return output;
}

std::string Pose6DOF::toStringRPY(const std::string& indent) const {
  std::string output;
  output += indent + "Time stamp = " + std::to_string(time_stamp.toSec()) + "\n";
  output += indent + "Position = (" + std::to_string(pos(0)) + ", " + std::to_string(pos(1)) + ", " + std::to_string(pos(2)) +
            ") with Norm = " + std::to_string(pos.norm()) + "\n";
  Eigen::Vector3d rpy = rot.toRotationMatrix().eulerAngles(0, 1, 2);
  output += indent + "Rot (RPY) = (" + std::to_string(rpy(0)) + ", " + std::to_string(rpy(1)) + ", " + std::to_string(rpy(2)) +
            ") with Norm = " + std::to_string(rpy.norm()) + "\n";

  return output;
}

void Pose6DOF::transformToFixedFrame(const std::string& tgt_frame, const std::string& src_frame, tf::TransformListener* tf_listener) {
  Pose6DOF pose_in_tgt = transformToFixedFrame(*this, tgt_frame, src_frame, tf_listener);
  pos = pose_in_tgt.pos;
  rot = pose_in_tgt.rot;
}

Pose6DOF Pose6DOF::transformToFixedFrame(
    const Pose6DOF& pose_in_src, const std::string& tgt_frame, const std::string& src_frame, tf::TransformListener* tf_listener) {
  tf::Pose tf_in_src(
      tf::Quaternion(pose_in_src.rot.x(), pose_in_src.rot.y(), pose_in_src.rot.z(), pose_in_src.rot.w()),
      tf::Vector3(pose_in_src.pos(0), pose_in_src.pos(1), pose_in_src.pos(2)));
  tf::Stamped<tf::Pose> tf_in_tgt;
  try {
    // By default takes most recent time stamp. It should aim for a target stamp
    tf_listener->transformPose(tgt_frame, tf::Stamped<tf::Pose>(tf_in_src, ros::Time(0), src_frame), tf_in_tgt);
  } catch (tf::TransformException e) {
  }

  Pose6DOF pose_in_tgt(tf_in_tgt, pose_in_src.time_stamp);
  pose_in_tgt.cov = pose_in_src.cov;
  return pose_in_tgt;
}

void Pose6DOF::fromEigenIsometry3(const Eigen::Isometry3d& T) {
  pos = Eigen::Vector3d(T.translation().x(), T.translation().y(), T.translation().z());
  rot = Eigen::Quaterniond(T.linear());
  rot.normalize();
}

void Pose6DOF::fromEigenMatrix(const Eigen::Matrix4d& T) {
  pos = Eigen::Vector3d(T(0, 3), T(1, 3), T(2, 3));
  Eigen::Matrix3d R = T.block(0, 0, 3, 3);  //.transpose();
  rot = Eigen::Quaterniond(R);
  rot.normalize();
}

void Pose6DOF::fromEigenMatrixInFixedFrame(
    const Eigen::Matrix4d& T, const std::string& tgt_frame, const std::string& src_frame, tf::TransformListener* tf_listener) {
  Pose6DOF pose_T_in_src(T);
  transformToFixedFrame(tgt_frame, src_frame, tf_listener);
}

void Pose6DOF::fromROSPoseInFixedFrame(
    const geometry_msgs::Pose& pose_msg, const std::string& tgt_frame, const std::string& src_frame, tf::TransformListener* tf_listener) {
  Pose6DOF pose_in_src(pose_msg);
  transformToFixedFrame(tgt_frame, src_frame, tf_listener);
}

void Pose6DOF::fromROSPose(const geometry_msgs::Pose& pose) {
  pos = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  rot = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  rot.normalize();
}

void Pose6DOF::fromROSPoseWithCovariance(const geometry_msgs::PoseWithCovariance& pose_cov) {
  pos = Eigen::Vector3d(pose_cov.pose.position.x, pose_cov.pose.position.y, pose_cov.pose.position.z);
  rot = Eigen::Quaterniond(
      pose_cov.pose.orientation.w, pose_cov.pose.orientation.x, pose_cov.pose.orientation.y, pose_cov.pose.orientation.z);
  rot.normalize();
  const double* cov_arr = pose_cov.covariance.data();
  cov = Eigen::Matrix<double, 6, 6>(cov_arr);
}

void Pose6DOF::fromROSTransform(const geometry_msgs::Transform& transform) {
  pos = Eigen::Vector3d(transform.translation.x, transform.translation.y, transform.translation.z);
  rot = Eigen::Quaterniond(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  rot.normalize();
}

void Pose6DOF::fromTFPose(const tf::Pose& pose) {
  pos = Eigen::Vector3d(pose.getOrigin().getX(), pose.getOrigin().getY(), pose.getOrigin().getZ());
  rot = Eigen::Quaterniond(pose.getRotation().w(), pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z());
  rot.normalize();
}

void Pose6DOF::fromTFTransform(const tf::Transform& transform) {
  pos = Eigen::Vector3d(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
  rot = Eigen::Quaterniond(
      transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
  rot.normalize();
}

Eigen::Matrix4d Pose6DOF::toEigenMatrix() const {
  Eigen::Matrix4d T;
  T.setIdentity();
  T.block<3, 3>(0, 0) = rot.toRotationMatrix();
  T.block<3, 1>(0, 2) = pos;
  return T;
}

Eigen::Isometry3d Pose6DOF::toEigenIsometry3() const {
  Eigen::Isometry3d T;
  T.setIdentity();
  T.linear() = rot.toRotationMatrix();
  T.translation() = pos;
  return T;
}

tf::Transform Pose6DOF::toTFTransform() const {
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
  transform.setRotation(tf::Quaternion(rot.x(), rot.y(), rot.z(), rot.w()));
  return transform;
}

tf::Pose Pose6DOF::toTFPose() const {
  tf::Pose pose;
  pose.setOrigin(tf::Vector3(pos(0), pos(1), pos(2)));
  pose.setRotation(tf::Quaternion(rot.x(), rot.y(), rot.z(), rot.w()));
  return pose;
}

geometry_msgs::Point Pose6DOF::toROSPoint() const {
  geometry_msgs::Point p;
  p.x = pos(0);
  p.y = pos(1);
  p.z = pos(2);

  return p;
}

geometry_msgs::Pose Pose6DOF::toROSPose() const {
  geometry_msgs::Pose pose;
  pose.position.x = pos(0);
  pose.position.y = pos(1);
  pose.position.z = pos(2);
  pose.orientation.x = rot.x();
  pose.orientation.y = rot.y();
  pose.orientation.z = rot.z();
  pose.orientation.w = rot.w();
  return pose;
}

geometry_msgs::PoseWithCovariance Pose6DOF::toROSPoseWithCovariance() const {
  geometry_msgs::PoseWithCovariance pose_cov;
  pose_cov.pose.position.x = pos(0);
  pose_cov.pose.position.y = pos(1);
  pose_cov.pose.position.z = pos(2);
  pose_cov.pose.orientation.x = rot.x();
  pose_cov.pose.orientation.y = rot.y();
  pose_cov.pose.orientation.z = rot.z();
  pose_cov.pose.orientation.w = rot.w();
  float* cov_arr;
  Eigen::Map<Eigen::Matrix<float, 6, 6>>(cov_arr, cov.rows(), cov.cols()) = cov.cast<float>();
  for (size_t i = 0; i < 36; i++) {
    pose_cov.covariance[i] = cov_arr[i];
  }
  return pose_cov;
}
