#ifndef FLOATING_CAMERA_PLUGIN_HH_
#define FLOATING_CAMERA_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/simbody/SimbodyJoint.hh>
#include <cstdio>
#include <gazebo/common/CommonTypes.hh>
#include <eigen3/Eigen/Dense>
//#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>

using IgnitionVector = ignition::math::Vector3d;
using IgnitionPose = ignition::math::Pose3d;
using IgnitionQuaternion = ignition::math::Quaterniond;

namespace gazebo {

inline Eigen::Vector3d vectorIgnition2Eigen(IgnitionVector vec) {
  Eigen::Vector3d out;
  out << vec.X(), vec.Y(), vec.Z();
  return out;
}

inline IgnitionVector vectorEigen2Ignition(Eigen::Vector3d vec) {
  IgnitionVector out(vec(0), vec(1), vec(2));
  return out;
}

inline Eigen::Matrix3d rotationIgnition2Eigen(IgnitionQuaternion quat) {
  Eigen::Quaterniond eig_quat(quat.W(), quat.X(), quat.Y(), quat.Z());
  return eig_quat.toRotationMatrix();
}

inline IgnitionQuaternion rotationEigen2Ignition(Eigen::Matrix3d rot) {
  Eigen::Quaterniond q(rot);
  IgnitionQuaternion ign_q(q.w(), q.x(), q.y(), q.z());
  return ign_q;
}

inline Eigen::Matrix3d hat(const Eigen::Vector3d _vec) {
  return (Eigen::Matrix3d() << 0.0, -_vec(2), _vec(1), _vec(2),
      0.0, -_vec(0), -_vec(1), _vec(0), 0.0)
      .finished();
}

class FloatingCameraPlugin : public ModelPlugin {

public:
  FloatingCameraPlugin();
  ~FloatingCameraPlugin() override;
  void OnRosMsg(const geometry_msgs::PoseWithCovarianceConstPtr &_msg);

protected:
  void CustomInit();
  void Reset() override;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const common::UpdateInfo &_info);

private:
  common::Time lastUpdateTime_, currentUpdateTime_;

  std::string link_name_;
  std::string namespace_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  /// \brief Reset the link pose
  IgnitionPose reset_pose_;

  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode;
  /// \brief A ROS subscriber
  ros::Subscriber rosSub;
  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;
  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;
  void QueueThread();
};

}

#endif // FLOATING_CAMERA_PLUGIN_HH_