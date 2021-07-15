#include <memory>

#include "floating_camera_plugin.hh"
namespace gazebo {

FloatingCameraPlugin::FloatingCameraPlugin() : ModelPlugin() {
  namespace_.clear();
}

FloatingCameraPlugin::~FloatingCameraPlugin() {
  updateConnection_.reset();
}

void FloatingCameraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  gzmsg << "[FloatingCameraPlugin] loading...\n";

  model_ = _model;
  world_ = model_->GetWorld();

  //disable physics
  world_->SetPhysicsEnabled(false);

  // look for "link" in the plugin parameters
  if (_sdf->HasElement("link")) {
    link_name_ = _sdf->GetElement("link")->Get<std::string>();
    gzmsg << "[FloatingCameraPlugin] link name: " << link_name_ << std::endl;
  } else
    gzerr << "[FloatingCameraPlugin] link not specified\n";
  // find the parent link pointer
  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr) gzthrow("[FloatingCameraPlugin] Cannot find  \"" << link_name_
                                                                         << "\".");
  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&FloatingCameraPlugin::OnUpdate, this, _1));
  // Initial pose to parent (primary) link
  reset_pose_ = link_->WorldPose();

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = nullptr;
    ros::init(argc, argv, "gazebo_client",
              ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode = std::make_unique<ros::NodeHandle>("gazebo_client");

  // Create a named topic, and subscribe to it.
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::PoseWithCovariance>(
          "/" + this->link_name_ + "/cmd_odom",
          1,
          boost::bind(&FloatingCameraPlugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->rosNode->subscribe(so);

  // Spin up the queue helper thread.
  this->rosQueueThread =
      std::thread(std::bind(&FloatingCameraPlugin::QueueThread, this));

}

void FloatingCameraPlugin::OnUpdate(const common::UpdateInfo &_info) {

}

void FloatingCameraPlugin::Reset() {
  // Reset the parent & child link
  link_->SetWorldPose(reset_pose_);
  link_->ResetPhysicsStates();

  gzdbg << "[FloatingCameraPlugin] reset!" << std::endl;
}

/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
/// of the Velodyne.
void FloatingCameraPlugin::OnRosMsg(const geometry_msgs::PoseWithCovarianceConstPtr &_msg) {
  //  this->SetVelocity(_msg->data);
  IgnitionPose pose_(_msg->pose.position.x,
                     _msg->pose.position.y,
                     _msg->pose.position.z,
                     _msg->pose.orientation.w,
                     _msg->pose.orientation.x,
                     _msg->pose.orientation.y,
                     _msg->pose.orientation.z);
  link_->SetWorldPose(pose_);
  link_->SetLinkStatic(true);
  ROS_INFO("received odometry");
}

/// \brief ROS helper function that processes messages
void FloatingCameraPlugin::QueueThread() {
  static const double timeout = 0.001;
  while (this->rosNode->ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(FloatingCameraPlugin);
}