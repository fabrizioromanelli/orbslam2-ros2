/**
 * @brief Fuser pose publisher node source code.
 *
 * @author Fabrizio Romanelli <fabrizio.romanelli@gmail.com>
 * @author Roberto Masocco <robmasocco@gmail.com>
 * 
 * @date Apr 23, 2022
 */

#include "fuser_ros2.hpp"

using namespace std::chrono_literals;

/* QoS profile for state data. */
rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

/**
 * @brief Creates an FuserNode.
 * 
 * @param pSLAM ORB_SLAM2 instance pointer.
 * @param realsense realsense instance pointer.
 * @param camera_pitch camera pitch angle in radians.
 */
FuserNode::FuserNode(ORB_SLAM2::System *pSLAM, RealSense *_realsense, float _camera_pitch) : Node(FUSERNAME), mpSLAM(pSLAM), realsense(_realsense), camera_pitch(_camera_pitch)
{
  // Initialize QoS profile.
  auto state_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

  // Initialize publishers.
#ifdef PX4
  vio_publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("VehicleVisualOdometry_PubSubTopic", 10);
#endif
  state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("FuserState", state_qos);

  // Create callback groups.
#ifdef PX4
  timestamp_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
#endif
  vio_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

#ifdef PX4
  // Subscribe to Timesync.
  auto timesync_sub_opt = rclcpp::SubscriptionOptions();
  timesync_sub_opt.callback_group = timestamp_clbk_group_;
  ts_sub_ = this->create_subscription<px4_msgs::msg::Timesync>(
      "Timesync_PubSubTopic",
      10,
      std::bind(&FuserNode::timestamp_callback, this, std::placeholders::_1),
      timesync_sub_opt);
#endif

  firstReset = true;
  orbPrevTs = -1;

  fuser = new Fuser();

  // Activate timer for VIO publishing.
  // VIO: 10 ms period.
  vio_timer_ = this->create_wall_timer(10ms, std::bind(&FuserNode::timer_vio_callback, this), vio_clbk_group_);

  // Compute camera values.
  cp_sin_ = sin(camera_pitch);
  cp_cos_ = cos(camera_pitch);

  RCLCPP_INFO(this->get_logger(), "Node initialized, camera pitch: %f", camera_pitch * 180.0f / M_PIf32);
}

/**
 * @brief Stores the latest PX4 timestamp.
 * 
 * @param msg Timesync message pointer.
 */
#ifdef PX4
void FuserNode::timestamp_callback(const px4_msgs::msg::Timesync::SharedPtr msg)
{
  timestamp_.store(msg->timestamp, std::memory_order_release);
}
#endif

/**
 * @brief Conversion between poses method for fuser state member.
 *
 * @param orbPose ORBSLAM2 Pose to be converted.
 * @param orbState ORBSLAM2 state to be propagated.
 * @param rs2Pose rs2 pose result of the conversion.
 */
void FuserNode::poseConversion(const ORB_SLAM2::HPose & orbPose, const unsigned int orbState, rs2_pose & rs2Pose) {
  rs2Pose.translation.x = orbPose.GetTranslation()[0];
  rs2Pose.translation.y = orbPose.GetTranslation()[1];
  rs2Pose.translation.z = orbPose.GetTranslation()[2];
  rs2Pose.rotation.x    = orbPose.GetRotation()[0];
  rs2Pose.rotation.y    = orbPose.GetRotation()[1];
  rs2Pose.rotation.z    = orbPose.GetRotation()[2];
  rs2Pose.rotation.w    = orbPose.GetRotation()[3];
  rs2Pose.tracker_confidence = orbState;
  return;
}

/**
 * @brief Conversion between poses method for fuser state member. Overloaded.
 *
 * @param rs2Pose rs2 pose to be converted.
 * @param pose Pose result of the conversion.
 */
void FuserNode::poseConversion(const rs2_pose & rs2Pose, Pose & pose) {
  pose.setTranslation(rs2Pose.translation.x, rs2Pose.translation.y, rs2Pose.translation.z);
  pose.setRotation(rs2Pose.rotation.w, rs2Pose.rotation.x, rs2Pose.rotation.y, rs2Pose.rotation.z);
  pose.setAccuracy(rs2Pose.tracker_confidence);
}

/**
 * @brief Conversion between poses method for fuser state member. Overloaded.
 *
 * @param pose Pose to be converted.
 * @param rs2Pose rs2 pose result of the conversion.
 */
void FuserNode::poseConversion(Pose & pose, rs2_pose & rs2Pose) {
  rs2Pose.translation.x = pose.getTranslation()[Pose::X];
  rs2Pose.translation.y = pose.getTranslation()[Pose::Y];
  rs2Pose.translation.z = pose.getTranslation()[Pose::Z];
  rs2Pose.rotation.w    = pose.getRotation().w();
  rs2Pose.rotation.x    = pose.getRotation().x();
  rs2Pose.rotation.y    = pose.getRotation().y();
  rs2Pose.rotation.z    = pose.getRotation().z();
  rs2Pose.tracker_confidence = pose.getAccuracy();
}

/**
 * @brief Publishes the latest VIO data to PX4 topics.
 */
void FuserNode::timer_vio_callback(void)
{
  //
  // Sensor fusion ready to go!
  //
  realsense->run();
  rs2_pose pose = realsense->getPose();

  cv::Mat irMatrix    = realsense->getIRLeftMatrix();
  cv::Mat depthMatrix = realsense->getDepthMatrix();

  // ORBSLAM2 fails if it's running! We need to reset it.
  if (!firstReset && mpSLAM->GetTrackingState() == ORB_SLAM2::Tracking::LOST) {
    mpSLAM->Reset();
  }

  // Pass the IR Left and Depth frames to the SLAM system
  ORB_SLAM2::HPose cameraPose = mpSLAM->TrackIRD(irMatrix, depthMatrix, realsense->getIRLeftTimestamp());
  unsigned int ORBState = (mpSLAM->GetTrackingState() == ORB_SLAM2::Tracking::OK) ? 3 : 0;
  rs2_pose orbPose;
  poseConversion(cameraPose, ORBState, orbPose);

  // The first time I receive a valid ORB-SLAM2 sample, I have to reset the T265 tracker.
  if (!cameraPose.empty() && firstReset) {
    realsense->resetPoseTrack();
    realsense->run();
    pose = realsense->getPose();
    firstReset = false;
  }

  Pose _camPose, _orbPose, orbSyncedPose, fusedPose;
  poseConversion(pose, _camPose);
  poseConversion(orbPose, _orbPose);

  // Sensor fusion
  fuser->synchronizer(orbPrevTs, realsense->getIRLeftTimestamp(), realsense->getPoseTimestamp(), orbPrevPose, _orbPose, orbSyncedPose);
  orbSyncedPose.setAccuracy(_orbPose.getAccuracy());
  fuser->fuse(_camPose, orbSyncedPose);
  fusedPose = fuser->getFusedPose();

  // Save the previous orb pose and timestamp
  poseConversion(orbPose, orbPrevPose);
  orbPrevTs = realsense->getIRLeftTimestamp();

#ifdef PX4
  uint64_t msg_timestamp = timestamp_.load(std::memory_order_acquire);
  px4_msgs::msg::VehicleVisualOdometry message{};

  // Set message timestamp (from Timesync).
  message.set__timestamp(msg_timestamp);
  message.set__timestamp_sample(msg_timestamp);

  // Set local frames of reference (these SHOULD be NED for PX4).
  message.set__local_frame(px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED);
  message.set__velocity_frame(px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED);

  // Set unnecessary data fields: velocities and covariances.
  message.q_offset[0] = NAN;
  message.pose_covariance[0] = NAN;
  message.pose_covariance[15] = NAN;
  message.set__vx(NAN);
  message.set__vy(NAN);
  message.set__vz(NAN);
  message.set__rollspeed(NAN);
  message.set__pitchspeed(NAN);
  message.set__yawspeed(NAN);
  message.velocity_covariance[0] = NAN;
  message.velocity_covariance[15] = NAN;

  // Get the rest from the last stored pose.
  // Conversion from VSLAM to NED is: [x y z]ned = [z x y]vslam.
  // Quaternions must follow the Hamiltonian convention.
  if (camera_pitch != 0.0f)
  {
    // TODO: With different camera pitch is to be implemented yet!
    #if 0
    // Correct orientation: rotate around camera axes in NED body frame.
    Eigen::Quaternionf q_orb_ned = {q_orb.w(), -q_orb.z(), -q_orb.x(), -q_orb.y()};
    auto orb_ned_angles = q_orb_ned.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Quaternionf q_roll = {cos(orb_ned_angles[0] / 2.0f),
                                  sin(orb_ned_angles[0] / 2.0f) * cp_cos_,
                                  0.0f,
                                  sin(orb_ned_angles[0] / 2.0f) * cp_sin_};
    Eigen::Quaternionf q_pitch = {cos(orb_ned_angles[1] / 2.0f),
                                  0.0f,
                                  sin(orb_ned_angles[1] / 2.0f),
                                  0.0f};
    Eigen::Quaternionf q_yaw = {cos(orb_ned_angles[2] / 2.0f),
                                sin(orb_ned_angles[2] / 2.0f) * -cp_sin_,
                                0.0f,
                                sin(orb_ned_angles[2] / 2.0f) * cp_cos_};
    Eigen::Quaternionf q = q_yaw * (q_pitch * q_roll);
    message.q = {q.w(), q.x(), q.y(), q.z()};

    // Correct position: rotate -camera_pitch around Y axis.
    message.set__x(cp_cos_ * Twc.at<float>(2) - cp_sin_ * Twc.at<float>(1));
    message.set__y(Twc.at<float>(0));
    message.set__z(cp_sin_ * Twc.at<float>(2) + cp_cos_ * Twc.at<float>(1));
    #endif
  } else {
    message.set__x(fusedPose.getTranslation()[0]);
    message.set__y(fusedPose.getTranslation()[1]);
    message.set__z(fusedPose.getTranslation()[2]);
    message.q = {(float)fusedPose.getRotation().w(), (float)fusedPose.getRotation().x(), (float)fusedPose.getRotation().y(), (float)fusedPose.getRotation().z()};
  }

  // Send it!
  vio_publisher_->publish(message);
#endif

  // Publish latest tracking state.
  std_msgs::msg::Int32 msg{};
  msg.set__data(_camPose.getAccuracy());
  state_publisher_->publish(msg);
}
