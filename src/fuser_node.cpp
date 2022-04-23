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
 */
FuserNode::FuserNode(ORB_SLAM2::System *pSLAM, float camera_pitch) : Node(FUSERNAME), mpSLAM(pSLAM), camera_pitch_(camera_pitch)
{
  // Initialize QoS profile.
  auto state_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

  // Initialize publishers.
#ifdef PX4
  vio_publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("VehicleVisualOdometry_PubSubTopic", 10);
#endif
  state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("ORBS2State", state_qos);

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

  // Activate timer for VIO publishing.
  // VIO: 50 ms period.
  vio_timer_ = this->create_wall_timer(50ms, std::bind(&FuserNode::timer_vio_callback, this), vio_clbk_group_);

  // Compute camera values.
  cp_sin_ = sin(camera_pitch_);
  cp_cos_ = cos(camera_pitch_);

  RCLCPP_INFO(this->get_logger(), "Node initialized, camera pitch: %f", camera_pitch_ * 180.0f / M_PIf32);
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
 * @brief Setter method for pose member.
 *
 * @param _pose Pose data to store.
 */
void FuserNode::setPose(cv::Mat _pose)
{
  poseMtx.lock();
  if (_pose.empty())
  {
    // SLAM lost tracking.
    orbslam2Pose = cv::Mat::eye(4, 4, CV_32F);
    RCLCPP_ERROR(this->get_logger(), "VSLAM tracking lost");
  }
  else
  {
    // Save latest pose.
    orbslam2Pose = _pose;
  }
  poseMtx.unlock();
}

/**
 * @brief Setter method for ORB_SLAM2 state member.
 *
 * @param _state New internal state to set.
 */
void FuserNode::setState(int32_t _state)
{
  stateMtx.lock();
  orbslam2State = _state;
  stateMtx.unlock();
}

/**
 * @brief Publishes the latest VIO data to PX4 topics.
 */
void FuserNode::timer_vio_callback(void)
{
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

  poseMtx.lock();

  // Get the rest from the last stored pose.
  cv::Mat Rwc = orbslam2Pose.rowRange(0, 3).colRange(0, 3).t();
  cv::Mat Twc = -Rwc * orbslam2Pose.rowRange(0, 3).col(3);

  Eigen::Matrix3f orMat;
  orMat(0, 0) = orbslam2Pose.at<float>(0, 0);
  orMat(0, 1) = orbslam2Pose.at<float>(0, 1);
  orMat(0, 2) = orbslam2Pose.at<float>(0, 2);
  orMat(1, 0) = orbslam2Pose.at<float>(1, 0);
  orMat(1, 1) = orbslam2Pose.at<float>(1, 1);
  orMat(1, 2) = orbslam2Pose.at<float>(1, 2);
  orMat(2, 0) = orbslam2Pose.at<float>(2, 0);
  orMat(2, 1) = orbslam2Pose.at<float>(2, 1);
  orMat(2, 2) = orbslam2Pose.at<float>(2, 2);
  Eigen::Quaternionf q_orb(orMat);

  // Conversion from VSLAM to NED is: [x y z]ned = [z x y]vslam.
  // Quaternions must follow the Hamiltonian convention.
  if (camera_pitch_ != 0.0f)
  {
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
  }
  else
  {
    message.set__x(Twc.at<float>(2));
    message.set__y(Twc.at<float>(0));
    message.set__z(Twc.at<float>(1));
    message.q = {q_orb.w(), -q_orb.z(), -q_orb.x(), -q_orb.y()};
  }
  poseMtx.unlock();

  // Send it!
  vio_publisher_->publish(message);
#endif

  // Publish latest tracking state.
  std_msgs::msg::Int32 msg{};
  stateMtx.lock();
  msg.set__data(orbslam2State);
  stateMtx.unlock();
  state_publisher_->publish(msg);
}
