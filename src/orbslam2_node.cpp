/**
 * @brief ORB_SLAM2 pose publisher node source code.
 *
 * @author Fabrizio Romanelli <fabrizio.romanelli@gmail.com>
 * @author Roberto Masocco <robmasocco@gmail.com>
 * 
 * @date May 26, 2021
 */

#include <math.h>

#ifdef TESTING
#include <cstdio>
#endif

#include "../include/orbslam2-ros2/orbslam2_ros2.hpp"

using namespace std::chrono_literals;

/* QoS profile for state data. */
rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

/**
 * @brief Creates an ORBSLAM2Node.
 * 
 * @param pSLAM ORB_SLAM2 instance pointer.
 * @param _sensorType Type of sensor in use.
 */
ORBSLAM2Node::ORBSLAM2Node(ORB_SLAM2::System *pSLAM,
                           ORB_SLAM2::System::eSensor _sensorType,
                           double camera_pitch,
                           int start_pad,
                           bool filter) : Node(ORB2NAME),
                                          mpSLAM(pSLAM),
                                          sensorType(_sensorType),
                                          camera_pitch_(camera_pitch),
                                          start_pad_(start_pad),
                                          filter_(filter)
{
    // Compute navigation offsets.
    x_offset_ = pads_pos[start_pad_ - 1][0];
    y_offset_ = pads_pos[start_pad_ - 1][1];
    if (start_pad_ == 7)
        z_offset_ = -LAND_PAD_7_ALT;
    else
        z_offset_ = 0.0f;

    // Initialize QoS profile.
    auto state_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

    // Initialize publishers.
    vio_publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("VehicleVisualOdometry_PubSubTopic", 1);
    state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("ORBS2State", state_qos);

    // Create callback groups.
    timestamp_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    vio_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscribe to Timesync.
    auto timesync_sub_opt = rclcpp::SubscriptionOptions();
    timesync_sub_opt.callback_group = timestamp_clbk_group_;
    ts_sub_ = this->create_subscription<px4_msgs::msg::Timesync>(
        "Timesync_PubSubTopic",
        1,
        std::bind(&ORBSLAM2Node::timestamp_callback, this, std::placeholders::_1),
        timesync_sub_opt);

    // Activate timer for VIO publishing.
    // VIO: 50 ms period.
    vio_timer_ = this->create_wall_timer(50ms, std::bind(&ORBSLAM2Node::timer_vio_callback, this), vio_clbk_group_);

    // Compute camera values.
    cp_sin_ = sin(camera_pitch_);
    cp_cos_ = cos(camera_pitch_);

    std::cout << "Camera pitch: " << (camera_pitch_ * 180.0 / M_PI) << "°, start pad: " << start_pad_ << std::endl;
    if (filter_)
        std::cout << "XYZ filtering ENABLED" << std::endl;
    else
        std::cout << "XYZ filtering DISABLED" << std::endl;

    RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Stores the latest PX4 timestamp.
 * 
 * @param msg Timesync message pointer.
 */
void ORBSLAM2Node::timestamp_callback(const px4_msgs::msg::Timesync::SharedPtr msg)
{
    timestamp_.store(msg->timestamp, std::memory_order_release);
}

/**
 * @brief Setter method for pose member.
 *
 * @param _pose Pose data to store.
 */
void ORBSLAM2Node::setPose(cv::Mat _pose)
{
    poseMtx.lock();
    if (_pose.empty())
    {
        // SLAM lost tracking.
        orbslam2Pose = cv::Mat::eye(4, 4, CV_64F);
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
void ORBSLAM2Node::setState(int32_t _state)
{
    stateMtx.lock();
    orbslam2State = _state;
    stateMtx.unlock();
}

/**
 * @brief Publishes the latest VIO data to PX4 topics.
 */
void ORBSLAM2Node::timer_vio_callback(void)
{
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

    Eigen::Matrix3d orMat;
    orMat(0, 0) = orbslam2Pose.at<double>(0, 0);
    orMat(0, 1) = orbslam2Pose.at<double>(0, 1);
    orMat(0, 2) = orbslam2Pose.at<double>(0, 2);
    orMat(1, 0) = orbslam2Pose.at<double>(1, 0);
    orMat(1, 1) = orbslam2Pose.at<double>(1, 1);
    orMat(1, 2) = orbslam2Pose.at<double>(1, 2);
    orMat(2, 0) = orbslam2Pose.at<double>(2, 0);
    orMat(2, 1) = orbslam2Pose.at<double>(2, 1);
    orMat(2, 2) = orbslam2Pose.at<double>(2, 2);
    Eigen::Quaterniond q_orb(orMat);

    // Conversion from VSLAM to NED is: [x y z]ned = [z x y]vslam.
    // Quaternions must follow the Hamiltonian convention.
    if (camera_pitch_ != 0.0)
    {
        // Correct orientation: rotate around camera axes in NED body frame,
        // and add start pad yaw.
        Eigen::Quaterniond q_orb_ned = {q_orb.w(), -q_orb.z(), -q_orb.x(), -q_orb.y()};
        auto orb_ned_angles = q_orb_ned.toRotationMatrix().eulerAngles(0, 1, 2);
        Eigen::Quaterniond q_roll = {cos(orb_ned_angles[0] / 2.0),
                                     sin(orb_ned_angles[0] / 2.0) * cp_cos_,
                                     0.0,
                                     sin(orb_ned_angles[0] / 2.0) * cp_sin_};
        Eigen::Quaterniond q_pitch = {cos(orb_ned_angles[1] / 2.0),
                                      0.0,
                                      sin(orb_ned_angles[1] / 2.0),
                                      0.0};
        Eigen::Quaterniond q_yaw = {cos(orb_ned_angles[2] / 2.0),
                                    sin(orb_ned_angles[2] / 2.0) * -cp_sin_,
                                    0.0,
                                    sin(orb_ned_angles[2] / 2.0) * cp_cos_};
        Eigen::Quaterniond q_map = q_yaw * (q_pitch * q_roll);
        message.q = {float(q_map.w()),
                     float(q_map.x()),
                     float(q_map.y()),
                     float(q_map.z())};

        // Correct position: rotate -camera_pitch around Y axis, then add start
        // pad offsets.
        double orb_x = cp_cos_ * Twc.at<double>(2) - cp_sin_ * Twc.at<double>(1);
        double orb_y = Twc.at<double>(0);
        double orb_z = cp_sin_ * Twc.at<double>(2) + cp_cos_ * Twc.at<double>(1);
        message.set__x(float(orb_x + x_offset_));
        message.set__y(float(orb_y + y_offset_));
        message.set__z(float(orb_z + z_offset_));
    }
    else
    {
        // Only account for map offsets on position and orientation.
        message.set__x(float(Twc.at<double>(2) + x_offset_));
        message.set__y(float(Twc.at<double>(0) + y_offset_));
        message.set__z(float(Twc.at<double>(1) + z_offset_));

        Eigen::Quaterniond q_map = {q_orb.w(), -q_orb.z(), -q_orb.x(), -q_orb.y()};
        message.q = {float(q_map.w()),
                     float(q_map.x()),
                     float(q_map.y()),
                     float(q_map.z())};
    }
    poseMtx.unlock();

    // Send it!
    vio_publisher_->publish(message);

#ifdef TESTING
    Eigen::Quaternionf q_msg = {message.q[0], message.q[1], message.q[2], message.q[3]};
    auto euler_ang = q_msg.toRotationMatrix().eulerAngles(0, 1, 2);
    printf("x:\t%f, y:\t%f, z:\t%f\n"
           "q = {\t%f\t%f\t%f\t%f\t}\n"
           "roll:\t%f°, pitch:\t%f°, yaw:\t%f°\n\n",
           message.x, message.y, message.z,
           message.q[0], message.q[1], message.q[2], message.q[3],
           euler_ang[0] * 180.0f / M_PIf32,
           euler_ang[1] * 180.0f / M_PIf32,
           euler_ang[2] * 180.0f / M_PIf32);
#endif

    // Publish latest tracking state.
    std_msgs::msg::Int32 msg{};
    stateMtx.lock();
    msg.set__data(orbslam2State);
    stateMtx.unlock();
    state_publisher_->publish(msg);
}
