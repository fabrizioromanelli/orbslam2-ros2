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
 * @brief Implementation of a filter for VIO data.
 *
 * @param sample New sample to add.
 * @param axis Axis index: [x y z] = [0 1 2].
 * @return Filtered sample along specified axis.
 */
float ORBSLAM2Node::orb_filter(float sample, int axis)
{
    // Compute new filtered sample
    float y_filtered = (-a_1_ * y_filtered_old_[axis] - a_0_ * y_filtered_old_old_[axis] + b_1_ * orb_data_old_[axis] + b_0_ * orb_data_old_old_[axis]);

    // Shift samples left one position
    for (int j = 0; j < N_ORB_BUFFER - 1; j++)
        orb_buffer_[axis][j] = orb_buffer_[axis][j + 1];

    // Store new sample
    orb_buffer_[axis][N_ORB_BUFFER - 1] = sample;

    // Update trusting threshold
    float no_trusting = 0.0;
    for (int j = 0; j < (N_ORB_BUFFER - 1); j++)
        no_trusting += abs(orb_buffer_[axis][N_ORB_BUFFER - 1 - j] - orb_buffer_[axis][N_ORB_BUFFER - 2 - j]);

    // Compute trust coefficients
    tau_dynamic_[axis] = max(tau_dynamic_min_, tau_dynamic_[axis] - 1.0f / 120.0f);
    if (no_trusting > tau_dynamic_thresh_[axis])
        tau_dynamic_[axis] = tau_dynamic_slow_;

    // Extract trusted sample
    float y_timevariant = tau_dynamic_[axis] * y_timevariant_old_[axis] + (1.0f - tau_dynamic_[axis]) + y_filtered;

    // Store latest values and samples
    y_timevariant_old_[axis] = y_timevariant;
    orb_data_old_old_[axis] = orb_data_old_[axis];
    orb_data_old_[axis] = sample;
    y_filtered_old_old_[axis] = y_filtered_old_[axis];
    y_filtered_old_[axis] = y_filtered;

    return y_timevariant;
}

/**
 * @brief Creates an ORBSLAM2Node.
 * 
 * @param pSLAM ORB_SLAM2 instance pointer.
 * @param _sensorType Type of sensor in use.
 */
ORBSLAM2Node::ORBSLAM2Node(ORB_SLAM2::System *pSLAM,
                           ORB_SLAM2::System::eSensor _sensorType,
                           float camera_pitch,
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

    std::cout << "Camera pitch: " << (camera_pitch_ * 180.0f / M_PIf32) << "°, start pad: " << start_pad_ << std::endl;
    if (filter_)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < N_ORB_BUFFER; j++)
                orb_buffer_[i][j] = 0.0f;
        }
        std::cout << "XYZ filtering ENABLED" << std::endl;
    }
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
        // Correct orientation: rotate around camera axes in NED body frame,
        // and add start pad yaw.
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
        Eigen::Quaternionf q_map = q_yaw * (q_pitch * q_roll);
        message.q = {q_map.w(), q_map.x(), q_map.y(), q_map.z()};

        // Correct position: rotate -camera_pitch around Y axis, then add start
        // pad offsets.
        float orb_x = cp_cos_ * Twc.at<float>(2) - cp_sin_ * Twc.at<float>(1);
        float orb_y = Twc.at<float>(0);
        float orb_z = cp_sin_ * Twc.at<float>(2) + cp_cos_ * Twc.at<float>(1);
        if (filter_)
        {
            message.set__x(orb_filter(orb_x + x_offset_, 0));
            message.set__y(orb_filter(orb_y + y_offset_, 1));
            message.set__z(orb_filter(orb_z + z_offset_, 2));
        }
        else
        {
            message.set__x(orb_x + x_offset_);
            message.set__y(orb_y + y_offset_);
            message.set__z(orb_z + z_offset_);
        }
    }
    else
    {
        // Only account for map offsets on position and orientation.
        if (filter_)
        {
            message.set__x(orb_filter(Twc.at<float>(2) + x_offset_, 0));
            message.set__y(orb_filter(Twc.at<float>(0) + y_offset_, 1));
            message.set__z(orb_filter(Twc.at<float>(1) + z_offset_, 2));
        }
        else
        {
            message.set__x(Twc.at<float>(2) + x_offset_);
            message.set__y(Twc.at<float>(0) + y_offset_);
            message.set__z(Twc.at<float>(1) + z_offset_);
        }

        Eigen::Quaternionf q_map = {q_orb.w(), -q_orb.z(), -q_orb.x(), -q_orb.y()};
        message.q = {q_map.w(),
                     q_map.x(),
                     q_map.y(),
                     q_map.z()};
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
