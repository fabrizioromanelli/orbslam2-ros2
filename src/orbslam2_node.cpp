/**
 * @brief ORB_SLAM2 pose publisher node source code.
 *
 * @author Fabrizio Romanelli <fabrizio.romanelli@gmail.com>
 * @author Roberto Masocco <robmasocco@gmail.com>
 * 
 * @date May 26, 2021
 */

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
                           ORB_SLAM2::System::eSensor _sensorType) : Node(ORB2NAME)
{
    // Initialize members.
    mpSLAM = pSLAM;
    sensorType = _sensorType;

    // Initialize QoS profile.
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

    // Initialize publishers.
#ifdef PX4
    vio_publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("VehicleVisualOdometry_PubSubTopic", 10);
#endif
    state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("orbslam2_state", qos);

    // Create callback groups.
#ifdef PX4
    timestamp_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
#endif
    state_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    vio_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

#ifdef PX4
    // Subscribe to Timesync.
    auto timesync_sub_opt = rclcpp::SubscriptionOptions();
    timesync_sub_opt.callback_group = timestamp_clbk_group_;
    ts_sub_ = this->create_subscription<px4_msgs::msg::Timesync>(
        "Timesync_PubSubTopic",
        10,
        std::bind(&ORBSLAM2Node::timestamp_callback, this, std::placeholders::_1),
        timesync_sub_opt);
#endif

    // Activate timers for state and VIO publishing.
    // VIO: 50 ms period.
    // State: 100 ms period.
    vio_timer_ = this->create_wall_timer(50ms, std::bind(&ORBSLAM2Node::timer_vio_callback, this), vio_clbk_group_);
    state_timer_ = this->create_wall_timer(100ms, std::bind(&ORBSLAM2Node::timer_state_callback, this), state_clbk_group_);

#ifdef EXTSAMPLER_QUAD
    // Initialize quadratic extrasamplers.
    ext_x = QuadFixTimeExtrasampler<double>(CAMERA_STIME);
    ext_y = QuadFixTimeExtrasampler<double>(CAMERA_STIME);
    ext_z = QuadFixTimeExtrasampler<double>(CAMERA_STIME);
    ext_q_w = QuadFixTimeExtrasampler<double>(CAMERA_STIME);
    ext_q_i = QuadFixTimeExtrasampler<double>(CAMERA_STIME);
    ext_q_j = QuadFixTimeExtrasampler<double>(CAMERA_STIME);
    ext_q_k = QuadFixTimeExtrasampler<double>(CAMERA_STIME);
#endif

    RCLCPP_INFO(this->get_logger(), "Node initialized.");
}

/**
 * @brief Returns the current time measured by the ROS 2 node, in seconds.
 *
 * @return Absolute time, in seconds.
 */
#if defined(EXTSAMPLER_LIN) || defined(EXTSAMPLER_QUAD)
#include <time.h>
#include <string.h>

double ORBSLAM2Node::get_time(void)
{
    double time = 0.0;
    struct timespec now;
    memset(&now, 0, sizeof(now));
    if (clock_gettime(CLOCK_MONOTONIC, &now) == -1)
    {
        RCLCPP_FATAL(this->get_logger(), "Failed to get time from system.");
        exit(EXIT_FAILURE);
    }
    time = (double)(now.tv_sec) + ((double)(now.tv_nsec) * 1e-9);
    return time;
}
#endif

/**
 * @brief Stores the latest PX4 timestamp.
 * 
 * @param msg Timesync message pointer.
 */
#ifdef PX4
void ORBSLAM2Node::timestamp_callback(const px4_msgs::msg::Timesync::SharedPtr msg)
{
    timestamp_.store(msg->timestamp, std::memory_order_release);
}
#endif

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
#if defined(EXTSAMPLER_LIN) || defined(EXTSAMPLER_QUAD)
        // Reset the extrapolators.
        ext_x.reset();
        ext_y.reset();
        ext_z.reset();
        ext_q_w.reset();
        ext_q_i.reset();
        ext_q_j.reset();
        ext_q_k.reset();
#endif
        RCLCPP_WARN(this->get_logger(), "VSLAM tracking lost.");
    }
    else
    {
        // Save latest pose.
        orbslam2Pose = _pose;

#if defined(EXTSAMPLER_LIN) || defined(EXTSAMPLER_QUAD)
        // Extract measurements from latest pose sample.
        cv::Mat Rwc = _pose.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat Twc = -Rwc * _pose.rowRange(0, 3).col(3);

        Eigen::Matrix3f orMat;
        orMat(0, 0) = _pose.at<float>(0, 0);
        orMat(0, 1) = _pose.at<float>(0, 1);
        orMat(0, 2) = _pose.at<float>(0, 2);
        orMat(1, 0) = _pose.at<float>(1, 0);
        orMat(1, 1) = _pose.at<float>(1, 1);
        orMat(1, 2) = _pose.at<float>(1, 2);
        orMat(2, 0) = _pose.at<float>(2, 0);
        orMat(2, 1) = _pose.at<float>(2, 1);
        orMat(2, 2) = _pose.at<float>(2, 2);
        Eigen::Quaternionf q(orMat);

        // Update extrapolators with latest sample data.
        double new_T = get_time();
        // Conversion from VSLAM to NED is: [x y z]ned = [z x y]vslam.
        // Quaternions must follow the Hamiltonian convention.
        ext_x.update_samples(new_T, Twc.at<float>(2));
        ext_y.update_samples(new_T, Twc.at<float>(0));
        ext_z.update_samples(new_T, Twc.at<float>(1));
        ext_q_w.update_samples(new_T, q.w());
        ext_q_i.update_samples(new_T, -q.z());
        ext_q_j.update_samples(new_T, -q.x());
        ext_q_k.update_samples(new_T, -q.y());
#endif
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
 * @brief Publishes the latest ORB_SLAM2 state value.
 */
void ORBSLAM2Node::timer_state_callback(void)
{
    std_msgs::msg::Int32 msg{};
    stateMtx.lock();
    msg.set__data(orbslam2State);
    stateMtx.unlock();
    state_publisher_->publish(msg);
}

/**
 * @brief Publishes the latest VIO data to PX4 topics.
 */
void ORBSLAM2Node::timer_vio_callback(void)
{
#ifdef PX4
    uint64_t msg_timestamp = timestamp_.load(std::memory_order_acquire);
    px4_msgs::msg::VehicleVisualOdometry message{};

    // Set message timestamp (from Timesync).
    message.set__timestamp(msg_timestamp);
    message.set__timestamp_sample(msg_timestamp);

    // Set local frames of reference (these SHOULD be NED for PX4).
    // Note that velocity is not sent.
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
#if defined(EXTSAMPLER_LIN) || defined(EXTSAMPLER_QUAD)
    // Get the rest from the extrapolators.
    double T = get_time(); // Extrapolator takes new absolute sampling time.
    message.set__x((float)(ext_x.get_sample(T)));
    message.set__y((float)(ext_y.get_sample(T)));
    message.set__z((float)(ext_z.get_sample(T)));
    message.q[0] = (float)(ext_q_w.get_sample(T));
    message.q[1] = (float)(ext_q_i.get_sample(T));
    message.q[2] = (float)(ext_q_j.get_sample(T));
    message.q[3] = (float)(ext_q_k.get_sample(T));
#else
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
    Eigen::Quaternionf q(orMat);

    // Conversion from VSLAM to NED is: [x y z]ned = [z x y]vslam.
    // Quaternions must follow the Hamiltonian convention.
    message.set__x(Twc.at<float>(2));
    message.set__y(Twc.at<float>(0));
    message.set__z(Twc.at<float>(1));
    message.q = {q.w(), -q.z(), -q.x(), -q.y()};
#endif
    poseMtx.unlock();

    // Send it!
    vio_publisher_->publish(message);
#endif
}
