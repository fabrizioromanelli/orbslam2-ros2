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
    vio_publisher_ = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("VehicleVisualOdometry_PubSubTopic", 10);
    state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("orbslam2_state", qos);

    // Create callback groups.
    timestamp_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    vio_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscribe to Timesync.
    auto timesync_sub_opt = rclcpp::SubscriptionOptions();
    timesync_sub_opt.callback_group = timestamp_clbk_group_;
    ts_sub_ = this->create_subscription<px4_msgs::msg::Timesync>(
        "Timesync_PubSubTopic",
        10,
        std::bind(&ORBSLAM2Node::timestamp_callback, this, std::placeholders::_1),
        timesync_sub_opt);

    // Activate timers for state and VIO publishing.
    // VIO: 50 ms period.
    // State: 100 ms period.
    vio_timer_ = this->create_wall_timer(50ms, std::bind(&ORBSLAM2Node::timer_vio_callback, this), vio_clbk_group_);
    state_timer_ = this->create_wall_timer(100ms, std::bind(&ORBSLAM2Node::timer_state_callback, this), state_clbk_group_);

    RCLCPP_INFO(this->get_logger(), "Node initialized.");
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

/** @brief Setter method for pose member.
 *
 * @param _pose Pose matrix to store.
 */
void ORBSLAM2Node::setPose(cv::Mat _pose)
{
    poseMtx.lock();
    orbslam2Pose = _pose;
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
    msg.data = orbslam2State;
    stateMtx.unlock();
    state_publisher_->publish(msg);
}

/**
 * @brief Publishes the latest VIO data to PX4 topics.
 */
void ORBSLAM2Node::timer_vio_callback(void)
{
    uint64_t msg_timestamp = timestamp_.load(std::memory_order_acquire);
    px4_msgs::msg::VehicleVisualOdometry message{};

    // Set message timestamp (from Timesync).
    message.timestamp = msg_timestamp;
    message.timestamp_sample = msg_timestamp;

    // Set local frames of reference (these SHOULD be NED for PX4).
    // Note that velocity is not sent.
    message.local_frame = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED;
    message.velocity_frame = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED;

    // Set unnecessary data fields: velocities and covariances.
    message.q_offset[0] = NAN;
    message.pose_covariance[0] = NAN;
    message.pose_covariance[15] = NAN;
    message.vx = NAN;
    message.vy = NAN;
    message.vz = NAN;
    message.rollspeed = NAN;
    message.pitchspeed = NAN;
    message.yawspeed = NAN;
    message.velocity_covariance[0] = NAN;
    message.velocity_covariance[15] = NAN;

    // Get the rest from the pose estimate. Some computations are necessary.
    poseMtx.lock();
    if (orbslam2Pose.empty())
    {
        orbslam2Pose = cv::Mat::eye(4, 4, CV_32F);
        message.x = NAN;
        message.y = NAN;
        message.z = NAN;
        message.q[0] = NAN;
    }
    else
    {
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
        message.x = Twc.at<float>(2);
        message.y = Twc.at<float>(0);
        message.z = Twc.at<float>(1);
        message.q = {q.w(), q.z(), q.x(), q.y()};
    }
    poseMtx.unlock();

    // Send it!
    vio_publisher_->publish(message);
}
