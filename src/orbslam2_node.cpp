#include "../include/orbslam2-ros2/orbslam2_ros2.hpp"

using namespace std::chrono_literals;

/* QoS profile for state data. */
rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

/**
 * @brief Creates an ORBSLAM2Node.
 * 
 * @param pSLAM ORB_SLAM2 instance pointer.
 * @param _sensorType Type of ORB_SLAM2 sensor in use.
 */
ORBSLAM2Node::ORBSLAM2Node(ORB_SLAM2::System *pSLAM, ORB_SLAM2::System::eSensor _sensorType) : Node(ORB2NAME)
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
