/**
 * @brief ORB_SLAM2 for ROS 2 nodes definitions.
 *
 * @author Fabrizio Romanelli <fabrizio.romanelli@gmail.com>
 * @author Roberto Masocco <robmasocco@gmail.com>
 *
 * @date May 26, 2021
 */

#include <memory>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <ORB_SLAM2/System.h>

/* Node names. */
#define ORB2NAME "orbslam2_node"
#define IMGRABNAME "image_grabber"

/* PX4 messages. */
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>

/* Pose messages. */
#include <std_msgs/msg/int32.hpp>

/* Image Grabber message filters stuff. */
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;

/**
 * @brief ORB_SLAM2 node: publishes pose estimates on ROS 2/PX4 topics.
 */
class ORBSLAM2Node : public rclcpp::Node
{
public:
    ORBSLAM2Node(ORB_SLAM2::System *pSLAM, ORB_SLAM2::System::eSensor _sensorType);

    void setPose(cv::Mat _pose);
    void setState(int32_t _state);

private:
    void timer_vio_callback(void);
    void timer_state_callback(void);
    void timestamp_callback(cons px4_msgs::msg::Timesync::SharedPtr msg);

    rclcpp::CallbackGroup::SharedPtr timestamp_clbk_group_;
    rclcpp::CallbackGroup::SharedPtr state_clbk_group_;
    rclcpp::CallbackGroup::SharedPtr vio_clbk_group_;

    rclcpp::TimerBase::SharedPtr vio_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;

    rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr vio_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;

    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr ts_sub_;

    std::atomic<uint64_t> timestamp_ = 0UL;

    std::mutex poseMtx;
    std::mutex stateMtx;

    ORB_SLAM2::System *mpSLAM;
    ORB_SLAM2::System::eSensor sensorType;
    int32_t orbslam2State = ORB_SLAM2::Tracking::eTrackingState::SYSTEM_NOT_READY;

    cv::Mat orbslam2Pose = cv::Mat::eye(4, 4, CV_32F);
}

/**
 * @brief Gets frames from the camera and calls ORB_SLAM2.
 */
class ImageGrabber : public rclcpp::Node
{
public:
    ImageGrabber(ORB_SLAM2::System *pSLAM,
                 std::shared_ptr<ORBSLAM2Node> pORBSLAM2Node,
                 ORB_SLAM2::System::eSensor sensorType,
                 bool irDepth);

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr &msgRGB,
                  const sensor_msgs::msg::Image::SharedPtr &msgD);
    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr &msgLeft,
                    const sensor_msgs::msg::Image::SharedPtr &msgRight);

    ORB_SLAM2::System *mpSLAM;
    std::shared_ptr<ORBSLAM2Node> mpORBSLAM2Node;

private:
    message_filters::Subscriber<sensor_msgs::msg::Image> stream1_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> stream2_sub_;
    std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync_;
};
