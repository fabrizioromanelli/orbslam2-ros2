/**
 * @brief ORB_SLAM2 for ROS 2 nodes definitions.
 *
 * @author Fabrizio Romanelli <fabrizio.romanelli@gmail.com>
 * @author Roberto Masocco <robmasocco@gmail.com>
 *
 * @date May 26, 2021
 */

#ifndef ORBSLAM2_ROS2_HPP
#define ORBSLAM2_ROS2_HPP

#include <memory>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <ORB_SLAM2/System.h>

#include "../../../navigation_data/navigation_data.hpp"

/* Node names. */
#define ORB2NAME "orbslam2_node"
#define IMGRABNAME "image_grabber"

/* PX4 messages. */
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>

/* State messages. */
#include <std_msgs/msg/int32.hpp>

/* Camera sampling messages. */
#ifdef BENCHMARK
#include <std_msgs/msg/bool.hpp>
#endif

/* Image Grabber message filters stuff. */
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;

/* ORB_SLAM2 data filter buffer size. */
#define N_ORB_BUFFER 4

/**
 * @brief ORB_SLAM2 node: publishes pose estimates on ROS 2/PX4 topics.
 */
class ORBSLAM2Node : public rclcpp::Node
{
public:
    ORBSLAM2Node(ORB_SLAM2::System *pSLAM,
                 ORB_SLAM2::System::eSensor _sensorType,
                 double camera_pitch,
                 int start_pad,
                 bool filter);

    void setPose(cv::Mat _pose);
    void setState(int32_t _state);

private:
    rclcpp::CallbackGroup::SharedPtr vio_clbk_group_;

    rclcpp::TimerBase::SharedPtr vio_timer_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;

    std::mutex poseMtx;
    std::mutex stateMtx;

    ORB_SLAM2::System *mpSLAM;
    ORB_SLAM2::System::eSensor sensorType;
    int32_t orbslam2State = ORB_SLAM2::Tracking::eTrackingState::SYSTEM_NOT_READY;

    cv::Mat orbslam2Pose = cv::Mat::eye(4, 4, CV_64F);

    double camera_pitch_;
    double cp_sin_, cp_cos_;

    int start_pad_;

    float x_offset_, y_offset_, z_offset_;

    bool filter_;

    const double a_0_ = 0.239018;
    const double a_1_ = -0.7379;
    const double b_0_ = 0.19080;
    const double b_1_ = 0.31028;
    const double tau_dynamic_thresh_[3] = {0.5, 0.7, 0.2};
    const double tau_thresh_min_[3] = {0.1, 0.1, 0.08};
    const double tau_dynamic_min_ = 0.1;
    const double tau_dynamic_slow_ = 0.98;

    double y_filtered_old_[3] = {0.0, 0.0, 0.0};
    double y_filtered_old_old_[3] = {0.0, 0.0, 0.0};
    double orb_data_old_[3] = {0.0, 0.0, 0.0};
    double orb_data_old_old_[3] = {0.0, 0.0, 0.0};
    double y_timevariant_old_[3] = {0.0, 0.0, 0.0};
    double tau_dynamic_[3] = {0.0, 0.0, 0.0};
    double orb_buffer_[3][N_ORB_BUFFER];

    void timer_vio_callback(void);

    void timestamp_callback(const px4_msgs::msg::Timesync::SharedPtr msg);

    double orb_filter(double sample, int axis);

    std::atomic<uint64_t> timestamp_;
    rclcpp::CallbackGroup::SharedPtr timestamp_clbk_group_;
    rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr vio_publisher_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr ts_sub_;
};

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

#ifdef BENCHMARK
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sampling_publisher_;
#endif
};

#endif
