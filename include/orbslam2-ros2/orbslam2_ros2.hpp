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

/* Preprocessor consistency checks. */
#if defined(EXTSAMPLER_LIN) && defined(EXTSAMPLER_QUAD)
#error "Only one extrasampler is allowed"
#endif

/* Linear extrapolation oversampling algorithm. */
#ifdef EXTSAMPLER_LIN
#include "../extrasampler/extrasampler_linear.hpp"
#ifndef SAMPLES
#define SAMPLES 8
#endif
#endif

/* Quadratic extrapolation oversampling algorithm. */
#ifdef EXTSAMPLER_QUAD
#include "../extrasampler/extrasampler_quadratic_fixed-time.hpp"
#ifndef CAMERA_STIME
#define CAMERA_STIME 0.067 // Camera sampling + processing time (accounts for ORB_SLAM2 computations too).
#endif
#endif

/* Node names. */
#define ORB2NAME "orbslam2_node"
#define IMGRABNAME "image_grabber"

/* PX4 messages. */
#ifdef PX4
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#endif

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

/**
 * @brief ORB_SLAM2 node: publishes pose estimates on ROS 2/PX4 topics.
 */
class ORBSLAM2Node : public rclcpp::Node
{
public:
    ORBSLAM2Node(ORB_SLAM2::System *pSLAM,
                 ORB_SLAM2::System::eSensor _sensorType,
                 int start_pad);

    void setPose(cv::Mat _pose);
    void setState(int32_t _state);

#if defined(EXTSAMPLER_LIN) || defined(EXTSAMPLER_QUAD)
    double get_time(void);
#endif

private:
    rclcpp::CallbackGroup::SharedPtr vio_clbk_group_;

    rclcpp::TimerBase::SharedPtr vio_timer_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;

    std::mutex poseMtx;
    std::mutex stateMtx;

    ORB_SLAM2::System *mpSLAM;
    ORB_SLAM2::System::eSensor sensorType;
    int32_t orbslam2State = ORB_SLAM2::Tracking::eTrackingState::SYSTEM_NOT_READY;

    cv::Mat orbslam2Pose = cv::Mat::eye(4, 4, CV_32F);

    int start_pad_;

    float r_1_1_, r_1_2_;
    float r_2_1_, r_2_2_;
    float x_offset_, y_offset_, z_offset_;
    Eigen::Quaternionf rot_offset_;

    void timer_vio_callback(void);

#ifdef PX4
    void timestamp_callback(const px4_msgs::msg::Timesync::SharedPtr msg);
    std::atomic<uint64_t> timestamp_;
    rclcpp::CallbackGroup::SharedPtr timestamp_clbk_group_;
    rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr vio_publisher_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr ts_sub_;
#endif

#ifdef EXTSAMPLER_QUAD
    QuadFixTimeExtrasampler<double> ext_x;
    QuadFixTimeExtrasampler<double> ext_y;
    QuadFixTimeExtrasampler<double> ext_z;
    QuadFixTimeExtrasampler<double> ext_q_w;
    QuadFixTimeExtrasampler<double> ext_q_i;
    QuadFixTimeExtrasampler<double> ext_q_j;
    QuadFixTimeExtrasampler<double> ext_q_k;
#endif

#ifdef EXTSAMPLER_LIN
    LinearExtrasampler<double, SAMPLES> ext_x;
    LinearExtrasampler<double, SAMPLES> ext_y;
    LinearExtrasampler<double, SAMPLES> ext_z;
    LinearExtrasampler<double, SAMPLES> ext_q_w;
    LinearExtrasampler<double, SAMPLES> ext_q_i;
    LinearExtrasampler<double, SAMPLES> ext_q_j;
    LinearExtrasampler<double, SAMPLES> ext_q_k;
#endif
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
