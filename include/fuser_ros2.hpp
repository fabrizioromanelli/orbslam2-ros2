/**
 * @brief Fuser for ROS 2 node definition.
 *
 * @author Fabrizio Romanelli <fabrizio.romanelli@gmail.com>
 * @author Roberto Masocco <robmasocco@gmail.com>
 *
 * @date Apr 23, 2022
 */

#ifndef FUSER_ROS2_HPP
#define FUSER_ROS2_HPP

#include <memory>
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <ORB_SLAM2/System.h>
#include "realsense.hpp"
#include "fuser.hpp"
#include "pose.hpp"

/* Node names. */
#define FUSERNAME "fuser_node"

/* PX4 messages. */
#ifdef PX4
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#endif

/* State messages. */
#include <std_msgs/msg/int32.hpp>

/**
 * @brief ORB_SLAM2 node: publishes pose estimates on ROS 2/PX4 topics.
 */
class FuserNode : public rclcpp::Node
{
public:
  FuserNode(ORB_SLAM2::System *pSLAM, RealSense *realsense, float camera_pitch);

  void poseConversion(const ORB_SLAM2::HPose &, const unsigned int, rs2_pose &);
  void poseConversion(const rs2_pose &, Pose &);
  void poseConversion(Pose &, rs2_pose &);

private:
  void timer_vio_callback(void);

  rclcpp::CallbackGroup::SharedPtr vio_clbk_group_;

  rclcpp::TimerBase::SharedPtr vio_timer_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;

  ORB_SLAM2::System *mpSLAM;
  int32_t fuserState = Pose::trackQoS::LOST;

  RealSense *realsense;

  bool firstReset;

  Pose orbPrevPose;
  rs2_time_t orbPrevTs;

  Fuser *fuser;

  float camera_pitch;
  float cp_sin_, cp_cos_;

#ifdef PX4
  void timestamp_callback(const px4_msgs::msg::Timesync::SharedPtr msg);
  std::atomic<uint64_t> timestamp_;
  rclcpp::CallbackGroup::SharedPtr timestamp_clbk_group_;
  rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr vio_publisher_;
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr ts_sub_;
#endif
};

#endif // FUSER_ROS2_HPP
