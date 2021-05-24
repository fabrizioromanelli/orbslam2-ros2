#include <memory>
#include <chrono>
#include <mutex>
#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <ORB_SLAM2/System.h>
#include "rmw/qos_profiles.h"
#include "std_msgs/msg/int32.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "rclcpp/rclcpp.hpp"

// Includes for PX4
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>

rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

using std::placeholders::_1;
using namespace std::chrono_literals;

class ORBSLAM2Node : public rclcpp::Node
{
  public:
    ORBSLAM2Node(ORB_SLAM2::System* pSLAM, ORB_SLAM2::System::eSensor _sensorType)
    : Node("orbslam2"), mpSLAM(pSLAM), sensorType(_sensorType)
    {
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

      // Create subscriber to the timesync topic of PX4
      // Possible known issue with PX4 timestamp sync. If the following is not working, try to:
      // subscribe to the VehicleOdometry message, copy their timestamp and timestamp_sample to
      // timestamp_ and then publishe the VehicleVisualOdometry with these timestamps.
      // Ref: https://discuss.px4.io/t/fastrtps-ros2-foxy-vehicle-visual-odometry-advertiser/19149/5
      ts_subscriber_   = this->create_subscription<px4_msgs::msg::Timesync>(
        "/Timesync_PubSubTopic", 10,
        [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
          timestamp_.store(msg->timestamp, std::memory_order_release);
      });

      // Create publishers with 50ms period for pose and 100ms period for state
      pose_publisher_  = this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("VehicleVisualOdometry_PubSubTopic", 10);
          pose_timer_  = this->create_wall_timer(50ms, std::bind(&ORBSLAM2Node::timer_pose_callback, this));
      state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("orbslam2_state", qos);
          state_timer_ = this->create_wall_timer(100ms, std::bind(&ORBSLAM2Node::timer_state_callback, this));
    }

    void setPose(cv::Mat);
    void setState(signed int);

  private:
    void timer_pose_callback();
    void timer_state_callback();

    ORB_SLAM2::System* mpSLAM;
    ORB_SLAM2::System::eSensor sensorType;
    rclcpp::TimerBase::SharedPtr pose_timer_, state_timer_;
    rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr ts_subscriber_;
    cv::Mat orbslam2Pose = cv::Mat::eye(4,4,CV_32F);
    signed int orbslam2State = ORB_SLAM2::Tracking::eTrackingState::SYSTEM_NOT_READY;
    std::mutex poseMtx;
    std::mutex stateMtx;
    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
};

void ORBSLAM2Node::setPose(cv::Mat _pose)
{
  poseMtx.lock();
  orbslam2Pose = _pose;
  poseMtx.unlock();
}

void ORBSLAM2Node::setState(signed int _state)
{
  stateMtx.lock();
  orbslam2State = _state;
  stateMtx.unlock();
}

void ORBSLAM2Node::timer_pose_callback()
{
  px4_msgs::msg::VehicleVisualOdometry message = px4_msgs::msg::VehicleVisualOdometry();
  uint64_t msg_timestamp = timestamp_.load(std::memory_order_acquire);

  message.timestamp = msg_timestamp;
  message.timestamp_sample = msg_timestamp;

  message.local_frame              = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED; // ! Needs to be investigated further.
  message.velocity_frame           = px4_msgs::msg::VehicleVisualOdometry::LOCAL_FRAME_NED;
  message.q_offset[0]              = NAN;
  message.pose_covariance [0]      = NAN;
  message.pose_covariance [15]     = NAN;
  message.vx                       = NAN;
  message.vy                       = NAN;
  message.vz                       = NAN;
  message.rollspeed                = NAN;
  message.pitchspeed               = NAN;
  message.yawspeed                 = NAN;
  message.velocity_covariance [0]  = NAN;
  message.velocity_covariance [15] = NAN;

  poseMtx.lock();
  if (orbslam2Pose.empty())
  {
    orbslam2Pose = cv::Mat::eye(4,4,CV_32F);
    message.x    = NAN;
    message.y    = NAN;
    message.z    = NAN;
    message.q[0] = NAN;
  }
  else
  {
    cv::Mat Rwc = orbslam2Pose.rowRange(0,3).colRange(0,3).t();
    cv::Mat Twc = -Rwc*orbslam2Pose.rowRange(0,3).col(3);

    Eigen::Matrix3f orMat;
    orMat(0,0) = orbslam2Pose.at<float>(0,0);
    orMat(0,1) = orbslam2Pose.at<float>(0,1);
    orMat(0,2) = orbslam2Pose.at<float>(0,2);
    orMat(1,0) = orbslam2Pose.at<float>(1,0);
    orMat(1,1) = orbslam2Pose.at<float>(1,1);
    orMat(1,2) = orbslam2Pose.at<float>(1,2);
    orMat(2,0) = orbslam2Pose.at<float>(2,0);
    orMat(2,1) = orbslam2Pose.at<float>(2,1);
    orMat(2,2) = orbslam2Pose.at<float>(2,2);
    Eigen::Quaternionf q(orMat);

    // Conversion from VSLAM to FRD is [x y z]frd = [z x y]vslam
    message.x = Twc.at<float>(2);
    message.y = Twc.at<float>(0);
    message.z = Twc.at<float>(1);
    message.q = {q.w(), q.z(), q.x(), q.y()};
  }
  poseMtx.unlock();

  pose_publisher_->publish(message);
}

void ORBSLAM2Node::timer_state_callback()
{
  auto message = std_msgs::msg::Int32();
  stateMtx.lock();
  message.data = orbslam2State;
  stateMtx.unlock();
  state_publisher_->publish(message);
}

class ImageGrabber
{
  public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, std::shared_ptr<ORBSLAM2Node> pORBSLAM2Node) : mpSLAM(pSLAM), mpORBSLAM2Node(pORBSLAM2Node){}

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr& msgRGB, const sensor_msgs::msg::Image::SharedPtr& msgD);
    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr& msgLeft, const sensor_msgs::msg::Image::SharedPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    std::shared_ptr<ORBSLAM2Node> mpORBSLAM2Node;
};

void ImageGrabber::GrabRGBD(const sensor_msgs::msg::Image::SharedPtr& msgRGB, const sensor_msgs::msg::Image::SharedPtr& msgD)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try
  {
    cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try
  {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }

  rclcpp::Time Ts = cv_ptrRGB->header.stamp;
  mpORBSLAM2Node->setPose(mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Ts.seconds()));
  mpORBSLAM2Node->setState(mpSLAM->GetTrackingState());
}

void ImageGrabber::GrabStereo(const sensor_msgs::msg::Image::SharedPtr& msgLeft, const sensor_msgs::msg::Image::SharedPtr& msgRight)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try
  {
    cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try
  {
    cv_ptrRight = cv_bridge::toCvShare(msgRight);
  }
  catch (cv_bridge::Exception& e)
  {
    return;
  }

  rclcpp::Time Ts = cv_ptrLeft->header.stamp;
  mpORBSLAM2Node->setPose(mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Ts.seconds()));
  mpORBSLAM2Node->setState(mpSLAM->GetTrackingState());
}

enum string_code {
  eStereo,
  eRGBD,
  eIRD
};

string_code hashit (std::string const& inString) {
  if (inString == "STEREO") return eStereo;
  if (inString == "RGBD") return eRGBD;
  if (inString == "IRD") return eIRD;
  return eStereo;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  bool irDepth = false;
  ORB_SLAM2::System::eSensor sensorType;
  switch (hashit(argv[3]))
  {
    case eStereo:
      sensorType = ORB_SLAM2::System::STEREO;
      break;
    case eIRD:
      irDepth = true;
    case eRGBD:
      sensorType = ORB_SLAM2::System::RGBD;
      break;

      break;
    default:
      break;
  }

  bool display;
  string dFlag(argv[4]);
  if (dFlag == "ON")
    display = true;
  else
    display = false;

  ORB_SLAM2::System SLAM(argv[1], argv[2], sensorType, display);
  auto nodePtr = std::make_shared<ORBSLAM2Node>(&SLAM, sensorType);

  ImageGrabber igb(&SLAM, nodePtr);
  std::string s1, s2;

  if (sensorType == ORB_SLAM2::System::RGBD)
  {
    if (irDepth)
    {
      s1 = "/vslam/infra1/image_rect_raw";
      s2 = "/vslam/aligned_depth_to_infra1/image_raw";
    }
    else
    {
      s1 = "/vslam/color/image_raw";
      s2 = "/vslam/aligned_depth_to_color/image_raw";
    }
  }
  else if (sensorType == ORB_SLAM2::System::STEREO)
  {
      s1 = "/vslam/infra1/image_rect_raw";
      s2 = "/vslam/infra2/image_rect_raw";
  }

  message_filters::Subscriber<sensor_msgs::msg::Image> stream1_sub(nodePtr.get(), s1, rmw_qos_profile_sensor_data);
  message_filters::Subscriber<sensor_msgs::msg::Image> stream2_sub(nodePtr.get(), s2, rmw_qos_profile_sensor_data);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), stream1_sub, stream2_sub);

  if (sensorType == ORB_SLAM2::System::RGBD)
    sync.registerCallback(&ImageGrabber::GrabRGBD, &igb);
  else if (sensorType == ORB_SLAM2::System::STEREO)
    sync.registerCallback(&ImageGrabber::GrabStereo, &igb);

  rclcpp::spin(nodePtr);
  rclcpp::shutdown();
  return 0;
}
