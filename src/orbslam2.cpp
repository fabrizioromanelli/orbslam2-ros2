#include <functional>
#include <queue>
#include <thread>
#include <mutex>
#include <memory>
#include <chrono>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <System.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "rclcpp/rclcpp.hpp"

#define MAX_IMU_BUFFER 7

using std::placeholders::_1;

class ORBSLAM2Subscriber : public rclcpp::Node
{
public:

  ORBSLAM2Subscriber(ORB_SLAM2::System* pSLAM, ORB_SLAM2::System::eSensor _sensorType)
  : Node("orbslam2_to_realsense_subscriber"), mpSLAM(pSLAM), sensorType(_sensorType)
  {
    rclcpp::QoS video_qos(10);
    video_qos.keep_last(10);
    video_qos.best_effort();
    video_qos.durability_volatile();

    if (sensorType != ORB_SLAM2::System::STEREO && sensorType != ORB_SLAM2::System::RGBD)
    {
      irLeftSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/infra1/image_rect_raw", video_qos, std::bind(&ORBSLAM2Subscriber::irLeft_callback, this, _1)
      );
    }

    if (this->sensorType == ORB_SLAM2::System::IMU_STEREO)
    {
      irRightSubscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/infra2/image_rect_raw", video_qos, std::bind(&ORBSLAM2Subscriber::irRight_callback, this, _1)
      );
    }
  }

  void runSLAM();

private:
  void irLeft_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock{mBufMutexLeft};
    std::cout << "ir" << std::endl;
    if (!imgLeftBuf.empty())
      imgLeftBuf.pop();
    imgLeftBuf.push(msg);
  }

  void irRight_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock{mBufMutexRight};
    if (!imgRightBuf.empty())
      imgRightBuf.pop();
    imgRightBuf.push(msg);
  }

  void imuSLAM();

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr irLeftSubscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr irRightSubscription_;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSubscription_;

  // Queues
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::queue<sensor_msgs::msg::Image::SharedPtr> imgLeftBuf, imgRightBuf;

  // Mutex
  std::mutex mBufMutex;
  std::mutex mBufMutexLeft, mBufMutexRight;

  ORB_SLAM2::System* mpSLAM;
  ORB_SLAM2::System::eSensor sensorType;
};

void ORBSLAM2Subscriber::runSLAM()
{
//   if (this->sensorType == ORB_SLAM2::System::IMU_STEREO || this->sensorType == ORB_SLAM2::System::IMU_RGBD)
//     imuSLAM();
}

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::msg::Image::SharedPtr& msgRGB, const sensor_msgs::msg::Image::SharedPtr& msgD);
    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr& msgLeft, const sensor_msgs::msg::Image::SharedPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
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
    mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Ts.seconds());
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
    mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Ts.seconds());
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

  ORB_SLAM2::System SLAM(argv[1], argv[2], sensorType, true);
  auto nodePtr = std::make_shared<ORBSLAM2Subscriber>(&SLAM, sensorType);

  ImageGrabber igb(&SLAM);
  std::string s1, s2;

  if (sensorType == ORB_SLAM2::System::RGBD)
  {
    if (irDepth)
    {
      s1 = "/camera/infra1/image_rect_raw";
      s2 = "/camera/aligned_depth_to_infra1/image_raw";
    }
    else
    {
      s1 = "/camera/color/image_raw";
      s2 = "/camera/aligned_depth_to_color/image_raw";
    }
  }
  else if (sensorType == ORB_SLAM2::System::STEREO)
  {
      s1 = "/camera/infra1/image_rect_raw";
      s2 = "/camera/infra2/image_rect_raw";
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
