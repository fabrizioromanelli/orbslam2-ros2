#include <iostream>
#include <cstdio>
#include <thread>

#include "../include/orbslam2-ros2/orbslam2_ros2.hpp"

void ImageGrabber::GrabRGBD(const sensor_msgs::msg::Image::SharedPtr &msgRGB, const sensor_msgs::msg::Image::SharedPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        return;
    }

    rclcpp::Time Ts = cv_ptrRGB->header.stamp;
    mpORBSLAM2Node->setPose(mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Ts.seconds()));
    mpORBSLAM2Node->setState(mpSLAM->GetTrackingState());
}

void ImageGrabber::GrabStereo(const sensor_msgs::msg::Image::SharedPtr &msgLeft, const sensor_msgs::msg::Image::SharedPtr &msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        return;
    }

    rclcpp::Time Ts = cv_ptrLeft->header.stamp;
    mpORBSLAM2Node->setPose(mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Ts.seconds()));
    mpORBSLAM2Node->setState(mpSLAM->GetTrackingState());
}

/* Helps with input argument parsing. */
enum string_code
{
    eStereo,
    eRGBD,
    eIRD
};

/**
 * @brief Parses sensor type input argument.
 *
 * @param inString Input argument.
 * @return Sensor type (from enum).
 */
string_code hashit(std::string const &inString)
{
    if (inString == "STEREO")
        return eStereo;
    if (inString == "RGBD")
        return eRGBD;
    if (inString == "IRD")
        return eIRD;
    return eStereo;
}

/* The works. */
int main(int argc, char **argv)
{
    // Disable buffering on stdout and stderr, for performance and correctness.
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    // Initialize ROS 2 connection.
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor orbslam_mt_executor;

    bool irDepth = false;
    ORB_SLAM2::System::eSensor sensorType;
    //! Input arguments check.
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

    //! REMOVE
    message_filters::Subscriber<sensor_msgs::msg::Image> stream1_sub(nodePtr.get(), s1, rmw_qos_profile_sensor_data);
    message_filters::Subscriber<sensor_msgs::msg::Image> stream2_sub(nodePtr.get(), s2, rmw_qos_profile_sensor_data);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), stream1_sub, stream2_sub);
    //! REMOVE

    if (sensorType == ORB_SLAM2::System::RGBD)
        sync.registerCallback(&ImageGrabber::GrabRGBD, &igb);
    else if (sensorType == ORB_SLAM2::System::STEREO)
        sync.registerCallback(&ImageGrabber::GrabStereo, &igb);

    orbslam_mt_executor.add_node(nodePtr);

    orbslam_mt_executor.spin();
    rclcpp::shutdown();
    return 0;
}
