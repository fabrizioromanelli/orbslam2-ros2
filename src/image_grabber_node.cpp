/**
 * @brief ImageGrabber node source code.
 *
 * @author Fabrizio Romanelli <fabrizio.romanelli@gmail.com>
 * @author Roberto Masocco <robmasocco@gmail.com>
 *
 * @date May 26, 2021
 */

#include "../include/orbslam2-ros2/orbslam2_ros2.hpp"

/**
 * @brief Creates an ImageGrabber node.
 *
 * @param pSLAM ORB_SLAM2 instance pointer.
 * @param pORBSLAM2Node Sibling ORBSLAM2Node pointer.
 * @param sensorType Type of sensor in use.
 * @param irDepth IR depth measurement availability flag.
 */
ImageGrabber::ImageGrabber(ORB_SLAM2::System *pSLAM,
                           std::shared_ptr<ORBSLAM2Node> pORBSLAM2Node,
                           ORB_SLAM2::System::eSensor sensorType,
                           bool irDepth) : Node("IMGRABNAME")
{
    // Set common members.
    mpSLAM = pSLAM;
    mpORBSLAM2Node = pORBSLAM2Node;

    // Subscribe to RealSense topics.
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
    stream1_sub.subscribe(this, s1, rmw_qos_profile_sensor_data);
    stream2_sub.subscribe(this, s2, rmw_qos_profile_sensor_data);

    // Create and configure streams synchronizer.
    sync_ = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(10), stream1_sub, stream2_sub);
    if (sensorType == ORB_SLAM2::System::RGBD)
        sync_->registerCallback(&ImageGrabber::GrabRGBD, this);
    else if (sensorType == ORB_SLAM2::System::STEREO)
        sync_->registerCallback(&ImageGrabber::GrabStereo, this);
    
    RCLCPP_INFO(this->get_logger(), IMGRABNAME " node initialized.");
}

/**
 * @brief Grabs RGBD frames and calls ORB_SLAM2.
 *
 * @param msgRGB Pointer to (a pointer to) the RGB image.
 * @param msgD Pointer to (a pointer to) the depth image.
 */
void ImageGrabber::GrabRGBD(const const sensor_msgs::msg::Image::SharedPtr &msgRGB,
                            const sensor_msgs::msg::Image::SharedPtr &msgD)
{
    // Copy the ROS image messages to cv::Mat.
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

    // Call ORB_SLAM2 and update local data.
    rclcpp::Time Ts = cv_ptrRGB->header.stamp;
    mpORBSLAM2Node->setPose(mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Ts.seconds()));
    mpORBSLAM2Node->setState(mpSLAM->GetTrackingState());
}

/**
 * @brief Grabs stereo frames and calls ORB_SLAM2.
 *
 * @param msgLeft Pointer to (a pointer to) the left image.
 * @param msgRight Pointer to (a pointer to) the right image.
 */
void ImageGrabber::GrabStereo(const sensor_msgs::msg::Image::SharedPtr &msgLeft,
                              const sensor_msgs::msg::Image::SharedPtr &msgRight)
{
    // Copy the ROS image messages to cv::Mat.
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

    // Call ORB_SLAM2 and update local data.
    rclcpp::Time Ts = cv_ptrLeft->header.stamp;
    mpORBSLAM2Node->setPose(mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Ts.seconds()));
    mpORBSLAM2Node->setState(mpSLAM->GetTrackingState());
}
