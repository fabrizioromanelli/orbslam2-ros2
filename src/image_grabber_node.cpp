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
 * @param irDepth IR depth measurement availability.
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
}
