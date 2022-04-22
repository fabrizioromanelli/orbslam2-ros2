/**
 * @brief ORB_SLAM2 for ROS 2 package executable code.
 *
 * @author Fabrizio Romanelli <fabrizio.romanelli@gmail.com>
 * @author Roberto Masocco <robmasocco@gmail.com>
 *
 * @date May 26, 2021
 */

#include <iostream>
#include <cstdio>
#include <thread>

#include "../include/orbslam2-ros2/orbslam2_ros2.hpp"

#ifdef SMT
#pragma message "Generating multithreaded VIO process"
#else
#pragma message "Generating single-threaded VIO process"
#endif

#ifdef EXTSAMPLER_LIN
#pragma message "Linear n-samples-filter extrasampler activated"
#endif

#ifdef EXTSAMPLER_QUAD
#pragma message "Quadratic fixed-sampling-time extrasampler activated"
#endif

#ifdef PX4
#pragma message "Activated publishers and subscribers for PX4 topics"
#endif

#ifdef BENCHMARK
#pragma message "Activated dummy publisher for VIO processing pipeline performance monitoring"
#endif

#ifdef TESTING
#pragma message "Activated testing features"
#endif

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

    // Parse input arguments (type of sensor, GUI on/off and camera pitch).
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
    default:
        break;
    }
    bool display;
    std::string dFlag(argv[4]);
    if (dFlag == "ON")
        display = true;
    else
        display = false;
    float camera_pitch = atof(argv[5]);

    // Create ORB_SLAM2 instance.
    ORB_SLAM2::System SLAM(argv[1], argv[2], sensorType, display);

    // Initialize ROS 2 connection and MT executor.
    rclcpp::init(argc, argv);
#ifdef SMT
    rclcpp::executors::MultiThreadedExecutor orbs2_mt_executor;
#else
    rclcpp::executors::SingleThreadedExecutor orbs2_st_executor;
#endif
    std::cout << "ROS 2 executor initialized" << std::endl;

    // Create ORBSLAM2Node.
    auto orbs2_node_ptr = std::make_shared<ORBSLAM2Node>(&SLAM,
                                                         sensorType,
                                                         camera_pitch);

#ifdef SMT
    orbs2_mt_executor.add_node(orbs2_node_ptr);
    orbs2_mt_executor.spin();
#else
    orbs2_st_executor.add_node(orbs2_node_ptr);
    orbs2_st_executor.spin();
#endif

    // Done!
    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
}
