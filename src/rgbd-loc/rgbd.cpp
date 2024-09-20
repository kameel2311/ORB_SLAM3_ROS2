#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"

#include "System.h"

// Global shutdown flag
std::atomic_bool g_shutdown_requested(false);

// Signal handler for SIGTERM and SIGINT
void signalHandler(int signum)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received signal: %d", signum);
    g_shutdown_requested = true;
    cout << "SIGNAL Recieved ... Shutting down..." << endl;
    rclcpp::shutdown();  // Initiates the ROS2 shutdown process
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    if(argc < 5)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings output_saving_file_path visualization_bool" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    // bool visualization = true;
    bool visualization;
    if (std::string(argv[4]) == "true")
    {
        std::cout << "Visualization is ON" << std::endl;
        visualization = true;
    }
    else
    {
        std::cout << "Visualization is OFF" << std::endl;
        visualization = false;
    }
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, visualization);

    // Pose Saving Directory
    const std::string saving_file_directory = argv[3];

    auto node = std::make_shared<RgbdSlamNode>(&SLAM, &saving_file_directory);
    std::cout << "============================ " << std::endl;

    // rclcpp::spin(node);
    // Spin the node, will exit when rclcpp::shutdown() is called
    while (rclcpp::ok() && !g_shutdown_requested)
    {
        rclcpp::spin_some(node);
        // Sleep or do other work
    }
    rclcpp::shutdown();

    return 0;
}
