#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>
#include<chrono>

using std::placeholders::_1;
using std::placeholders::_2;
// using std::placeholders::_3;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify, const std::string* saving_file_directory)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    m_saving_file_directory = saving_file_directory;

    if (doRectify){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }
    // left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/robot_interface/front_camera/infra1/image_rect_raw");
    // right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/robot_interface/front_camera/infra2/image_rect_raw");
    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/ir_left");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(this, "/ir_right");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);

    int writter_flag = 0;

    // TODO: VO MODE NOT YET SUPPORTED, NEEDS REFACTORING ON THE BACKEND SIDE
    // cout << "Turning On Localizaiton Mode " << endl;
    // m_SLAM->ActivateLocalizationMode();
    // cout << "Localization Mode Activated " << endl;
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    // m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    cout << "Saving Trajectory" << endl;
    m_SLAM->SaveTrajectoryTUM(*m_saving_file_directory);
    cout << "Deleting classes" << endl;
    delete m_SLAM;
    m_SLAM = nullptr;

    // Clear/reset shared pointers
    if (syncApproximate) {
        syncApproximate.reset();
    }

    if (left_sub) {
        left_sub.reset();
    }

    if (right_sub) {
        right_sub.reset();
    }
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    auto start_time = std::chrono::high_resolution_clock::now(); 
    // Copy the ros rgb image message to cv::Mat.
    // cout << "Received stereo pair with timestamp: " << msgLeft->header.stamp.sec << "." << msgLeft->header.stamp.nanosec << endl;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        // cout << "Left image size: " << cv_ptrLeft->image.cols << "x" << cv_ptrLeft->image.rows << endl;
    }
    catch (cv_bridge::Exception& e)
    {
        // cout << "cv_bridge exception: " << e.what() << endl;
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
        // cout << "Right image size: " << cv_ptrRight->image.cols << "x" << cv_ptrRight->image.rows << endl;
    }
    catch (cv_bridge::Exception& e)
    {
        // cout << "cv_bridge exception: " << e.what() << endl;
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (doRectify){
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
    }
    else
    {
        // cout << "Not rectifying images" << endl;
        // if (writter_flag == 0 || writter_flag % 100 == 0)
        // {
        //     cout << "Processing stereo pair with timestamp: " << msgLeft->header.stamp.sec << "." << msgLeft->header.stamp.nanosec << endl;
        // }
        m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
        // writter_flag++;
        // cout << "Finished tracking stereo pair" << endl;
    }
    double total = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).count();
    processing_times_.push_back(total);
    if(processing_times_.size() > 10) processing_times_.pop_front();

    if(++frame_count_ % 10 == 0)
    {
        double avg_processing_time = std::accumulate(processing_times_.begin(), processing_times_.end(), 0.0) / processing_times_.size();
        RCLCPP_INFO(this->get_logger(), "Avg processing time (last 10 frames): %.3f ms", avg_processing_time * 1000.0);
    }
}
