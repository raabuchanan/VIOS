/***************************************************************************************
 *
 *  Author:        Russell Buchanan, russellb@student.ethz.ch
 *
 *  File:           ros_stereo.cc
 *
 *  Description:    Test funciton for visual-inertial pipeline
 *
 *
 ***************************************************************************************/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <stdint.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

#define DO_RECTIFY true
#define USE_BODY_FRAME true

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle* nh):mpSLAM(pSLAM), pnh(nh){


        mPosePub = pnh->advertise<geometry_msgs::TransformStamped>("/orb_slam_2_ros_node/transform_cam", 10);
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    cv::Mat M1l,M2l,M1r,M2r;
    ros::NodeHandle* pnh;
    ros::Publisher mPosePub;
    cv::Mat mT_C_B, mT_B_C, mT_B_V;
};

class ImuGrabber
{
public:
    ImuGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImu(const sensor_msgs::Imu& imu);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc != 4)
    {
        cerr << "Usage: rosrun VIOS Stereo path_to_vocabulary path_to_settings visualization" << endl;
        cerr << "Example: rosrun VIOS Stereo Vocabulary/ORBvoc.txt Settings/EuRoC.yaml true " << endl;
        ros::shutdown();
        return 1;
    }

    bool visualization;
    istringstream(argv[3]) >> std::boolalpha >> visualization;    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,visualization);

    ros::NodeHandle nh;

    ImageGrabber image_grb(&SLAM, &nh);
    ImuGrabber imu_grb(&SLAM);

    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    if(DO_RECTIFY)
    {
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r, T_R_L, Q_;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Distortion parameters for stereo rectification are missing!"
                 << endl;
            return -1;
        }

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["T_RIGHT_LEFT"] >> T_R_L;


        if (P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty())
        {
            if(!T_R_L.empty()){
                cerr << "WARNING: Rectification matrices are missing. Calculating now."
                     << endl;
                cv::stereoRectify(K_l, D_l, K_r, D_r, cv::Size(cols_l, rows_l),
                                T_R_L.rowRange(0, 3).colRange(0, 3),
                                T_R_L.col(3).rowRange(0, 3), R_l, R_r, P_l, P_r, Q_);
            }
            cerr << "ERROR: Stereo extrinsic transform missing!"
                 << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,image_grb.M1l,image_grb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,image_grb.M1r,image_grb.M2r);
    }


    if(USE_BODY_FRAME)
    {
        cv::Mat T_C_B, T_B_V;

        fsSettings["T_LEFT_IMU"] >> T_C_B;
        fsSettings["T_IMU_VICON"] >> T_B_V;

        if (T_C_B.empty() || T_B_V.empty())
        {
            cerr << "ERROR: Inertial extrinsic transform missing!" << endl;
            return -1;
        }

        image_grb.mT_C_B = T_C_B;
        image_grb.mT_B_C = T_C_B.inv();
        image_grb.mT_B_V = T_B_V;


        image_grb.mT_C_B.convertTo(image_grb.mT_C_B, CV_32F);
        image_grb.mT_B_C.convertTo(image_grb.mT_B_C, CV_32F);
        image_grb.mT_B_V.convertTo(image_grb.mT_B_V, CV_32F);
    }

    ros::Subscriber imu_sub = nh.subscribe("/imu0",10,&ImuGrabber::GrabImu,&imu_grb);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/cam0/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/cam1/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(5), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&image_grb,_1,_2));
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv::Mat T_W_C;

    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(DO_RECTIFY)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        T_W_C = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        T_W_C = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    if(!T_W_C.empty())
    {
        cv::Mat T_W_V;
        geometry_msgs::TransformStamped msg;

        if (USE_BODY_FRAME)
        {
            T_W_V = mT_B_V * mT_B_C * T_W_C.inv();
        }
        else
        {
            T_W_V = T_W_C.inv();
        }

        msg.header = msgLeft->header;
        msg.header.frame_id = "world";
        msg.child_frame_id = "cam0";

        msg.transform.translation.x = T_W_V.at<float>(0,3);
        msg.transform.translation.y = T_W_V.at<float>(1,3);
        msg.transform.translation.z = T_W_V.at<float>(2,3);

        // TODO Quaternion rotations
        msg.transform.rotation.x = 0;
        msg.transform.rotation.y = 0;
        msg.transform.rotation.z = 0;
        msg.transform.rotation.w = 1;

        mPosePub.publish(msg);
    }
}


void ImuGrabber::GrabImu(const sensor_msgs::Imu& imu)
{    
    struct ORB_SLAM2::ImuMeasurement NewMeasurement;

    uint64_t seconds = imu.header.stamp.sec;
    uint64_t nseconds = imu.header.stamp.nsec;

    seconds = seconds%100000;

    NewMeasurement.TimeStamp = seconds*1000 + nseconds/1000000;

    // Coordinate system switched to Orb Slam in world frame
    NewMeasurement.AngularVelocity(0) = imu.angular_velocity.x;
    NewMeasurement.AngularVelocity(1) = imu.angular_velocity.y;
    NewMeasurement.AngularVelocity(2) = imu.angular_velocity.z;

    NewMeasurement.LinearAcceleration(0) = imu.linear_acceleration.x;
    NewMeasurement.LinearAcceleration(1) = imu.linear_acceleration.y;
    NewMeasurement.LinearAcceleration(2) = imu.linear_acceleration.z;


    ORB_SLAM2::MotionModel* mpMotionModel = mpSLAM->GetMotionModeler();
    mpMotionModel->IntegrateImuMeasurement(&NewMeasurement);
}

