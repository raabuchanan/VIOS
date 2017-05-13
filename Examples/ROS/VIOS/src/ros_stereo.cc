/** *************************************************************************************
 *
 *  Authors:        Russell Buchanan, russellb@student.ethz.ch
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
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

#define VISUALIZATION true 
#define USE_IMU true

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, ros::NodeHandle* pnh):mpSLAM(pSLAM), nh(pnh){


        pose = nh->advertise<geometry_msgs::TransformStamped>("/orb_slam_2_ros_node/transform_cam", 100);
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    ros::NodeHandle* nh;
    ros::Publisher pose;
    Eigen::Matrix4d T_C_B_, T_B_C_, T_S_V_;
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
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,USE_IMU,VISUALIZATION);

    ros::NodeHandle nh;

    ImageGrabber image_grb(&SLAM, &nh);
    ImuGrabber imu_grb(&SLAM);


    Eigen::Matrix4d T_C_B, T_B_C, T_V_S;
        

    // I calculated for euroc
     //   T_C_B << 0.01476960368055319, 0.9996668546037443, -0.02116692263358451, 0.06610086169028019,
     // -0.99986528036748, 0.01491752462460677, 0.006847523255539449, -0.013698980112073279,
     // 0.0071610001243047556, 0.02106293582886804, 0.9997525057790498, -0.0007667891785872767,
     // 0.0, 0.0, 0.0, 1.0;

     // Provided from euroc
     // T_C_B << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
     //   0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
     //  -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
     //   0.0, 0.0, 0.0, 1.0;


    // using for SERVO for now
    T_C_B << 1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;

    // using for SERVO for now
    T_V_S << 1,0,0,0,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;

    // V2 Vicon
   // T_V_S << 0.33638, -0.01749,  0.94156,  0.06901,
   //       -0.02078, -0.99972, -0.01114, -0.02781,
   //        0.94150, -0.01582, -0.33665, -0.12395,
   //            0.0,      0.0,      0.0,      1.0;

  // MH5 leica
   // T_V_S << 1.0, 0.0, 0.0,  7.48903e-02,
   //       0.0, 1.0, 0.0, -1.84772e-02,
   //       0.0, 0.0, 1.0, -1.20209e-01,
   //       0.0, 0.0, 0.0,  1.0;



     image_grb.T_C_B_ = T_C_B;
     image_grb.T_B_C_ = T_C_B.inverse();

    image_grb.T_S_V_ = T_V_S;



    stringstream ss(argv[3]);
	ss >> boolalpha >> image_grb.do_rectify;

    if(image_grb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
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
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,image_grb.M1l,image_grb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,image_grb.M1r,image_grb.M2r);
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

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.

    //cout << "Got Stereo Image" << endl;

    cv::Mat T_C_W_opencv;

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

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        T_C_W_opencv = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        T_C_W_opencv = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }


    Eigen::Matrix4d T_C_W, T_W_C;

    cv2eigen(T_C_W_opencv,T_C_W);

    T_W_C = T_S_V_*T_B_C_*T_C_W.inverse(); // Use this for euroc

    // Eigen::Quaterniond q(T_W_C.block<3,3>(0,0));

    geometry_msgs::TransformStamped msg;

    msg.header = msgLeft->header;
    msg.header.frame_id = "world";
    // Setting the child and parent frames
    msg.child_frame_id = "cam0";


    msg.transform.translation.x = T_W_C(0,3);
    msg.transform.translation.y = T_W_C(1,3);
    msg.transform.translation.z = T_W_C(2,3);

    // msg.transform.translation.x = T_W_C(2,3);
    // msg.transform.translation.y = -1*T_W_C(0,3);
    // msg.transform.translation.z = -1*T_W_C(1,3);

    msg.transform.rotation.x = 0;//q.x();
    msg.transform.rotation.y = 0;//q.y();
    msg.transform.rotation.z = 0;//q.z();
    msg.transform.rotation.w = 1;//q.w();

    pose.publish(msg);
    

    //cout << "T_C_W_opencv " << endl << T_C_W_opencv << endl;
    //cout << "T_W_C_opencv " << endl << T_C_W_opencv.inv() << endl;

    //cv2eigen(T_C_W_opencv,m);

    //yaw = 180*atan2(m(1,0),m(0,0))/3.14;
    //pitch = 180*atan2(-1*m(2,0),sqrt(m(2,1)*m(2,1) + m(2,2)*m(2,2)))/3.14;
    //roll = 180*atan2(m(2,1),m(2,2))/3.14;

    //cout << "ORB SLAM 2" << endl;
    //cout << "X: " << m(0,3) << " Y: " << m(1,3) << " Z: " << m(2,3) << endl;
    //cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << endl;
    //cout << endl;

}


void ImuGrabber::GrabImu(const sensor_msgs::Imu& imu)
{    
    //cout << "Got IMU data" << endl;

    struct ORB_SLAM2::IMUMeas newMeas;
    newMeas.ang_vel = Eigen::Vector3d::Zero(3);
    newMeas.lin_acc = Eigen::Vector3d::Zero(3);

    ORB_SLAM2::MotionModel* mpMotionModel = mpSLAM->GetMotionModeler();

    uint64_t seconds = imu.header.stamp.sec;
    uint64_t nseconds = imu.header.stamp.nsec;

    seconds = seconds%100000;

    newMeas.time_stamp = seconds*1000 + nseconds/1000000;
/*
    // Coordinate system switched to Orb Slam
    newMeas.ang_vel(0) = -1*imu.angular_velocity.x;
    newMeas.ang_vel(1) = -1*imu.angular_velocity.y;
    newMeas.ang_vel(2) = imu.angular_velocity.z;

    newMeas.lin_acc(0) = -1*imu.linear_acceleration.x;
    newMeas.lin_acc(1) = -1*imu.linear_acceleration.y;
    newMeas.lin_acc(2) = imu.linear_acceleration.z;
*/

/*
    // Coordinate system switched to Orb Slam in world frame
    newMeas.ang_vel(0) = -1*imu.angular_velocity.y;
    newMeas.ang_vel(1) = -1*imu.angular_velocity.z;
    newMeas.ang_vel(2) = imu.angular_velocity.x;

    newMeas.lin_acc(0) = -1*imu.linear_acceleration.y;
    newMeas.lin_acc(1) = -1*imu.linear_acceleration.z;
    newMeas.lin_acc(2) = imu.linear_acceleration.x;
*/

    // Coordinate system switched to Orb Slam in world frame
    newMeas.ang_vel(0) = imu.angular_velocity.x;
    newMeas.ang_vel(1) = imu.angular_velocity.y;
    newMeas.ang_vel(2) = imu.angular_velocity.z;

    newMeas.lin_acc(0) = imu.linear_acceleration.x;
    newMeas.lin_acc(1) = imu.linear_acceleration.y;
    newMeas.lin_acc(2) = imu.linear_acceleration.z;

    mpMotionModel->NewMeasurement(&newMeas);
    mpMotionModel->ProcessIMUMeas();
    
}

