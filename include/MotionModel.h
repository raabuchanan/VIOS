/** *************************************************************************************
 *
 *  Authors:        Russell Buchanan, russellb@student.ethz.ch
 *
 *  File:           MotionModel.h
 *
 *  Description:    Class definition for Motion Modeler
 *
 *
 ***************************************************************************************/
#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <stdint.h>
#include <iostream>
#include <mutex>
#include <unistd.h>
#include <math.h> 
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include "Tracking.h"
#include "Manifold.h"



using namespace std;

namespace ORB_SLAM2
{

class Tracking;

// Raw IMU measurement in Body frame
struct IMUMeas
{
	uint64_t time_stamp; // in nsec
	Eigen::Vector3d ang_vel;
	Eigen::Vector3d lin_acc;
};

// In world frame
struct IMUData
{
	Eigen::Matrix3d R_WB;
	Eigen::Vector3d pos;
	Eigen::Vector3d vel;
};

class MotionModel
{
public:
	MotionModel(const string &strSettingPath);
	void ProcessIMUMeas();
	void setTracker(Tracking* pTracking_);
	void Run();
	void NewMeasurement(struct IMUMeas* new_meas);
	void GetMotionModel(cv::Mat& rot, cv::Mat& pos, cv::Mat& vel, float& dt_total, float& dt2_total);

private:
	double gyro_noise_density_; 		// [ rad / s / sqrt(Hz) ]   ( gyro "white noise" )
	double gyro_rand_walk_;  		// [ rad / s^2 / sqrt(Hz) ] ( gyro bias diffusion )
	double acc_noise_density_;		// [ m / s^2 / sqrt(Hz) ]   ( accel "white noise" )
	double acc_rand_walk_;   		// [ m / s^3 / sqrt(Hz) ].  ( accel bias diffusion )
	struct IMUMeas ms_; //measurement state
	struct IMUData ds_; //data state
	Tracking* mpTracker_;
	bool new_meas_;
	double dt_sum_;
	double dt2_sum_;
	Eigen::Vector3d eta_gd_;
	Eigen::Vector3d eta_ad_;
	Eigen::Vector3d b_g_;
	Eigen::Vector3d b_a_;
	Eigen::Vector3d g;
	void sendToROS();
	void printPose();
	uint64_t last_time_;
	int init_count;
	void ResetIntegration();
	Eigen::Vector3d g_;

};

} //namespace ORB_SLAM

#endif