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
#include "Manifold.h"
#include <Eigen/Dense>

#include "Tracking.h"

// Gravity in Zurich, Switzerland
// https://www.metas.ch/metas/en/home/dok/gravitationszonen.html
#define GRAVITATIONAL_ACCELERATION -9.80740

using namespace std;

namespace ORB_SLAM2
{

class Tracking;

// Raw IMU measurements in Body frame
struct ImuMeasurement
{
    uint64_t TimeStamp;  // in nsec
    Eigen::Vector3d AngularVelocity;
    Eigen::Vector3d LinearAcceleration;
};

// In world frame
struct MotionIncrement
{
    Eigen::Matrix3d dR_W_B;
    Eigen::Vector3d dPosition;
    Eigen::Vector3d dVelocity;
};

class MotionModel
{
public:
	MotionModel(const string &strSettingPath);
	void IntegrateImuMeasurement(struct ImuMeasurement* NewMeasurement);
	void SetTracker(Tracking* pTracking);
	void NewMeasurement(struct ImuMeasurement* NewMeasurement);
	void GetMotionModel(cv::Mat& dR, cv::Mat& dPos, cv::Mat& dVel, float& dTimeSum, float& dTime2Sum);

private:
	void ResetIntegration();

	struct ImuMeasurement mCurrentMeasurement; //measurement state
	struct MotionIncrement mCurrentMotion; //data state
	Tracking* mpTracker;
	double mdTimeSum;
	double mdTime2Sum;
	Eigen::Vector3d mAccelBias;
	Eigen::Vector3d mGyroBias;
	Eigen::Vector3d mGravity_B;
	uint64_t mLastTime;
	int mInitCount;

	Eigen::Matrix3d mR_I_C, mR_C_B;
};

} //namespace ORB_SLAM

#endif