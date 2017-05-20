/**
 **************************************************************************************
 *
 *  Authors:        Russell Buchanan, russellb@student.ethz.ch
 *
 *  File:           MotionModel.cc
 *
 *  Description:    Class member functions for Motion thread which integrates
 *potition
 *					IMU data
 *
 *
 ***************************************************************************************/

#include "MotionModel.h"

using namespace std;

namespace ORB_SLAM2
{
MotionModel::MotionModel(const string& strSettingPath)
{
	mInitCount = 0;

    mdTimeSum = 0;
    mdTime2Sum = 0;
    mLastTime = 0;

    mCurrentMeasurement.AngularVelocity = Eigen::Vector3d::Zero(3);
    mCurrentMeasurement.LinearAcceleration = Eigen::Vector3d::Zero(3);

    mCurrentMotion.dPosition = Eigen::Vector3d::Zero(3);
    mCurrentMotion.dVelocity = Eigen::Vector3d::Zero(3);

    mCurrentMotion.dR_W_B = Eigen::Matrix3d::Identity(3, 3);

    mAccelBias = Eigen::Vector3d::Zero(3);
    mGyroBias = Eigen::Vector3d::Zero(3);

    mGravity = Eigen::Vector3d::Zero(3);
    mGravity(2) = GRAVITATIONAL_ACCELERATION;
}

void MotionModel::IntegrateImuMeasurement(struct ImuMeasurement* NewMeasurement)
{
    if (mInitCount < 50)
    {
        mLastTime = NewMeasurement->TimeStamp;
        mAccelBias += NewMeasurement->LinearAcceleration + mGravity;
        mGyroBias += NewMeasurement->AngularVelocity;
        mInitCount++;

        if (mInitCount == 50)
        {
            mAccelBias = mAccelBias / 50;
            mGyroBias = mGyroBias / 50;
        }

        // cout << "Accel Bias" << mAccelBias << endl;
        // cout << "Vel Bias" << mGyroBias << endl;
    }
    else
    {
        float dt = (NewMeasurement->TimeStamp - mLastTime) / 1000.0;

        // cout << "NewMeasurement->TimeStamp " << NewMeasurement->TimeStamp << endl;
        // cout << "mLastTime " << mLastTime << endl;
        // cout << "dt " << dt << endl;

        if (abs(dt - 0.005) > 0.0001)
        {
            printf("dt: %f this time: %ld last time: %ld\n", dt,
                   NewMeasurement->TimeStamp, mLastTime);
        }

        if(mdTimeSum + dt > 0.15){
            cout << "Resetting motion model" << endl;
            ResetIntegration();
        }

        mLastTime = NewMeasurement->TimeStamp;

        Eigen::Matrix3d dR =
            ExpMap((NewMeasurement->AngularVelocity - mGyroBias) * dt);

        mCurrentMotion.dVelocity +=
            mCurrentMotion.dR_W_B *
            (NewMeasurement->LinearAcceleration - mAccelBias) * dt;

        mCurrentMotion.dPosition +=
            3 * mCurrentMotion.dR_W_B *
            (NewMeasurement->LinearAcceleration - mAccelBias) * dt * dt / 2;

        mCurrentMotion.dR_W_B = mCurrentMotion.dR_W_B * dR;

        mdTimeSum += dt;
        mdTime2Sum += dt * dt;
    }
}

void MotionModel::SetTracker(Tracking* pTracking) { mpTracker = pTracking; }

void MotionModel::NewMeasurement(struct ImuMeasurement* NewMeasurement)
{
    mCurrentMeasurement.TimeStamp = NewMeasurement->TimeStamp;

    mCurrentMeasurement.AngularVelocity = NewMeasurement->AngularVelocity;
    mCurrentMeasurement.LinearAcceleration = NewMeasurement->LinearAcceleration;

    //IntegrateImuMeasurement();
}

void MotionModel::GetMotionModel(cv::Mat& rotInc, cv::Mat& posInc,
                                 cv::Mat& velInc, float& timeInc,
                                 float& time2Inc)
{
    cv::eigen2cv(mCurrentMotion.dR_W_B, rotInc);
    cv::eigen2cv(mCurrentMotion.dPosition, posInc);
    cv::eigen2cv(mCurrentMotion.dVelocity, velInc);

    rotInc.convertTo(rotInc, CV_32F);
    posInc.convertTo(posInc, CV_32F);
    velInc.convertTo(velInc, CV_32F);

    timeInc = mdTimeSum;
    time2Inc = mdTime2Sum;

    ResetIntegration();
}

void MotionModel::ResetIntegration()
{
    mCurrentMotion.dPosition = Eigen::Vector3d::Zero(3);
    mCurrentMotion.dVelocity = Eigen::Vector3d::Zero(3);
    mCurrentMotion.dR_W_B = Eigen::Matrix3d::Identity(3, 3);
    mdTimeSum = 0;
    mdTime2Sum = 0;
}

}  // namespace ORB_SLAM