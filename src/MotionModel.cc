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

    mGravity_B = Eigen::Vector3d::Zero(3);
    mGravity_B(2) = GRAVITATIONAL_ACCELERATION;

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);


    cv::Mat T_C_I, T_I_C, T_C_B, T_I_B;
    fSettings["T_CAM0_IMU"] >> T_C_I;
    fSettings["T_CAM0_BODY"] >> T_C_B;
    fSettings["T_IMU_BODY"] >> T_I_B;

    T_C_I.convertTo(T_C_I, CV_32F);
    T_C_B.convertTo(T_C_B, CV_32F);
    T_I_B.convertTo(T_I_B, CV_32F);


    if(T_I_B.empty()){
        T_I_B = T_C_I.inv() * T_C_B;
    }
    
    cv::cv2eigen(T_I_B.rowRange(0,3).colRange(0,3), mR_I_B);

}

void MotionModel::IntegrateImuMeasurement(struct ImuMeasurement* NewMeasurement)
{
    if (mInitCount < 50)
    {
        mLastTime = NewMeasurement->TimeStamp;
        mAccelBias += NewMeasurement->LinearAcceleration + mR_I_B * mGravity_B;
        mGyroBias += NewMeasurement->AngularVelocity;
        mInitCount++;

        if (mInitCount == 50)
        {
            mAccelBias = mAccelBias / 50;
            mGyroBias = mGyroBias / 50;
        }

        #ifdef DEBUG
        cout << "NewMeasurement->LinearAcceleration" << endl << NewMeasurement->LinearAcceleration << endl;
        cout << "NewMeasurement->AngularVelocity" << endl << NewMeasurement->AngularVelocity << endl;
        cout << "mR_I_C * mR_C_B * mGravity_B" << endl << mR_I_B * mGravity_B << endl;
        cout << "mInitCount " << mInitCount << endl;
        cout << "mAccelBias" << endl << mAccelBias << endl;
        cout << "mGyroBias" << endl << mGyroBias << endl;
        #endif


    }
    else
    {
        float dt = (NewMeasurement->TimeStamp - mLastTime) / 1000.0;

        // cout << "NewMeasurement->TimeStamp " << NewMeasurement->TimeStamp << endl;
        // cout << "mLastTime " << mLastTime << endl;
        // cout << "dt " << dt << endl;
        #ifdef DEBUG
        if (abs(dt - 0.005) > 0.0001)
        {
            printf("dt: %f this time: %ld last time: %ld\n", dt,
                   NewMeasurement->TimeStamp, mLastTime);
        }
        #endif

        if(mdTimeSum + dt > 0.1){
            #ifdef DEBUG
            cout << "Too long since last motion model request, resetting motion model" << endl;
            #endif
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
    mCurrentMotion.dR_W_B = Eigen::Matrix3d::Identity(3,3);
    mdTimeSum = 0;
    mdTime2Sum = 0;
}

}  // namespace ORB_SLAM