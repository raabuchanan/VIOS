/** *************************************************************************************
 *
 *  Authors:        Russell Buchanan, russellb@student.ethz.ch
 *
 *  File:           MotionModel.cc
 *
 *  Description:    Class member functions for Motion thread which integrates potition
 *					IMU data
 *
 *
 ***************************************************************************************/

#include "MotionModel.h"

using namespace std;

namespace ORB_SLAM2
{
	MotionModel::MotionModel(const string &strSettingPath)
	{
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

		// rate_ 				= fSettings["IMU.rate"];
		// gyro_noise_density_ = fSettings["IMU.gyroscope_noise_density"];
		// gyro_rand_walk_ 	= fSettings["IMU.gyroscope_random_walk"];
		// acc_noise_density_ 	= fSettings["IMU.accelerometer_noise_density"];
		// acc_rand_walk_ 		= fSettings["IMU.accelerometer_random_walk"];

		// cout << "- rate_: " 				<< rate_ 				<< endl;
		// cout << "- gyro_noise_density_: " 	<< gyro_noise_density_ 	<< endl;
		// cout << "- gyro_rand_walk_: " 		<< gyro_rand_walk_ 		<< endl;
		// cout << "- acc_noise_density_: " 	<< acc_noise_density_ 	<< endl;
		// cout << "- acc_rand_walk_: " 		<< acc_rand_walk_ 		<< endl;

	    dt_sum_ = 0;
	    dt2_sum_ = 0;
	    last_time_ = 0;

	    // TODO Figure out why this crashes if after the Eigen allocations below
	    //pub_ = n_.advertise<geometry_msgs::PoseStamped>("/pose", 1);

	    ms_.ang_vel = Eigen::Vector3d::Zero(3);
	    ms_.lin_acc = Eigen::Vector3d::Zero(3);

	    ds_.pos = Eigen::Vector3d::Zero(3);
	    ds_.vel = Eigen::Vector3d::Zero(3);


	    ds_.R_WB = Eigen::Matrix3d::Identity(3,3);

	    eta_ad_ = Eigen::Vector3d::Zero(3);
	    eta_gd_ = Eigen::Vector3d::Zero(3);

	    b_a_ = Eigen::Vector3d::Zero(3);
	    b_g_ = Eigen::Vector3d::Zero(3);

    	// Eigen::Matrix3d initialPose;

    	// initialPose << 1.0000, 0, 0,
					//    0, 0.9397, -0.3420,
					//    0, 0.3420, 0.9397;

	    g_ = Eigen::Vector3d::Zero(3);
	    g_(2) = -9.80740;

	    
	    // g_ = initialPose*g; // needed for imu0

	    new_meas_ = false;

	    init_count = 0;
	}


	void MotionModel::Run()
	{
		int count = 0;
		while(1)
		{
			if(new_meas_)
			{
				//cout << "processing" <<endl;
				ProcessIMUMeas();
				new_meas_ = false;
			}

			//usleep(10); //1ms
/*
			if (count > 100)
			{
				printPose();
				count = 0;
			}

			count++;
*/
		}

	}

	void MotionModel::ProcessIMUMeas()
	{
		if(init_count < 50)
		{
			last_time_ = ms_.time_stamp;
			b_a_ += ms_.lin_acc + g_;
			b_g_ += ms_.ang_vel;
			init_count++;

			if(init_count == 50)
			{
				b_a_ = b_a_/50;
				b_g_ = b_g_/50;
			}

			//cout << "Accel Bias" << b_a_ << endl;
			//cout << "Vel Bias" << b_g_ << endl;

		}
		else
		{

			float dt = (ms_.time_stamp - last_time_)/1000.0;
			

			if(abs(dt-0.005)>0.0001){
				printf("dt: %f this time: %ld last time: %ld\n", dt, ms_.time_stamp, last_time_);
			}

			last_time_ = ms_.time_stamp;
			

			// cout << "ms_.lin_acc - b_a_ = " << endl << ms_.lin_acc - b_a_ << endl;
			// cout << "ms_.ang_vel - b_a_ = " << endl << ms_.ang_vel - b_g_ << endl;
			
			Eigen::Vector3d omegadt = (ms_.ang_vel - b_g_ - eta_gd_)*dt;
			Eigen::Matrix3d dR = ExpMap(omegadt);

			ds_.vel += ds_.R_WB*(ms_.lin_acc - b_a_ - eta_ad_)*dt;

			ds_.pos += 3*ds_.R_WB*(ms_.lin_acc - b_a_ - eta_ad_)*dt*dt/2;

			ds_.R_WB = ds_.R_WB*dR;

			dt_sum_ += dt;
			dt2_sum_ += dt*dt;

			// cout << "dt " << dt << endl;
			// cout << "dR " << endl << ds_.R_WB << endl;
			// cout << "dpos " << endl << ds_.pos << endl;

			// if(dt_sum_ > 0.2)
			// 	ResetIntegration();

		}

	}

	void MotionModel::setTracker(Tracking *pTracking)
	{
	    mpTracker_=pTracking;
	}

	void MotionModel::NewMeasurement(struct IMUMeas* new_meas)
	{

		ms_.time_stamp = new_meas->time_stamp;

		ms_.ang_vel = new_meas->ang_vel;
		ms_.lin_acc = new_meas->lin_acc;

		new_meas_ = true;
	}

    void MotionModel::GetMotionModel(cv::Mat& rot, cv::Mat& pos, cv::Mat& vel, float& dt_total, float& dt2_total)
	{

		cv::eigen2cv(ds_.R_WB,rot);
		cv::eigen2cv(ds_.pos,pos);
		cv::eigen2cv(ds_.vel,vel);

		rot.convertTo(rot,CV_32F);
		pos.convertTo(pos, CV_32F);
		vel.convertTo(vel, CV_32F);

		dt_total = dt_sum_;
		dt2_total = dt2_sum_;

		ResetIntegration();


	}

	void MotionModel::ResetIntegration()
	{
	    ds_.pos = Eigen::Vector3d::Zero(3);
	    ds_.vel = Eigen::Vector3d::Zero(3);
	    ds_.R_WB = Eigen::Matrix3d::Identity(3,3);
	    dt_sum_ = 0;
	    dt2_sum_ = 0;
	}


} //namespace ORB_SLAM