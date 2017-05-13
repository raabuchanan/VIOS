/** *************************************************************************************
 *
 *  Authors:        Russell Buchanan, russellb@student.ethz.ch
 *
 *  File:           Manifold.cc
 *
 *  Description:    
 *
 *
 ***************************************************************************************/

#include "Manifold.h"


using namespace std;

namespace ORB_SLAM2
{

/* Maps vector in R3 to skew symetric matrix in so3 eq1 in Forster*/
Eigen::Matrix3d Hat(const Eigen::Vector3d& vec)
{
	Eigen::Matrix3d m;

	m <<  	  0, -vec(2),  vec(1),
		 vec(2), 	  0,  -vec(0),
		-vec(1), vec(0),		0;

	return m;
}

/* Maps skew symetric matrix in so3 to vector in R3 */
Eigen::Vector3d Vee(const Eigen::Matrix3d& mat)
{
	Eigen::Vector3d v;

	if(!mat.isApprox(-1*mat.transpose()))
	{
		v = Eigen::Vector3d::Zero(3);
		printf("Matrix is not Skew Symetric\n");
		return v;
	}

	v << -mat(1,2), mat(0,2), mat(0,1);
}

Eigen::Matrix3d ExpMap(const Eigen::Vector3d& omegadt)
{

	double theta = omegadt.norm(); //l2 norm
	//printf("NORM: %f\n", theta);
	double w = cos(theta/2);
	double coeff;

	if(theta < 1.5e-5)
	{
		//printf("[Manifold] WARNING: Small change in angle detected! \n");
		coeff = 0.5 - theta*theta/48; // Taylor Expansion
	}
	else
	{
		coeff = sin(theta/2)/theta; //Expmap
	}

	Eigen::Quaterniond quat(w,coeff*omegadt.x(),coeff*omegadt.y(),coeff*omegadt.z());

	quat.normalize();

	return quat.toRotationMatrix();

}

Eigen::Matrix3d RightJac(const Eigen::Vector3d& phi)
{
	Eigen::Matrix3d Jr = Eigen::Matrix3d::Identity();
	double nphi = phi.norm();
	Eigen::Matrix3d hphi = Hat(phi);

	if(nphi < 1.5e-5)
	{
		return Jr;
	}
	else
	{
		Jr = Eigen::Matrix3d::Identity() - (1-cos(nphi))/(nphi*nphi)*hphi
		   + (nphi - sin(nphi))/(nphi*nphi*nphi)*hphi*hphi;
	}

	return Jr;

}



}

