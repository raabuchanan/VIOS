

#ifndef MANIFOLD_H
#define MANIFOLD_H

#include <iostream>
#include <Eigen/Dense>


namespace ORB_SLAM2
{
Eigen::Matrix3d ExpMap(const Eigen::Vector3d& omegadt);

Eigen::Matrix3d Hat(const Eigen::Vector3d& vec);

Eigen::Vector3d Vee(const Eigen::Matrix3d& mat);
}


#endif