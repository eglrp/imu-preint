//
//  SO3_lib.h
//  
//
//  Created by Shin Fang on 28/1/18.
//

#ifndef SO3_lib_h
#define SO3_lib_h

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

Matrix3d skew(const Vector3d&v){
	Matrix3d m;
	m.fill(0.);
	m(0,1) = -v(2);
	m(0,2) = v(1);
	m(1,2) = -v(0);
	m(1,0) = v(2);
	m(2,0) = -v(1);
	m(2,1) = v(0);
	return m;
}
/*
Vector3d QuaternionToAngleAxis(Eigen::Quaterniond&p){
	Vector3d Angle_Axis;

	const double q1 = p.x();
	const double q2 = p.y();
	const double q3 = p.z();
	const double sin_squared = q1 * q1 + q2 * q2 + q3 * q3;

	if (sin_squared>0.0){
		const double sin_theta = sqrt(sin_squared);
		const double k = 2.0 * atan2(sin_theta,p.w())/sin_theta;
		Angle_Axis[0] = q1 * k;
		Angle_Axis[1] = q2 * k;
		Angle_Axis[2] = q3 * k;


	}


	return Angle_Axis;
}*/


#endif /* SO3_lib_h */
