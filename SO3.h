//
//  SO3.h
//  
//
//  Created by Shin Fang on 26/1/18.
//

#ifndef SO3_h
#define SO3_h

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





#endif /* SO3_h */
