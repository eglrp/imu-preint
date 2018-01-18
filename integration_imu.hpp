//
//  integration_imu.hpp
//  
//
//  Created by Shin Fang on 29/11/17.
//

#ifndef integration_imu_hpp
#define integration_imu_hpp

#include <istream>
#include <map>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "eigen3/unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h"



namespace ceres{
namespace examples{

struct Pose3d{

	Eigen::Vector3d v_x;
	Eigen::Vector3d v_y;
	Eigen::Quaterniond q;

	static std::string name(){
		return "VEC_R3:QUAT";
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

std::istream& operator>>(std::istream& input, Pose3d& pose){
	input >> pose.v_x.x() >> pose.v_x.y() >>pose.v_x.z() >> pose.v_y.x() >> pose.v_y.y()>> pose.v_y.z()>> pose.q.w() >> pose.q.x() >> pose.q.y() >> pose.q.z();

	return input;
}

typedef std::map<int,Pose3d, std::less<int>,
				Eigen::aligned_allocator<std::pair<const int, Pose3d> > > MapOfPoses;




struct Constraint3d{

	Eigen::Vector3d o;


	static::std::string name(){
		return "EDGE_SO3:VECTOR";
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



std::istream& operator>>(std::istream& input, Constraint3d& constraint){
	input>>constraint.o.x() >>constraint.o.y()>>constraint.o.z();


	return input;
}

typedef std::vector<Constraint3d,Eigen::aligned_allocator<Constraint3d> >VectorOfConstraints;




}//examples
}//ceres
#endif /* integration_imu_hpp */
