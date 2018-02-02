//
//  preintegration_func.hpp
//  
//
//  Created by Shin Fang on 27/1/18.
//

#ifndef preintegration_func_hpp
#define preintegration_func_hpp

#include <iostream>
#include <vector>
#include "Eigen/Core"
#include "integration_imu.hpp"


#ifndef BASIC_MATRIX
#define BASIC_MATRIX
Eigen::Matrix3d I3x3 = Eigen::Matrix<double,3,3>::Identity();
Eigen::Matrix3d Z3x3 = Eigen::Matrix<double,3,3>::Zero();

#endif //BASIC MATRIX

class preintegration_func{
		//can be assessed by outside class
	public:

		preintegration_func();

		~preintegration_func()
		{
			std::cout<<"IMU Preintegration has been destructed"<<std::endl;
		}

		//to be assessed in int main()
		Eigen::Vector3d bias_g;
		std::map<int,Eigen::Quaterniond> Preintegrated_omega;
		Eigen::Quaternion<double> preint_measurement;

		std::map<int,Eigen::Quaterniond> Preintegration(const VectorOfConstraints& constraint, MapOfPoses* poses);

		//reset the imupreintegration to the original state for every k samples
		void Reset();

		void BuildOptimizationProblem(std::map<int,Eigen::Quaterniond>& preint, MapOfPoses* poses);

		void bias_g_write (Eigen::Vector3d new_bias_g);
		//{bias_g = new_bias_g;}

		void preintegrated_measurement_write (std::map<int,Eigen::Quaterniond> new_preint_measurements) {Preintegrated_omega = new_preint_measurements;}

		Eigen::Vector3d bias_g_read() { return bias_g;}

		Eigen::Matrix3d Dexp(Eigen::Vector3d theta);



	private:

		//Eigen::Vector3d bias_g;
		Eigen::Matrix3d delta_Rij;
		//std::map<int,Eigen::Quaterniond> Preintegrated_omega;
		int count;
		int index;
		int bias_index;

		struct Derivative{
			//bias of gyroscope
			Eigen::Matrix3d DRij_Dbg;

		}df_dx;
};


#endif /* preintegration_func_hpp */
