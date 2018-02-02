//
//  preintegration_func.cpp
//  
//
//  Created by Shin Fang on 27/1/18.
//


#include "Eigen/Core"
#include <iostream>
#include <vector>
#include <map>

#include "preintegration_func.hpp"
#include "SO3_lib.h"

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "pose_error.h"

//Initialization
preintegration_func::preintegration_func()
{
	bias_g << 0.03,0.05,0.07;
	count = 0;
	index = 1;
	bias_index = 0;

	//ini attitude
	delta_Rij = I3x3;

};

void preintegration_func::Reset(){

	// reset after every emerge of R samples
	delta_Rij = I3x3;
}

void preintegration_func::bias_g_write (Eigen::Vector3d new_bias_g){
	bias_g = new_bias_g;
}



//function to preintegrate imu measurement
//preintegrated imu measurement this time, included bias correction as well
//this is the reason why the value is no longer 4.161468e-01 0 -9.092974e-01 0
std::map<int,Eigen::Quaterniond> preintegration_func::Preintegration(const VectorOfConstraints& a, MapOfPoses* map)
{
	std::map<int,Eigen::Quaterniond>::iterator it = Preintegrated_omega.begin();
	int bias_pointer = 0;
for (VectorOfConstraints::const_iterator constraints_iter = a.begin();constraints_iter!=a.end();++constraints_iter)
	{

			count = count +1;

			const Constraint3d& gyro = *constraints_iter;
			MapOfPoses::iterator current_bias = map->find(bias_pointer);
			//std::cout<<"bias_g in preintegrated measurement function"<<bias_g<<std::endl;
			//std::cout<<bias_pointer<<": Current Bias In Preintegrated "<<current_bias->second.b<<std::endl;

			Eigen::Vector3d gyroCorrect = gyro.o - current_bias->second.b;

			//Eigen::Vector3d gyroCorrect = gyro.o;
			//std::cout<<"index"<<index-1<<"bias in preintegrated measurement"<<current_bias->second.b<<std::endl;
			//std::cout<<"bias g"<<bias_g<<std::endl;

			Eigen::Matrix3d Rk_k_1 = skew(gyroCorrect);
			delta_Rij = delta_Rij * Rk_k_1.exp();


			if (count%4==0)
			{
				Eigen::Quaterniond omega_q(delta_Rij);
				omega_q.normalize();
				Preintegrated_omega[index] = omega_q;
				/*std::cout<<omega_q.w()<<std::endl;
				std::cout<<omega_q.x()<<std::endl;
				std::cout<<omega_q.y()<<std::endl;
				std::cout<<omega_q.z()<<std::endl;*/
				index = index + 1;
				bias_pointer = bias_pointer + 1;
				Reset();					}

		} // for loop

		//count = 0;
		index = 1;
		bias_pointer = 0;
		//bias_index = 0 ;
		return Preintegrated_omega;

}

//This function has been migrated to the main
void preintegration_func::BuildOptimizationProblem(std::map<int,Eigen::Quaterniond>& temp_map, MapOfPoses* poses)
{
		ceres::Problem problem;
		ceres::Solver::Options solver_options;
		ceres::Solver::Summary summary;
		solver_options.minimizer_progress_to_stdout = true;
		solver_options.max_num_consecutive_invalid_steps = 1000;
		solver_options.max_num_iterations = 1;


		ceres::LossFunction* loss_function = NULL;
		ceres::LocalParameterization *quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

		int index = 0;

		for (std::map<int,Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Pose3d> > >::const_iterator attitude_iterator= poses->begin();attitude_iterator!=poses->end();++attitude_iterator)
		{

			const std::map<int,Pose3d, std::less<int>,Eigen::aligned_allocator<std::pair<const int,Pose3d> > >::value_type pair = *attitude_iterator;
			MapOfPoses::iterator current_pose_to_optimize = poses->find(index);

			Eigen::Quaternion<double> vector_b_x;
			Eigen::Quaternion<double> vector_b_y;
			Eigen::Quaternion<double> Constraint_imu;
			Eigen::Vector3d bias_vec ;


			bias_vec<<0.1,0.1,0.1;


			vector_b_x = Eigen::Quaternion<double>::Quaternion(0,pair.second.v_x.x(),pair.second.v_x.y(),pair.second.v_x.z());
			vector_b_y = Eigen::Quaternion<double>::Quaternion(0,pair.second.v_y.x(),pair.second.v_y.y(),pair.second.v_y.z());
			//bias_vec <<pair.second.b.x(),pair.second.b.y(),pair.second.b.z();


			ceres::CostFunction* cost_function_x = ceres::examples::AttitudeError_x::Create(vector_b_x);
			//ceres::CostFunction* cost_function_y = AttitudeError_y::Create(vector_b_y);
			//ceres::CostFunction* cost_function_bias = ceres::examples::Bias::Create(bias_g);


			problem.AddResidualBlock(cost_function_x,loss_function,current_pose_to_optimize->second.q.coeffs().data());
			problem.SetParameterization(current_pose_to_optimize->second.q.coeffs().data(),quaternion_local_parameterization);
			//problem.AddResidualBlock(cost_function_y,loss_function,current_pose_to_optimize->second.q.coeffs().data());
			//problem.SetParameterization(current_pose_to_optimize->second.q.coeffs().data(),quaternion_local_parameterization);

			//problem.AddResidualBlock(cost_function_bias, loss_function, current_pose_to_optimize->second.b.data());

			if (index>0){

			std::map<int,Eigen::Quaterniond>::iterator preint_constraint = temp_map.find(index);

			Constraint_imu = Eigen::Quaternion<double>::Quaternion(preint_constraint->second.w(),preint_constraint->second.x(),preint_constraint->second.y(),preint_constraint->second.z());

			std::cout<<"constraint"<<Constraint_imu.w()<<std::endl;
			std::cout<<"constraint"<<Constraint_imu.x()<<std::endl;
			std::cout<<"constraint"<<Constraint_imu.y()<<std::endl;
			std::cout<<"constraint"<<Constraint_imu.z()<<std::endl;
			MapOfPoses::iterator prev_pose_to_optimize = poses->find(index-1);



			//ceres::CostFunction* constraint_cost_function = IMUFunctor::Create(Constraint_imu);
			//problem.AddResidualBlock(constraint_cost_function, loss_function, prev_pose_to_optimize->second.q.coeffs().data(),current_pose_to_optimize->second.q.coeffs().data());
			//problem.SetParameterization(current_pose_to_optimize->second.q.coeffs().data(),quaternion_local_parameterization);
			//problem.SetParameterization(prev_pose_to_optimize->second.q.coeffs().data(),quaternion_local_parameterization);
			}
			index = index +1;

			/*std::cout<<"value of bias prior to optimization"<<current_pose_to_optimize->second.b<<std::endl;
			ceres::Solve(solver_options,&problem,&summary);
			std::cout<<"value of bias"<<current_pose_to_optimize->second.b<<std::endl;
			Eigen::Vector3d new_bias;
			new_bias = current_pose_to_optimize->second.b;
			std::cout<<"new bias"<<new_bias<<std::endl;

			*/

			//bias_g_write(new_bias);

			//std::cout<<current_pose_to_optimize->second.b<<std::endl;



		}


}

Eigen::Matrix3d preintegration_func::Dexp(Eigen::Vector3d theta)
{
	double phi = theta.norm();
	double phi_inv = 1/phi;

	Eigen::Matrix3d phix,Jr;
	phix = skew(theta);


	if (phi<0.00001){
		return Jr;
	}

	else{
		Jr = I3x3 - (1-cos(phi))*phi_inv*phi_inv*phix+ (phi-sin(phi))*phi_inv*phi_inv*phi_inv*phix*phix;
	}
	return Jr;

}




