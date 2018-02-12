 //
//  integration_imu.cpp
//
//
//  Created by Shin Fang on 29/11/17.
//5:35PM TUESDAY
//

#include <iostream>
#include <fstream>
#include <string>

#include <map>
#include <vector>
#include "Eigen/Core"
#include <Eigen/Dense>
#include <Eigen/Geometry>



#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "read_file.h"
#include "glog/logging.h"
#include "pose_error.h"
#include "integration_imu.hpp"
#include "preintegration_func.hpp"
#include "preintegration_func.cpp"

DEFINE_string(input,"/Users/shinfang/Documents/MATLAB/preintegration_theory/synthetic_data.txt","The file is similar to g2o format");
//preintegration_func IMU_Integration;
namespace ceres{

	namespace examples{

	bool OutputPoses(const std::string& filename, const MapOfPoses* poses){
	    				std::fstream outfile;
	    				outfile.open(filename.c_str(), std::istream::out);

	    for (std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::const_iterator poses_iter= poses->begin();poses_iter!=poses->end();++poses_iter){

	    		const std::map<int,Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::value_type& pair = *poses_iter;


	    		outfile << pair.second.q.w()<<" "<<pair.second.q.x()<<" "<<pair.second.q.y()<<" "<<pair.second.q.z()<<" "<<pair.second.b.x()<<" "<<pair.second.b.y()<<" "<<pair.second.b.z()<<" "<<std::endl;


	    		}
	    return true;

	    }


	}//examples
}//ceres

int main(int argc, char** argv)

{
    google::InitGoogleLogging(argv[0]);
    CERES_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);

    MapOfPoses poses;
    VectorOfConstraints constraints;
    std::map<int,Eigen::Quaterniond> Preintegrated_Map;
    //preintegration_func IMU_Integration;

    ceres::examples::ReadFile(FLAGS_input,&poses,&constraints);
    //IMU_Integration.Preintegrated_omega = IMU_Integration.Preintegration(constraints,&poses);
/*
    for (std::map<int, Eigen::Quaterniond>::const_iterator poses_iter= IMU_Integration.Preintegrated_omega.begin();poses_iter!=IMU_Integration.Preintegrated_omega.end();++poses_iter){
    					std::cout<<poses_iter->first<<":"<<poses_iter->second.w()<<std::endl;
    					std::cout<<poses_iter->first<<":"<<poses_iter->second.x()<<std::endl;
    					std::cout<<poses_iter->first<<":"<<poses_iter->second.y()<<std::endl;
    					std::cout<<poses_iter->first<<":"<<poses_iter->second.z()<<std::endl;
    }*/

    ceres::Problem problem;
    ceres::Solver::Options solver_options;
    	ceres::Solver::Summary summary;
    	solver_options.minimizer_progress_to_stdout = true;
    	solver_options.max_num_consecutive_invalid_steps = 1000;
   //solver_options.max_num_iterations = 1;

    	/*for (VectorOfConstraints::const_iterator constraints_iter = constraints.begin();constraints_iter!=constraints.end();++constraints_iter)
    		{

    		//Eigen::Map<const Eigen::Matrix<T,3,1> > bias_hat_i_(bias_hat_i);

    		const Constraint3d& gyro = *constraints_iter;
    		std::cout<<"GYRO"<< gyro.o.x()<<std::endl;
    		std::cout<<"GYRO"<< gyro.o.y()<<std::endl;
    		std::cout<<"GYRO"<< gyro.o.z()<<std::endl;
    		std::cout<<"GYRO x y z"<< gyro.o<<std::endl;
    		}*/

    	ceres::LossFunction* loss_function = NULL;
    	ceres::LossFunction* loss_function_outlier = new ceres::CauchyLoss(0.3);
    	ceres::LocalParameterization *quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

    int pointer_index =0;
    //int preintegrate_index = 1;

   // std::map<int,Eigen::Quaterniond> temp_map = IMU_Integration.Preintegrated_omega;
   // std::map<int,Eigen::Quaterniond> &temp_map = IMU_Integration.Preintegrated_omega;

    //initialization of the variable to store the first R
    Eigen::Quaternion<double> current_optimized_pose;
    Eigen::Quaternion<double> prev_optimized_pose;


    //optimization begins here
    //prev_optimized_pose = Eigen::Quaternion<double>::Quaternion(1,0,0,0);
 for (std::map<int,Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Pose3d> > >::const_iterator attitude_iterator= poses.begin();attitude_iterator!=poses.end();++attitude_iterator)
 {

	const std::map<int,Pose3d, std::less<int>,Eigen::aligned_allocator<std::pair<const int,Pose3d> > >::value_type pair = *attitude_iterator;
    	MapOfPoses::iterator current_pose_to_optimize = poses.find(pointer_index);
    	std::cout<<"pointer index"<<pointer_index<<std::endl;
    	//std::cout<<"CURRENT POSE"<<current_pose_to_optimize->second.q.w()<<std::endl;

    	Eigen::Quaternion<double> vector_b_x;
    	Eigen::Quaternion<double> vector_b_y;
    Eigen::Quaternion<double> Constraint_imu;
    Eigen::Vector3d bias_vec;


    	bias_vec<<0.1,0.1,0.1;
    	vector_b_x = Eigen::Quaternion<double>::Quaternion(0,pair.second.v_x.x(),pair.second.v_x.y(),pair.second.v_x.z());
    	vector_b_y = Eigen::Quaternion<double>::Quaternion(0,pair.second.v_y.x(),pair.second.v_y.y(),pair.second.v_y.z());

    	ceres::CostFunction* cost_function_x = ceres::examples::AttitudeError_x::Create(vector_b_x,pointer_index);
    	ceres::CostFunction* cost_function_y = ceres::examples::AttitudeError_y::Create(vector_b_y);

    		//???? do i really need to pass the updated bias_g
    		//return bias just for the preintegrated function to recompute the preintegrated omega measurement
    		///ceres::CostFunction* cost_function_bias = ceres::examples::Bias::Create(bias_vec);


    	problem.AddResidualBlock(cost_function_x,loss_function,current_pose_to_optimize->second.q.coeffs().data());
    	problem.SetParameterization(current_pose_to_optimize->second.q.coeffs().data(),quaternion_local_parameterization);

    	problem.AddResidualBlock(cost_function_y,loss_function,current_pose_to_optimize->second.q.coeffs().data());
    	problem.SetParameterization(current_pose_to_optimize->second.q.coeffs().data(),quaternion_local_parameterization);


    	if (pointer_index>0)
    	{

    			MapOfPoses::iterator prev_pose_to_optimize = poses.find(pointer_index-1);

    	//std::cout<<"Print value here"<<IMU_Integration.Preintegrated_omega.find(pointer_index)->second.w()<<std::endl;

    	ceres::CostFunction* imu_cost_function = ceres::examples::IMUFunctor_Bias::Create(constraints,pointer_index);
    	problem.AddResidualBlock(imu_cost_function, loss_function, prev_pose_to_optimize->second.q.coeffs().data(),current_pose_to_optimize->second.q.coeffs().data(),prev_pose_to_optimize->second.b.data());
    	problem.SetParameterization(current_pose_to_optimize->second.q.coeffs().data(),quaternion_local_parameterization);
    problem.SetParameterization(prev_pose_to_optimize->second.q.coeffs().data(),quaternion_local_parameterization);

    	//cost function for bias
    ceres::CostFunction* cost_function_bias = ceres::examples::Bias::Create(bias_vec);
	problem.AddResidualBlock(cost_function_bias, loss_function, prev_pose_to_optimize->second.b.data(),current_pose_to_optimize->second.b.data());
	//std::cout<<"prev estimated attitude"<<prev_pose_to_optimize->second.q.w()<<prev_pose_to_optimize->second.q.x()<<std::endl;
    	} //end if for adding residual block for edges preintegrated omega measurements
    	MapOfPoses::iterator prev_pose_to_optimize_op = poses.find(pointer_index-1);

   	ceres::Solve(solver_options,&problem,&summary);
    	//std::cout<<"bias estimate"<<current_pose_to_optimize->second.b.x()<<std::endl;
    	std::cout<<"estimated attitude"<<current_pose_to_optimize->second.q.w()<<current_pose_to_optimize->second.q.x()<<current_pose_to_optimize->second.q.y()<<current_pose_to_optimize->second.q.z()<<std::endl;
    	std::cout<<"prev after solving estimated attitude :"<<prev_pose_to_optimize_op->second.q.w()<<" "<<prev_pose_to_optimize_op->second.q.x()<<std::endl;
    	pointer_index = pointer_index + 1;


    	//IMU_Integration.Preintegrated_omega = IMU_Integration.Preintegration(constraints,&poses);

    	/*
    	for (std::map<int, Eigen::Quaterniond>::const_iterator poses_iter= IMU_Integration.Preintegrated_omega.begin();poses_iter!=IMU_Integration.Preintegrated_omega.end();++poses_iter){
    	  std::cout<<poses_iter->first<<":"<<poses_iter->second.w()<<std::endl;
    	   std::cout<<poses_iter->first<<":"<<poses_iter->second.x()<<std::endl;
    	    std::cout<<poses_iter->first<<":"<<poses_iter->second.y()<<std::endl;
    	    	std::cout<<poses_iter->first<<":"<<poses_iter->second.z()<<std::endl; }*/

   	std::cout<<"bias estimate x"<<current_pose_to_optimize->second.b.x()<<std::endl;
   	std::cout<<"bias estimate y"<<current_pose_to_optimize->second.b.y()<<std::endl;
   	std::cout<<"bias estimate z"<<current_pose_to_optimize->second.b.z()<<std::endl;


    	}//end for loop


 	 std::cout<<summary.FullReport()<<std::endl;
	 //ceres::examples::BuildOptimizationProblem_Main(Preintegrated_Map,&poses);
 	 ceres::examples::OutputPoses("/Users/shinfang/Documents/MATLAB/preintegration_theory/synthetic_data_optimized.txt",&poses);

 	 std::fstream outfile;
 	 outfile.open("poses_print_here",std::istream::out);
 	 for(std::map<int,Pose3d, std::less<int>,Eigen::aligned_allocator<std::pair<const int,Pose3d> > >::const_iterator print_iter=poses.begin();print_iter!=poses.end();++print_iter){
 		 const std::map<int,Pose3d,std::less<int>,Eigen::aligned_allocator<std::pair<const int,Pose3d> > >::value_type& pair_print = *print_iter;
 		 outfile<< pair_print.second.q.w()<<std::endl;
 	 }


    	//IMU_Integration.BuildOptimizationProblem(Preintegrated_Map,&poses);

    //Preintegrated_Map = IMU_Integration.Preintegration(constraints,&poses);


	return 0;

}

