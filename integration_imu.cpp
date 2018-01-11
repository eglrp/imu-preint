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
#include "pose_error.h"
#include "glog/logging.h"
#include "integration_imu.hpp"


DEFINE_string(input,"/Users/shinfang/Documents/MATLAB/preintegration_theory/omega_noisy.txt","The file is similar to g2o format");

namespace ceres{
    namespace examples{

    bool OutputPoses(const std::string& filename,const MapOfPoses& poses)
        {
            std::fstream outfile;
            outfile.open(filename.c_str(), std::istream::out);


            for (std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::const_iterator poses_iter = poses.begin();
                 poses_iter != poses.end(); ++poses_iter) {

                const std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Pose3d> > >::value_type& pair =*poses_iter;

                outfile << pair.second.q.w()<<" "<<pair.second.q.x()<<" "<< pair.second.q.y()<<" "<<pair.second.q.z() <<" "<<std::endl;

                if(poses_iter-> first == 5)
                {
                    std::cout<<pair.first<<std::endl;
                }
            }


            return true;
        }


 void BuildOptimizationProblem(std::map<int,Eigen::Quaterniond> &temp_map,MapOfPoses* poses, ceres::Solver::Summary* summary){


	 	 	 std::fstream outfile;
	         outfile.open("poses_at_each_step.txt", std::istream::out);

	 	 	ceres::Problem problem;
	 	 	ceres::Solver::Options solver_options;
	 	 	solver_options.minimizer_progress_to_stdout = true;
	 	 	solver_options.max_num_consecutive_invalid_steps = 1000;

	 	 	//ceres::Solver::Summary summary;

            ceres::LossFunction* loss_function = NULL;
            ceres::LocalParameterization *quaternion_local_parameterization = new EigenQuaternionParameterization;

            std::map<int,Eigen::Quaterniond>::iterator it= temp_map.begin();
            	/*
             for (it=temp_map.begin(); it!=temp_map.end(); ++it)
             {
            	 std::cout<<"print constraint"<<std::endl;
             std::cout<<it->second.w()<<std::endl;
             std::cout<<it->second.x()<<std::endl;
             std::cout<<it->second.y()<<std::endl;
             std::cout<<it->second.z()<<std::endl;

             }
			*/
            int index=0;

for (std::map<int,Pose3d,std::less<int>, Eigen::aligned_allocator<std::pair<const int,Pose3d> > >::const_iterator poses_iter =poses->begin();poses_iter!=poses->end();++poses_iter)
  {
                const std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Pose3d> > >::value_type& pair =*poses_iter;


                //Vertex -> Measured value   Poses -> Estimated value  temp_map -> Constraint (IMU Preintegration value)

                 MapOfPoses::iterator current_pose_to_optimize = poses->find(index);



                  Eigen::Quaternion<double> Omega_Q;
                  Eigen::Quaternion<double> Constraint_imu;

                  Omega_Q = Eigen::Quaternion<double>::Quaternion(pair.second.q.w(),pair.second.q.x(),pair.second.q.y(),pair.second.q.z());
                  	  	  /*
                  current_pose_to_optimize -> second.q.w () =0.6536;
                  current_pose_to_optimize -> second.q.x () =0;
                  current_pose_to_optimize -> second.q.y () =0.7568;
                  current_pose_to_optimize -> second.q.z () =0;
                  std::cout<<"answer"<<current_pose_to_optimize->second.q.w()<<std::endl;*/
                  ///Constraint_imu = Eigen::Quaternion<double>::Quaternion(preint_constraint->second.w(),preint_constraint->second.x(),preint_constraint->second.y(),preint_constraint->second.z());

                  ceres::CostFunction* cost_function = PoseError::Create(index,Omega_Q);

                  problem.AddResidualBlock(cost_function,loss_function,current_pose_to_optimize->second.q.coeffs().data());

                  problem.SetParameterization(current_pose_to_optimize->second.q.coeffs().data(),quaternion_local_parameterization);


                 //Enter if loop when the next pose data is obtained.

                 if(index>0){



                	 std::map<int,Eigen::Quaterniond>::iterator preint_constraint= temp_map.find(index);
                	 Constraint_imu = Eigen::Quaternion<double>::Quaternion(preint_constraint->second.w(),preint_constraint->second.x(),preint_constraint->second.y(),preint_constraint->second.z());


                 ceres::CostFunction* constraint_cost_function = IMUFunctor::Create(Constraint_imu);

                 MapOfPoses::iterator prev_pose_to_optimize = poses->find(index-1);


                 problem.AddResidualBlock(constraint_cost_function,
                		 	 	 	 	 	 	 loss_function,
												 prev_pose_to_optimize->second.q.coeffs().data(),
												 	 current_pose_to_optimize->second.q.coeffs().data());

                 problem.SetParameterization(current_pose_to_optimize->second.q.coeffs().data(),quaternion_local_parameterization);
                 problem.SetParameterization(prev_pose_to_optimize->second.q.coeffs().data(),quaternion_local_parameterization);
                 }

                 ceres::Solve(solver_options,&problem,summary);


                 for (std::map<int,Pose3d,std::less<int>, Eigen::aligned_allocator<std::pair<const int,Pose3d> > >::const_iterator poses_op_iter =poses->begin();poses_op_iter!=poses->end();++poses_op_iter){

                	 	 const std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int,Pose3d> > >::value_type& pair_op =*poses_op_iter;

                	 	 outfile <<index<<" "<<pair_op.first<<" "<<pair_op.second.q.w()<<" "<<pair_op.second.q.x()<<" "<< pair_op.second.q.y()<<" "<<pair_op.second.q.z() <<std::endl;

                 }


                 index=index+1;


                //std::cout<<pair.first<<std::endl;
                //std::cout<<pair.second.q.w()<<std::endl;
                //std::cout<<pair.second.q.coeffs().data()<<std::endl;

            }//for loop

        }//BuildOptimizationProblem

 }//namespace examples
}//namespace ceres




int main(int argc, char** argv)

{

    google::InitGoogleLogging(argv[0]);
    CERES_GFLAGS_NAMESPACE::ParseCommandLineFlags(&argc, &argv, true);


    ceres::examples::MapOfPoses poses;

    ceres::examples::VectorOfConstraints constraints;
    ceres::examples::VectorOfConstraints c;

    //Poses to store as Estimated Pose
    ceres::examples::ReadFile(FLAGS_input,&poses,&constraints);
    //Vertex to store as Measured Pose



    int count =0;
    int index=1;

    //map to store preintegrated omega
    std::map<int,Eigen::Quaterniond> Preintegrated_omega;
    std::map<int,Eigen::Quaterniond>::iterator it= Preintegrated_omega.begin();




    Eigen::Matrix<double,3,3> originOmega = Eigen::Matrix<double,3,3>::Identity();
    Eigen::Matrix<double,3,3> Omegahat = Eigen::Matrix<double,3,3>::Zero();
    Eigen::Matrix<double,3,3> Omegahatx = Eigen::Matrix<double,3,3>::Zero();

    //Eigen::Quaterniond omega_q;

    //Integrate measured omega (IMU)
    for(ceres::examples::VectorOfConstraints::iterator constraints_iter =constraints.begin();constraints_iter!=constraints.end(); ++constraints_iter)
    {
        const ceres::examples::Constraint3d& constraint = *constraints_iter;
        count = count +1;

        Omegahat <<0, -constraint.o.z(), constraint.o.y(),
        constraint.o.z(),0,-constraint.o.x(),
        -constraint.o.y(),constraint.o.x(),0;

        Omegahatx = originOmega*Omegahat.exp();
        originOmega = Omegahatx;

        if (count%4==0)
        {
        		std::cout<<Omegahatx<<std::endl;
        		Eigen::Matrix3d m;
        		m<<Omegahatx(0,0),Omegahatx(0,1),Omegahatx(0,2),
        				Omegahatx(1,0),Omegahatx(1,1),Omegahatx(1,2),
						Omegahatx(2,0),Omegahatx(2,1),Omegahatx(2,2);

        		std::cout<<m<<std::endl;
        		Eigen::Quaterniond omega_q(m);

            omega_q.normalize();

            std::cout<<"omega_q"<<omega_q.w()<<std::endl;
            std::cout<<"omega_q"<<omega_q.y()<<std::endl;
            omega_q.normalize();
            Preintegrated_omega[index] = omega_q;
            index = index+1;
            originOmega = Eigen::Matrix<double,3,3>::Identity();
        }

    }//for loop

    /*
    for (it=Preintegrated_omega.begin();it!=Preintegrated_omega.end();++it)
     {

     std::cout<<"pereint"<<it->second.w()<<std::endl;
     std::cout<<"pereint"<<it->second.x()<<std::endl;
     std::cout<<"pereint"<<it->second.y()<<std::endl;
     std::cout<<"pereint"<<it->second.z()<<std::endl;
     }*/
    ceres::Solver::Summary summary;

    //ceres::Problem problem;
    ceres::examples::BuildOptimizationProblem(Preintegrated_omega,&poses,&summary);
    //ceres::Solver::Options solver_options;
    //solver_options.minimizer_progress_to_stdout = true;
    //solver_options.max_num_consecutive_invalid_steps = 1000;

   // ceres::Solver::Summary summary;
    //ceres::Solve(solver_options,&problem,&summary);
    std::cout<<summary.FullReport()<<std::endl;


    ceres::examples::OutputPoses("poses_optimized.txt",poses);

    return 0;
}

