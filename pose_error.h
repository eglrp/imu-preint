
#ifndef EXAMPLES_ERROR
#define EXAMPLES_ERROR

#include "Eigen/Core"
#include "ceres/autodiff_cost_function.h"

#include "integration_imu.hpp"

namespace ceres {
namespace examples {

class IMUFunctor{

	public:
	IMUFunctor(const Eigen::Quaternion<double> constraint) : constraint_(constraint) {}

	template <typename T>
	bool operator() (const T* const robot_pose_estimated_prev, const T* const robot_pose_estimated_curr, T* residual) const{

  	 // std::cout<<"constraint in functor"<<constraint_.w()<<std::endl;
  	  //std::cout<<constraint_.x()<<std::endl;
  	  //std::cout<<constraint_.y()<<std::endl;
  	  //std::cout<<constraint_.z()<<std::endl;

	Eigen::Map<const Eigen::Quaternion<T> > pose_hat_curr(robot_pose_estimated_curr);
	Eigen::Map<const Eigen::Quaternion<T> > pose_hat_prev(robot_pose_estimated_prev);
	Eigen::Quaternion<T> constraint_q = constraint_.template cast<T>();

	Eigen::Quaternion<T> delta = constraint_q.conjugate() * pose_hat_prev.conjugate() * pose_hat_curr;
	//Eigen::Quaternion<T> delta = (pose_hat_curr.conjugate() * pose_hat_prev * constraint_q);


	/*
	std::cout<<"delta"<<delta.w()<<std::endl;
	std::cout<<"delta"<<delta.x()<<std::endl;
	std::cout<<"delta"<<delta.y()<<std::endl;
	std::cout<<"delta"<<delta.z()<<std::endl;
	std::cout<<"HI"<<std::endl;*/
	Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);
	residuals = T(8.0)* delta.vec();

	std::cout<<"current robot pose W"<<robot_pose_estimated_curr[0]<<std::endl;
	std::cout<<robot_pose_estimated_curr[1]<<std::endl;
	std::cout<<robot_pose_estimated_curr[2]<<std::endl;
	std::cout<<robot_pose_estimated_curr[3]<<std::endl;


	std::cout<<"previous robot pose W"<<robot_pose_estimated_prev[0]<<std::endl;
	std::cout<<robot_pose_estimated_prev[1]<<std::endl;
	std::cout<<robot_pose_estimated_prev[2]<<std::endl;
	std::cout<<robot_pose_estimated_prev[3]<<std::endl;


	return true;

	}

	static ceres::CostFunction* Create(const Eigen::Quaternion<double> constraint){
		return new ceres::AutoDiffCostFunction<IMUFunctor,3,4,4>(
				new IMUFunctor(constraint));
	}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
	const Eigen::Quaternion<double> constraint_;

};

int i=0;
class PoseError{
	public:

	PoseError(const int index, const Eigen::Quaternion<double> omega_observed) : index_(index),omega_observed_(omega_observed){ }

      template <typename T>
            bool operator() (const T* const robot_pose_estimated, T* residual) const{


    	 // Eigen::Map<const Eigen::Quaternion<T> > omega_observed_b(omega_observed_);

    	  //Eigen::Quaternion<T> omega_measured_q = Eigen::Quaternion<T> >(w_,x_,y_,z_);
    	  /*
    	  std::cout<<"omega"<<omega_observed_.w()<<std::endl;
    	  std::cout<<omega_observed_.x()<<std::endl;
    	  std::cout<<omega_observed_.y()<<std::endl;
    	  std::cout<<omega_observed_.z()<<std::endl;*/
    	  Eigen::Map<const Eigen::Quaternion<T> > pose_hat(robot_pose_estimated);
    	  Eigen::Quaternion<T> omega_observed_inverse = omega_observed_.template cast<T>();
    	  Eigen::Quaternion<T> omega_obs_invs = omega_observed_inverse.conjugate();

    	  Eigen::Quaternion<T> delta_q = omega_obs_invs * pose_hat;
    	  /*
    	  std::cout<<"delta Pose"<<delta_q.w()<<std::endl;
    	  	std::cout<<"delta Pose"<<delta_q.x()<<std::endl;
    	  	std::cout<<"delta Pose"<<delta_q.y()<<std::endl;
    	  	std::cout<<"delta Pose"<<delta_q.z()<<std::endl;*/
    	  Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);
    	  residuals = T(8.0) * delta_q.vec();

    		std::cout<<"current robot pose R vertex"<<robot_pose_estimated[0]<<std::endl;
    		std::cout<<robot_pose_estimated[1]<<std::endl;
    		std::cout<<robot_pose_estimated[2]<<std::endl;
    		std::cout<<robot_pose_estimated[3]<<std::endl;




                return true;

            }

            static ceres::CostFunction* Create(
            		const int index,
				const Eigen::Quaternion<double> omega_observed)
            {
                return new ceres::AutoDiffCostFunction<PoseError,3,4>
                (new PoseError(index,omega_observed));

            }
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW


private:
       const int index_;
       const Eigen::Quaternion<double> omega_observed_;

        };





}  // namespace examples
}  // namespace ceres

#endif  // EXAMPLES_CERES_READ_G2O_H_
