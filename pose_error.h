
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
/*
	std::cout<<"current robot pose W"<<robot_pose_estimated_curr[0]<<std::endl;
	std::cout<<robot_pose_estimated_curr[1]<<std::endl;
	std::cout<<robot_pose_estimated_curr[2]<<std::endl;
	std::cout<<robot_pose_estimated_curr[3]<<std::endl;


	std::cout<<"previous robot pose W"<<robot_pose_estimated_prev[0]<<std::endl;
	std::cout<<robot_pose_estimated_prev[1]<<std::endl;
	std::cout<<robot_pose_estimated_prev[2]<<std::endl;
	std::cout<<robot_pose_estimated_prev[3]<<std::endl;
*/

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

class AttitudeError_x{
	public:

	AttitudeError_x(const Eigen::Quaternion<double> vec_b_x) : vec_b_x_ (vec_b_x){ }

      template <typename T>
            bool operator() (const T* const robot_pose_estimated, T* residual) const{


    	  Eigen::Matrix<T,3,1> p_x_world;
    	  p_x_world << T(1),T(0),T(0);

    	  Eigen::Map<const Eigen::Quaternion<T> > pose_hat(robot_pose_estimated);
    	  Eigen::Quaternion<T> vec_b_constant = vec_b_x_.template cast<T>();

/*
    	  std::cout<<"current robot pose R vertex"<<robot_pose_estimated[0]<<std::endl;
    	  std::cout<<robot_pose_estimated[1]<<std::endl;
    	  std::cout<<robot_pose_estimated[2]<<std::endl;
    	  std::cout<<robot_pose_estimated[3]<<std::endl;*/



    	  Eigen::Quaternion<T> vector_rotated = pose_hat * vec_b_constant * pose_hat.conjugate();
    	  Eigen::Map<Eigen::Matrix<T,3,1> >residuals(residual);
    	  residuals = p_x_world - vector_rotated.vec();

/*
    		std::cout<<"current robot pose R vertex"<<robot_pose_estimated[0]<<std::endl;
    		std::cout<<robot_pose_estimated[1]<<std::endl;
    		std::cout<<robot_pose_estimated[2]<<std::endl;
    		std::cout<<robot_pose_estimated[3]<<std::endl;*/
    		return true;

            }

            static ceres::CostFunction* Create(

				const Eigen::Quaternion<double> vec_b_x)
            {
                return new ceres::AutoDiffCostFunction<AttitudeError_x,3,4>(new AttitudeError_x(vec_b_x));

            }
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW


private:
       const Eigen::Quaternion<double> vec_b_x_;

        };


class AttitudeError_y{
	public:
	AttitudeError_y(const Eigen::Quaternion<double> vector_b_y):vector_b_y_(vector_b_y){}

		template <typename T>
		bool operator() (const T* const robot_pose_estimated, T* residual) const{

			Eigen::Map<const Eigen::Quaternion<T> > attitude_hat(robot_pose_estimated);
			Eigen::Matrix<T,3,1> p_y_world;
			p_y_world << T(0),T(1),T(0);

			Eigen::Map<const Eigen::Quaternion<T> > pose_hat(robot_pose_estimated);
			Eigen::Quaternion<T> vector_b_y_constant = vector_b_y_.template cast<T>();

			Eigen::Quaternion<T> vector_rotated = pose_hat * vector_b_y_constant * pose_hat.conjugate();

			std::cout<<"robot pose"<<std::endl;
			std::cout<<robot_pose_estimated[0]<<std::endl;
			std::cout<<robot_pose_estimated[1]<<std::endl;
			std::cout<<robot_pose_estimated[2]<<std::endl;
			std::cout<<robot_pose_estimated[3]<<std::endl;

			Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);
			residuals = p_y_world - vector_rotated.vec();

			return true;
		}

		static ceres::CostFunction* Create(
		const Eigen::Quaternion<double> vector_b_y){
			return new ceres::AutoDiffCostFunction<AttitudeError_y,3,4>(new AttitudeError_y(vector_b_y));
		}
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		const Eigen::Quaternion<double> vector_b_y_;

};


}  // namespace examples
}  // namespace ceres

#endif  // EXAMPLES_CERES_READ_G2O_H_
