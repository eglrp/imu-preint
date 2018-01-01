
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

	Eigen::Map<const Eigen::Quaternion<T> > pose_hat_curr(robot_pose_estimated_curr);
	Eigen::Map<const Eigen::Quaternion<T> > pose_hat_prev(robot_pose_estimated_prev);
	Eigen::Quaternion<T> constraint_q = constraint_.template cast<T>();

	Eigen::Quaternion<T> delta = pose_hat_prev * pose_hat_curr.conjugate() * constraint_q;
	Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);
	residuals = delta.vec();



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


class PoseError{
	public:

	PoseError(const int index, const Eigen::Quaternion<double> omega_observed) : index_(index),omega_observed_(omega_observed){ }

      template <typename T>
            bool operator() (const T* const robot_pose_estimated, T* residual) const{


    	 // Eigen::Map<const Eigen::Quaternion<T> > omega_observed_b(omega_observed_);

    	  //Eigen::Quaternion<T> omega_measured_q = Eigen::Quaternion<T> >(w_,x_,y_,z_);

    	  Eigen::Map<const Eigen::Quaternion<T> > pose_hat(robot_pose_estimated);
    	  Eigen::Quaternion<T> omega_observed_inverse = omega_observed_.template cast<T>();
    	  Eigen::Quaternion<T> omega_obs_invs = omega_observed_inverse.conjugate();
    	  Eigen::Quaternion<T> delta_q = omega_obs_invs * pose_hat;
    	  Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);
    	  residuals = delta_q.vec();





    	  	  	  //Eigen::Quaternion<T> omega_measured =
               // Eigen::Map<const Eigen::Quaternion<T> > pose_hat(robot_pose_estimated);
               // Eigen::Quaternion<T> delta_q = pose_hat.conjugate() * t_omega_measured_;




               // Eigen::Quaternion<T> robot_pose_inverse = pose_hat.conjugate();
                //Eigen::Quaternion<T> robot_pose_inverse = t_omega_measured_.q.conjugate();
                //Eigen::Quaternion<T> measured = t_omega_measured_.template cast<T>();
                //Eigen::Quaternion<T> delta_q = robot_pose_inverse.template cast<T>() * pose_hat;



                //residual in quaternion
               // Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);

               // residuals = delta_q.vec();

                /*
                residuals[0] = aux(0);
                residuals[1] = aux(1);
                residuals[2] = aux(2);
*/
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
