
#ifndef EXAMPLES_ERROR
#define EXAMPLES_ERROR

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ceres/autodiff_cost_function.h"

#include "integration_imu.hpp"
#include "preintegration_func.hpp"
#include "SO3_lib.h"

namespace ceres {
namespace examples {

//preintegration_func IMU_Integration;

class IMUFunctor_Bias{
	public:
	IMUFunctor_Bias(const VectorOfConstraints& a, int index): omega_vector(a), index_w(index){}

	template <typename T>
	bool operator() (const T* const robot_pose_estimated_prev, const T* const robot_pose_estimated_curr, const T* const bias_hat_i, T* residual)const{

		int c = 0;
		int index_preint = 1;
		std::map<int,Eigen::Quaternion<T> > Preintegrated_omega_bias;
		//typename std::map<int,Eigen::Quaternion<T> >::iterator it_bias = Preintegrated_omega_bias.begin();
		Eigen::Matrix<T,3,3> delta_Rij = Eigen::Matrix<T,3,3>::Identity();
		//Eigen::Map<const Eigen::Matrix<T,3,1> > bias_hat_ii_(bias_hat_i);
		//std::cout<<"bias"<<bias_hat_i_<<std::endl;

		//Recompute preintegrated omega with optimized bias
for (VectorOfConstraints::const_iterator constraints_iter = omega_vector.begin();constraints_iter!=omega_vector.end();++constraints_iter)
	{

	Eigen::Map<const Eigen::Matrix<T,3,1> > bias_hat_i_(bias_hat_i);

	c = c +1;
	const Constraint3d& gyro = *constraints_iter;

	/*
	T gyro_x = gyro.o.x();
	T gyro_y = gyro.o.y();
	T gyro_z = gyro.o.z();*/

	//Eigen::Matrix<T,3,1> gyroCorrect;
	//gyroCorrect[0] = gyro_x - bias_hat_ii_[0];
	//gyroCorrect[1] = gyro_y - bias_hat_ii_[1];
	//gyroCorrect[2] = gyro_z - bias_hat_ii_[2];

	//MapOfPoses::iterator current_bias = bias_map->find(bias_preint_pointer);


	//Eigen::Matrix<T,3,1> gyroCorrect = gyro.o- bias_hat_i_;
	//Eigen::Matrix<T,3,1> gyroCorrect = gyro.o.template cast<T>() -bias_hat_i_;
	//Eigen::Matrix<T,3,1> gyroCorrect = gyro.o.template cast<T>();
	Eigen::Matrix<T,3,1> gyroCorrect = gyro.o.template cast<T>() - bias_hat_i_;
	//Eigen::Matrix<T,3,1> gyroCorrect = gyroCorrect_ - bias_hat_i_;

    Eigen::Matrix<T,3,3> skew_gyro = Eigen::Matrix<T,3,3>::Zero();
			skew_gyro(0,1) = -gyroCorrect[2];
			skew_gyro(0,2) = gyroCorrect[1];
			skew_gyro(1,2) = -gyroCorrect[0];
			skew_gyro(1,0) = gyroCorrect[2];
			skew_gyro(2,0) = -gyroCorrect[1];
			skew_gyro(2,1) = gyroCorrect[0];

			T phi = gyroCorrect.norm();
			T phi_inv = T(1.0) /phi;

		Eigen::Matrix<T,3,3> iden = Eigen::Matrix<T,3,3>::Identity();
		Eigen::Matrix<T,3,3> exp_gyro;



		if (phi>0.000174)
		{

		Eigen::Matrix<T,3,3> first_term = sin(phi) * phi_inv * skew_gyro;
		Eigen::Matrix<T,3,3> second_term = (T(1.0) - cos(phi)) * phi_inv * phi_inv  * skew_gyro * skew_gyro;
		exp_gyro = iden + first_term + second_term;
		}
		else
		{

		T const_1 = T(1.0) - T(1/6)* phi* phi + T(1/120)*phi*phi*phi*phi;
		T const_2 = T(0.5) - T(1/24)* phi*phi + T(1/720) * phi*phi*phi*phi;
		exp_gyro = iden + const_1*skew_gyro + const_2 * skew_gyro* skew_gyro;
		}

		delta_Rij = delta_Rij * exp_gyro;
		if (c%4 ==0)
		{
			//std::cout<<"DELTA"<<delta_Rij<<std::endl;
			Eigen::Quaternion<T> omega_q(delta_Rij);
			omega_q.normalize();
			//std::cout<<"OMEGA"<<omega_q.w()<<" "<<omega_q.x()<<" "<<omega_q.y()<<" "<<omega_q.z()<<" "<<std::endl;
			Preintegrated_omega_bias[index_preint] = omega_q;
			index_preint = index_preint + 1;
			delta_Rij = Eigen::Matrix<T,3,3>::Identity();
		} //end if loop for the computation
	}//end for loop preintegrated measurement
	index_preint = 1;

	typename std::map<int,Eigen::Quaternion<T> >::iterator it = Preintegrated_omega_bias.find(index_w);
	Eigen::Quaternion<T> constraint_from_map;

			//std::cout<<"constraint"<<constraint_from_map.w()<<std::endl;

			constraint_from_map.w() = it->second.w();
			constraint_from_map.x() = it->second.x();
			constraint_from_map.y() = it->second.y();
			constraint_from_map.z() = it->second.z();

			//std::cout<<"C:"<<c<<" "<<"IMU"<<constraint_from_map.w()<<std::endl;

			Eigen::Quaternion<T> constraint_q = constraint_from_map.template cast<T>();
			Eigen::Map<const Eigen::Quaternion<T> > pose_hat_curr(robot_pose_estimated_curr);
			Eigen::Map<const Eigen::Quaternion<T> > pose_hat_prev(robot_pose_estimated_prev);

			Eigen::Quaternion<T> delta = constraint_from_map.conjugate() * pose_hat_prev.conjugate() * pose_hat_curr;
			//Eigen::Quaternion<T> delta = pose_hat_prev.conjugate() * pose_hat_curr;

			Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);
			residuals = T(8.0)* delta.vec();


		return true;
	}
	static ceres::CostFunction* Create(const VectorOfConstraints& a, int index){
			return new ceres::AutoDiffCostFunction<IMUFunctor_Bias,3,4,4,3>(
					new IMUFunctor_Bias(a,index));
		}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
	const VectorOfConstraints& omega_vector;
	//MapOfPoses* bias_map;
	int index_w;
};


class IMUFunctor{

	public:
	IMUFunctor(std::map<int,Eigen::Quaterniond> &temp_map,const Eigen::Quaternion<double> constraint_1, int index):x_(temp_map),constraint_1_(constraint_1),index_(index){}

	template <typename T>
	bool operator() (const T* const robot_pose_estimated_prev, const T* const robot_pose_estimated_curr, T* residual) const{

		//std::map<int,Eigen::Quaterniond>::value_type pair = *constraint_1_;
		//std::cout<<pair.second.w()<<std::endl;
		//std::cout<<"map"<<*constraint_1_<<std::endl;

	//std::cout<<"Enter constraint in imu functor"<<std::endl;

	std::map<int,Eigen::Quaterniond>::iterator it = x_.find(index_);

	Eigen::Quaternion<double> constraint_from_map = Eigen::Quaternion<double>::Quaternion(it->second.w(),it->second.x(),it->second.y(),it->second.z());

	/*
	std::cout<<constraint_from_map.w()<<std::endl;
	std::cout<<constraint_from_map.x()<<std::endl;
	std::cout<<constraint_from_map.y()<<std::endl;
	std::cout<<constraint_from_map.z()<<std::endl;*/
	Eigen::Quaternion<T> constraint_q = constraint_from_map.template cast<T>();

	/*
	std::cout<<constraint_1_.w()<<std::endl;
	std::cout<<constraint_1_.x()<<std::endl;
	std::cout<<constraint_1_.y()<<std::endl;
	std::cout<<constraint_1_.z()<<std::endl;*/
	Eigen::Map<const Eigen::Quaternion<T> > pose_hat_curr(robot_pose_estimated_curr);
	Eigen::Map<const Eigen::Quaternion<T> > pose_hat_prev(robot_pose_estimated_prev);
	//Eigen::Quaternion<T> constraint_q = constraint_1_.template cast<T>();

	Eigen::Quaternion<T> delta = constraint_q.conjugate() * pose_hat_prev.conjugate() * pose_hat_curr;
	//Eigen::Quaternion<T> delta = (pose_hat_curr.conjugate() * pose_hat_prev * constraint_q);*/



	Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);
	residuals = T(8.0)* delta.vec();


	return true;

	}

	static ceres::CostFunction* Create(std::map<int,Eigen::Quaterniond> &temp_map,const Eigen::Quaternion<double> constraint_1,int index)

	{
		return new ceres::AutoDiffCostFunction<IMUFunctor,3,4,4>(
				new IMUFunctor(temp_map,constraint_1,index));
	}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
	std::map<int,Eigen::Quaterniond> &x_;
	const Eigen::Quaternion<double> constraint_1_;
	int index_;


};

//int i=0;

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

class Bias{
	public:
	Bias(const Eigen::Vector3d bias):bias_(bias){}

	template <typename T>
	bool operator() (const T* const bias_hat_i, const T* const bias_hat_j, T* residual) const{


		Eigen::Map<const Eigen::Matrix<T,3,1> > bias_hat_i_(bias_hat_i);
		Eigen::Map<const Eigen::Matrix<T,3,1> > bias_hat_j_(bias_hat_j);
		Eigen::Map<Eigen::Matrix<T,3,1> > residuals(residual);

		residuals = T(10.0)* (bias_hat_i_- bias_hat_j_);
		//residuals = bias_hat_i_- bias_hat_j_;
		return true;
	}



	static ceres::CostFunction* Create(
		const Eigen::Vector3d bias_	){
			return new ceres::AutoDiffCostFunction<Bias,3,3,3>(new Bias(bias_));

	}
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	const Eigen::Vector3d bias_;
};


}  // namespace examples
}  // namespace ceres

#endif  // EXAMPLES_CERES_READ_G2O_H_
