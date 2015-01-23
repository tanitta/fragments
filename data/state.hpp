#pragma once

// #include <boost/numeric/ublas/vector.hpp>
// #include <boost/math/quaternion.hpp>
#ifdef Success
  #undef Success
#endif
#include "Eigen/Dense"
#include "Eigen/Geometry"
namespace fragments {
	namespace data {
		//! 状態を定義
		/*!
		 * 位置，速度，角度，角速度といった状態を格納する
		 */
		class State {
			public:
				bool is_active_;
				Eigen::Vector3d position_;
				// Eigen::Quaternion<double> orientation_;
				Eigen::Vector3d linear_velocity_;
				Eigen::Vector3d angular_velocity_;

				State():
					position_(Eigen::Vector3d::Zero(3)),
					linear_velocity_(Eigen::Vector3d::Zero(3)),
					angular_velocity_(Eigen::Vector3d::Zero(3)){};
				virtual ~State(){};
		};
	} // namespace data
} // namespace fragments
