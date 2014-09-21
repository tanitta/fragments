#pragma once

#include <boost/numeric/ublas/vector.hpp>
#include <boost/math/quaternion.hpp>
namespace fragments {
	namespace data {
		class State {
			public:
				bool is_active_;
				boost::numeric::ublas::vector<float> position_;
				boost::math::quaternion<float> orientation_;
				boost::numeric::ublas::vector<float> linear_velocity_;
				boost::numeric::ublas::vector<float> angular_velocity_;

				State():
					position_(3),
					linear_velocity_(3),
					angular_velocity_(3){};
				virtual ~State(){};
		};
	} // namespace data
} // namespace fragments
