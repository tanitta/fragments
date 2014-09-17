#pragma once
#include <boost/numeric/ublas/matrix.hpp>
namespace fragments {
	namespace data {
		class Attribute {
			public:
				const float mass_;
				const boost::numeric::ublas::matrix<float> matrix_;
				Attribute():mass_(25),
				matrix_(3,3){};
				virtual ~Attribute(){};
		};
	} // namespace data
} // namespace fragments
