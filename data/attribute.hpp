#pragma once
#include <boost/numeric/ublas/matrix.hpp>
namespace fragments {
	namespace data {
		//! 属性を表す
		/*!
		 * 重量や慣性テンソル等の属性を表す
		 */
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
