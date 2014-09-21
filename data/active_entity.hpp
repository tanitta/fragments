#pragma once

#include <boost/numeric/ublas/vector.hpp>
#include <shape.hpp>
#include <attribute.hpp>
#include <state.hpp>
#include <static_entity.hpp>
namespace fragments {
	namespace data {
		class ActiveEntity{
			public:
				const fragments::data::Shape shape_;
				fragments::data::Attribute attribute_;
				fragments::data::State state_;

				boost::numeric::ublas::vector<float> box_size_max_;
				boost::numeric::ublas::vector<float> box_size_min_;

				ActiveEntity(fragments::data::Shape shape = fragments::data::Shape::SQUARE):
					shape_(shape),
					state_(),
					attribute_(){};
				virtual ~ActiveEntity(){};
		};
	} // namespace data
} // namespace fragments
