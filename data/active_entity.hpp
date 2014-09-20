#pragma once
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

				ActiveEntity(fragments::data::Shape shape = fragments::data::Shape::SQUARE):
					shape_(shape),
					state_(),
					attribute_(){};
				virtual ~ActiveEntity(){};
		};
	} // namespace data
} // namespace fragments
