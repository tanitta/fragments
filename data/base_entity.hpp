#pragma once
#include <collidable.hpp>
#include <shape.hpp>
#include <state.hpp>
#include <attribute.hpp>

namespace fragments {
	namespace data {
		class BaseEntity {
			public:
				fragments::data::Collidable collidable_;
				fragments::data::Shape shape_;
				fragments::data::State state_;
				fragments::data::Attribute attribute_;

				BaseEntity():
					collidable_(),
					shape_(),
					state_(),
					attribute_(){};

				virtual ~BaseEntity(){};

				void Setup(){};
		};
	} // namespace data
} // namespace fragments
