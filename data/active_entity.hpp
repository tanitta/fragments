#pragma once
#include <shape.hpp>
#include <attribute.hpp><`0`>
#include <state.hpp><`0`>
#include <static_entity.hpp>
namespace fragments {
	namespace data {
		class ActiveEntity{
			public:
				const fragments::data::Shape shape_;
				fragments::data::Attribute attribute_;
				fragments::data::State state_;

				vector< shared_ptr< fragments::data::StaticEntity > > static_entity_;

				ActiveEntity(fragments::data::Shape shape):
					shape_(shape),
					state_(),
					attribute_(),
					static_entity_(0){};
				virtual ~ActiveEntity(){};

				void PushStaticEntity(shared_ptr< fragments::data::StaticEntity > ptr){};

		};
	} // namespace data
} // namespace fragments
