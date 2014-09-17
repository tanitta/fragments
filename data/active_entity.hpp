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

				vector<fragments::data::StaticEntity*> static_entity_ptrs_;

				ActiveEntity(fragments::data::Shape shape):
					shape_(shape),
					state_(),
					attribute_(),
					static_entity_ptrs_(0){};
				virtual ~ActiveEntity(){};

				void PushStaticEntity(fragments::data::StaticEntity* ptr){
					static_entity_ptrs_.push_back(ptr);
				};

		};
	} // namespace data
} // namespace fragments
