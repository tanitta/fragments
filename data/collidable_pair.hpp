#pragma once
#include <active_entity.hpp>
#include <static_entity.hpp>
namespace fragments {
	namespace data {
		class CollidablePair {
			private:
				fragments::data::StaticEntity& static_entity_ref_;
				fragments::data::ActiveEntity& active_entity_ref_;
			public:
				CollidablePair(fragments::data::StaticEntity& static_entity_ref, fragments::data::ActiveEntity& active_entity_ref):
			static_entity_ref_(static_entity_ref),active_entity_ref_(active_entity_ref){};
				virtual ~CollidablePair(){};
		};
	} // namespace data
} // namespace fragments
