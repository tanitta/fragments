#pragma once

#include <base_entity.hpp>
#include <static_entity.hpp>
namespace fragments {
	namespace data {
		class ActiveEntity : public fragments::data::BaseEntity {
			public:
				vector< shared_ptr< fragments::data::StaticEntity > > static_entity_;

				ActiveEntity():
					static_entity_(0){};
				virtual ~ActiveEntity(){};

				void PushStaticEntity(shared_ptr< fragments::data::StaticEntity > ptr){};

		};
	} // namespace data
} // namespace fragments
