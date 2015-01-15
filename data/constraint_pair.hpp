#pragma once
#include <iostream>
#include <base_entity.hpp>
namespace fragments {
	namespace data {
		class ConstraintPair {
			private:
				std::vector<fragments::data::BaseEntity*> base_entity_ptrs_;

			public:
				ConstraintPair():base_entity_ptrs_(){};
				virtual ~ConstraintPair(){};

				fragments::data::BaseEntity& operator[](int i){
					return *base_entity_ptrs_[i];
				};
		};
	} // namespace data
} // namespace fragments
