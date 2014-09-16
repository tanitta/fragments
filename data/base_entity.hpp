#pragma once

#include <shape.hpp>
#include <state.hpp>
#include <attribute.hpp>

namespace fragments {
	namespace data {
		class BaseEntity {
			public:
				BaseEntity(){};

				virtual ~BaseEntity(){};

				void Setup(){};
		};
	} // namespace data
} // namespace fragments
