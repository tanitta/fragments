#pragma once

namespace fragments {
	namespace data {
		class Config {
			public:
				bool is_gravity_;
				bool is_air_resistance_;

				Config():
					is_gravity_(true),
					is_air_resistance_(true){};
				virtual ~Config(){};
		};
	} // namespace data
} // namespace fragments
