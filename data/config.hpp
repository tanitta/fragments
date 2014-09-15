#pragma once

namespace fragments {
	namespace data {
		class Config {
			public:
				bool is_gravity_;
				bool is_air_resistance_;
				int frame_rate_;
				Config():
					is_gravity_(true),
					is_air_resistance_(true),
					frame_rate_(30){
				};
				virtual ~Config(){};
		};
	} // namespace data
} // namespace fragments
