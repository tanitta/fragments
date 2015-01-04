#pragma once
#include <data/active_entity.hpp>
namespace fragments {
	class Integrator {
		public:
			Integrator(){};
			virtual ~Integrator(){};

			void Setup(){};
			void Update(std::vector<fragments::data::ActiveEntity>& active_entities){
				for (auto i : active_entities) {
					i.Integrate(1.0/30.0);
				}
			};
	};
} // namespace fragments
