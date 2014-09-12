#pragma once
namespace fragments {
	class Integrator {
		public:
			Integrator(){};
			virtual ~Integrator(){};

			void Setup(){};
			void Update(){};
	};
} // namespace fragments
