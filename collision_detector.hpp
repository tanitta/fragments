#pragma once

#include <broad_phase.hpp>
#include <narrow_phase.hpp>
namespace fragments {
	class CollisionDetector {
		private:
			fragments::BroadPhase broad_phase_;
			fragments::NarrowPhase narrow_phase_;
		public:
			CollisionDetector():
				broad_phase_(),
				narrow_phase_(){};
			virtual ~CollisionDetector(){};

			void Setup(){
				broad_phase_.Setup();
				narrow_phase_.Setup();
			};

			void Update(){
				broad_phase_.Update();
				narrow_phase_.Update();
			};
	};
} // namespace fragments
