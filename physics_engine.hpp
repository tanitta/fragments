#pragma once
#include <force_adder.hpp>
#include <collision_detector.hpp>
#include <constraint_solver.hpp>
#include <integrator.hpp>

namespace fragments {
	class PhysicsEngine {
		private:
			fragments::ForceAdder force_adder_;
			fragments::CollisionDetector collision_detector_;
			fragments::ConstraintSolver constraint_solver_;
			fragments::Integrator integrator_;

		public:
			PhysicsEngine():
				force_adder_(),
				collision_detector_(),
				constraint_solver_(),
				integrator_(){};
			virtual ~PhysicsEngine(){};

			void Setup(){
				collision_detector_.Setup();
				constraint_solver_.Setup();
				integrator_.Setup();
			};
			void Update(){
				collision_detector_.Update();
				constraint_solver_.Update();
				integrator_.Update();
			};

	};
} // namespace fragments
