#pragma once
#include <collision_detector.hpp><`0`>
#include <constraint_solver.hpp><`0`>
#include <integrator.hpp>

namespace fragments {
	class PhysicsEngine {
		private:
			fragments::CollisionDetector collision_detector_;
			fragments::ConstraintSolver constraint_solver_;
			fragments::Integrator integrator_;

		public:
			PhysicsEngine():
				collision_detector_(),
				constraint_solver_(),
				integrator_(){};
			virtual ~PhysicsEngine(){};

			void Update(){};

	};
} // namespace fragments
