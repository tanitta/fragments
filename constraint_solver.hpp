#pragma once

namespace fragments {
	class ConstraintSolver {
		public:
			ConstraintSolver(){};
			virtual ~ConstraintSolver(){};

			void Setup(){};
			void Update(){};
	};
} // namespace fragments
