module fragments.constraintsolver;

import armos;
import fragments.constraintpair;

/++
+/
class ConstraintSolver(NumericType){
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		void solve(ref ConstraintPair!N[] constraintPairs){
			setup(constraintPairs);
			iterate(constraintPairs);
		}
	}//public

	private{
		int _iterations = 10;
		
		void iterate(ref ConstraintPair!N[] constraintPairs){
			//iteration
			for (int i = 0; i < _iterations; i++) {
				foreach (ref constraintPair; constraintPairs) {
					// linear
					foreach (ref constraint; constraintPair.linearConstraints) {
	
					}

					// angular
					foreach (ref constraint; constraintPair.angularConstraints) {

					}
				}

			}
		}
		
		void setup(ref ConstraintPair!N[] constraintPairs){
			foreach (ref constraintPair; constraintPairs) {
				// linear
				foreach (ref constraint; constraintPair.linearConstraints) {
					
				}
				
				// angular
				foreach (ref constraint; constraintPair.angularConstraints) {

				}
			}
		}
	}//private
}//class ConstraintSolver
