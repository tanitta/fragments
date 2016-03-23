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
			//preprocess
			foreach (ref constraintPair; constraintPairs) {
				{
					// auto rA = 
					// auto r_tilda = [
					// 	M33(
					// 			[0, ]
					// 	)
					// ]
					// constraintPair.k = 
					// 	(constraintPair.entities[0].massInv + constraintPair.entities[1].massInv) * M33.identity 
					// 	- ;
				}
				
				// linear
				foreach (ref constraint; constraintPair.linearConstraints) {
					
				}
				
				// angular
				foreach (ref constraint; constraintPair.angularConstraints) {
					
				}
			}
			
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
	}//public

	private{
		int _iterations = 10;
		
		void setup(){
			
		}
	}//private
}//class ConstraintSolver
