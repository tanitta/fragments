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
		void solve(
			ref CollisionConstraintPair!N[] collisionConstraintPairs,
			ref ConstraintPair!N[] constraintPairs,
		){
			setup(collisionConstraintPairs, constraintPairs);
			iterate(collisionConstraintPairs, constraintPairs);
		}
	}//public

	private{
		int _iterations = 10;
		
		void setup(
			ref CollisionConstraintPair!N[] collisionConstraintPairs, 
			ref ConstraintPair!N[] constraintPairs,
		){
			foreach (ref collisionConstraintPair; collisionConstraintPairs) {
				
			}
			
			// foreach (ref constraintPair; constraintPairs) {
			// 	// linear
			// 	foreach (ref constraint; constraintPair.linearConstraints) {
			//		
			// 	}
			//	
			// 	// angular
			// 	foreach (ref constraint; constraintPair.angularConstraints) {
			//
			// 	}
			// }
		}
		
		void iterate(
			ref CollisionConstraintPair!N[] collisionConstraintPairs, 
			ref ConstraintPair!N[] constraintPairs,
		){
			
			import fragments.entity;
			//iteration
			for (int i = 0; i < _iterations; i++) {
				// collisionConstraint;
				foreach (ref collisionConstraintPair; collisionConstraintPairs) {
					DynamicEntity!N entity = collisionConstraintPair.entity;
					
					V3[2] deltaVelocities = collisionConstraintPair.collisionConstraint.deltaVelocities(entity);
					entity.deltaLinearVelocity  = entity.deltaLinearVelocity + deltaVelocities[0];
					entity.deltaAngularVelocity = entity.deltaAngularVelocity + deltaVelocities[1];
				}
				
				// foreach (ref constraintPair; constraintPairs) {
				// 	// linear
				// 	foreach (ref constraint; constraintPair.linearConstraints) {
				//
				// 	}
				//
				// 	// angular
				// 	foreach (ref constraint; constraintPair.angularConstraints) {
				//
				// 	}
				// }

			}
		}
		
		void updateVelocities(
			ref CollisionConstraintPair!N[] collisionConstraintPairs, 
			ref ConstraintPair!N[] constraintPairs,
		){
			foreach (ref constraintPair; constraintPairs) {
			}
			foreach (ref collisionConstraintPair; collisionConstraintPairs) {
			}
		}
	}//private
}//class ConstraintSolver
