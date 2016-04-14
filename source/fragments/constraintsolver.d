module fragments.constraintsolver;

import armos;
import fragments.constraintpair;
import fragments.entity;

/++
+/
class ConstraintSolver(NumericType){
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		/++
		++/
		void unitTime(in N t){_unitTime = t;}
		
		/++
		++/
		void solve(
			ref CollisionConstraintPair!N[] collisionConstraintPairs,
			ref ConstraintPair!N[] constraintPairs,
		){
			preProcess(collisionConstraintPairs, constraintPairs);
			iterate(collisionConstraintPairs, constraintPairs);
			postProcess(collisionConstraintPairs, constraintPairs);
		}
	}//public

	private{
		N _unitTime;
		int _iterations = 10;

		void preProcess(
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
		
		void postProcess(
			ref CollisionConstraintPair!N[] collisionConstraintPairs, 
			ref ConstraintPair!N[] constraintPairs,
		)in{
			import std.math;
			assert(!isNaN(_unitTime));
		}body{
			import fragments.contactpoint;
			foreach (ref collisionConstraintPair; collisionConstraintPairs) {
				auto entity = collisionConstraintPair.entity;
				
				updateVelocitiesFromDelta(entity);
				initDeltaVelocity(entity);
				
				immutable contactPoint = collisionConstraintPair.contactPoint;
				
				// immutable depth = ( entity.position + entity.linearVelocity * _unitTime - contactPoint.coordination).dotProduct(contactPoint.normal);
				// immutable depth = (entity.linearVelocity * _unitTime).dotProduct(contactPoint.normal);
				
				// immutable depth = (contactPoint.applicationPoint - contactPoint.coordination).dotProduct(contactPoint.normal);
				
				immutable depth = contactPoint.distance * contactPoint.normal;
				entity.position = entity.position + depth;
				
				import std.stdio;
				writeln("depth\t", depth);
			}
		}
		
		void updateVelocitiesFromDelta(DynamicEntity!(N) entity){
			entity.linearVelocity = entity.linearVelocity + entity.deltaLinearVelocity;
			entity.angularVelocity = entity.angularVelocity + entity.deltaAngularVelocity;
		}
		
		void initDeltaVelocity(DynamicEntity!(N) entity){
			entity.deltaLinearVelocity = V3.zero;
			entity.deltaAngularVelocity = V3.zero;
		}
	}//private
}//class ConstraintSolver

/++
+/
private struct SolverBody(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		CollisionConstraintPair!N[] collisionConstraintPairs;
		DynamicEntity!N dynamicEntity;
		V3 bias;
		
		V3 deltaLinearVelocity = V3.zero;
		V3 deltaAngularVelocity = V3.zero;
	}//public

	private{
	}//private
}//struct SolverBody
unittest{
	assert(
		__traits(compiles, (){
			alias N = double;
			auto solverBody = SolverBody!N();
		})
	);
}
