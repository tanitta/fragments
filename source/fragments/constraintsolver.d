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
		DynamicEntity!N[] _dynamicEntities;

		void preProcess(
			ref CollisionConstraintPair!N[] collisionConstraintPairs, 
			ref ConstraintPair!N[] constraintPairs,
		){
			foreach (ref collisionConstraintPair; collisionConstraintPairs) {
				_dynamicEntities ~= collisionConstraintPair.entity;
			}
			import std.algorithm.iteration:uniq;
			import std.array;
			_dynamicEntities = _dynamicEntities.uniq.array;
			
			foreach (ref entity; _dynamicEntities) {
				entity.bias = V3.zero;
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
					
					//collision
					{
						V3[2] deltaVelocities = collisionConstraintPair.collisionConstraint.deltaVelocities(entity);
						entity.deltaLinearVelocity  = entity.deltaLinearVelocity + deltaVelocities[0];
						entity.deltaAngularVelocity = entity.deltaAngularVelocity + deltaVelocities[1];
						
						import std.math;
						immutable maxFriction = fabs(collisionConstraintPair.collisionConstraint.currentImpulse) * collisionConstraintPair.dynamicFriction;
						foreach (ref frictionConstraint; collisionConstraintPair.frictionConstraints) {
							frictionConstraint.lowerLimit = -maxFriction;
							frictionConstraint.upperLimit = maxFriction;
						}
					}
					
					//friction
					{
						foreach (int index, frictionConstraint; collisionConstraintPair.frictionConstraints) {
							V3[2] deltaVelocities = frictionConstraint.deltaVelocities(entity);
							entity.deltaLinearVelocity  = entity.deltaLinearVelocity + deltaVelocities[0];
							entity.deltaAngularVelocity = entity.deltaAngularVelocity + deltaVelocities[1];
						}
					}
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
				immutable depth = contactPoint.distance * contactPoint.normal;
				if(entity.bias.norm < depth.norm){
					entity.bias = depth;
				}
				
				import std.stdio;
				writeln("depth\t", depth.norm);
			}
			
			foreach (entity; _dynamicEntities) {
				entity.position = entity.position + entity.bias;
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
