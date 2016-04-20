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
			ref DynamicEntity!N[]           dynamicEntities,
			ref CollisionConstraintPair!N[] collisionConstraintPairs,
			ref LinkConstraintPair!N[]      linkConstraintPairs,
			ref LinearImpulseConstraint!N[] linearImpulseConstraints,
		){
			_iterations.iterate(
				collisionConstraintPairs,
				linkConstraintPairs,
				linearImpulseConstraints
			);
			
			postProcess(
				collisionConstraintPairs,
				linkConstraintPairs,
				dynamicEntities
			);
		}
	}//public

	private{
		N _unitTime;
		int _iterations = 10;
		DynamicEntity!N[] _dynamicEntities;
	}//private
}//class ConstraintSolver

private void postProcess(N)(
	ref CollisionConstraintPair!N[] collisionConstraintPairs, 
	ref LinkConstraintPair!N[]      linkConstraintPairs,
	ref DynamicEntity!N[]           dynamicEntities,
){
	import std.algorithm.iteration:each;
	dynamicEntities.each!(entity => entity.updateVelocitiesFromDelta);
	dynamicEntities.each!(entity => entity.initDeltaVelocity);
	
	collisionConstraintPairs.updateBias;
}

private void updateBias(N)(
	ref CollisionConstraintPair!N[] collisionConstraintPairs, 
){
	//bias
	import std.algorithm.iteration:map, uniq, each;
	import std.array:array;
	DynamicEntity!N[] collisionDynamicEntities = collisionConstraintPairs
		.map!(collisionConstraintPair => collisionConstraintPair.entity)
		.uniq
		.array;
	
	alias V3 = ar.Vector!(N, 3);
	collisionDynamicEntities.each!(entity => entity.bias = V3.zero);
	
	import fragments.contactpoint;
	foreach (ref collisionConstraintPair; collisionConstraintPairs) {
		auto entity = collisionConstraintPair.entity;
		
		const contactPoint = collisionConstraintPair.contactPoint;
		immutable depth = contactPoint.distance * contactPoint.normal;
		if(depth.dotProduct(entity.bias) < depth.norm){
			entity.bias = entity.bias - (depth.dotProduct(entity.bias) - depth.norm)*depth.normalized;
		}
	}
	
	collisionDynamicEntities.each!(entity => entity.position = entity.position + entity.bias * 1.001);
}

private void iterate(N)(
	in int iterations, 
	ref CollisionConstraintPair!N[] collisionConstraintPairs, 
	ref LinkConstraintPair!N[]      linkConstraintPairs,
	ref LinearImpulseConstraint!N[] linearImpulseConstraints,
){
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	import fragments.entity;
	//iteration
	for (int i = 0; i < iterations; i++) {
		//impulse
		foreach (ref linearImpulseConstraint; linearImpulseConstraints) {
			import std.conv;
			auto entity = linearImpulseConstraint.entity;
			V3[2] deltaVelocities = linearImpulseConstraint.deltaVelocities(entity);
			entity.deltaLinearVelocity  = entity.deltaLinearVelocity + deltaVelocities[0]/iterations.to!N;
			entity.deltaAngularVelocity = entity.deltaAngularVelocity + deltaVelocities[1]/iterations.to!N;
		}
		
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
	}
}

private void updateVelocitiesFromDelta(N)(DynamicEntity!(N) entity){
	entity.linearVelocity = entity.linearVelocity + entity.deltaLinearVelocity;
	entity.angularVelocity = entity.angularVelocity + entity.deltaAngularVelocity;
}

private void initDeltaVelocity(N)(DynamicEntity!(N) entity){
	alias V3 = ar.Vector!(N, 3);
	entity.deltaLinearVelocity = V3.zero;
	entity.deltaAngularVelocity = V3.zero;
}
