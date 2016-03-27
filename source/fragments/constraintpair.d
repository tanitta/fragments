module fragments.constraintpair;

import fragments.entity;
import fragments.contactpoint;
import armos;

/++
++/
struct ConstraintPair(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	
	public{
		this(DynamicEntity!(N) entityA, DynamicEntity!(N) entityB = null){
			_entities[0] = entityA;
			_entities[1] = entityB;
		}
		
		DynamicEntity!(N)[2] entities(){return _entities;};
		
		Constraint!(N)[3] linearConstraints;
		Constraint!(N)[3] angularConstraints;
		
	}//public

	private{
		DynamicEntity!(N)[2] _entities;
	}//private
}//class ConstraintPair

/++
+/
struct Constraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias ConstraintCondition = V3[2] delegate(ConstraintPair!N);
	public{
		// ContactPoint!(N) contactPoint;
		this(
			in V3 axis,
			in V3 initialImpulse,
			in ConstraintCondition constraintCondition,
		){
			_axis = axis;
			_initialImpulse = initialImpulse;
			_constraintCondition = constraintCondition;
		}
		
		@property{
			V3 axis(){return _axis;};
			V3 initialImpulse(){return _initialImpulse;};
			ConstraintCondition constraintCondition(){return _constraintCondition;};
		}
	}//public

	private{
		V3 _axis;
		V3 _initialImpulse;
		ConstraintCondition _constraintCondition;
	}//private
}//struct Constraint

/++
+/
struct CollisionConstraintPair(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		/// 
		this(
			ref DynamicEntity!N dynamicEntity,
			ref StaticEntity!N staticEntity,
			ref ContactPoint!(N) contactPoint,
			ref N unitTime, 
		){
			_entity = dynamicEntity;
			
			//set constraints
			V3 applicationPoint = contactPoint.coordination - dynamicEntity.position;
			
			immutable relativeVelocity = dynamicEntity.linearVelocity + dynamicEntity.angularVelocity.vectorProduct(applicationPoint);
			
			import std.math;
			immutable bias = 0.1;
			immutable slop = N(0);
			import std.stdio;
			contactPoint.distance.writeln;
			immutable biasTerm = (bias * fmax(N(0), contactPoint.distance+slop))/unitTime;
			
			collisionConstraint = CollisionConstraint!N(
				relativeVelocity,
				jacDiagInv(
					applicationPoint,
					dynamicEntity.massInv,
					dynamicEntity.inertiaGlobalInv,
					staticEntity.normal
					),
				applicationPoint, 
				staticEntity.normal,
				biasTerm,
			);
			
			// friction constraint
			// 	(staticEntity.vertices[1] - staticEntity.vertices[0]).normalized, 
			
			// friction constraint
			// 	constraints[0].axis.vectorProduct(constraints[1].axis), 
		}
		
		DynamicEntity!(N) entity(){return _entity;};
		
		CollisionConstraint!(N) collisionConstraint;
	}//public

	private{
		DynamicEntity!(N) _entity;
		
		N jacDiagInv(
			V3 applicationPoint,
			N massInv,
			M33 inertiaGlobalInv, 
			V3 normal, 
		)const in{
			import std.math;
			assert(!isNaN(applicationPoint[0]));
			assert(!isNaN(massInv));
			assert(!isNaN(inertiaGlobalInv[0][0]));
			assert(!isNaN(normal[0]));
		}body{
			M33 k;
			{
				auto rCrossMatrix = M33(
					[0,                    -applicationPoint[2], applicationPoint[1] ],
					[applicationPoint[2],  0,                    -applicationPoint[0]],
					[-applicationPoint[1], applicationPoint[0],  0                   ],
				);
				k = massInv * M33.identity - rCrossMatrix * inertiaGlobalInv * rCrossMatrix;
			}
			
			return N(1)/((k * normal).dotProduct(normal));
		}
	}//private
}//struct CollisionConstraintPair

/++
+/
struct CollisionConstraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	public{
		/// 
		this(
			V3 velocity,
			N jacDiagInv,
			V3 applicationPoint, 
			V3 direction, 
			N biasTerm, 
		)in{
			import std.math;
			assert(!isNaN(velocity[0]));
			assert(!isNaN(jacDiagInv));
			assert(!isNaN(applicationPoint[0]));
			assert(!isNaN(direction[0]));
			assert(!isNaN(biasTerm));
		}body{
			_jacDiagInv = jacDiagInv;
			_applicationPoint = applicationPoint;
			_direction = direction;
			_biasTerm = biasTerm;
			
			_initialImpulse = -impulse(velocity);
		}
		
		/// 
		V3[2] deltaVelocities(DynamicEntity!N dynamicEntity)
		in{
			assert(dynamicEntity);
			import std.math;
			assert(!isNaN(dynamicEntity.deltaLinearVelocity[0]));
			assert(!isNaN(dynamicEntity.deltaAngularVelocity[0]));
			assert(!isNaN(_applicationPoint[0]));
		}body{
			V3 deltaVelocity = dynamicEntity.deltaLinearVelocity + dynamicEntity.deltaAngularVelocity.vectorProduct(_applicationPoint);
			
			import std.algorithm;
			N deltaImpluse = (_initialImpulse - impulse(deltaVelocity)).clamp(0, N.nan);
			
			V3 deltaLinearVelocity = deltaImpluse * dynamicEntity.massInv * _direction;
			V3 deltaAngularVelocity = deltaImpluse * dynamicEntity.inertiaGlobalInv * _applicationPoint.vectorProduct(_direction);
			
			V3[2] v = [
				deltaLinearVelocity,
				deltaAngularVelocity,
			];
			return v;
		};
	}//public

	private{
		N _jacDiagInv;
		V3 _applicationPoint;
		V3 _direction;
		N _biasTerm;
		
		N _initialImpulse;
		
		N impulse(V3 deltaVelocity)
		in{
			import std.math;
			assert(!isNaN(deltaVelocity[0]));
		}body{
			N impulse = _jacDiagInv * (_direction.dotProduct(deltaVelocity) - _biasTerm);
			
			return impulse;
		}
	}//private
}//struct CollisionConstraint

/++
+/
struct FrictionConstraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias ConstraintCondition = V3[2] delegate(ConstraintPair!N);
	public{
		this(
			N jacDiagInv,
			V3 aplicationPoint, 
			V3 direction, 
		){
		}
		
		V3[2] deltaVelocities(){
			V3[2] v = [V3.zero, V3.zero];
			return v;
		};
		
		N lowerLimit;
		N upperLimit;
	}//public

	private{
		V3 _initialImpulse;
	}//private
}//struct CollisionConstraint
