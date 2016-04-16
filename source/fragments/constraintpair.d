module fragments.constraintpair;

import fragments.entity;
import fragments.contactpoint;
import armos;

/++
+/
class LinkConstraintPair(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	
	public{
		this(){}
	}//public

	private{
		DynamicEntity!N[2] _dynamicEntities;
		LinkConstraint!N[6] _linkConstraints;
	}//private
}//class LinkConstraintPair

/++
+/
struct LinkConstraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	
	public{
	}//public

	private{
	}//private
}//struct LinkConstraint



/++
+/
struct LinearImpulseConstraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	
	public{
		this(DynamicEntity!N dynamicEntity, in V3 impulse){
			this.entity = dynamicEntity; 
			_impulse = impulse;
		}
		/// 
		V3[2] deltaVelocities(DynamicEntity!N dynamicEntity)
		in{
			assert(dynamicEntity);
			import std.math;
			assert(!isNaN(dynamicEntity.deltaLinearVelocity[0]));
		}body{
			immutable V3 deltaLinearVelocity = -_impulse * dynamicEntity.massInv;
			immutable V3[2] v = [
				deltaLinearVelocity,
				V3.zero,
			];
			return v;
		};
		
		DynamicEntity!N entity;
	}//public

	private{
		V3 _impulse;
		N _initialImpulse;
	}//private
}//struct ForceConstraint

/++
+/
// template forceConstraintPair(NumericType) {
// 	alias N = NumericType;
// 	alias V3 = ar.Vector!(N, 3);
// 	public{
// 		ConstraintPair!N forceConstraintPair(DynamicEntity!N entity, in V3 force){
// 			auto constraintPair = ConstraintPair!N(entity);
// 			// constraintPair.linearConstraints[0] = Constraint!N(
// 			// 	force.normalized
// 			// );
// 			return constraintPair;
// 		}
// 	}//public
//
// 	private{
// 	}//private
// }//template forceConstraintPair
unittest{
	alias N = double;
	alias V3 = ar.Vector!(N, 3);
	
	import fragments.material;
	auto material = new Material!N;
	
	import fragments.square;
	auto entity = new Square!N(material);
	
	// assert(__traits(compiles,(){
	// 	ConstraintPair!N forceConstraint = forceConstraintPair!N(entity, V3.zero);
	// }));
}


/++
++/
// struct ConstraintPair(NumericType) {
// 	alias N = NumericType;
// 	alias V3 = ar.Vector!(N, 3);
//	
// 	public{
// 		this(DynamicEntity!(N) entityA, DynamicEntity!(N) entityB = null){
// 			_entities[0] = entityA;
// 			_entities[1] = entityB;
// 		}
//		
// 		DynamicEntity!(N)[2] entities(){return _entities;};
//		
// 		Constraint!(N)[3] linearConstraints;
// 		Constraint!(N)[3] angularConstraints;
//		
// 	}//public
//
// 	private{
// 		DynamicEntity!(N)[2] _entities;
// 	}//private
// }//class ConstraintPair

/++
+/
// struct Constraint(NumericType) {
// 	alias N = NumericType;
// 	alias V3 = ar.Vector!(N, 3);
// 	alias ImpulseFunction = N delegate(V3 deltaVelocity);
//	
// 	public{
// 		// ContactPoint!(N) contactPoint;
// 		this(
// 			in V3 initialDeltaVelocity,
// 			in V3 applicationPoint, 
// 			in V3 axis,
// 			in N biasTerm, 
// 			in ImpulseFunction impulseFunction,
// 		){
// 			_impulseFunction = impulseFunction;
// 			_axis = axis;
// 			_initialImpulse = _impulseFunction(initialDeltaVelocity);
// 		}
//		
// 		@property{
// 			V3 axis(){return _axis;};
// 			ImpulseFunction _impulseFunction;
// 			V3 initialImpulse(){return _initialImpulse;};
// 		}
// 	}//public
//
// 	private{
// 		V3 _axis;
// 		V3 _initialImpulse;
// 	}//private
// }//struct Constraint

/++
+/
struct CollisionConstraintPair(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		/++
		+/
		this(
			ref DynamicEntity!N dynamicEntity,
			ref StaticEntity!N staticEntity,
			ref ContactPoint!(N) contactPoint,
			ref N unitTime, 
		){
			_entity = dynamicEntity;
			_contactPoint = contactPoint;
			
			//set constraints
			immutable V3 applicationPoint = contactPoint.applicationPoint - dynamicEntity.position;
			
			immutable relativeVelocity = dynamicEntity.linearVelocity + dynamicEntity.angularVelocity.vectorProduct(applicationPoint);
			
			
			import std.math;
			immutable bias = 0.00;
			immutable slop = N(0.0);
			immutable biasTerm = (bias * fmax(N(0), contactPoint.distance+slop))/unitTime;
			
			immutable jacDiagInv = this.jacDiagInv(
				applicationPoint,
				dynamicEntity.massInv,
				dynamicEntity.inertiaGlobalInv,
				staticEntity.normal
			);
			
			collisionConstraint = CollisionConstraint!N(
				relativeVelocity,
				jacDiagInv, 
				applicationPoint, 
				staticEntity.normal,
				biasTerm,
			);
			
			// friction constraint
			// _dynamicFriction = dynamicEntity.material.
			
			_staticFriction = (dynamicEntity.material.staticFriction * staticEntity.material.staticFriction)^^N(0.5);
			_dynamicFriction = (dynamicEntity.material.dynamicFriction * staticEntity.material.dynamicFriction)^^N(0.5);
			
			
			V3[2] frictionAxes;
			frictionAxes[0] = (staticEntity.vertices[1] - staticEntity.vertices[0]).normalized;
			frictionAxes[1] = frictionAxes[0].vectorProduct(staticEntity.normal);
			
			foreach (int index, frictionAxis; frictionAxes) {
				frictionConstraints[index] = FrictionConstraint!N(
					relativeVelocity,
					jacDiagInv,
					applicationPoint,
					frictionAxis,
				);
			}
		}
		
		/++
		+/
		ContactPoint!N contactPoint(){return _contactPoint;}
		
		/++
		+/
		DynamicEntity!(N) entity(){return _entity;};
		
		/++
		+/
		CollisionConstraint!(N) collisionConstraint;
		
		/++
		+/
		N dynamicFriction()const{
			return _dynamicFriction;
		}
		
		/++
		+/
		N staticFriction()const{
			return _staticFriction;
		}
		
		FrictionConstraint!(N)[2] frictionConstraints;
	}//public

	private{
		DynamicEntity!(N) _entity;
		ContactPoint!(N) _contactPoint;
		
		N _staticFriction;
		N _dynamicFriction;
		
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
			immutable rCrossMatrix = M33(
				[0,                    -applicationPoint[2], applicationPoint[1] ],
				[applicationPoint[2],  0,                    -applicationPoint[0]],
				[-applicationPoint[1], applicationPoint[0],  0                   ],
			);
			immutable M33 k = massInv * M33.identity - rCrossMatrix * inertiaGlobalInv * rCrossMatrix;
			
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
			in V3 velocity,
			in N jacDiagInv,
			in V3 applicationPoint, 
			in V3 direction, 
			in N biasTerm, 
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
			immutable V3 deltaVelocity = dynamicEntity.deltaLinearVelocity + dynamicEntity.deltaAngularVelocity.vectorProduct(_applicationPoint);
			
			import std.algorithm;
			immutable N deltaImpluse = (_initialImpulse - impulse(deltaVelocity)).clamp(0, N.nan);
			_currentImpulse = deltaImpluse;
			
			immutable V3 deltaLinearVelocity = deltaImpluse * dynamicEntity.massInv * _direction;
			immutable V3 deltaAngularVelocity = deltaImpluse * dynamicEntity.inertiaGlobalInv * _applicationPoint.vectorProduct(_direction);
			
			immutable V3[2] v = [
				deltaLinearVelocity,
				deltaAngularVelocity,
			];
			return v;
		};
		
		N currentImpulse()const{
			return _currentImpulse;
		};
	}//public

	private{
		N _jacDiagInv;
		V3 _applicationPoint;
		V3 _direction;
		N _biasTerm;
		
		N _initialImpulse;
		
		N impulse(V3 deltaVelocity)const
		in{
			import std.math;
			assert(!isNaN(deltaVelocity[0]));
		}body{
			immutable N impulse = _jacDiagInv * (_direction.dotProduct(deltaVelocity) - _biasTerm);
			
			return impulse;
		}
		
		N _currentImpulse = N(0);
	}//private
}//struct CollisionConstraint

/++
+/
struct FrictionConstraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	public{
		this(
			in V3 velocity,
			in N jacDiagInv,
			in V3 applicationPoint, 
			in V3 direction, 
		)in{
			import std.math;
			assert(!isNaN(velocity[0]));
			assert(!isNaN(jacDiagInv));
			assert(!isNaN(applicationPoint[0]));
			assert(!isNaN(direction[0]));
		}body{
			_jacDiagInv = jacDiagInv;
			_applicationPoint = applicationPoint;
			_direction= direction;
			
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
			immutable V3 deltaVelocity = dynamicEntity.deltaLinearVelocity + dynamicEntity.deltaAngularVelocity.vectorProduct(_applicationPoint);
			
			import std.algorithm;
			immutable N deltaImpluse = (_initialImpulse - impulse(deltaVelocity)).clamp(lowerLimit, upperLimit);
			
			immutable V3 deltaLinearVelocity = deltaImpluse * dynamicEntity.massInv * _direction;
			immutable V3 deltaAngularVelocity = deltaImpluse * dynamicEntity.inertiaGlobalInv * _applicationPoint.vectorProduct(_direction);
			
			immutable V3[2] v = [
				deltaLinearVelocity,
				deltaAngularVelocity,
			];
			return v;
		};
		
		N lowerLimit;
		N upperLimit;
	}//public

	private{
		N _jacDiagInv;
		V3 _applicationPoint;
		V3 _direction;
		
		N _initialImpulse;
		
		N impulse(in V3 deltaVelocity)const
		in{
			import std.math;
			assert(!isNaN(deltaVelocity[0]));
		}body{
			immutable N impulse = _jacDiagInv * (_direction.dotProduct(deltaVelocity));
			
			return impulse;
		}
	}//private
}//struct CollisionConstraint
