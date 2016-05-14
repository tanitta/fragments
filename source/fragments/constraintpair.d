module fragments.constraintpair;

import fragments.entity;
import fragments.contactpoint;
import armos;

/++
+/
template BallJoint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	public{
		LinkConstraintPair!N BallJoint(
			DynamicEntity!N entityA, DynamicEntity!N entityB, 
			in V3 applicationPointA, in V3 applicationPointB,
		){
			return new LinkConstraintPair!N(
				entityA, entityB,
				applicationPointA, applicationPointB, 
				[
					LinearLinkConstraint!N(V3(1, 0, 0)), 
					LinearLinkConstraint!N(V3(0, 1, 0)), 
					LinearLinkConstraint!N(V3(0, 0, 1)), 
				],
				[]
			);
		};
	}//public

	private{
	}//private
}//template BallJoint
unittest{
	import fragments.square;
	import fragments.material;
	alias N = double;
	alias V3 = ar.Vector!(N, 3);
	auto material = new Material!N;
	auto entityA = new Square!N(material);
	auto entityB = new Square!N(material);
	assert(__traits(compiles, {
		auto ballJoint = BallJoint!N(
			entityA, entityB,
			V3(0, 0, 1), 
			V3(0, 0, -1), 
		);
	}));
}

/++
++/
template FixedJoint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	public{
		LinkConstraintPair!N FixedJoint(
			DynamicEntity!N entityA, DynamicEntity!N entityB, 
			in V3 applicationPointA, in V3 applicationPointB,
		){
			return new LinkConstraintPair!N(
				entityA, entityB,
				applicationPointA, applicationPointB, 
				[
					LinearLinkConstraint!N(V3(1, 0, 0)), 
					LinearLinkConstraint!N(V3(0, 1, 0)), 
					LinearLinkConstraint!N(V3(0, 0, 1)), 
				],
				[
					AngularLinkConstraint!N(V3(1, 0, 0)), 
					AngularLinkConstraint!N(V3(0, 1, 0)), 
					AngularLinkConstraint!N(V3(0, 0, 1)), 
				]
			);
		};
	}//public

	private{
	}//private
}//template BallJoint

/++
+/
class LinkConstraintPair(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		/++
		++/
		this(
			DynamicEntity!N entityA, DynamicEntity!N entityB,
			in V3 localApplicationPointA, in V3 localApplicationPointB,
			LinearLinkConstraint!N[] linearLinkConstraints, 
			AngularLinkConstraint!N[] angularLinkConstraints, 
		){
			_dynamicEntities[0] = entityA;
			_dynamicEntities[1] = entityB;
			
			_localApplicationPoints[0] = localApplicationPointA;
			_localApplicationPoints[1] = localApplicationPointB;
			
			_linearLinkConstraints = linearLinkConstraints;
			_angularLinkConstraints = angularLinkConstraints;
		}
		
		/++
		++/
		DynamicEntity!N[] dynamicEntities(){
			return _dynamicEntities;
		}
		
		/++
		++/
		void update(in N unitTime){
			updateRotatedLocalApplicationPoints;
			updateMassAndInertiaTermInv;
			updateConstraints(unitTime);
		}
		
		/++
		++/
		LinearLinkConstraint!N[] linearLinkConstraints(){
			return _linearLinkConstraints;
		}
		
		/++
		++/
		AngularLinkConstraint!N[] angularLinkConstraints(){
			return _angularLinkConstraints;
		}
		
	}//public

	private{
		DynamicEntity!N[2] _dynamicEntities;
		M33 _massAndInertiaTermInv;
		V3[2] _localApplicationPoints;
		V3[2] _rotatedLocalApplicationPoints;
		
		V3 _localDirection;
		V3 _rotatedLocalDirection;
		
		LinearLinkConstraint!N[] _linearLinkConstraints;
		AngularLinkConstraint!N[] _angularLinkConstraints;
		
		void updateRotatedLocalApplicationPoints(){
			foreach (int index, ref rotatedLocalApplicationPoint; _rotatedLocalApplicationPoints) {
				rotatedLocalApplicationPoint = _dynamicEntities[index].orientation.rotatedVector(_localApplicationPoints[index]);
			}
		}
		
		void updateMassAndInertiaTermInv()in{
			import std.math;
			assert(!isNaN(_dynamicEntities[0].inertiaGlobalInv[0][0]));
			assert(!isNaN(_dynamicEntities[0].massInv));
			assert(!isNaN(_dynamicEntities[1].inertiaGlobalInv[0][0]));
			assert(!isNaN(_dynamicEntities[1].massInv));
			assert(!isNaN(_rotatedLocalApplicationPoints[0][0]));
			assert(!isNaN(_rotatedLocalApplicationPoints[1][0]));
		}body{
			_massAndInertiaTermInv = M33.zero;
			for (int i = 0; i < 2; i++) {
				_massAndInertiaTermInv = _massAndInertiaTermInv + massAndInertiaTermInv(
					_rotatedLocalApplicationPoints[i],
					_dynamicEntities[i].massInv,
					_dynamicEntities[i].inertiaGlobalInv
				);
			}
		}
		
		void updateConstraints(in N unitTime){
			
			import std.math;
			{
				immutable velocity = (_dynamicEntities[0].linearVelocity + _dynamicEntities[0].angularVelocity.vectorProduct(_rotatedLocalApplicationPoints[0]))-
				(_dynamicEntities[1].linearVelocity + _dynamicEntities[1].angularVelocity.vectorProduct(_rotatedLocalApplicationPoints[1]));
			
				immutable gain = N(0.5);
				immutable slop = N(0);
				immutable distance = (_dynamicEntities[1].position+_rotatedLocalApplicationPoints[1])-(_dynamicEntities[0].position+_rotatedLocalApplicationPoints[0]);


				foreach (ref linearLinkConstraint; _linearLinkConstraints) {
					linearLinkConstraint.update(_dynamicEntities[0].orientation, _massAndInertiaTermInv, _rotatedLocalApplicationPoints);
					linearLinkConstraint.updateBias(gain, slop, distance, unitTime);
					linearLinkConstraint.updateInitialImpulse(velocity);
				}
			}
			{
				immutable velocity = _dynamicEntities[0].angularVelocity - _dynamicEntities[1].angularVelocity;
				// immutable velocity = V3.zero;
				import std.stdio;
				writeln("ang : ", velocity.norm);
				
				immutable gain = N(0.0);
				immutable slop = N(0);
				// immutable distance = (_dynamicEntities[1].orientation)-(_dynamicEntities[0].orientation);
				immutable distance = _dynamicEntities[1].orientation;
				foreach (ref angularLinkConstraint; _angularLinkConstraints) {
					angularLinkConstraint.update(_dynamicEntities[0].orientation, _massAndInertiaTermInv, _rotatedLocalApplicationPoints);
					angularLinkConstraint.updateBias(gain, slop, -distance, unitTime);
					angularLinkConstraint.updateInitialImpulse(velocity);
				}
			}
		}
	}//private
}//class LinkConstraintPair
unittest{
	import fragments.square;
	import fragments.material;
	alias N = double;
	auto material = new Material!N;
	auto entityA = new Square!N(material);
	auto entityB = new Square!N(material);
	// assert(__traits(compiles, {
	// 	auto linkConstraintPair = new LinkConstraintPair!N(entityA, entityB);
	// }));
}
unittest{
	import fragments.square;
	import fragments.material;
	alias N = double;
	auto material = new Material!N;
	auto entityA = new Square!N(material);
	auto entityB = new Square!N(material);
	// auto linkConstraintPair = new LinkConstraintPair!N(entityA, entityB);
}

/++
+/
struct LinearLinkConstraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	alias Q = ar.Quaternion!(N);
	
	public{
		/++
		+/
		this(in V3 localDirection){
			_localDirection = localDirection;
		}
		
		/++
		+/
		V3[2][2] deltaVelocities(DynamicEntity!N[] entities)const in{
			import std.math;
			assert(!isNaN(_initialImpulse));
		}body{
			immutable V3 deltaVelocity = (entities[0].deltaLinearVelocity + entities[0].deltaAngularVelocity.vectorProduct(_applicationPoints[0]))
			- (entities[1].deltaLinearVelocity + entities[1].deltaAngularVelocity.vectorProduct(_applicationPoints[1]));
			
			import std.algorithm;
			immutable N deltaImpluse = (_initialImpulse - impulse(deltaVelocity));
			
			// immutable V3 deltaLinearVelocity = deltaImpluse * dynamicEntity.massInv * _direction;
			// immutable V3 deltaAngularVelocity = deltaImpluse * dynamicEntity.inertiaGlobalInv * _applicationPoint.vectorProduct(_rotatedDirection);
			
			immutable V3[2][2] v = [
				[
					deltaImpluse * entities[0].massInv * _rotatedDirection,
					deltaImpluse * entities[0].inertiaGlobalInv * _applicationPoints[0].vectorProduct(_rotatedDirection)
				], 
				[
					deltaImpluse * entities[1].massInv * _rotatedDirection,
					deltaImpluse * entities[1].inertiaGlobalInv * _applicationPoints[1].vectorProduct(_rotatedDirection)
				], 
			];
			return v;
		}
		
		/++
		+/
		void update(in Q orientation, in M33 massAndInertiaTermInv, in V3[] rotatedLocalApplicationPoints)in{
			import std.math;
			assert(!isNaN(orientation[0]));
			assert(!isNaN(massAndInertiaTermInv[0][0]));
		}body{
			_rotatedDirection = orientation.rotatedVector(_localDirection);
			_jacDiagInv = N(1)/((massAndInertiaTermInv*_rotatedDirection).dotProduct(_rotatedDirection));
			_applicationPoints = rotatedLocalApplicationPoints;
		}
		
		void updateInitialImpulse(in V3 velocity)in{
			import std.math;
			assert(!isNaN(velocity[0]));
		}body{
			_initialImpulse = -impulse(velocity);
		}
		
		void updateBias(in N gain, in N slop, in V3 distance, in N unitTime){
			import std.math;
			_biasTerm = (gain * (_rotatedDirection.dotProduct(distance)))/unitTime;
		};
		
		/++
		+/
		void localDirection(in V3 localDirection){
			_localDirection = localDirection;
		}
	}//public

	private{
		N _initialImpulse;
		
		V3 _localDirection;
		V3 _rotatedDirection;
		
		N _jacDiagInv;
		V3[2] _applicationPoints;
		
		N _biasTerm;
		
		N impulse(in V3 deltaVelocity)const
		in{
			import std.math;
			assert(!isNaN(deltaVelocity[0]));
		}body{
			immutable N impulse = _jacDiagInv * (_rotatedDirection.dotProduct(deltaVelocity) - _biasTerm);
			return impulse;
		}
	}//private
}//struct LinkConstraint
unittest{
	import fragments.square;
	import fragments.material;
	alias N = double;
	auto material = new Material!N;
	auto entityA = new Square!N(material);
	auto entityB = new Square!N(material);
	
	assert(__traits(compiles, {
		auto linkConstraint= LinearLinkConstraint!N();
	}));
}

/++
+/
struct AngularLinkConstraint(NumericType){
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	alias Q = ar.Quaternion!(N);
	
	public{
		/++
		+/
		this(in V3 localDirection){
			_localDirection = localDirection;
		}
		
		/++
		+/
		V3[2][2] deltaVelocities(DynamicEntity!N[] entities)const in{
			import std.math;
			assert(!isNaN(_initialImpulse));
		}out(v){
			import std.math;
			assert(!isNaN(v[0][0][0]));
			assert(!isNaN(v[0][1][0]));
			assert(!isNaN(v[1][0][0]));
			assert(!isNaN(v[1][1][0]));
		}body{
			immutable V3 deltaVelocity = (entities[0].deltaAngularVelocity - entities[1].deltaAngularVelocity);
			import std.stdio;
			entities[0].deltaAngularVelocity.writeln;
			// immutable V3 deltaVelocity = V3.zero;
			
			import std.algorithm;
			immutable N deltaImpluse = (_initialImpulse - impulse(deltaVelocity))*0.05;
			writeln("deltaImpluse : ", deltaImpluse);
			
			immutable V3[2][2] v = [
				[
					// deltaImpluse * entities[0].massInv * _applicationPoints[0].vectorProduct(_rotatedDirection),
					// deltaImpluse * entities[0].massInv * _rotatedDirection,
					V3.zero, 
					deltaImpluse * entities[0].inertiaGlobalInv * _rotatedDirection,
					// -deltaImpluse * entities[0].inertiaGlobalInv * _applicationPoints[0].vectorProduct(_rotatedDirection)
				], 
				[
					// deltaImpluse * entities[1].massInv * _applicationPoints[1].vectorProduct(_rotatedDirection),
					// deltaImpluse * entities[1].massInv * _rotatedDirection,
					V3.zero,
					deltaImpluse * entities[1].inertiaGlobalInv * _rotatedDirection,
					// -deltaImpluse * entities[1].inertiaGlobalInv * _applicationPoints[1].vectorProduct(_rotatedDirection)
				], 
			];
				
			// immutable V3[2][2] v = [
			// 	[
			// 		V3.zero, 
			// 		V3.zero, 
			// 	], 
			// 	[
			// 		V3.zero, 
			// 		V3.zero, 
			// 	], 
			// ];
			return v;
		}
		
		/++
		+/
		void update(in Q orientation, in M33 massAndInertiaTermInv, in V3[] rotatedLocalApplicationPoints)in{
			import std.math;
			assert(!isNaN(orientation[0]));
			assert(!isNaN(massAndInertiaTermInv[0][0]));
		}body{
			_rotatedDirection = orientation.rotatedVector(_localDirection);
			_jacDiagInv = N(1)/((massAndInertiaTermInv*_rotatedDirection).dotProduct(_rotatedDirection));
			_applicationPoints = rotatedLocalApplicationPoints;
		}
		
		void updateInitialImpulse(in V3 velocity)in{
			import std.math;
			assert(!isNaN(velocity[0]));
		}body{
			_initialImpulse = -impulse(velocity);
		}
		
		void updateBias(in N gain, in N slop, in Q distance, in N unitTime){
			import std.math;
			// _biasTerm = (gain * (_rotatedDirection.dotProduct(distance)))/unitTime;
			_biasTerm = N(0);
			import std.stdio;
			writeln("bias : ", _biasTerm);
			// import std.stdio;
			// _biasTerm.writeln;
		};
		
		/++
		+/
		void localDirection(in V3 localDirection){
			_localDirection = localDirection;
		}
	}//public

	private{
		N _initialImpulse;
		
		V3 _localDirection;
		V3 _rotatedDirection;
		
		N _jacDiagInv;
		V3[2] _applicationPoints;
		
		N _biasTerm;
		
		N impulse(in V3 deltaVelocity)const
		in{
			import std.math;
			assert(!isNaN(deltaVelocity[0]));
		}body{
			immutable N impulse = _jacDiagInv * (_rotatedDirection.dotProduct(deltaVelocity) - _biasTerm);
			return impulse;
		}
	}//private
}//struct LinkConstraint

/++
+/
struct LinearImpulseConstraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	
	public{
		/// 
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
			immutable V3 deltaLinearVelocity = _impulse * dynamicEntity.massInv;
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
}//struct LinearImpulseConstraint

/++
+/
struct AngularImpulseConstraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	
	public{
		/// 
		this(DynamicEntity!N dynamicEntity, in V3 impulse){
			this.entity = dynamicEntity; 
			_impulse = impulse;
		}
		
		/// 
		V3[2] deltaVelocities(DynamicEntity!N dynamicEntity)
		in{
			assert(dynamicEntity);
			import std.math;
			assert(!isNaN(dynamicEntity.deltaAngularVelocityVelocity[0]));
		}body{
			immutable V3 deltaAngularVelocity = _impulse * dynamicEntity.inertiaGlobalInv;
			immutable V3[2] v = [
				V3.zero,
				deltaAngularVelocity
			];
			return v;
		};
		
		DynamicEntity!N entity;
	}//public

	private{
		V3 _impulse;
		N _initialImpulse;
	}//private
}//struct AngularImpulseConstraint

/++
+/
struct CollisionConstraintPair(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		this(
			ref DynamicEntity!N dynamicEntity,
			ref ContactPoint!N contactPoint,
		){
			_dynamicEntity = dynamicEntity;
			
			auto staticEntity = contactPoint.staticEntity;
			
			immutable V3 applicationPoint = contactPoint.applicationPoint - dynamicEntity.position;
			immutable relativeVelocity = dynamicEntity.linearVelocity + dynamicEntity.angularVelocity.vectorProduct(applicationPoint);
				
			immutable bias = 0.0;
			immutable slop = N(0);
			import std.math;
			immutable unitTime = 1.0/30.0;
			// immutable biasTerm = (bias * fmax(N(0), contactPoint.distance+slop))/unitTime;
			immutable biasTerm = N(0);

			immutable jacDiagInv = jacDiagInv(
				applicationPoint,
				dynamicEntity.massInv,
				dynamicEntity.inertiaGlobalInv,
				staticEntity.normal
			);
			
			//set constraints
			collisionConstraint = CollisionConstraint!N(
				relativeVelocity,
				jacDiagInv, 
				applicationPoint, 
				staticEntity.normal,
				biasTerm,
			);
			
			// friction constraint
			{
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
		}
		
		this(DynamicEntity!N dynamicEntity){
			_dynamicEntity = dynamicEntity;
		}
		
		void update(
			in ContactPoint!(N)[] contactPoints,
		){
			_isColliding = contactPoints.length > 0;
			_depth = V3.zero;
			
			if(isColliding){
				const contactPoint = contactPoints[0];
				
				immutable V3 applicationPoint = contactPoint.applicationPoint - _dynamicEntity.position;
				immutable relativeVelocity = _dynamicEntity.linearVelocity + _dynamicEntity.angularVelocity.vectorProduct(applicationPoint);

				// immutable bias = 0.0;
				// immutable slop = N(0);
				// import std.math;
				// immutable biasTerm = (bias * fmax(N(0), contactPoint.distance+slop))/unitTime;
				immutable biasTerm = N(0);

				const staticEntity = contactPoints[0].staticEntity;
				
				immutable jacDiagInv = jacDiagInv(
					applicationPoint,
					_dynamicEntity.massInv,
					_dynamicEntity.inertiaGlobalInv,
					staticEntity.normal
				);

				//set constraints
				collisionConstraint = CollisionConstraint!N(
					relativeVelocity,
					jacDiagInv, 
					applicationPoint, 
					staticEntity.normal,
					biasTerm,
				);
				updateFrictionConstraints(staticEntity, relativeVelocity, applicationPoint, jacDiagInv);
				
				_depth = contactPoint.distance * contactPoint.normal;
			}
		};
		
		bool isColliding()const{return _isColliding;}
		
		/++
		+/
		DynamicEntity!(N) entity(){return _dynamicEntity;};
		
		/++
		+/
		CollisionConstraint!(N) collisionConstraint;
		
		/++
		+/
		V3 depth()const{
			return _depth;
		};
		
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
		DynamicEntity!(N) _dynamicEntity;
		
		V3 _depth;
		
		bool _isColliding;
		
		N _staticFriction;
		N _dynamicFriction;
		
		void updateFrictionConstraints(
			in StaticEntity!N staticEntity,
			in V3 relativeVelocity,
			in V3 applicationPoint,
			in N jacDiagInv,
		){
			_staticFriction = (_dynamicEntity.material.staticFriction * staticEntity.material.staticFriction)^^N(0.5);
			_dynamicFriction = (_dynamicEntity.material.dynamicFriction * staticEntity.material.dynamicFriction)^^N(0.5);

			V3[2] frictionAxes;
			if(relativeVelocity.vectorProduct(staticEntity.normal).norm > 0){
				frictionAxes[0] = relativeVelocity.vectorProduct(staticEntity.normal).normalized;
				frictionAxes[1] = frictionAxes[0].vectorProduct(staticEntity.normal);
			}else{
				frictionAxes[0] = (staticEntity.vertices[1] - staticEntity.vertices[0]).normalized;
				frictionAxes[1] = frictionAxes[0].vectorProduct(staticEntity.normal);
			}

			foreach (int index, frictionAxis; frictionAxes) {
				frictionConstraints[index] = FrictionConstraint!N(
					relativeVelocity,
					jacDiagInv,
					applicationPoint,
					frictionAxis,
				);
			}
		}
	}//private
}//struct CollisionConstraintPair

/++
+/
struct CollisionConstraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	public{
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
			// immutable N oldImpulse = _accumImpulse;
			// _accumImpulse = (oldImpulse + (_initialImpulse - impulse(deltaVelocity)).clamp(0, N.max)).clamp(0, N.max);
			// immutable N deltaImpluse = _accumImpulse - oldImpulse;
			
			immutable N deltaImpluse = (_initialImpulse - impulse(deltaVelocity)).clamp(N(0), N.max);
			
			// import std.stdio;
			// _accumImpulse.writeln;
			
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
		N _accumImpulse = N(0);
		
		N impulse(in V3 deltaVelocity)const
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
		V3[2] deltaVelocities(DynamicEntity!N dynamicEntity)const
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

private N jacDiagInv(N, V3 = ar.Vector!(N, 3), M33 = ar.Matrix!(N, 3, 3))(
	in V3 applicationPoint,
	in N massInv,
	in M33 inertiaGlobalInv, 
	in V3 normal, 
)in{
	import std.math;
	assert(!isNaN(applicationPoint[0]));
	assert(!isNaN(massInv));
	assert(!isNaN(inertiaGlobalInv[0][0]));
	assert(!isNaN(normal[0]));
}body{
	immutable M33 k = massAndInertiaTermInv(
		applicationPoint,
		massInv,
		inertiaGlobalInv, 
	);
	return N(1)/((k * normal).dotProduct(normal));
}

private M33 massAndInertiaTermInv(N, V3 = ar.Vector!(N, 3), M33 = ar.Matrix!(N, 3, 3))(
	in V3 applicationPoint,
	in N massInv,
	in M33 inertiaGlobalInv, 
){
	immutable rCrossMatrix = applicationPoint.crossMatrix;
	return massInv * M33.identity - rCrossMatrix * inertiaGlobalInv * rCrossMatrix;
}

private M33 massAndInertiaTermInv(N, V3 = ar.Vector!(N, 3), M33 = ar.Matrix!(N, 3, 3))(
	in V3 applicationPointA,
	in N massInvA,
	in M33 inertiaGlobalInvA, 
	in V3 applicationPointB,
	in N massInvB,
	in M33 inertiaGlobalInvB, 
){
	return massAndInertiaTermInv(applicationPointA, massInvA, inertiaGlobalInvA) + massAndInertiaTermInv(applicationPointB, massInvB, inertiaGlobalInvB);
}

private M33 crossMatrix(V3, M33 = ar.Matrix!(typeof(V3[0]), 3, 3))(in V3 vector){
	return M33(
		[0,                    -vector[2], vector[1] ],
		[vector[2],  0,                    -vector[0]],
		[-vector[1], vector[0],  0                   ],
	);
}
