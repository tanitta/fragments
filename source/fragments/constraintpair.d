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
			in V3 direction
		){
			return new LinkConstraintPair!N(entityA, entityB, applicationPointA, applicationPointB, direction);
		};
	}//public

	private{
	}//private
}//template BallJoint
unittest{
	import fragments.square;
	import fragments.material;
	alias N = double;
	auto material = new Material!N;
	auto entityA = new Square!N(material);
	auto entityB = new Square!N(material);
	// assert(__traits(compiles, {
	// 	auto ballJoint = BallJoint!N(entityA, entityB);
	// }));
}

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
			in V3 localDirection
		){
			_dynamicEntities[0] = entityA;
			_dynamicEntities[1] = entityB;
			
			_localApplicationPoints[0] = localApplicationPointA;
			_localApplicationPoints[1] = localApplicationPointB;
		}
		
		/++
		++/
		DynamicEntity!N[] dynamicEntities(){
			return _dynamicEntities;
		}
		
		/++
		++/
		void update(){
			updateRotatedLocalApplicationPoints;
			updateMassAndInertiaTermInv;
			updateConstraints;
		}
		
		/++
		++/
		void addLinearLinkConstraint(LinkConstraint!N[] linearConstraint){
			_linearLinkConstraints ~= linearConstraint;
		}
		
		/++
		++/
		void addAngularLinkConstraint(LinkConstraint!N[] angularConstraint){
			_angularLinkConstraints ~= angularConstraint;
		}
		
		/++
		++/
		LinkConstraint!N[] linearLinkConstraints(){
			return _linearLinkConstraints;
		}
		
		/++
		++/
		LinkConstraint!N[] angularLinkConstraints(){
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
		
		LinkConstraint!N[] _linearLinkConstraints;
		LinkConstraint!N[] _angularLinkConstraints;
		
		void updateRotatedLocalApplicationPoints(){
			foreach (int index, ref rotatedLocalApplicationPoint; _rotatedLocalApplicationPoints) {
				rotatedLocalApplicationPoint = _dynamicEntities[index].orientation.rotatedVector(_localApplicationPoints[index]);
			}
		}
		
		void updateMassAndInertiaTermInv(){
			_massAndInertiaTermInv = M33.zero;
			for (int i = 0; i < 2; i++) {
				_massAndInertiaTermInv = _massAndInertiaTermInv + massAndInertiaTermInv(
					_rotatedLocalApplicationPoints[i],
					_dynamicEntities[i].massInv,
					_dynamicEntities[i].inertiaGlobalInv
				);
			}
		}
		
		void updateConstraints(){
			foreach (linearLinkConstraint; _linearLinkConstraints) {
				linearLinkConstraint.update(_rotatedLocalDirection, _massAndInertiaTermInv);
			}
			foreach (angularLinkConstraint; _angularLinkConstraints) {
				angularLinkConstraint.update(_rotatedLocalDirection, _massAndInertiaTermInv);
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
struct LinkConstraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		this(V3 rotatedConstraintDirection, in M33 massAndInertiaTermInv){
			_direction = rotatedConstraintDirection;
			_massAndInertiaTermInv = massAndInertiaTermInv;
			_jacDiagInv = N(1)/((_massAndInertiaTermInv*_direction).dotProduct(_direction));
		}
		
		V3[2] deltaVelocities(DynamicEntity!N entityA, DynamicEntity!N entityB){
			immutable V3[2] v = [
				V3.zero, 
				V3.zero, 
			];
			return v;
		}
		
		void update(in V3 rotatedConstraintDirection, in M33 massAndInertiaTermInv){
			_direction = rotatedConstraintDirection;
			_massAndInertiaTermInv = massAndInertiaTermInv;
			_jacDiagInv = N(1)/((_massAndInertiaTermInv*_direction).dotProduct(_direction));
		}
	}//public

	private{
		N _initialImpulse;
		V3 _direction;
		M33 _massAndInertiaTermInv;
		N _jacDiagInv;
		V3[2] _applicationPoints;
		
		N impulse(V3 deltaVelocity)const
		in{
			import std.math;
			assert(!isNaN(deltaVelocity[0]));
		}body{
			immutable N impulse = _jacDiagInv * _direction.dotProduct(deltaVelocity);
			
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
		auto linkConstraint= LinkConstraint!N();
	}));
}

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
				
				// friction constraint
				{
					_staticFriction = (_dynamicEntity.material.staticFriction * staticEntity.material.staticFriction)^^N(0.5);
					_dynamicFriction = (_dynamicEntity.material.dynamicFriction * staticEntity.material.dynamicFriction)^^N(0.5);

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
