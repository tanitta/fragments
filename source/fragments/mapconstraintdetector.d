module fragments.mapconstraintdetector;

import fragments.constraintpair;
import fragments.contactpoint;
import fragments.entity;
import fragments.polygon;
import fragments.boundingbox;
import armos;

/++
++/
class MapConstraintDetector(NumericType){
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias M33 = ar.Matrix!(N, 3, 3);
	public{
		/++
		++/
		void setStaticEntities(ref StaticEntity!(N)[] staticEntities){
			_root = AABBNode!(N)(staticEntities);
		}
		
		/++
		++/
		CollisionConstraintPair!(N)[] detectCollisionConstraintPairs(DynamicEntity!(N)[] dynamicEntities){
			// broad phase
			import std.algorithm;
			CollidablePair!(N)[] collidablePairs;
			detectCollidablePairs(collidablePairs, dynamicEntities);
			
			return generatedCollidableConstraintPairs(collidablePairs);
		}
		
		void draw(){
			ar.pushStyle;
			_root.draw;
			ar.popStyle;
		}
		
		void unitTime(N t){_unitTime = t;}
	}//public

	private{
		AABBNode!N _root;
		
		N _unitTime;
		
		CollidablePair!N[] detectCollidablePairs(ref CollidablePair!(N)[] collidablePairs, DynamicEntity!(N)[] dynamicEntities){
			foreach (dynamicEntity; dynamicEntities) {
				foreach (polygon; _root.detectCollidableStaticEntities(dynamicEntity.boundingBox)) {
					collidablePairs ~= CollidablePair!(N)(dynamicEntity, polygon);
				}
			}
			return collidablePairs;
		}
		
		CollisionConstraintPair!N[] generatedCollidableConstraintPairs(ref CollidablePair!(N)[] collidablePairs){
			CollisionConstraintPair!(N)[] collisionConstraintPairs;
			foreach (collidablePair; collidablePairs) {
				foreach (contactPoint; collidablePair.dynamicEntity.contactPoints(collidablePair.staticEntity)) {
					collisionConstraintPairs ~= CollisionConstraintPair!N(
						collidablePair.dynamicEntity,
						collidablePair.staticEntity,
						contactPoint,
						_unitTime,
					);
				}
			}
			return collisionConstraintPairs;
		}
		
		// CollisionConstraintPair!N generatedCollidableConstraintPair(ref CollidablePair!(N) collidablePair, ref ContactPoint!(N) contactPoint)in{assert(_unitTime > N(0));}body{
		// 	alias M33 = ar.Matrix!(N, 3, 3);
		//		
		// 	CollisionConstraintPair!N collisionConstraintPair = CollisionConstraintPair!N(collidablePair.dynamicEntity);
		//	
		// 	V3 r = contactPoint.coordination - collidablePair.dynamicEntity.position;
		//	
		// 	M33 k;
		// 	{
		// 		auto rCrossMatrix = M33(
		// 			[0,     -r[2], r[1] ],
		// 			[r[2],  0,     -r[0]],
		// 			[-r[1], r[0],  0    ],
		// 			);
		// 		k = (collidablePair.dynamicEntity.massInv + N(0)) * M33.identity - rCrossMatrix * collidablePair.dynamicEntity.inertiaGlobalInv * rCrossMatrix;
		// 	}
		//	
		// 	N bias = contactPoint.distance / _unitTime;
		//	
		// 	collisionConstraintPair.linearConstraints[0] = collisionConstraint!(0)(
		// 		collidablePair, 
		// 		collisionConstraintPair, 
		// 		contactPoint,
		// 		bias,
		// 		k, 
		// 	);
		//	
		// 	collisionConstraintPair.linearConstraints[1] = collisionConstraint!(1)(
		// 		collidablePair, 
		// 		collisionConstraintPair, 
		// 		contactPoint,
		// 		bias,
		// 		k, 
		// 	);
		//	
		// 	collisionConstraintPair.linearConstraints[2] = collisionConstraint!(2)(
		// 		collidablePair, 
		// 		collisionConstraintPair, 
		// 		contactPoint,
		// 		bias,
		// 		k, 
		// 	);
		//	
		// 	return constraintPair;
		// }
		
		Constraint!N collisionConstraint(int Axis)(
			ref CollidablePair!(N) collidablePair,
			ref ConstraintPair!(N) constraintPair,
			ref ContactPoint!(N) contactPoint,
			ref N bias,
			ref M33 k
		)if(0 <= Axis || Axis <= 2){
			auto initialImpulse = V3.zero;
			
			static if(Axis == 0){
				return Constraint!N(
					(collidablePair.staticEntity.vertices[1] - collidablePair.staticEntity.vertices[0]).normalized, 
					initialImpulse, 
					(ConstraintPair!N constraintPair){
						V3[2] v = [V3.zero, V3.zero];
						return v;
					}
				);
			}else static if(Axis == 1){
				return Constraint!N(
					constraintPair.linearConstraints[0].axis.vectorProduct(constraintPair.linearConstraints[1].axis), 
					initialImpulse, 
					(ConstraintPair!N constraintPair){
						V3[2] v = [V3.zero, V3.zero];
						return v;
					}
				);
			}else{
				return Constraint!N(
					constraintPair.linearConstraints[0].axis.vectorProduct(constraintPair.linearConstraints[1].axis), 
					initialImpulse, 
					(ConstraintPair!N constraintPair){
						V3[2] v = [V3.zero, V3.zero];
						return v;
					}
				);
			}
		}
	}//private
}//class MapConstraintDetector

private CollidablePair!(N)[] broadPhase(N)(in DynamicEntity!(N)[] dynamicEntities, in AABBNode!(N) node){
	CollidablePair!(N)[] array;
	return array;
}

private ConstraintPair!(N)[] narrowPhase(N)(CollidablePair!(N)[] collidablePairs){
	ConstraintPair!(N)[] array;
	return array;
}

private struct CollidablePair(NumericType) {
	alias N = NumericType;
	public{
		this(DynamicEntity!(N) d, StaticEntity!(N) s){
			dynamicEntity = d;
			staticEntity = s;
		}
		
		DynamicEntity!(N) dynamicEntity;
		StaticEntity!(N) staticEntity;
	}//public

	private{
	}//private
}//struct CollidablePair

private struct AABBNode(NumericType){
	alias N = NumericType;
	public{
		/++
		++/
		this(StaticEntity!(N)[] staticEntities)in{assert( staticEntities.length > 0);}body{
			import std.array;
			import std.math;
			
			ar.Vector!(N, 3) start = staticEntities[0].boundingBox.start;
			ar.Vector!(N, 3) end = staticEntities[0].boundingBox.end;

			foreach (staticEntity; staticEntities) {
				for (int axis = 0; axis < 3; axis++) {
					start[axis] = fmin(start[axis], staticEntity.boundingBox.start[axis] );
					end[axis] = fmax(end[axis], staticEntity.boundingBox.end[axis] );
				}
			}
			_boundingBox = BoundingBox!(N)(start, end);
			
			if(staticEntities.length > 1){
				staticEntities.sortStaticEntitiesWithAxis!((in N a, in N b){return a<b;})(_boundingBox.majorAxis);
					
				auto splitPoint= staticEntities.length / 2;
				_nexts ~= AABBNode!(N)(staticEntities[0 .. splitPoint]);
				_nexts ~= AABBNode!(N)(staticEntities[splitPoint.. $]);
			}else{
				_staticEntity = staticEntities[0];
			}
		}
		
		bool isLeaf()const{
			return (_nexts.length == 0);
		}
		
		StaticEntity!(N)[] detectCollidableStaticEntities(in BoundingBox!(N) boundingBox){
			StaticEntity!(N)[] array;
			detectCollidableStaticEntitiesRecursively(boundingBox, array);
			return array;
		}
		
		void draw(){
			drawRecursively;
		}
	}//public

	private{
		AABBNode!(N)[] _nexts;
		StaticEntity!(N) _staticEntity;
		
		BoundingBox!(N) _boundingBox;
		
		void detectCollidableStaticEntitiesRecursively(in BoundingBox!(N) boundingBox, ref StaticEntity!(N)[] array){
			if(boundingBox & _boundingBox){
				if(isLeaf){
					array ~= _staticEntity;
				}else{
					foreach (nextNode; _nexts) {
						nextNode.detectCollidableStaticEntitiesRecursively(boundingBox, array);
					}
				}
			}
		}
		
		void drawRecursively(){
			if(isLeaf){
				ar.setColor(0, 255, 64);
				_boundingBox.drawBoundingBox;
			}else{
				ar.setColor(64, 64, 64);
				_boundingBox.drawBoundingBox;
				foreach (nextNode; _nexts) {
					nextNode.drawRecursively;
				}
			}
		};
	}//private
}//struct AABBNode 

private void drawBoundingBox(B)(B boundingBox){
	with( boundingBox ){
		ar.drawLine(start[0], start[1], start[2], end[0], start[1], start[2]);
		ar.drawLine(start[0], start[1], start[2], start[0], end[1], start[2]);
		ar.drawLine(start[0], start[1], start[2], start[0], start[1], end[2]);

		ar.drawLine(start[0], end[1], start[2], end[0], end[1], start[2]);
		ar.drawLine(start[0], end[1], start[2], start[0], end[1], end[2]);

		ar.drawLine(end[0], end[1], end[2], start[0], end[1], end[2]);
		ar.drawLine(end[0], end[1], end[2], end[0], start[1], end[2]);
		ar.drawLine(end[0], end[1], end[2], end[0], end[1], start[2]);

		ar.drawLine(end[0], start[1], end[2], start[0], start[1], end[2]);
		ar.drawLine(end[0], start[1], end[2], end[0], start[1], start[2]);
		
		ar.drawLine(start[0], end[1], end[2], start[0], start[1], end[2]);
		ar.drawLine(end[0], end[1], start[2], end[0], start[1], start[2]);
	}
}

private int majorAxis(Numeric)(in BoundingBox!(Numeric) boundingbox){
	import std.algorithm;
	import std.math;
	auto vec = boundingbox.end-boundingbox.start;
	return ( reduce!(fmax)(vec.data).among(vec[0], vec[1], vec[2])-1 );
}
unittest{
	alias V = ar.Vector3d;
	auto axis = majorAxis(BoundingBox!(double)(V(0, 0, 0), V(2, 3, 1)));
	assert(axis == 1);
}

private void sortStaticEntitiesWithAxis(alias Func, N)(StaticEntity!(N)[] staticEntities, int axis){
	bool func(in StaticEntity!(N) a, in StaticEntity!(N) b){
		return Func(a.boundingBox.center[axis], b.boundingBox.center[axis]);
	}
	
	import std.algorithm;
	staticEntities.sort!func;
}

unittest{
	import armos;
	import fragments.material;
	auto material = new Material!(double);
	alias V = ar.Vector3d;
	StaticEntity!(double)[] polygons = [ 
		new Polygon!(double)([V(2, 1, 2), V(4, 4, 2), V(2, 1, 4)], material), 
		new Polygon!(double)([V(5, 4, 5), V(3, 3, 3), V(6, 5, 7)], material), 
		new Polygon!(double)([V(0, -1, 0), V(1, -1, 0), V(1, 0, 2)], material), 
	];
	
	auto resultPolygonsX = [ 
		new Polygon!(double)([V(0, -1, 0), V(1, -1, 0), V(1, 0, 2)], material), 
		new Polygon!(double)([V(2, 1, 2), V(4, 4, 2), V(2, 1, 4)], material), 
		new Polygon!(double)([V(5, 4, 5), V(3, 3, 3), V(6, 5, 7)], material), 
	];
	
	// sortPolygonsWithAxis!((in double a, in double b){return a<b;}, 0)(polygons);
	polygons.sortStaticEntitiesWithAxis!((in double a, in double b){return a<b;})(0);
	
	foreach (int index, p; polygons) {
		assert (p.boundingBox.center == resultPolygonsX[index].boundingBox.center);
	}
	
}
