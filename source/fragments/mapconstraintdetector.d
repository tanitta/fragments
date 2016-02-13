module fragments.mapconstraintdetector;
import fragments.constraintpair;
import fragments.entity;
import fragments.polygon;
import fragments.boundingbox;
import armos;

/++
++/
class MapConstraintDetector(NumericType){
	alias N = NumericType;
	public{
		/++
		++/
		void setStaticEntities(StaticEntity!(N)[] staticEntities){
			_root = AABBNode!(N)(staticEntities);
		}
		
		/++
		++/
		ConstraintPair!(N)[] detectConstraintPairs(DynamicEntity!(N)[] dynamicEntities){
			import std.algorithm;
			CollidablePair!(N)[] collidablePairs;
			detectCollidablePair(collidablePairs, dynamicEntities);
			return broadPhase(dynamicEntities, _root).narrowPhase;
		}
	}//public

	private{
		AABBNode!(N) _root;
		
		CollidablePair[] detectCollidablePair(ref CollidablePair!(N)[] collidablePairs, DynamicEntity!(N)[] dynamicEntities){
			foreach (dynamicEntity; dynamicEntities) {
				foreach (polygon; _root.detectCollidableStaticEntities(dynamicEntity.boundingBox)) {
					collidablePairs ~= CollidablePair(dynamicEntity, polygon);
				}
			}
			return collidablePairs;
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
			_dynamicEntity = d;
			_staticEntity = s;
		}
	}//public

	private{
		DynamicEntity!(N) _dynamicEntity;
		StaticEntity!(N) _staticEntity;
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
			ar.Vector!(N, 3) start = ar.Vector!(N, 3)(0, 0, 0);
			ar.Vector!(N, 3) end = ar.Vector!(N, 3)(0, 0, 0);

			foreach (staticEntity; staticEntities) {
				for (int axis = 0; axis < 3; axis++) {
					start[axis] = fmin(start[axis], staticEntity.boundingBox.start[axis] );
					end[axis] = fmax(end[axis], staticEntity.boundingBox.end[axis] );
				}
			}
			_boundingBox = BoundingBox(start, end);
			
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
		
		StaticEntity!(N)[] detectCollidableStaticEntities(in BoundingBox boundingBox){
			StaticEntity!(N)[] array;
			detectCollidableStaticEntitiesRecursively(boundingBox, array);
			return array;
		}
	}//public

	private{
		AABBNode!(N)[] _nexts;
		StaticEntity!(N) _staticEntity;
		
		BoundingBox!(N) _boundingBox;
		
		void detectCollidableStaticEntitiesRecursively(in BoundingBox boundingBox, ref StaticEntity!(N)[] array){
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
	}//private
}//struct AABBNode 

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
	alias V = ar.Vector3d;
	StaticEntity!(double)[] polygons = [ 
		new Polygon!(double)([V(2, 1, 2), V(4, 4, 2), V(2, 1, 4)]), 
		new Polygon!(double)([V(5, 4, 5), V(3, 3, 3), V(6, 5, 7)]), 
		new Polygon!(double)([V(0, -1, 0), V(1, -1, 0), V(1, 0, 2)]), 
	];
	
	auto resultPolygonsX = [ 
		new Polygon!(double)([V(0, -1, 0), V(1, -1, 0), V(1, 0, 2)]), 
		new Polygon!(double)([V(2, 1, 2), V(4, 4, 2), V(2, 1, 4)]), 
		new Polygon!(double)([V(5, 4, 5), V(3, 3, 3), V(6, 5, 7)]), 
	];
	
	// sortPolygonsWithAxis!((in double a, in double b){return a<b;}, 0)(polygons);
	polygons.sortStaticEntitiesWithAxis!((in double a, in double b){return a<b;})(0);
	
	foreach (int index, p; polygons) {
		assert (p.boundingBox.center == resultPolygonsX[index].boundingBox.center);
	}
	
}
