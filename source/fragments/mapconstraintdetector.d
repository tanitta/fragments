module fragments.mapconstraintdetector;
import fragments.constraintpair;
import fragments.entity;
import fragments.polygon;
import fragments.boundingbox;
import armos;
/++
++/
class MapConstraintDetector {
	public{
		/++
		++/
		void setStaticEntities(StaticEntity[] staticEntities){
			_root = AABBNode(staticEntities);
		}
		/++
		++/
		ConstraintPair[] detectConstraintPairs(DynamicEntity[] dynamicEntities){
			import std.algorithm;
			CollidablePair[] collidablePairs;
			detectCollidablePair(collidablePairs, dynamicEntities);
			return broadPhase(dynamicEntities, _root).narrowPhase;
		}
	}//public

	private{
		AABBNode _root;
		
		CollidablePair[] detectCollidablePair(ref CollidablePair[] collidablePairs, DynamicEntity[] dynamicEntities){
			foreach (dynamicEntity; dynamicEntities) {
				foreach (polygon; _root.detectCollidableStaticEntities(dynamicEntity.boundingBox)) {
					collidablePairs ~= CollidablePair(dynamicEntity, polygon);
				}
			}
			return collidablePairs;
		}
		
	}//private
}//class MapConstraintDetector
unittest{
	
}


private CollidablePair[] broadPhase(in DynamicEntity[] dynamicEntities, in AABBNode node){
	CollidablePair[] array;
	return array;
}

private ConstraintPair[] narrowPhase(CollidablePair[] collidablePairs){
	ConstraintPair[] array;
	return array;
}


/++
++/
private struct CollidablePair {
	public{
		this(DynamicEntity d, StaticEntity s){
			_dynamicEntity = d;
			_staticEntity = s;
		}
	}//public

	private{
		DynamicEntity _dynamicEntity;
		StaticEntity _staticEntity;
	}//private
}//struct CollidablePair


/++
++/
private struct AABBNode{
	public{
		/++
		++/
		this(StaticEntity[] staticEntities)in{assert( staticEntities.length > 0);}body{
			import std.array;
			import std.math;
			ar.Vector3d start = ar.Vector3d(0, 0, 0);
			ar.Vector3d end = ar.Vector3d(0, 0, 0);

			foreach (staticEntity; staticEntities) {
				for (int axis = 0; axis < 3; axis++) {
					start[axis] = fmin(start[axis], staticEntity.boundingBox.start[axis] );
					end[axis] = fmax(end[axis], staticEntity.boundingBox.end[axis] );
				}
			}
			_boundingBox = BoundingBox(start, end);
			
			if(staticEntities.length > 1){
				staticEntities.sortStaticEntitiesWithAxis!((in double a, in double b){return a<b;})(_boundingBox.majorAxis);
					
				auto splitPoint= staticEntities.length / 2;
				_nexts ~= AABBNode(staticEntities[0 .. splitPoint]);
				_nexts ~= AABBNode(staticEntities[splitPoint.. $]);
			}else{
				_staticEntity = staticEntities[0];
			}
		}
		
		bool isLeaf()const{
			return (_nexts.length == 0);
		}
		
		StaticEntity[] detectCollidableStaticEntities(in BoundingBox boundingBox){
			StaticEntity[] array;
			detectCollidableStaticEntitiesRecursively(boundingBox, array);
			return array;
		}
	}//public

	private{
		AABBNode[] _nexts;
		StaticEntity _staticEntity;
		
		BoundingBox _boundingBox;
		
		void detectCollidableStaticEntitiesRecursively(in BoundingBox boundingBox, ref StaticEntity[] array){
			if(boundingBox & _boundingBox){
				if(isLeaf){
					import std.stdio;
					"leaf".writeln;
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
// unittest{
// 	alias V = ar.Vector3d;
//	
// 	Polygon[] polygons = [ 
// 		new Polygon([V(2, 1, 2), V(4, 4, 2), V(2, 1, 4)]), 
// 		new Polygon([V(6, 5, 6), V(4, 4, 2), V(2, 1, 4)]), 
// 		new Polygon([V(6, 5, 6), V(8, 8, 6), V(2, 1, 4)]), 
// 	];
//	
// 	auto tree = AABBNode(polygons);
// 	import std.stdio;
// 	tree.detectCollidablePolygons(BoundingBox(V(0, 0, 0), V(10, 10, 10)))[2].boundingBox.start.print;
// }

private int majorAxis(in BoundingBox boundingbox){
	import std.algorithm;
	import std.math;
	auto vec = boundingbox.end-boundingbox.start;
	return ( reduce!(fmax)(vec.data).among(vec[0], vec[1], vec[2])-1 );
}
unittest{
	alias V = ar.Vector3d;
	auto axis = majorAxis(BoundingBox(V(0, 0, 0), V(2, 3, 1)));
	assert(axis == 1);
}

private void sortStaticEntitiesWithAxis(alias Func)(StaticEntity[] staticEntities, int axis){
	bool func(in StaticEntity a, in StaticEntity b){
		return Func(a.boundingBox.center[axis], b.boundingBox.center[axis]);
	}
	
	import std.algorithm;
	staticEntities.sort!func;
}

unittest{
	import armos;
	alias V = ar.Vector3d;
	StaticEntity[] polygons = [ 
		new Polygon([V(2, 1, 2), V(4, 4, 2), V(2, 1, 4)]), 
		new Polygon([V(5, 4, 5), V(3, 3, 3), V(6, 5, 7)]), 
		new Polygon([V(0, -1, 0), V(1, -1, 0), V(1, 0, 2)]), 
	];
	
	auto resultPolygonsX = [ 
		new Polygon([V(0, -1, 0), V(1, -1, 0), V(1, 0, 2)]), 
		new Polygon([V(2, 1, 2), V(4, 4, 2), V(2, 1, 4)]), 
		new Polygon([V(5, 4, 5), V(3, 3, 3), V(6, 5, 7)]), 
	];
	
	// sortPolygonsWithAxis!((in double a, in double b){return a<b;}, 0)(polygons);
	polygons.sortStaticEntitiesWithAxis!((in double a, in double b){return a<b;})(0);
	
	foreach (int index, p; polygons) {
		assert (p.boundingBox.center == resultPolygonsX[index].boundingBox.center);
	}
	
}
