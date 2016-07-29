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
	
	import std.algorithm:map;
	import std.array:array;
	import std.array:join;
	public{
		/++
		++/
		void setStaticEntities(ref StaticEntity!N[] staticEntities){
			_root = AABBNode!N(staticEntities);
		}
		
		/++
		++/
		StaticEntity!N[] detectedStaticEntities(DynamicEntity!N dynamicEntity){
			import std.algorithm.iteration:map;
			import std.array:array;
			return detectedCollidablePairs(dynamicEntity).map!(pair => pair.staticEntity).array;
		}
		
		/++
		++/
		void unitTime(in N t){_unitTime = t;}
	}//public

	private{
		AABBNode!N _root;
		
		N _unitTime;
		
		CollidablePair!N[] detectedCollidablePairs(DynamicEntity!(N) dynamicEntity){
			return _root.detectCollidableStaticEntities(dynamicEntity.boundingBox)
				.map!(polygon => CollidablePair!(N)(dynamicEntity, polygon))
				.array;
		}
		
		CollidablePair!N[] detectedCollidablePairs(DynamicEntity!(N)[] dynamicEntities){
			return dynamicEntities
				.map!(entity => detectedCollidablePairs(entity))
				.join;
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
		this(StaticEntity!(N)[] staticEntities)in{assert( staticEntities.length > 0, "empty entities");}body{
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
					
				immutable splitPoint= staticEntities.length / 2;
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
				ar.color(0, 255, 64);
				_boundingBox.drawBoundingBox;
			}else{
				ar.color(64, 64, 64);
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
	immutable vec = boundingbox.end-boundingbox.start;
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
