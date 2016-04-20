module fragments.square;

import armos;
import fragments.entity;
import fragments.contactpoint;
import fragments.boundingbox;
import fragments.material;
import fragments.ray;

/++
++/
class Square(NumericType) : DynamicEntity!(NumericType){
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias Q = ar.Quaternion!(N);
	mixin DynamicEntityProperties!(N);
	
	public{
		this(in Material!(N) m, in N size = N(1.0)){
			_material = m;

			_margin = V3(0.5, 0.5, 0.5);
			
			//set some rays used in method : contactPoints
			_rays = [
				V3(size,  0, 0    ), 
				V3(-size, 0, 0    ), 
				V3(0,     0, size ), 
				V3(0,     0, -size), 
		
				V3(size,  0, size ), 
				V3(-size, 0, -size), 
				V3(size,  0, -size), 
				V3(-size, 0, size ), 
			];
			
			// _collisionConstraintPairs
		}
		
		void updateCollisionConstraintPairs(in StaticEntity!(N)[] staticEntities){
			// {
			// 	ContactPoint!N[] points;
			// 	detectContactPoint(
			// 		_positionPre,
			// 		_position,
			// 		staticEntity,
			// 		points 
			// 	);
			//	
			// 	if(points.length > 0){
			// 		_collisionConstraintPairs[0].update(staticEntity, points[0]);
			// 	}
			// }
			//
			// foreach (int index, ray; _rays) {
			// 	ContactPoint!N[] points;
			// 	immutable isDetectStaticRay = detectContactPoint(
			// 		_position,
			// 		_orientation.rotatedVector(ray)+_position, 
			// 		staticEntity, 
			// 		points
			// 	);
			//	
			// 	// if(!isDetectStaticRay){
			// 	// 	immutable V3 rayBeginGlobal = _orientationPre.rotatedVector(ray)+_positionPre;
			// 	// 	immutable V3 rayEndGlobal   = _orientation.rotatedVector(ray)+_position;
			// 	// 	detectContactPoint(
			// 	// 		rayBeginGlobal,
			// 	// 		rayEndGlobal, 
			// 	// 		staticEntity, 
			// 	// 		points
			// 	// 	);
			// 	// }
			//	
			// 	if(points.length > 0){
			// 		_collisionConstraintPairs[index+1].update(staticEntity, points[0]);
			// 	}
			// }
		}
		
		///
		ContactPoint!(N)[] contactPoints(in StaticEntity!(N) staticEntity)const{
			ContactPoint!(N)[] points;
			
			detectContactPoint(
				_positionPre,
				_position,
				staticEntity, 
				points
			);
				
			foreach (ray; _rays) {
				immutable isDetectStaticRay = detectContactPoint(
					_position,
					_orientation.rotatedVector(ray)+_position, 
					staticEntity, 
					points
				);
				
				// if(!isDetectStaticRay){
				// 	immutable V3 rayBeginGlobal = _orientationPre.rotatedVector(ray)+_positionPre;
				// 	immutable V3 rayEndGlobal   = _orientation.rotatedVector(ray)+_position;
				// 	detectContactPoint(
				// 		rayBeginGlobal,
				// 		rayEndGlobal, 
				// 		staticEntity, 
				// 		points
				// 	);
				// }
			}
			
			
			// points.writeln;
			return points;
		}
	}//public
	
	private{
		V3[8] _rays;
	}//private
}//class Chip
unittest{
	import fragments.material;
	auto material = new Material!(double);
	auto square = new Square!(double)(material);
};

private ContactPoint!(N)[] detectedContactPoints(N, V3 = ar.Vector!(N, 3))(
	in V3 rayBeginGlobal,
	in V3 rayEndGlobal,
	in StaticEntity!(N) staticEntity
){
	ContactPoint!(N)[] points;
	detectContactPoint(
		rayBeginGlobal,
		rayEndGlobal,
		staticEntity,
		points,
	);
	return points;
}

private bool detectMostCloselyContactPoint(N, V3 = ar.Vector!(N, 3), StaticEntities)(
	in V3 rayBeginGlobal,
	in V3 rayEndGlobal,
	in StaticEntities staticEntities,
	ref ContactPoint!N[] contactPoints, 
){
	bool isDetect = false;
	
	ContactPoint!N[] points;
	foreach (staticEntity; staticEntities) {
		isDetect= isDetect|| detectContactPoint(
			rayBeginGlobal,
			rayEndGlobal,
			staticEntity,
			points, 
		);
	}
	if(isDetect){
		int closelyIndex = 0;
		N closelyDistance = N.max;
		foreach (int index, point; points) {
			if( point.distance < closelyDistance ){
				closelyIndex = index;
				closelyDistance = point.distance;
			}
		}
		contactPoints ~= points[closelyIndex];
	}
	// import std.math;
	// import std.algorithm:reduce;
	// contactPoints = points.reduce!((a, b) => a.distance<b.distance?a:b);
	return isDetect;
}
unittest{
	alias N = double;
	alias V3 = ar.Vector!(N, 3);
	
	import fragments.material;
	auto material = new Material!(double);
	
	import fragments.polygon;
	Polygon!N[] polygons;
	auto exceptedPolygon = new Polygon!N(
		[
			V3(1, -4, 1), 
			V3(1, 4, 1), 
			V3(1, 0, -2), 
		],
		material
	);

	polygons ~= exceptedPolygon;
	polygons ~= new Polygon!N(
		[
			V3(2, -4, 1), 
			V3(2, 4, 1), 
			V3(2, 0, -2), 
		],
		material
	);
	
	{
		V3 begin = V3(0, 0, 0);
		V3 end = V3(10, 0, 0);
		
		ContactPoint!N[] contactPoints;
		detectMostCloselyContactPoint(begin, end, polygons, contactPoints);
		assert(contactPoints[0].staticEntity == exceptedPolygon);
	}
	{
		V3 begin = V3(0, 0, 0);
		V3 end = V3(-10, 0, 0);
		
		ContactPoint!N[] contactPoints;
		detectMostCloselyContactPoint(begin, end, polygons, contactPoints);
		assert(contactPoints.length == 0);
	}
}

private bool detectContactPoint(N, V3 = ar.Vector!(N, 3))(
	in V3 rayBeginGlobal,
	in V3 rayEndGlobal,
	in StaticEntity!(N) staticEntity,
	ref ContactPoint!(N)[] points, 
	in V3 applicationPoint = null
){
	bool isSuccess = false;
	
	immutable p0 = V3.zero;
	immutable p1 = staticEntity.vertices[1] -staticEntity.vertices[0];
	immutable p2 = staticEntity.vertices[2] -staticEntity.vertices[0];
	
	immutable pBegin = rayBeginGlobal - staticEntity.vertices[0];
	immutable pEnd = rayEndGlobal - staticEntity.vertices[0];
	immutable pNormal= staticEntity.normal;
	
	immutable isCollidingLineToPlane = ( (pBegin.dotProduct(pNormal)) * (pEnd.dotProduct(pNormal)) <= N(0) );
	if( isCollidingLineToPlane ){
		import std.math;
		immutable d1 = pNormal.dotProduct(pBegin);
		immutable d2 = pNormal.dotProduct(pEnd);
		immutable a = fabs(d1)/(fabs(d1)+fabs(d2));
		immutable pContact = (N(1)-a)*pBegin + a*pEnd;
		
		immutable isBuried = (d2 <= 0 && 0 <= d1);
		// immutable isBuried = (d2 < 0);
		immutable isIncludedInPolygon =
		( ( p1 - p0 ).vectorProduct(pContact-p0).dotProduct(pNormal) >= N(0) )&&
		( ( p2 - p1 ).vectorProduct(pContact-p1).dotProduct(pNormal) >= N(0) )&&
		( ( p0 - p2 ).vectorProduct(pContact-p2).dotProduct(pNormal) >= N(0) );
		
		if(isBuried && isIncludedInPolygon){
			const contactPoint = ContactPoint!(N)(
				pContact + staticEntity.vertices[0],
				pNormal, 
				-d2,
				(!isNaN( applicationPoint[0] ))?applicationPoint:rayEndGlobal,
				staticEntity
			);
			
			points ~= contactPoint;
			isSuccess = true;
		}
	}
	
	return isSuccess;
}
