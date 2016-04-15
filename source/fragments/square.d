module fragments.square;

import armos;
import fragments.entity;
import fragments.contactpoint;
import fragments.boundingbox;
import fragments.material;

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
		}
		
		///
		ContactPoint!(N)[] contactPoints(in StaticEntity!(N) staticEntity)const{
			// import std.stdio;
			// "bb".writeln;
			ContactPoint!(N)[] points;
			
			foreach (ray; _rays) {
				immutable isDetectStaticRay = detectContactPoint(
					_position,
					_orientation.rotatedVector(ray)+_position, 
					staticEntity, 
					points
				);
				
				if(!isDetectStaticRay){
					immutable V3 rayVelocity = (angularVelocity*0.33).vectorProduct( _orientation.rotatedVector(ray) ) + linearVelocity*0.33;
					immutable V3 rayBeginGlobal    = _orientation.rotatedVector(ray)+_position - rayVelocity;
					immutable V3 rayEndGlobal    = _orientation.rotatedVector(ray)+_position;
					
					detectContactPoint(
						rayBeginGlobal,
						rayEndGlobal, 
						staticEntity, 
						points
					);
				}
			}
			
			detectContactPoint(
				_position-linearVelocity,
				_position,
				staticEntity, 
				points
			);
			
			// points.writeln;
			return points;
		}
	}//public
	
	private{
		V3[8] _rays;
		
		bool detectContactPoint(
			in V3 rayBeginGlobal,
			in V3 rayEndGlobal,
			in StaticEntity!(N) staticEntity,
			ref ContactPoint!(N)[] points, 
			in V3 applicationPoint = null
		)const{
			immutable p0 = V3.zero;
			immutable p1 = staticEntity.vertices[1] -staticEntity.vertices[0];
			immutable p2 = staticEntity.vertices[2] -staticEntity.vertices[0];
			
			immutable pBegin = rayBeginGlobal - staticEntity.vertices[0];
			immutable pEnd = rayEndGlobal - staticEntity.vertices[0];
			immutable pNormal= staticEntity.normal;
			
			immutable isCollidingLineToPlane = ( (pBegin.dotProduct(pNormal)) * (pEnd.dotProduct(pNormal)) <= 0 );
			if( isCollidingLineToPlane ){
				import std.math;
				immutable d1 = pNormal.dotProduct(pBegin);
				immutable d2 = pNormal.dotProduct(pEnd);
				immutable a = fabs(d1)/(fabs(d1)+fabs(d2));
				immutable pContact = (N(1)-a)*pBegin + a*pEnd;
				
				immutable isBuried = (d2 < 0 && 0 <= d1);
				// immutable isBuried = (d2 < 0);
				immutable isIncludedInPolygon =
				( ( p1 - p0 ).vectorProduct(pContact-p0).dotProduct(pNormal) >= N(0) )&&
				( ( p2 - p1 ).vectorProduct(pContact-p1).dotProduct(pNormal) >= N(0) )&&
				( ( p0 - p2 ).vectorProduct(pContact-p2).dotProduct(pNormal) >= N(0) );
				
				if(isBuried && isIncludedInPolygon){
					immutable contactPoint = ContactPoint!(N)(
						pContact + staticEntity.vertices[0],
						// rayEndGlobal, 
						// rayBeginGlobal, 
						pNormal, 
						-d2,
						(!isNaN( applicationPoint[0] ))?applicationPoint:rayEndGlobal
					);
					
					points ~= contactPoint;
					return true;
				}
				return false;
			}
			return false;
		}
	}//private
}//class Chip
unittest{
	import fragments.material;
	auto material = new Material!(double);
	auto square = new Square!(double)(material);
};
