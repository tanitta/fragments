module fragments.square;

import armos;
import fragments.entity;
import fragments.contactpoint;
import fragments.boundingbox;
import fragments.material;

template DynamicEntityProperties(NumericType){
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias Q = ar.Quaternion!(N);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		///
		N mass()const{return _mass;}

		///
		void mass(in N m){
			_mass = m;
			_massInv = N(1)/m;
		}
		
		///
		M33 inertia()const{return _inertia;};
		
		///
		void inertia(in M33 inertia){
			_inertia  = inertia;
			_inertiaGlobal = _orientation.matrix33*_inertia*_orientation.matrix33.inverse;
			_inertiaGlobalInv = _inertiaGlobal.inverse;
		};
		
		///
		M33 inertiaGlobal()const{return _inertiaGlobal;};
		
		///
		N massInv()const{
			return _massInv;
		}
		
		M33 inertiaGlobalInv()const{
			return _inertiaGlobalInv;
		}
		
		///
		V3 position()const{return _position;};
		
		///
		void position(in V3 position){_position = position;};
		
		///
		V3 positionPre()const{return _positionPre;};
		
		///
		V3 linearVelocity()const{return _linearVelocity;};
		
		///
		void linearVelocity(in V3 linearVelocity){_linearVelocity = linearVelocity;};
		
		///
		Q orientation()const{return _orientation;};
		
		///
		void orientation(in Q orientation){_orientation = orientation;};
		
		///
		Q orientationPre()const{return _orientationPre;};
		
		///
		V3 angularVelocity()const{return _angularVelocity;};
		
		///
		void angularVelocity(in V3 angularVelocity){_angularVelocity = angularVelocity;};
		
		///
		BoundingBox!(N) boundingBox()const{return _boundingBox;}
		
		///
		const( Material!(N) ) material()const{
			return _material;
		}
		
		V3 deltaLinearVelocity()const{
			return _deltaLinearVelocity;
		}
		
		void deltaLinearVelocity(in V3 v){
			_deltaLinearVelocity = v;
		};
		
		V3 deltaAngularVelocity()const{
			return _deltaAngularVelocity;
		}
		
		void deltaAngularVelocity(in V3 v){
			_deltaAngularVelocity = v;
		}
		
		///
		void updateProperties(in N unitTime){
			_positionPre = _position;
			_orientationPre = _orientation;
			_inertiaGlobal = _orientation.matrix33*_inertiaGlobal*_orientation.matrix33.inverse;
			_inertiaGlobalInv = _inertiaGlobal.inverse;
			_boundingBox = BoundingBox!(N)(_position, _position-linearVelocity*unitTime, _margin);
		}
	}//public
	
	private{
		N   _mass;
		N   _massInv;
		V3  _position = V3.zero;
		V3  _positionPre = V3.zero;
		V3  _linearVelocity = V3.zero;
		Q   _orientation = Q.unit;
		Q   _orientationPre = Q.unit;
		V3  _angularVelocity = V3.zero;
		M33 _inertia;
		M33 _inertiaGlobal;
		M33 _inertiaGlobalInv;
		BoundingBox!(N) _boundingBox;
		const( Material!(N) ) _material;
		V3 _margin = V3.zero;
		V3 _deltaLinearVelocity = V3.zero;
		V3 _deltaAngularVelocity = V3.zero;
	}//private
}

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
				V3(size,  0, 0), 
				V3(-size, 0, 0), 
				V3(0,     0, size), 
				V3(0,     0, -size), 
		
				V3(size,  0, size), 
				V3(-size, 0, -size), 
				V3(size,  0, -size), 
				V3(-size, 0, size), 
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
					V3 rayVelocity = angularVelocity.vectorProduct( _orientation.rotatedVector(ray) );
					V3 rayBeginGlobal    = _orientation.rotatedVector(ray)+_position - rayVelocity;
					V3 rayEndGlobal    = _orientation.rotatedVector(ray)+_position;

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
		
		bool detectContactPoint(in V3 rayBeginGlobal, in V3 rayEndGlobal, in StaticEntity!(N) staticEntity, ref ContactPoint!(N)[] points)const{
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
						pNormal, 
						-d2
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
