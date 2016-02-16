module fragments.square;
import armos;
import fragments.entity;
import fragments.contactpoint;
import fragments.boundingbox;

template DynamicEntityProperties(NumericType){
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias Q = ar.Quaternion!(N);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		///
		N mass()const{return _mass;}

		///
		void mass(in N m){_mass = m;}
		
		///
		M33 inertia()const{return _inertia;};
		
		///
		void inertia(in M33 inertia){_inertia  = inertia;};
		
		///
		M33 inertiaGlobal()const{return _inertiaGlobal;};
		
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
		void updateProperties(){
			_positionPre = _position;
			_orientationPre = _orientation;
			_inertiaGlobal = _orientation.matrix33*_inertiaGlobal*_orientation.matrix33.inverse;
			_boundingBox = BoundingBox!(N)(_position, _positionPre, _margin);
		}
	}//public
	
	private{
		N   _mass;
		V3  _position;
		V3  _positionPre;
		V3  _linearVelocity;
		Q   _orientation;
		Q   _orientationPre;
		V3  _angularVelocity;
		M33 _inertia;
		M33 _inertiaGlobal;
		BoundingBox!(N) _boundingBox;
		V3 _margin;
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
		this(in N size = N(1.0)){
			_margin = V3(0.5, 0.5, 0.5);
			
			//set some rays used in method : contactPoints
			_rays ~= V3(size,  0, 0);
			_rays ~= V3(-size, 0, 0);
			_rays ~= V3(0,     0, size);
			_rays ~= V3(0,     0, -size);
			
			_rays ~= V3(size,  0, size);
			_rays ~= V3(-size, 0, -size);
			_rays ~= V3(size,  0, -size);
			_rays ~= V3(-size, 0, size);
		}
		///
		ContactPoint!(N)[] contactPoints(in StaticEntity!(N) staticEntity)const{
			ContactPoint!(N)[] points;
			foreach (ray; _rays) {
				auto rayGlobal = _orientation.rotatedVector(ray)+_position;
				
				auto p0 = V3.zero;
				auto p1 = staticEntity.vertices[1] -staticEntity.vertices[0];
				auto p2 = staticEntity.vertices[2] -staticEntity.vertices[0];
				
				auto pBegin = _position - staticEntity.vertices[0];
				auto pEnd = rayGlobal - staticEntity.vertices[0];
				auto pNormal= staticEntity.normal;
				
				auto isCollidingLineToPlane = ( (pBegin.dotProduct(pNormal)) * (pEnd.dotProduct(pNormal)) <= 0 );
				if( isCollidingLineToPlane ){
					import std.math;
					auto d1 = pNormal.dotProduct(pBegin);
					auto d2 = pNormal.dotProduct(pEnd);
					auto a = fabs(d1)/(fabs(d1)+fabs(d2));
					auto pContact = (N(1)-a)*pBegin + a*pEnd;
					
					auto isBuried = (d2 <= 0);
					auto isIncludedInPolygon =
					( ( p1 - p0 ).vectorProduct(pContact-p0).dotProduct(pNormal) > N(0) )&&
					( ( p2 - p1 ).vectorProduct(pContact-p1).dotProduct(pNormal) > N(0) )&&
					( ( p0 - p2 ).vectorProduct(pContact-p2).dotProduct(pNormal) > N(0) );
					if(isBuried && isIncludedInPolygon){
						auto contactPoint = ContactPoint!(N)();
						
						contactPoint.coordination = pContact + staticEntity.vertices[0];
						contactPoint.distance = -d2;
						contactPoint.normal = pNormal;
						
						points ~= contactPoint;
					}
				}
			}
			return points;
		}
	}//public
	
	private{
		V3[] _rays;
	}//private
}//class Chip
unittest{
	auto square = new Square!(double);
}
