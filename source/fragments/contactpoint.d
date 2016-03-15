module fragments.contactpoint;

import armos;

/++
++/
struct ContactPoint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	public{
		///
		this(in V3 coordination, in V3 normal, in N distance){
			_coordination = coordination;
			_normal = normal;
			_distance = distance;
		}
		
		///
		V3 coordination()const{return _coordination;};
		
		///
		V3 normal()const{return _normal;};
		
		///
		N distance()const{return _distance;};
	}//public

	private{
		///
		V3 _coordination;
		
		///
		V3 _normal;
		
		///
		N _distance;
	}//private
}//struct ContactPoint
