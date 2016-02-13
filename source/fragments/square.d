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
		V3 position()const{return _position;};
		
		///
		void position(in V3 position){_position = position;};
		
		///
		V3 linearVelocity()const{return _linearVelocity;};
		
		///
		void linearVelocity(in V3 linearVelocity){_linearVelocity = linearVelocity;};
		
		///
		Q orientation()const{return _orientation;};
		
		///
		void orientation(in Q orientation){_orientation = orientation;};
		
		///
		V3 angularVelocity()const{return _angularVelocity;};
		
		///
		void angularVelocity(in V3 angularVelocity){_angularVelocity = angularVelocity;};
	}//public
	private{
		N   _mass;
		V3  _position;
		V3  _linearVelocity;
		Q   _orientation;
		V3  _angularVelocity;
		M33 _inertia;
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
		///
		BoundingBox!(N) boundingBox()const{return _boundingBox;};
		
		///
		ContactPoint!(N)[] contactPoints(in StaticEntity!(N) staticEntity)const{
			ContactPoint!(N)[] points;
			return points;
		};
	}//public
	private{
		BoundingBox!(N) _boundingBox;
	}//private
}//class Chip
unittest{
	auto square = new Square!(double);
}
