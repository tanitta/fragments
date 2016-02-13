module fragments.contactpoint;
import armos;
/++
++/
struct ContactPoint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	public{
		///
		V3 coordination;
		
		///
		V3 normal;
		
		///
		N distance;
	}//public

	private{
	}//private
}//struct ContactPoint
