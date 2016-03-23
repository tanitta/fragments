module fragments.constraintpair;

import fragments.entity;
import fragments.contactpoint;
import armos;

/++
++/
struct ConstraintPair(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	
	public{
		DynamicEntity!(N)[2] entities;
		
		Constraint!(N)[3] linearConstraints;
		Constraint!(N)[3] angularConstraints;
		
		N k;
	}//public

	private{
	}//private
}//class ConstraintPair

/++
+/
struct Constraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	public{
		// ContactPoint!(N) contactPoint;
		this(
			in V3 axis,
			in N lowerLimit,
			in N upperLimit,
			in N delegate() force,
		){
			_axis = axis;
			_lowerLimit = lowerLimit;
			_upperLimit= upperLimit;
			_force = force;
		}
		
		@property{
			V3 axis(){return _axis;};
			N lowerLimit(){return _lowerLimit;};
			N upperLimit(){return _upperLimit;};
			N delegate() force(){return _force;};
		}
	}//public

	private{
		V3 _axis;
		N _lowerLimit;
		N _upperLimit;
		N delegate() _force;
	}//private
}//struct Constraint
