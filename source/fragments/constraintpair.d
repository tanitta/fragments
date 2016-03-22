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
		
		Entity!(N)[2] entities;
		
		Constraint!(N)[3] linearConstraints;
		Constraint!(N)[3] angularConstraints;
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
			in N bias,
			in N damper,
			in N spring,
			in N lowerLimit,
			in N upperLimit,
		){
			_axis = axis;
			_lowerLimit = lowerLimit;
			_upperLimit= upperLimit;
			_damper = damper;
			_spring = spring;
			_bias = bias;
		}
		
		@property{
			V3 axis(){return _axis;};
			N lowerLimit(){return _lowerLimit;};
			N upperLimit(){return _upperLimit;};
			N damper(){return _damper;};
			N spring(){return _spring;};
			N bias(){return _bias;};
		}
	}//public

	private{
		V3 _axis;
		N _lowerLimit;
		N _upperLimit;
		N _damper;
		N _spring;
		N _bias;
	}//private
}//struct Constraint
