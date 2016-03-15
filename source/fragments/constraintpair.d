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
		V3 axis;
		N lowerLimit;
		N upperLimit;;
		N damper;
		N spring;
	}//public

	private{
	}//private
}//struct Constraint
