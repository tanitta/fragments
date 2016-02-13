module fragments.constraintpair;
import fragments.entity;
import fragments.contactpoint;

/++
++/
struct ConstraintPair(NumericType) {
	alias N = NumericType;
	public{
		Entity!(N)[2] entities;
		ContactPoint!(N) contactPoint;
	}//public

	private{
	}//private
}//class ConstraintPair

