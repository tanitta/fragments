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
		this(DynamicEntity!(N) entityA, DynamicEntity!(N) entityB = null){
			_entities[0] = entityA;
			_entities[1] = entityB;
		}
		
		DynamicEntity!(N)[2] entities(){return _entities;};
		
		Constraint!(N)[3] linearConstraints;
		Constraint!(N)[3] angularConstraints;
		
	}//public

	private{
		DynamicEntity!(N)[2] _entities;
		// N _k;
		// void k(){_k = 0;}
	}//private
}//class ConstraintPair

/++
+/
struct Constraint(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias ConstraintCondition = V3[2] delegate(ConstraintPair!N);
	public{
		// ContactPoint!(N) contactPoint;
		this(
			in V3 axis,
			in V3 initialImpulse,
			in ConstraintCondition constraintCondition,
		){
			_axis = axis;
			_initialImpulse = initialImpulse;
			_constraintCondition = constraintCondition;
		}
		
		@property{
			V3 axis(){return _axis;};
			V3 initialImpulse(){return _initialImpulse;};
			ConstraintCondition constraintCondition(){return _constraintCondition;};
		}
	}//public

	private{
		V3 _axis;
		V3 _initialImpulse;
		ConstraintCondition _constraintCondition;
	}//private
}//struct Constraint
