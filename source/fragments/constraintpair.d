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
			
			if(entityB){
				// k();
			}else{
				// k();
			}
		}
		
		// DynamicEntity!(N)[2] entities(){return _entities;};
		// N k(){return _k;};
		
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
	public{
		// ContactPoint!(N) contactPoint;
		this(
			in V3 axis,
			in N delegate(ConstraintPair!N constraintPair) force,
		){
			_axis = axis;
			_force = force;
		}
		
		@property{
			V3 axis(){return _axis;};
			N delegate(ConstraintPair!N constraintPair) force(){return _force;};
		}
	}//public

	private{
		V3 _axis;
		N delegate(ConstraintPair!N constraintPair) _force;
	}//private
}//struct Constraint
