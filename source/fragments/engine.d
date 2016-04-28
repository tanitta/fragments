module fragments.engine;

import fragments.entity;
import fragments.mapconstraintdetector;
import fragments.constraintsolver;
import fragments.integrator;
import fragments.constraintpair;

/++
++/
class Engine(NumericType){
	alias N = NumericType;
	
	public{
		/++
		++/
		this(){
			_mapConstraintDetector = new MapConstraintDetector!(N);
			_integrator = new Integrator!(N);
			_constraintSolver = new ConstraintSolver!N;
			
			_unitTime = N(1.0/30.0);
			this.unitTime = _unitTime;
		}
		
		/++
		++/
		void unitTime(in N t){
			_unitTime = t;
			_mapConstraintDetector.unitTime = t;
			_constraintSolver.unitTime = t;
			_integrator.unitTime = t;
		}
		
		/++
		++/
		N unitTime()const{return _unitTime;}
		
		/++
		++/
		void staticEntities(StaticEntity!(N)[] staticEntities){
			_mapConstraintDetector.setStaticEntities(staticEntities);
		};
		
		/++
		++/
		void update(
			ref DynamicEntity!N[] dynamicEntities,
			LinkConstraintPair!N[] linkConstraintPairs, 
			ref LinearImpulseConstraint!N[] linearImpulseConstraints,
		){
			import std.algorithm:map, each;
			linkConstraintPairs.each!((ref pair) => pair.update(_unitTime));
			
			dynamicEntities.each!((ref entity) => entity.updateCollisionConstraintPairs(_mapConstraintDetector.detectedStaticEntities(entity)));
				
			_constraintSolver.solve(
				dynamicEntities, 
				linkConstraintPairs,
				linearImpulseConstraints, 
			);
			
			dynamicEntities.each!(entity => entity.updateStatus(_integrator, _mapConstraintDetector));
		};
	}//public

	private{
		N _unitTime;
		MapConstraintDetector!N _mapConstraintDetector;
		Integrator!N _integrator;
		ConstraintSolver!N _constraintSolver;
		
	}//private
}//class Engine
