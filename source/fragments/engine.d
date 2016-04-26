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
			linkConstraintPairs.each!(pair => pair.update);
			
			dynamicEntities.each!((ref entity) => entity.updateCollisionConstraintPairs(_mapConstraintDetector.detectedStaticEntities(entity)));
				
			_constraintSolver.solve(
				dynamicEntities, 
				linkConstraintPairs,
				linearImpulseConstraints, 
			);
			
			import std.algorithm.iteration:filter;
			import std.array:array;
			auto collidingEntities = dynamicEntities.filter!(entity => entity.isColliding).array;
			foreach(entity; collidingEntities){
				entity.updateCollisionConstraintPairs(_mapConstraintDetector.detectedStaticEntities(entity));
			}
			
			dynamicEntities.each!(entity => entity.updateEntityStatus(_integrator));
		};
	}//public

	private{
		N _unitTime;
		MapConstraintDetector!N _mapConstraintDetector;
		Integrator!N _integrator;
		ConstraintSolver!N _constraintSolver;
		
	}//private
}//class Engine

private void updateEntityStatus(N)(DynamicEntity!N entity, Integrator!N integrator){
	if(!entity.isColliding){
		entity.updatePreStatus;
	}
	
	integrator.integrate(entity);
	
	if(!entity.isColliding){
		entity.updatePreVelocity;
	}
	
	entity.updateProperties;
}
