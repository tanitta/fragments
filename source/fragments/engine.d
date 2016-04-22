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
			ref DynamicEntity!(N)[] dynamicEntities,
			ref LinkConstraintPair!N[] linkConstraintPairs
		){
			{
				import armos;
				alias V3 = ar.Vector!(N, 3);
				
				import std.algorithm:map, each;
				import std.array:array,join;
				LinearImpulseConstraint!N[] linearImpulseConstraints = dynamicEntities.map!(entity => LinearImpulseConstraint!N(entity, V3(0, -9.8*entity.mass*_unitTime, 0))).array;
				
				dynamicEntities.each!((ref entity) => entity.updateCollisionConstraintPairs(_mapConstraintDetector.detectedStaticEntities(entity)));
				
				CollisionConstraintPair!N[] collisionConstraintPairs;
				
				_constraintSolver.solve(
					dynamicEntities, 
					collisionConstraintPairs,
					linkConstraintPairs,
					linearImpulseConstraints, 
				);
			}
			
			import std.algorithm.iteration:each;
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
	entity.updatePreStatus;
	integrator.integrate(entity);
	entity.updatePreVelocity;
	entity.updateProperties;
}
