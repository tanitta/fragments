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
			_integrator.unitTime = t;
		}
		
		/++
		++/
		N unitTime(){return _unitTime;}
		
		/++
		++/
		void setStaticEntities(StaticEntity!(N)[] staticEntities){
			_mapConstraintDetector.setStaticEntities( staticEntities );
		};
		
		/++
		++/
		void update(ref DynamicEntity!(N)[] dynamicEntities, ref ConstraintPair!N[] constraintPairs){
			{
				auto collisionConstraintPairs = _mapConstraintDetector.detectCollisionConstraintPairs( dynamicEntities );
				_constraintSolver.solve(collisionConstraintPairs, constraintPairs);
			}
			
			_integrator.integrate( dynamicEntities );
			
			foreach (entity; dynamicEntities) {
				with(entity){
					updateProperties;
				}
			}
		};
		
		void draw(){
			_mapConstraintDetector.draw;
		}
	}//public

	private{
		N _unitTime;
		MapConstraintDetector!N _mapConstraintDetector;
		Integrator!N _integrator;
		ConstraintSolver!N _constraintSolver;
	}//private
}//class Engine
