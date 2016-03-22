module fragments.engine;
import fragments.entity;
import fragments.mapconstraintdetector;
import fragments.constraintsolver;
import fragments.integrator;

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
			
			_unitTime = N(0.0);
			this.unitTime = _unitTime;
		}
		
		/++
		++/
		void unitTime(in N t){
			_unitTime = t;
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
		void update( ref DynamicEntity!(N)[] dynamicEntities ){
			{
				auto collisionConstraintPairs = _mapConstraintDetector.detectConstraintPairs( dynamicEntities );
				_constraintSolver.solve(collisionConstraintPairs);
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
