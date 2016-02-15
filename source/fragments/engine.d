module fragments.engine;
import fragments.entity;
import fragments.mapconstraintdetector;
import fragments.integrator;

/++
++/
class Engine(NumericType){
	alias N = NumericType;
	
	public{
		/++
		++/
		this(){
			_unitTime = N(0.0);
			_mapConstraintDetector = new MapConstraintDetector!(N);
			_integrator = new Integrator!(N);
			_integrator.unitTime = _unitTime;
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
		void setStaticEntities(ref StaticEntity!(N)[] staticEntities){
			_mapConstraintDetector.setStaticEntities( staticEntities );
		};
		
		/++
		++/
		void update( ref DynamicEntity!(N)[] dynamicEntities ){
			_mapConstraintDetector.detectConstraintPairs( dynamicEntities );
			_integrator.integrate( dynamicEntities );
		};
	}//public

	private{
		N _unitTime;
		MapConstraintDetector!(N) _mapConstraintDetector;
		Integrator!(N) _integrator;
	}//private
}//class Engine
