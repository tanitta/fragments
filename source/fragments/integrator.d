module fragments.integrator;
import fragments.entity;
import armos;

/++
++/
class Integrator(NumericType) {
	alias N = NumericType;
	alias Q = ar.Quaternion!(N);
	
	public{
		/++
		++/
		this(){
			_unitTime = N(0);
		}
		
		/++
		++/
		void unitTime(N t){_unitTime = t;}
		
		/++
		++/
		N unitTime()const{return _unitTime;}
		unittest{
			auto integrator = new Integrator!(double)(1);
			assert(integrator.unitTime == 1);
			integrator.unitTime = 2;
			assert(integrator.unitTime == 2);
		}
		
		void integrate(ref DynamicEntity!(N)[] dynamicEntities){
			foreach (entity; dynamicEntities) {
				integratePosition(entity);
				integrateOrientation(entity);
			}
		}
	}//public

	private{
		N _unitTime;
		
		void integratePosition(DynamicEntity!(N) entity){
			with(entity){
				position = position + linearVelocity * _unitTime;
			}
		}
		
		void integrateOrientation(DynamicEntity!(N) entity){
			with(entity){
				if(angularVelocity.norm > 0.0){
					auto qAngularVelocityPerUnitTime = Q.angleAxis(angularVelocity.norm*_unitTime, angularVelocity.normalized);
					orientation = qAngularVelocityPerUnitTime * orientation;
				}
			}
		}
	}//private
}//class Integrator

private X euler(X, T)(X x, X v, T unitTime){
	return x + v * unitTime;
}

// private X modifiedEuler(X, T)(X x, X v, T unitTime){
// 	return x;
// }
