module fragments.integrator;
import fragments.entity;
import armos;

/++
++/
class Integrator(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
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
				updateVelocitiesFromDelta(entity);
				initDeltaVelocity(entity);
				integratePosition(entity);
				integrateOrientation(entity);
			}
		}
	}//public

	private{
		N _unitTime;
		
		void updateVelocitiesFromDelta(DynamicEntity!(N) entity){
			entity.linearVelocity = entity.linearVelocity + entity.deltaLinearVelocity;
			entity.angularVelocity = entity.angularVelocity + entity.deltaAngularVelocity;
		}
		
		void initDeltaVelocity(DynamicEntity!(N) entity){
			entity.deltaLinearVelocity = V3.zero;
			entity.deltaAngularVelocity = V3.zero;
		}
		
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

