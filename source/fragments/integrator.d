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
		void unitTime(in N t){_unitTime = t;}
		
		/++
		++/
		N unitTime()const{return _unitTime;}
		unittest{
			auto integrator = new Integrator!(double)();
			assert(integrator.unitTime == 1.0);
			integrator.unitTime = 2.0;
			assert(integrator.unitTime == 2.0);
		}
		
		void integrate(ref DynamicEntity!(N)[] dynamicEntities){
			import std.algorithm.iteration:each;
			dynamicEntities.each!(entity => integrate(entity));
		}
		
		void integrate(DynamicEntity!(N) entity){
			integratePosition(entity);
			integrateOrientation(entity);
		}
	}//public

	private{
		N _unitTime;
		
		void integratePosition(DynamicEntity!(N) entity){
			with(entity){
				// position = position + (linearVelocity + linearVelocityPre) * 0.5 * _unitTime;
				position = position + linearVelocity * _unitTime;
			}
		}
		
		void integrateOrientation(DynamicEntity!(N) entity){
			with(entity){
				if(angularVelocity.norm > 0.0){
					// immutable angularVelocityAvg = (angularVelocity + angularVelocityPre) * 0.5;
					// immutable qAngularVelocityPerUnitTime = Q.angleAxis(angularVelocityAvg.norm*_unitTime, angularVelocityAvg.normalized);
					
					immutable qAngularVelocityPerUnitTime = Q.angleAxis(angularVelocity.norm*_unitTime, angularVelocity.normalized);
					orientation = qAngularVelocityPerUnitTime * orientation;
				}
			}
		}
	}//private
}//class Integrator

