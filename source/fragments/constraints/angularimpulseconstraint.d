module fragments.constraints.angularimpulseconstraint;

import armos.math;
import fragments.entity;

/++
+/
struct AngularImpulseConstraint(NumericType) {
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    
    public{
        /// 
        this(DynamicEntity!N dynamicEntity, in V3 impulse){
            this.entity = dynamicEntity; 
            _impulse = impulse;
        }
        
        /// 
        V3[2] deltaVelocities(DynamicEntity!N dynamicEntity)
        in{
            assert(dynamicEntity);
            import std.math;
            assert(!isNaN(dynamicEntity.deltaAngularVelocityVelocity[0]));
        }body{
            immutable V3 deltaAngularVelocity = _impulse * dynamicEntity.inertiaGlobalInv;
            immutable V3[2] v = [
                V3.zero,
                deltaAngularVelocity
            ];
            return v;
        };
        
        ///
        DynamicEntity!N entity;
    }//public

    private{
        V3 _impulse;
        N _initialImpulse;
    }//private
}//struct AngularImpulseConstraint
