module fragments.constraints.linearimpulseconstraint;

import armos.math;
import fragments.entity;

/++
+/
struct LinearImpulseConstraint(NumericType) {
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
            assert(!isNaN(dynamicEntity.deltaLinearVelocity[0]));
        }body{
            immutable V3 deltaLinearVelocity = _impulse * dynamicEntity.massInv;
            immutable V3[2] v = [
                deltaLinearVelocity,
                V3.zero,
            ];
            return v;
        };
        
        DynamicEntity!N entity;
    }//public

    private{
        V3 _impulse;
        N _initialImpulse;
    }//private
}//struct LinearImpulseConstraint
