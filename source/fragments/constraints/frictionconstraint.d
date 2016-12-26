module fragments.constraints.frictionconstraint;

import armos.math;
import fragments.entity;

/++
+/
struct FrictionConstraint(NumericType) {
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    public{
        ///
        this(
            in V3 velocity,
            in N jacDiagInv,
            in V3 applicationPoint, 
            in V3 direction, 
        )in{
            import std.math;
            assert(!isNaN(velocity[0]));
            assert(!isNaN(jacDiagInv));
            assert(!isNaN(applicationPoint[0]));
            assert(!isNaN(direction[0]));
        }body{
            _jacDiagInv = jacDiagInv;
            _applicationPoint = applicationPoint;
            _direction= direction;
            
            _initialImpulse = -impulse(velocity);
        }
        
        /// 
        V3[2] deltaVelocities(DynamicEntity!N dynamicEntity)const
        in{
            assert(dynamicEntity);
            import std.math;
            assert(!isNaN(dynamicEntity.deltaLinearVelocity[0]));
            assert(!isNaN(dynamicEntity.deltaAngularVelocity[0]));
            assert(!isNaN(_applicationPoint[0]));
        }body{
            immutable V3 deltaVelocity = dynamicEntity.deltaLinearVelocity + dynamicEntity.deltaAngularVelocity.vectorProduct(_applicationPoint);
            
            import std.algorithm;
            immutable N deltaImpluse = (_initialImpulse - impulse(deltaVelocity)).clamp(lowerLimit, upperLimit);
            
            immutable V3 deltaLinearVelocity = deltaImpluse * dynamicEntity.massInv * _direction;
            immutable V3 deltaAngularVelocity = deltaImpluse * dynamicEntity.inertiaGlobalInv * _applicationPoint.vectorProduct(_direction);
            
            immutable V3[2] v = [
                deltaLinearVelocity,
                deltaAngularVelocity,
            ];
            return v;
        };
        
        N lowerLimit;
        N upperLimit;
    }//public

    private{
        N _jacDiagInv;
        V3 _applicationPoint;
        V3 _direction;
        
        N _initialImpulse;
        
        N impulse(in V3 deltaVelocity)const
        in{
            import std.math;
            assert(!isNaN(deltaVelocity[0]));
        }body{
            immutable N impulse = _jacDiagInv * (_direction.dotProduct(deltaVelocity));
            
            return impulse;
        }
    }//private
}//struct CollisionConstraint

