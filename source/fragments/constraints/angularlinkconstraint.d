module fragments.constraints.angularlinkconstraint;

import armos.math;
import fragments.entity;

/++
+/
struct AngularLinkConstraint(NumericType){
    alias N   = NumericType;
    alias V3  = Vector!(N, 3);
    alias M33 = Matrix!(N, 3, 3);
    alias Q   = Quaternion!(N);
    
    public{
        /++
        +/
        this(in V3 localDirection){
            _localDirection = localDirection;
        }
        
        /++
        +/
        V3[2][2] deltaVelocities(in DynamicEntity!N[] entities)const in{
            import std.math;
            assert(!isNaN(_initialImpulse));
        }out(v){
            import std.math;
            assert(!isNaN(v[0][0][0]));
            assert(!isNaN(v[0][1][0]));
            assert(!isNaN(v[1][0][0]));
            assert(!isNaN(v[1][1][0]));
        }body{
            immutable V3 deltaVelocity = (entities[0].deltaAngularVelocity - entities[1].deltaAngularVelocity);
            
            import std.algorithm;
            immutable N deltaImpluse = (_initialImpulse - impulse(deltaVelocity))*0.05;
            
            return [
                [
                    V3.zero, 
                    deltaImpluse * entities[0].inertiaGlobalInv * _rotatedDirection,
                ], 
                [
                    V3.zero,
                    deltaImpluse * entities[1].inertiaGlobalInv * _rotatedDirection,
                ], 
            ];
        }
        
        /++
        +/
        void update(in Q orientation, in M33 massAndInertiaTermInv, in V3[] rotatedLocalApplicationPoints)in{
            import std.math;
            assert(!isNaN(orientation[0]));
            assert(!isNaN(massAndInertiaTermInv[0][0]));
        }body{
            _rotatedDirection = orientation.rotatedVector(_localDirection);
            _jacDiagInv = N(1)/((massAndInertiaTermInv*_rotatedDirection).dotProduct(_rotatedDirection));
            _applicationPoints = rotatedLocalApplicationPoints;
        }
        
        ///
        void updateInitialImpulse(in V3 velocity)in{
            import std.math;
            assert(!isNaN(velocity[0]));
        }body{
            _initialImpulse = -impulse(velocity);
        }
        
        ///
        void updateBias(in N gain, in N slop, in Q distance, in N unitTime){
            immutable N spring = 0.5;
            import std.math;
            immutable referenceVectorA = _localDirection.OrthogonalNormalizedVector;
            //TODO
            immutable referenceVectorB = distance.rotatedVector(referenceVectorA);
            immutable deflection = referenceVectorA.vectorProduct(referenceVectorB);

            immutable tmp = _localDirection.dotProduct(deflection);
            if(tmp > N(0)){
                _biasTerm = referenceVectorA.angle(referenceVectorB);
            }else{
                _biasTerm = -referenceVectorA.angle(referenceVectorB);
            }
        };
        
        /++
        +/
        void localDirection(in V3 localDirection){
            _localDirection = localDirection;
        }
    }//public

    private{
        N _initialImpulse;
        
        V3 _localDirection; // ローカル拘束軸
        V3 _rotatedDirection; // ワールド拘束軸
        
        N _jacDiagInv;
        V3[2] _applicationPoints;
        
        N _biasTerm;
        
        N impulse(in V3 deltaVelocity)const
        in{
            import std.math;
            assert(!isNaN(deltaVelocity[0]));
        }body{
            immutable N damper = 1.0;
            immutable N impulse = _jacDiagInv * (_rotatedDirection.dotProduct(deltaVelocity)*damper - _biasTerm);
            return impulse;
        }
        
    }//private
}//struct LinkConstraint

private V3 OrthogonalNormalizedVector(V3)(in V3 v){
    immutable tmp = V3(v[1], v[2], v[0]);
    return v.vectorProduct(tmp).normalized;
}
unittest{
    immutable v = Vector!(double, 3)(1, 2, 3);
    immutable orthogonalNormalizedVector = OrthogonalNormalizedVector(v);
    import std.math;
    assert(approxEqual(v.dotProduct(orthogonalNormalizedVector), 0));
}

