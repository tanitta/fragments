module fragments.constraints.linearlinkconstraint;

import armos.math;
import fragments.entity;

/++
+/
struct LinearLinkConstraint(NumericType) {
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
        V3[2][2] deltaVelocities(DynamicEntity!N[] entities)const in{
            import std.math;
            assert(!isNaN(_initialImpulse));
        }body{
            immutable V3 deltaVelocity = (entities[0].deltaLinearVelocity + entities[0].deltaAngularVelocity.vectorProduct(_applicationPoints[0]))
                                       - (entities[1].deltaLinearVelocity + entities[1].deltaAngularVelocity.vectorProduct(_applicationPoints[1]));
            
            import std.algorithm;
            immutable N deltaImpluse = (_initialImpulse - impulse(deltaVelocity));
            
            // immutable V3 deltaLinearVelocity = deltaImpluse * dynamicEntity.massInv * _direction;
            // immutable V3 deltaAngularVelocity = deltaImpluse * dynamicEntity.inertiaGlobalInv * _applicationPoint.vectorProduct(_rotatedDirection);
            
            immutable V3[2][2] v = [
                [
                    deltaImpluse * entities[0].massInv * _rotatedDirection,
                    deltaImpluse * entities[0].inertiaGlobalInv * _applicationPoints[0].vectorProduct(_rotatedDirection)
                ], 
                [
                    deltaImpluse * entities[1].massInv * _rotatedDirection,
                    deltaImpluse * entities[1].inertiaGlobalInv * _applicationPoints[1].vectorProduct(_rotatedDirection)
                ], 
            ];
            return v;
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
        void updateBias(in N gain, in N slop, in V3 distance, in N unitTime){
            import std.math;
            _biasTerm = (gain * (_rotatedDirection.dotProduct(distance)))/unitTime;
        };
        
        /++
        +/
        void localDirection(in V3 localDirection){
            _localDirection = localDirection;
        }
    }//public

    private{
        N _initialImpulse;
        
        V3 _localDirection;
        V3 _rotatedDirection;
        
        N _jacDiagInv;
        V3[2] _applicationPoints;
        
        N _biasTerm;
        
        N impulse(in V3 deltaVelocity)const
        in{
            import std.math;
            assert(!isNaN(deltaVelocity[0]));
        }body{
            immutable N impulse = _jacDiagInv * (_rotatedDirection.dotProduct(deltaVelocity) - _biasTerm);
            return impulse;
        }
    }//private
}//struct LinkConstraint
unittest{
    import fragments.square;
    import fragments.material;
    alias N = double;
    auto material = new Material!N;
    auto entityA = new Square!N(material);
    auto entityB = new Square!N(material);
    
    assert(__traits(compiles, {
        auto linkConstraint= LinearLinkConstraint!N();
    }));
}
