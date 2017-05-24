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
        V3[2][2] deltaVelocities(in DynamicEntity!N[] entities)const in{
            import std.math;
            assert(!isNaN(_initialImpulse));
        }out(v){
            import std.math;
            assert(!isNaN(v[0][0][0]));
            assert(!isNaN(v[0][1][0]));
            assert(!isNaN(v[1][0][0]));
            assert(!isNaN(v[1][1][0]));
        } body{
            immutable V3 deltaVelocity = (entities[0].deltaLinearVelocity + entities[0].deltaAngularVelocity.vectorProduct(_applicationPoints[0]))
                                       - (entities[1].deltaLinearVelocity + entities[1].deltaAngularVelocity.vectorProduct(_applicationPoints[1]));
            
            import std.algorithm;
            immutable N deltaImpluse = (_initialImpulse - impulse(deltaVelocity));
            
            return [
                [
                    deltaImpluse * entities[0].massInv * _rotatedDirection,
                    deltaImpluse * entities[0].inertiaGlobalInv * _applicationPoints[0].vectorProduct(_rotatedDirection)
                ], 
                [
                    deltaImpluse * entities[1].massInv * _rotatedDirection,
                    deltaImpluse * entities[1].inertiaGlobalInv * _applicationPoints[1].vectorProduct(_rotatedDirection)
                ], 
            ];
        }
        
        /++
        +/
        ref typeof(this) update(in DynamicEntity!N entityA, in DynamicEntity!N entityB, in V3[] rotatedLocalApplicationPoints){
            _rotatedDirection = entityA.orientation.rotatedVector(_localDirection);
            _applicationPoints = rotatedLocalApplicationPoints;
            // _jacDiagInv = N(1)/((massAndInertiaTermInv(rotatedLocalApplicationPoints[0], entityA.massInv, entityA.inertiaGlobalInv, rotatedLocalApplicationPoints[1], entityB.massInv, entityB.inertiaGlobalInv)*_rotatedDirection).dotProduct(_rotatedDirection));
            import fragments.constraints.utils;
            _jacDiagInv = linearLinkJacDiagInv(
                entityA.orientation, entityB.orientation,
                _applicationPoints[0], _applicationPoints[1],
                _localDirection,
                entityA.inertiaGlobalInv, entityB.inertiaGlobalInv,
                entityA.massInv, entityB.massInv
            );
            return this;
        }
        
        ///
        ref typeof(this) updateInitialImpulse(in V3 velocity)in{
            import std.math;
            assert(!isNaN(velocity[0]));
        }body{
            _initialImpulse = -impulse(velocity);
            return this;
        }
        
        ///
        ref typeof(this) updateBias(in V3 distance, in N unitTime){
            import std.math;
            _biasTerm = (_spring * (_rotatedDirection.dotProduct(distance)))/unitTime;
            return this;
        };
        
        /++
        +/
        ref typeof(this) localDirection(in V3 localDirection){
            _localDirection = localDirection;
            return this;
        }

        ///
        ref typeof(this) spring(in N s){
            _spring = s;
            return this;
        }

        ///
        N spring()const{
            return _spring;
        }

        ///
        ref typeof(this) damper(in N d){
            _damper = d;
            return this;
        }

        ///
        N damper()const{
            return _damper;
        }
    }//public

    private{
        N _spring = 0.5;
        N _damper = 1.0;

        N _erp = 0.5;
        N _cfm = 1.0;

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
            immutable N impulse = _jacDiagInv * (_rotatedDirection.dotProduct(deltaVelocity)*_damper - _biasTerm);
            return impulse;
        }
    }//private
}//struct LinkConstraint
unittest{
    import fragments.entities.square;
    import fragments.material;
    alias N = double;
    auto material = new Material!N;
    auto entityA = new Square!N(material);
    auto entityB = new Square!N(material);
    
    assert(__traits(compiles, {
        auto linkConstraint= LinearLinkConstraint!N();
    }));
}
