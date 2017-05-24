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
            immutable N deltaImpluse = (_initialImpulse - impulse(deltaVelocity));
            
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
        ref typeof(this) update(in DynamicEntity!N entityA, in DynamicEntity!N entityB)in{
            import std.math;
            // assert(!isNaN(orientation[0]));
            // assert(!isNaN(inertiaTermInv[0][0]));
        }body{
            import fragments.constraints.utils: inertiaAroundAxis,
                                                angularLinkJacDiagInv;
            _rotatedDirection = entityA.orientation.rotatedVector(_localDirection);
            // _jacDiagInv = N(1)/((inertiaTermInv*_rotatedDirection).dotProduct(_rotatedDirection));
            // _jacDiagInv = N(1)/(inertiaGlobalA.inertiaAroundAxis(_rotatedDirection) + inertiaGlobalB.inertiaAroundAxis(_rotatedDirection));
            _jacDiagInv = angularLinkJacDiagInv(
                _localDirection,
                entityA.orientation, entityB.orientation,
                entityA.inertiaGlobalInv, entityB.inertiaGlobalInv
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
        ref typeof(this) updateBias(in N distance, in N unitTime){
            // // Extract rotational element around _localDirection.
            // import std.math;
            // import fragments.constraints.utils:orthogonalNormalizedVector;
            // immutable referenceVectorA = _localDirection.orthogonalNormalizedVector;
            //
            // //Project rotated referenceVector to plane.
            // // TODO move angleQuaternion outside
            // immutable angleQuaternion = Q.angleAxis(- distance + _angle, _localDirection);
            // immutable pt = angleQuaternion.rotatedVector(referenceVectorA);
            // immutable d = pt.dotProduct(_localDirection);
            // immutable referenceVectorB =  (-_localDirection * d + pt).normalized;
            //
            // immutable deflection = referenceVectorA.vectorProduct(referenceVectorB);
            // immutable sign = _localDirection.dotProduct(deflection);
            // if(sign > N(0)){
            //     _biasTerm = referenceVectorA.angle(referenceVectorB)*_spring/unitTime;
            // }else{
            //     _biasTerm = -referenceVectorA.angle(referenceVectorB)*_spring/unitTime;
            // }

            // if(_localDirection == V3(0, 0, 1)){
            //     _biasTerm = 0.1;
            // }else{
            //     _biasTerm = (distance+_angle)*_spring/unitTime;
            // }
            _biasTerm = (_angle-distance)*_spring/unitTime;
            return this;
        };
        
        ///
        ref typeof(this) localDirection(in V3 localDirection){
            _localDirection = localDirection;
            return this;
        }

        ///
        V3 localDirection()const{
            return _localDirection;
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

        ///
        ref typeof(this) angle(in N a){
            _angle = a;
            return this;
        }

        ///
        N angle()const{
            return _angle;
        };
    }//public

    private{
        N _angle = N(0);

        N _spring = 0.5;
        N _damper = 1.0;

        N _initialImpulse;
        
        V3 _localDirection; // ローカル拘束軸
        V3 _rotatedDirection; // ワールド拘束軸
        
        N _jacDiagInv;
        
        N _biasTerm;
        
        N impulse(in V3 deltaVelocity)const
        in{
            import std.math;
            assert(!isNaN(deltaVelocity[0]));
        }body{
            immutable N impulse = _jacDiagInv * (_rotatedDirection.dotProduct(deltaVelocity)*_damper - _biasTerm)*0.2;
            // immutable N impulse = _jacDiagInv * (_rotatedDirection.dotProduct(deltaVelocity));
            // immutable N impulse = _jacDiagInv * (_rotatedDirection.dotProduct(deltaVelocity))*0.1;
            return impulse;
        }
        
    }//private
}//struct LinkConstraint
