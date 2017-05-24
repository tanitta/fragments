module fragments.constraintpairs.linearlinkconstraintpair;

import armos.math;
import fragments.constraintpairs.linkconstraintpair;
import fragments.entity;
import fragments.constraints.utils;
import fragments.contactpoint;
import fragments.constraints;

///
class LinearLinkConstraintPair(NumericType) : LinkConstraintPair!(NumericType){
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    alias M33 = Matrix!(N, 3, 3);
    alias Q   = Quaternion!(N);
   
    public{
        /++
        ++/
        this(
            DynamicEntity!N entityA, DynamicEntity!N entityB,
            in V3 localApplicationPointA, in V3 localApplicationPointB,
            in Q orientationalOffsetA, 
            in Q orientationalOffsetB, 
            LinearLinkConstraint!N[] linearLinkConstraints, 
            // AngularLinkConstraint!N[] angularLinkConstraints, 
        ){
            _dynamicEntities[0] = entityA;
            _dynamicEntities[1] = entityB;
           
            _localApplicationPoints[0] = localApplicationPointA;
            _localApplicationPoints[1] = localApplicationPointB;
           
            _linearLinkConstraints = linearLinkConstraints;
            _angularLinkConstraints = [AngularLinkConstraint!N(V3(1, 0, 0)), 
                                       AngularLinkConstraint!N(V3(0, 1, 0)), 
                                       AngularLinkConstraint!N(V3(0, 0, 1))];

            _orientationalOffsets[0] = orientationalOffsetA;
            _orientationalOffsets[1] = orientationalOffsetB;
        }
       
        /++
        ++/
        DynamicEntity!N[] dynamicEntities(){
            return _dynamicEntities;
        }
       
        /++
        ++/
        void update(in N unitTime){
            updateRotatedLocalApplicationPoints;
            updateMassAndInertiaTermInv;
            updateConstraints(unitTime);
        }
       
        /++
        ++/
        LinearLinkConstraint!N[] linearLinkConstraints(){
            return _linearLinkConstraints;
        }
       
        /++
        ++/
        AngularLinkConstraint!N[] angularLinkConstraints(){
            return _angularLinkConstraints;
        }
       
    }//public

    private{
        DynamicEntity!N[2] _dynamicEntities;
        M33 _massAndInertiaTermInv;
        // M33 _inertiaTermInv;
        V3[2] _localApplicationPoints;
        V3[2] _rotatedLocalApplicationPoints;
       
        V3 _localDirection;
        V3 _rotatedLocalDirection;
       
        LinearLinkConstraint!N[]  _linearLinkConstraints;
        AngularLinkConstraint!N[] _angularLinkConstraints;
        Q[2] _orientationalOffsets;
       
        void updateRotatedLocalApplicationPoints(){
            foreach (int index, ref rotatedLocalApplicationPoint; _rotatedLocalApplicationPoints) {
                rotatedLocalApplicationPoint = _dynamicEntities[index].orientation.rotatedVector(_localApplicationPoints[index]);
            }
        }
       
        void updateMassAndInertiaTermInv()in{
            import std.math;
            assert(!isNaN(_dynamicEntities[0].inertiaGlobalInv[0][0]));
            assert(!isNaN(_dynamicEntities[0].massInv));
            assert(!isNaN(_dynamicEntities[1].inertiaGlobalInv[0][0]));
            assert(!isNaN(_dynamicEntities[1].massInv));
            assert(!isNaN(_rotatedLocalApplicationPoints[0][0]));
            assert(!isNaN(_rotatedLocalApplicationPoints[1][0]));
        }body{
            _massAndInertiaTermInv = M33.zero;
            for (int i = 0; i < 2; i++) {
                _massAndInertiaTermInv = _massAndInertiaTermInv + massAndInertiaTermInv(
                    _rotatedLocalApplicationPoints[i],
                    _dynamicEntities[i].massInv,
                    _dynamicEntities[i].inertiaGlobalInv
                );
            }

            // _inertiaTermInv = M33.zero;
            // for (int i = 0; i < 2; i++) {
            //     _inertiaTermInv = _inertiaTermInv + inertiaAroundAxis(
            //         _rotatedLocalDirection, 
            //         _dynamicEntities[i].inertiaGlobalInv
            //     );
            //
            //     // _inertiaTermInv = _inertiaTermInv + _rotatedLocalDirection *
            // }
        }
       
        void updateConstraints(in N unitTime){
           
            import std.math;
            //LinearConstraints
            {
                immutable velocity = (_dynamicEntities[0].linearVelocity + _dynamicEntities[0].angularVelocity.vectorProduct(_rotatedLocalApplicationPoints[0]))
                                    -(_dynamicEntities[1].linearVelocity + _dynamicEntities[1].angularVelocity.vectorProduct(_rotatedLocalApplicationPoints[1]));
           
                immutable distance = (_dynamicEntities[1].position+_rotatedLocalApplicationPoints[1])
                                    -(_dynamicEntities[0].position+_rotatedLocalApplicationPoints[0]);

                foreach (ref linearLinkConstraint; _linearLinkConstraints) {
                    linearLinkConstraint.spring(1.0)
                                        .damper(1.0);
                    linearLinkConstraint.update(_dynamicEntities[0], _dynamicEntities[1], _rotatedLocalApplicationPoints);
                    linearLinkConstraint.updateBias(distance, unitTime);
                    linearLinkConstraint.updateInitialImpulse(velocity);
                }
            }

            //AngularConstraints
            {
                immutable velocity = _dynamicEntities[0].angularVelocity - _dynamicEntities[1].angularVelocity;
                immutable Q distance = _orientationalOffsets[0]*rotationDifference(_dynamicEntities[1].orientation*_orientationalOffsets[1],
                                                          _dynamicEntities[0].orientation);

                foreach (size_t i, ref angularLinkConstraint; _angularLinkConstraints) {
                    immutable angle = calcAngle(angularLinkConstraint.localDirection, distance);
                    angularLinkConstraint.spring(0.5)
                                         .damper(1.0)
                                         .update(_dynamicEntities[0], _dynamicEntities[1])
                                         .updateBias(angle, unitTime)
                                         .updateInitialImpulse(velocity);
                }
            }
        }

        N calcAngle(in V3 axis, in Q q){
            N angle = 0;
            import std.math;
            import std.stdio;
            if(axis == V3(1, 0, 0)){
                V3 r = q.rotatedVector(V3(0, 1, 0));
                angle = atan2(r.z, r.y);
            }else if(axis == V3(0, 1, 0)){
                V3 r = q.rotatedVector(V3(0, 0, 1));
                angle = atan2(r.x, r.z);
            }else if(axis == V3(0, 0, 1)){
                V3 r = q.rotatedVector(V3(1, 0, 0));
                angle = atan2(r.y, r.x);
            }else{
                assert(0);
            }
            return angle;
        }
    }//private
}//class LinearLinkConstraintPair
