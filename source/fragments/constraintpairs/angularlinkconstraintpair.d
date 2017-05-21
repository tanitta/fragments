module fragments.constraintpairs.angularlinkconstraintpair;

import armos.math;
import fragments.constraintpairs.linkconstraintpair;
import fragments.entity;

///
class AngularLinkConstraintPair(NumericType) : LinkConstraintPair!(NumericType){
    alias N   = NumericType;
    alias V3  = Vector!(N, 3);
    alias M33 = Matrix!(N, 3, 3);
    alias Q   = Quaternion!(N);
   
    public{
        /++
        ++/
        this(
            DynamicEntity!N entityA, DynamicEntity!N entityB,
            in V3 localApplicationPointA, in V3 localApplicationPointB,
            in N offset,
            AngularLinkConstraint!N angularLinkConstraint, 
        ){
            _dynamicEntities[0] = entityA;
            _dynamicEntities[1] = entityB;
           
            _localApplicationPoints[0] = localApplicationPointA;
            _localApplicationPoints[1] = localApplicationPointB;
           
            _angularLinkConstraints = [angularLinkConstraint];

            immutable axis1 = _angularLinkConstraints.localDirection.orthogonalNormalizedVector;
            immutable axis2 = axis1.orthogonalNormalizedVector;

            _linearLinkConstraints  = [LinearLinkConstraint!N(_angularLinkConstraints.localDirection), 
                                       LinearLinkConstraint!N(axis1),
                                       LinearLinkConstraint!N(axis1),
                                       LinearLinkConstraint!N(axis2),
                                       LinearLinkConstraint!N(axis2)];

            import std.algorithm:map;
            import std.array:array;
            _initialAngularAxises = _angularLinkConstraints.map!(c => c.localDirection)
                                                           .array;
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
        V3[] _initialAngularAxises;
       
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

                // immutable distance = _dynamicEntities[1].orientation*_dynamicEntities[0].orientation.inverse;
                immutable Q distance = rotationDifference(_dynamicEntities[1].orientation,
                                                          _dynamicEntities[0].orientation);

                N[] angles = new N[](_angularLinkConstraints.length);
               
                Q angleQuaternion = Q.unit;
                foreach (size_t i, ref angularLinkConstraint; _angularLinkConstraints) {
                    //Project rotated referenceVector to plane.
                    // immutable angleQuaternion = Q.angleAxis(_angle, _initialAngularAxises[i]);

                    _angularLinkConstraints[i].localDirection = angleQuaternion.rotatedVector(_initialAngularAxises[i]);
                    angles[i] = calcAngle(_initialAngularAxises[i], distance*angleQuaternion.inverse);
                    // angleQuaternion = angleQuaternion * Q.angleAxis(-angles[i], _angularLinkConstraints[i].localDirection);
                    angleQuaternion = angleQuaternion*Q.angleAxis(angles[i], _initialAngularAxises[i]);
                    // angleQuaternion = angleQuaternion*Q.angleAxis(angles[i], _angularLinkConstraints[i].localDirection);

                    // Extract rotational element around _localDirection.
                    // immutable referenceVectorA = _initialAngularAxises[i].orthogonalNormalizedVector;
                    // angles[i] = 0



                    angularLinkConstraint.spring(0.5)
                                         .damper(1.0);
                    angularLinkConstraint.update(_dynamicEntities[0], _dynamicEntities[1]);
                    angularLinkConstraint.updateBias(angles[i], unitTime);
                    angularLinkConstraint.updateInitialImpulse(velocity);

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
                writeln("X ", angle/PI);
                // angle = 0;
            }else if(axis == V3(0, 1, 0)){
                V3 r = q.rotatedVector(V3(0, 0, 1));
                angle = atan2(r.x, r.z);
                writeln("Y ", angle/PI);
                // angle = 0;
            }else if(axis == V3(0, 0, 1)){
                V3 r = q.rotatedVector(V3(1, 0, 0));
                angle = atan2(r.y, r.x);
                writeln("Z ", angle/PI);
                angle = 0;
            }else{
                assert(0);
            }
            return angle;
        }
    }//private
}//class AngularLinkConstraintPair
