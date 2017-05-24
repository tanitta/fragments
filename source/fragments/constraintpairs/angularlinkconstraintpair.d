module fragments.constraintpairs.angularlinkconstraintpair;

import armos.math;
import fragments.constraintpairs.linkconstraintpair;
import fragments.entity;
import fragments.constraints.utils;
import fragments.contactpoint;
import fragments.constraints;

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
            in Q orientationalOffsetA, 
            in Q orientationalOffsetB, 
            AngularLinkConstraint!N angularLinkConstraint, 
        ){
            _dynamicEntities[0] = entityA;
            _dynamicEntities[1] = entityB;
           
            _angularLinkConstraints = [angularLinkConstraint];

            immutable orientationalOffsettedLocalDirectionA = orientationalOffsetA.rotatedVector(_angularLinkConstraints[0].localDirection);
            immutable orientationalOffsettedLocalDirectionB = orientationalOffsetB.rotatedVector(_angularLinkConstraints[0].localDirection);

            _angularLinkConstraints[0].localDirection = orientationalOffsettedLocalDirectionA;

            immutable axis1 = orientationalOffsettedLocalDirectionA.orthogonalNormalizedVector;
            immutable axis2 = axis1.orthogonalNormalizedVector;


            _linearLinkConstraints  = [LinearLinkConstraint!N(orientationalOffsettedLocalDirectionA), 
                                       LinearLinkConstraint!N(axis1), // minus
                                       LinearLinkConstraint!N(axis1), // plus
                                       LinearLinkConstraint!N(axis2), // minus
                                       LinearLinkConstraint!N(axis2), // plus
                                      ];

            // _angularLinkConstraints[0].localDirection = orientationalOffsetA.rotatedVector(_angularLinkConstraints[0].localDirection());

            _localApplicationPoints[Offset.Zero][0]  = localApplicationPointA;
            _localApplicationPoints[Offset.Zero][1]  = localApplicationPointB;

            _localApplicationPoints[Offset.Plus][0]  = localApplicationPointA + orientationalOffsettedLocalDirectionA * offset;
            _localApplicationPoints[Offset.Plus][1]  = localApplicationPointB + orientationalOffsettedLocalDirectionB * offset;

            _localApplicationPoints[Offset.Minus][0] = localApplicationPointA - orientationalOffsettedLocalDirectionA * offset;
            _localApplicationPoints[Offset.Minus][1] = localApplicationPointB - orientationalOffsettedLocalDirectionB * offset;

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
        ///
        enum Offset {
            Zero,
            Minus, 
            Plus,
        }//enum Offset
        DynamicEntity!N[2] _dynamicEntities;

        M33[Offset] _massAndInertiaTermInvs;

        V3[2][3] _localApplicationPoints;
        V3[2][3] _rotatedLocalApplicationPoints;
       
        V3 _localDirection;
       
        LinearLinkConstraint!N[]  _linearLinkConstraints;
        AngularLinkConstraint!N[] _angularLinkConstraints;

        Q[2] _orientationalOffsets;
       
        void updateRotatedLocalApplicationPoints(){
            import std.traits;
            foreach (offset; [EnumMembers!Offset]) {
                foreach (int index, ref rotatedLocalApplicationPoint; _rotatedLocalApplicationPoints[offset]) {
                    rotatedLocalApplicationPoint = _dynamicEntities[index].orientation.rotatedVector(_localApplicationPoints[offset][index]);
                }
            }
        }
       
        void updateMassAndInertiaTermInv()in{
            import std.math;
            assert(!isNaN(_dynamicEntities[0].inertiaGlobalInv[0][0]));
            assert(!isNaN(_dynamicEntities[0].massInv));
            assert(!isNaN(_dynamicEntities[1].inertiaGlobalInv[0][0]));
            assert(!isNaN(_dynamicEntities[1].massInv));
            assert(!isNaN(_rotatedLocalApplicationPoints[Offset.Zero][0][0]));
            assert(!isNaN(_rotatedLocalApplicationPoints[Offset.Zero][1][0]));
        }body{
            import std.traits;
            foreach (offset; [EnumMembers!Offset]) {
                _massAndInertiaTermInvs[offset]= M33.zero;
                for (int i = 0; i < 2; i++) {
                    _massAndInertiaTermInvs[offset] = _massAndInertiaTermInvs[offset]
                                                    + massAndInertiaTermInv(_rotatedLocalApplicationPoints[offset][i],
                                                                            _dynamicEntities[i].massInv,
                                                                            _dynamicEntities[i].inertiaGlobalInv);
                }
            }
        }
       
        void updateConstraints(in N unitTime){
            import std.math;
            //LinearConstraints
            {
                import std.traits;
                V3[3] velocities;
                V3[3] distances;
                foreach (offset; [EnumMembers!Offset]) {
                    velocities[offset] = (_dynamicEntities[0].linearVelocity + _dynamicEntities[0].angularVelocity.vectorProduct(_rotatedLocalApplicationPoints[offset][0]))
                                        -(_dynamicEntities[1].linearVelocity + _dynamicEntities[1].angularVelocity.vectorProduct(_rotatedLocalApplicationPoints[offset][1]));

                    distances[offset]  = (_dynamicEntities[1].position+_rotatedLocalApplicationPoints[offset][1])
                                        -(_dynamicEntities[0].position+_rotatedLocalApplicationPoints[offset][0]);
                }
            
                foreach (ref size_t index, offset; [Offset.Zero, Offset.Minus, Offset.Plus, Offset.Minus, Offset.Plus]){
                    _linearLinkConstraints[index].spring(1.0)
                                                 .damper(1.0)
                                                 .update(_dynamicEntities[0], _dynamicEntities[1], _rotatedLocalApplicationPoints[offset])
                                                 .updateBias(distances[offset], unitTime)
                                                 .updateInitialImpulse(velocities[offset]);
                }
            }

            //AngularConstraints
            {
                immutable velocity = _dynamicEntities[0].angularVelocity - _dynamicEntities[1].angularVelocity;

                // immutable distance = _dynamicEntities[1].orientation*_dynamicEntities[0].orientation.inverse;
                immutable Q distance = _orientationalOffsets[0]*rotationDifference(_dynamicEntities[1].orientation*_orientationalOffsets[1],
                                                          _dynamicEntities[0].orientation);
                Q angleQuaternion = Q.unit;
                foreach (size_t i, ref angularLinkConstraint; _angularLinkConstraints) {
                    immutable angle = calcAngle(_angularLinkConstraints[i].localDirection, distance);

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
            if(axis[0] > 0.5){
                V3 r = q.rotatedVector(V3(0, 1, 0));
                angle = atan2(r.z, r.y);
            }else if(axis[1] > 0.5){
                V3 r = q.rotatedVector(V3(0, 0, 1));
                angle = atan2(r.x, r.z);
            }else if(axis[2] > 0.5){
                V3 r = q.rotatedVector(V3(1, 0, 0));
                angle = atan2(r.y, r.x);
            }else if(axis[0] < -0.5){
                V3 r = q.rotatedVector(V3(0, 1, 0));
                angle = -atan2(r.z, r.y);
            }else if(axis[1] < -0.5){
                V3 r = q.rotatedVector(V3(0, 0, 1));
                angle = -atan2(r.x, r.z);
            }else if(axis[2] < -0.5){
                V3 r = q.rotatedVector(V3(1, 0, 0));
                angle = -atan2(r.y, r.x);
            }else{
                assert(0);
            }
             return angle;
        }
    }//private
}//class AngularLinkConstraintPair
