module fragments.constraints.utils;

import armos.math;

///
N collisionJacDiagInv(N, V3 = Vector!(N, 3), M33 = Matrix!(N, 3, 3))(
    in V3 applicationPoint,
    in N massInv,
    in M33 inertiaGlobalInv, 
    in V3 normal, 
)in{
    import std.math;
    assert(!isNaN(applicationPoint[0]));
    assert(!isNaN(massInv));
    assert(!isNaN(inertiaGlobalInv[0][0]));
    assert(!isNaN(normal[0]));
}body{
    immutable M33 k = massAndInertiaTermInv(
        applicationPoint,
        massInv,
        inertiaGlobalInv, 
    );
    return N(1)/((k * normal).dotProduct(normal));
}

//TODO inv?
///
N linearLinkJacDiagInv(N, V3 = Vector!(N, 3), M33 = Matrix!(N, 3, 3), Q = Quaternion!N)(
    in Q orientationA, 
    in Q orientationB,
    in V3 targetGlobalPosA,
    in V3 targetGlobalPosB,
    in V3 jointAxis,
    in M33 inertiaInvA,
    in M33 inertiaInvB,
    in N massInvA,
    in N massInvB
){
    immutable jA = (targetGlobalPosA.vectorProduct(orientationA.rotatedVector(-jointAxis)));
    immutable jB = (targetGlobalPosB.vectorProduct(orientationB.rotatedVector(jointAxis)));
    immutable minvJtA = inertiaInvA.inertiaAroundAxis(jA);
    immutable minvJtB = inertiaInvB.inertiaAroundAxis(jB);
    immutable r = (massInvA + minvJtA+ massInvB + minvJtB);
    assert(r > N(0));
    return r;
}
unittest{
    alias N = double;
    alias M33 = Matrix!(N, 3, 3);
    alias V3 = Vector!(N, 3);
    assert(__traits(compiles, {
        immutable r = linearLinkJacDiagInv(M33.identity, M33.identity, V3.zero, V3.zero, V3(0, 0, 1), M33.identity, M33.identity, N(1), N(1));
    }));
}

//TODO
///
auto  angularLinkJacDiagInv(V3, M33, Q)(
    in V3 jointAxis,
    in Q orientationA, 
    in Q orientationB,
    in M33 inertiaInvA,
    in M33 inertiaInvB
){
    immutable jA = orientationA.rotatedVector(jointAxis);
    immutable jB = orientationB.rotatedVector(-jointAxis);
    immutable minvJtA = inertiaInvA.inertiaAroundAxis(jA);
    immutable minvJtB = inertiaInvB.inertiaAroundAxis(jB);
    immutable r = minvJtA + minvJtB;
    alias N = V3.elementType;
    assert(r > N(0));
    return r;
}

///
M33 massAndInertiaTermInv(N, V3 = Vector!(N, 3), M33 = Matrix!(N, 3, 3))(
    in V3 applicationPoint,
    in N massInv,
    in M33 inertiaGlobalInv, 
){
    immutable rCrossMatrix = applicationPoint.crossMatrix;
    return massInv * M33.identity - rCrossMatrix * inertiaGlobalInv * rCrossMatrix;
}

///
M33 massAndInertiaTermInv(N, V3 = Vector!(N, 3), M33 = Matrix!(N, 3, 3))(
    in V3 applicationPointA,
    in N massInvA,
    in M33 inertiaGlobalInvA, 
    in V3 applicationPointB,
    in N massInvB,
    in M33 inertiaGlobalInvB, 
){
    return massAndInertiaTermInv(applicationPointA, massInvA, inertiaGlobalInvA) + massAndInertiaTermInv(applicationPointB, massInvB, inertiaGlobalInvB);
}

///
M33 crossMatrix(V3, M33 = Matrix!(typeof(V3[0]), 3, 3))(in V3 vector){
    return M33(
        [ 0,          -vector[2], vector[1]  ],
        [ vector[2],  0,          -vector[0] ],
        [ -vector[1], vector[0],  0          ],
    );
}

///
N inertiaAroundAxis(V3, M33, N = V3.elementType)(
    in M33 inertia, 
    in V3 axis,
){
    immutable v = V3(
        axis.x * inertia[0][0] + axis.y * inertia[1][0] + axis.z * inertia[2][0],
        axis.x * inertia[0][1] + axis.y * inertia[1][1] + axis.z * inertia[2][1],
        axis.x * inertia[0][2] + axis.y * inertia[1][2] + axis.z * inertia[2][2],
        // axis.x * inertia[0][0] + axis.y * inertia[0][1] + axis.z * inertia[0][2],
        // axis.x * inertia[1][0] + axis.y * inertia[1][1] + axis.z * inertia[1][2],
        // axis.x * inertia[2][0] + axis.y * inertia[2][1] + axis.z * inertia[2][2],
    );
    return v.x * axis.x + v.y * axis.y + v.z * axis.z;
}
unittest{
    alias N = double;
    alias M33 = Matrix!(N, 3, 3);
    alias V3 = Vector!(N, 3);
    auto v = V3(1, 2, 3);
    auto m = M33(
        [1, 2, 3],
        [2, 3, 4],
        [3, 4, 5]
    );
    assert(inertiaAroundAxis(m, v) == 132);
}
