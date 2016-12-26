module fragments.constraints.utils;

import armos.math;

N jacDiagInv(N, V3 = Vector!(N, 3), M33 = Matrix!(N, 3, 3))(
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

M33 massAndInertiaTermInv(N, V3 = Vector!(N, 3), M33 = Matrix!(N, 3, 3))(
    in V3 applicationPoint,
    in N massInv,
    in M33 inertiaGlobalInv, 
){
    immutable rCrossMatrix = applicationPoint.crossMatrix;
    return massInv * M33.identity - rCrossMatrix * inertiaGlobalInv * rCrossMatrix;
}

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

M33 crossMatrix(V3, M33 = Matrix!(typeof(V3[0]), 3, 3))(in V3 vector){
    return M33(
        [ 0,          -vector[2], vector[1]  ],
        [ vector[2],  0,          -vector[0] ],
        [ -vector[1], vector[0],  0          ],
    );
}
