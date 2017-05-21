module fragments.constraintpairs.linkconstraintpair;

import armos.math;

import fragments.entity;
import fragments.constraints.utils;
import fragments.contactpoint;
import fragments.constraints;

/++
+/
interface LinkConstraintPair(NumericType) {
    private {
        alias N = NumericType;
        alias V3 = Vector!(N, 3);
        alias M33 = Matrix!(N, 3, 3);
        alias Q   = Quaternion!(N);
    }
    
    public{
        ///
        DynamicEntity!N[] dynamicEntities();

        ///
        void update(in N unitTime);

        ///
        LinearLinkConstraint!N[] linearLinkConstraints();
        
        ///
        AngularLinkConstraint!N[] angularLinkConstraints();
        
    }//public
}//interface LinkConstraintPair

// unittest{
//     import fragments.entities.square;
//     import fragments.material;
//     alias N = double;
//     auto material = new Material!N;
//     auto entityA = new Square!N(material);
//     auto entityB = new Square!N(material);
//     // assert(__traits(compiles, {
//     //     auto linkConstraintPair = new LinkConstraintPair!N(entityA, entityB);
//     // }));
// }
// unittest{
//     import fragments.entities.square;
//     import fragments.material;
//     alias N = double;
//     auto material = new Material!N;
//     auto entityA = new Square!N(material);
//     auto entityB = new Square!N(material);
//     // auto linkConstraintPair = new LinkConstraintPair!N(entityA, entityB);
// }
//
// // TODO Need to rewrite.
// Vector!(N, 3) matrixToEulerXYZ(N)(in Matrix!(N, 3, 3) matrix){
// 	//	// rot =  cy*cz          -cy*sz           sy
// 	//	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
// 	//	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
// 	//
//     Vector!(N, 3) xyz;
// 	N fi = getMatrixElem(matrix,2);
//     import std.math;
// 	if (fi < N(1.0f))
// 	{
// 		if (fi > N(-1.0f))
// 		{
// 			xyz[0] = atan2(-getMatrixElem(matrix,5),getMatrixElem(matrix,8));
// 			xyz[1] = asin(getMatrixElem(matrix,2));
// 			xyz[2] = atan2(-getMatrixElem(matrix,1),getMatrixElem(matrix,0));
// 			// return true;
// 		}
// 		else
// 		{
// 			// WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
// 			xyz[0] = -atan2(getMatrixElem(matrix,3),getMatrixElem(matrix,4));
// 			xyz[1] = -PI*0.5;
// 			xyz[2] = N(0.0);
// 			// return false;
// 		}
// 	}
// 	else
// 	{
// 		// WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
// 		xyz[0] = atan2(getMatrixElem(matrix,3),getMatrixElem(matrix,4));
// 		xyz[1] = PI*0.5;
// 		xyz[2] = 0.0;
// 	}
// 	// return false;
//     return xyz;
// }
//
// // TODO Need to rewrite.
// N getMatrixElem(N)(in Matrix!(N, 3, 3) matrix, in size_t index){
// 	immutable size_t i = (index%3);
// 	immutable size_t j = (index/3);
// 	return matrix[i][j];
// }
