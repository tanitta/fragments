module fragments.joint;

import armos.math;
import fragments.entity;
import fragments.constraintpairs.linkconstraintpair;
import fragments.constraints.linearlinkconstraint;
import fragments.constraints.angularlinkconstraint;

/++
+/
template BallJoint(NumericType) {
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    alias M33 = Matrix!(N, 3, 3);
    public{
        LinkConstraintPair!N BallJoint(
            DynamicEntity!N entityA, DynamicEntity!N entityB, 
            in V3 applicationPointA, in V3 applicationPointB,
        ){
            return new LinkConstraintPair!N(
                entityA, entityB,
                applicationPointA, applicationPointB, 
                [
                    LinearLinkConstraint!N(V3(1, 0, 0)), 
                    LinearLinkConstraint!N(V3(0, 1, 0)), 
                    LinearLinkConstraint!N(V3(0, 0, 1)), 
                ],
                []
            );
        };
    }//public

    private{
    }//private
}//template BallJoint
unittest{
    import fragments.square;
    import fragments.material;
    alias N = double;
    alias V3 = Vector!(N, 3);
    auto material = new Material!N;
    auto entityA = new Square!N(material);
    auto entityB = new Square!N(material);
    assert(__traits(compiles, {
        auto ballJoint = BallJoint!N(
            entityA, entityB,
            V3(0, 0, 1), 
            V3(0, 0, -1), 
        );
    }));
}

/++
++/
template FixedJoint(NumericType) {
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    alias M33 = Matrix!(N, 3, 3);
    public{
        LinkConstraintPair!N FixedJoint(
            DynamicEntity!N entityA, DynamicEntity!N entityB, 
            in V3 applicationPointA, in V3 applicationPointB,
        ){
            return new LinkConstraintPair!N(
                entityA, entityB,
                applicationPointA, applicationPointB, 
                [
                    LinearLinkConstraint!N(V3(1, 0, 0)), 
                    LinearLinkConstraint!N(V3(0, 1, 0)), 
                    LinearLinkConstraint!N(V3(0, 0, 1)), 
                ],
                [
                    AngularLinkConstraint!N(V3(1, 0, 0)), 
                    AngularLinkConstraint!N(V3(0, 1, 0)), 
                    AngularLinkConstraint!N(V3(0, 0, 1)), 
                ]
            );
        };
    }//public

    private{
    }//private
}//template BallJoint
unittest{
    import fragments.square;
    import fragments.material;
    alias N = double;
    alias V3 = Vector!(N, 3);
    auto material = new Material!N;
    auto entityA = new Square!N(material);
    auto entityB = new Square!N(material);
    assert(__traits(compiles, {
        auto fixedJoint = FixedJoint!N(
            entityA, entityB,
            V3(0, 0, 1), 
            V3(0, 0, -1), 
        );
    }));
}
