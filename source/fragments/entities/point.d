module fragments.entities.point;

import armos.math;
import fragments.entity;
import fragments.material;

/++
++/
class Point(NumericType) : DynamicEntity!(NumericType){
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    alias Q = Quaternion!N;
    mixin DynamicEntityProperties!N;
    
    public{
        this(in Material!(N) m, in N size = N(1.0)){
            _material = m;

            _margin = V3(0.5, 0.5, 0.5);
            
            import fragments.constraintpairs.collisionconstraintpair;
            _collisionConstraintPairs = [CollisionConstraintPair!N(this)];
        }
        
        void updateCollisionConstraintPairs(in StaticEntity!N[] staticEntities)
        in{
            assert(_collisionConstraintPairs.length == 8);
        }body{
            _isColliding = false;
            import fragments.contactpoint:ContactPoint;
            ContactPoint!N[] points;
            immutable V3 rayBeginGlobal = _orientationPre.rotatedVector(V3.zero)+_positionPre;
            immutable V3 rayEndGlobal   = _orientation.rotatedVector(V3.zero)+_position;
            import fragments.geometryhelper:detectMostCloselyContactPoint;
            detectMostCloselyContactPoint(
                    rayBeginGlobal,
                    rayEndGlobal, 
                    staticEntities, 
                    points
                    );
            _isColliding = _isColliding || points.length > 0;
            _collisionConstraintPairs[0].update(points);
        }
    }//public
}//class Point
unittest{
    assert( __traits(compiles, {
         import fragments.material;
         auto material = new Material!(double);
         auto point = new Point!(double)(material);
    }));
};

