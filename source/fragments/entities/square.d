module fragments.entities.square;

import armos.math;
import fragments.entity;
import fragments.contactpoint;
import fragments.material;

/++
++/
class Square(NumericType) : DynamicEntity!(NumericType){
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    alias Q = Quaternion!N;
    mixin DynamicEntityProperties!N;
    
    public{
        this(in Material!(N) m, in N size = N(1.0)){
            _material = m;

            _margin = V3(0.5, 0.5, 0.5);
            
            _rays = [
                V3(size,  0, 0    ), 
                V3(-size, 0, 0    ), 
                V3(0,     0, size ), 
                V3(0,     0, -size), 
        
                V3(size,  0, size ), 
                V3(-size, 0, -size), 
                V3(size,  0, -size), 
                V3(-size, 0, size ), 
            ];
            
            import fragments.constraintpairs.collisionconstraintpair;
            _collisionConstraintPairs = [
                CollisionConstraintPair!N(this), 
                CollisionConstraintPair!N(this), 
                CollisionConstraintPair!N(this), 
                CollisionConstraintPair!N(this), 
                CollisionConstraintPair!N(this), 
                CollisionConstraintPair!N(this), 
                CollisionConstraintPair!N(this), 
                CollisionConstraintPair!N(this), 
            ];
        }
        
        void updateCollisionConstraintPairs(in StaticEntity!N[] staticEntities)
        in{
            assert(_collisionConstraintPairs.length == 8);
        }body{
            _isColliding = false;
            foreach (int index, ray; _rays) {
                ContactPoint!N[] points;
                immutable V3 rayBeginGlobal = _orientationPre.rotatedVector(ray)+_positionPre;
                immutable V3 rayEndGlobal   = _orientation.rotatedVector(ray)+_position;
                import fragments.geometryhelper:detectMostCloselyContactPoint;
                detectMostCloselyContactPoint(
                    rayBeginGlobal,
                    rayEndGlobal, 
                    staticEntities, 
                    points
                );
                _isColliding = _isColliding || points.length > 0;
                _collisionConstraintPairs[index].update(points);
            }
        }
    }//public
    
    private{
        V3[8] _rays;
    }//private
}//class Square
unittest{
    import fragments.material;
    auto material = new Material!(double);
    auto square = new Square!(double)(material);
};

