module fragments.entities.disk;

import armos.math;
import fragments.entity;
import fragments.material;

/++
+/
class Disk(NumericType) : DynamicEntity!(NumericType){
    alias N  = NumericType;
    alias V3 = Vector!(N, 3);
    alias Q  = Quaternion!(N);
    mixin DynamicEntityProperties!(N);
    
    public{
        this(in Material!N m, in N radius = N(1.0)){
            _radius = radius;
            _material = m;
            _margin = V3(0.5, 0.5, 0.5);

            import fragments.constraintpairs.collisionconstraintpair;
            _collisionConstraintPairs = [
                CollisionConstraintPair!N(this),
                CollisionConstraintPair!N(this)
            ];
        }

        void updateCollisionConstraintPairs(in StaticEntity!N[] staticEntities)
        in{
            assert(_collisionConstraintPairs.length == 8);
        }body{
            _isColliding = false;
            updateCollisionConstraintPairsCenterRay(staticEntities);
            updateCollisionConstraintPairsCircleRay(staticEntities);
        }
    }//public

    private{
        N _radius;

        void updateCollisionConstraintPairsCenterRay(in StaticEntity!N[] staticEntities){
            import fragments.contactpoint:ContactPoint;
            ContactPoint!N[] points;
            immutable V3 rayBeginGlobal = _orientationPre.rotatedVector(V3.zero)+_positionPre;
            immutable V3 rayEndGlobal   = _orientation.rotatedVector(V3.zero)+_position;
            import fragments.geometryhelper:detectMostCloselyContactPoint;
            N margin = 0.003;
            detectMostCloselyContactPoint(
                rayBeginGlobal,
                rayEndGlobal, 
                staticEntities, 
                points, 
                margin
            );
            _isColliding = _isColliding || points.length > 0;
            _collisionConstraintPairs[0].update(points);
        }

        void updateCollisionConstraintPairsCircleRay(in StaticEntity!N[] staticEntities){
            import fragments.contactpoint:ContactPoint;
            ContactPoint!N[] points;
            ContactPoint!N[] pointsTmp;
            bool isDetected = false;
            foreach (staticEntity; staticEntities) {
                V3 rayPre = staticEntityDirectionOnDiskLocal(_positionPre, _orientationPre, staticEntity)*_radius;
                V3 ray    = staticEntityDirectionOnDiskLocal(_position,    _orientation,    staticEntity)*_radius;
                import std.math:isNaN;
                // if(rayPre.norm.isNaN || ray.norm.isNaN) continue;
                if(rayPre.norm.isNaN) rayPre = V3(0, 0, 0);
                if(ray.norm.isNaN)    ray    = V3(0, 0, 0);

                immutable V3 rayBeginGlobal = _orientationPre.rotatedVector(rayPre)+_positionPre;
                // immutable V3 rayBeginGlobal = _orientationPre.rotatedVector(V3.zero)+_positionPre;
                immutable V3 rayEndGlobal   = _orientation.rotatedVector(ray)+_position;

                import fragments.geometryhelper:detectContactPoint;
                N margin = 0.003;
                isDetected =  isDetected || detectContactPoint(
                    rayBeginGlobal,
                    rayEndGlobal,
                    staticEntity,
                    pointsTmp, 
                    margin
                );
            }
            import fragments.geometryhelper:closestContactPoint;
            if(isDetected) points ~= pointsTmp.closestContactPoint;
            _isColliding = _isColliding || points.length > 0;
            _collisionConstraintPairs[1].update(points);
        }
    }//private
}//class Disk

private V3 staticEntityDirectionOnDiskLocal(V3, Q, N)(in V3 position, in Q orientation, in StaticEntity!N staticEntity){
    auto directionLocal = orientation.inverse.rotatedVector(-staticEntity.normal);
    return V3(directionLocal.x, 0, directionLocal.z).normalized;
}
