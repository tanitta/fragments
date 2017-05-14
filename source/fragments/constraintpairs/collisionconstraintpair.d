module fragments.constraintpairs.collisionconstraintpair;

import armos.math;

import fragments.entity;
import fragments.constraints.utils;
import fragments.contactpoint;
import fragments.constraints;

/++
+/
struct CollisionConstraintPair(NumericType) {
    alias N   = NumericType;
    alias V3  = Vector!(N, 3);
    alias M33 = Matrix!(N, 3, 3);
    
    public{
        ///
        this(
            ref DynamicEntity!N dynamicEntity,
            ref ContactPoint!N contactPoint,
        ){
            _dynamicEntity = dynamicEntity;
            
            
            immutable V3 applicationPoint = contactPoint.applicationPoint - dynamicEntity.position;
            immutable relativeVelocity = dynamicEntity.linearVelocity + dynamicEntity.angularVelocity.vectorProduct(applicationPoint);
                
            immutable bias = 0.0;
            immutable slop = N(0);
            import std.math;
            immutable unitTime = 1.0/30.0;
            // immutable biasTerm = (bias * fmax(N(0), contactPoint.distance+slop))/unitTime;
            immutable biasTerm = N(0);
            
            auto staticEntity = contactPoint.staticEntity;

            immutable jacDiagInv = collisionJacDiagInv(
                applicationPoint,
                dynamicEntity.massInv,
                dynamicEntity.inertiaGlobalInv,
                staticEntity.normal
            );
            
            //set constraints
            collisionConstraint = CollisionConstraint!N(
                relativeVelocity,
                jacDiagInv, 
                applicationPoint, 
                staticEntity.normal,
                biasTerm,
            );
        }
        
        ///
        this(DynamicEntity!N dynamicEntity){
            _dynamicEntity = dynamicEntity;
        }
        
        ///
        void update(
            in ContactPoint!(N)[] contactPoints,
        ){
            _isColliding = contactPoints.length > 0;
            _depth = V3.zero;
            
            if(isColliding){
                const contactPoint = contactPoints[0];
                
                immutable V3 applicationPoint = contactPoint.applicationPoint - _dynamicEntity.position;
                immutable relativeVelocity = _dynamicEntity.linearVelocity + _dynamicEntity.angularVelocity.vectorProduct(applicationPoint);

                // immutable bias = 0.0;
                // immutable slop = N(0);
                // import std.math;
                // immutable biasTerm = (bias * fmax(N(0), contactPoint.distance+slop))/unitTime;
                immutable biasTerm = N(0);

                const staticEntity = contactPoints[0].staticEntity;
                
                immutable jacDiagInv = collisionJacDiagInv(
                    applicationPoint,
                    _dynamicEntity.massInv,
                    _dynamicEntity.inertiaGlobalInv,
                    staticEntity.normal
                );

                //set constraints
                collisionConstraint = CollisionConstraint!N(
                    relativeVelocity,
                    jacDiagInv, 
                    applicationPoint, 
                    staticEntity.normal,
                    biasTerm,
                );
                updateFrictionConstraints(staticEntity, relativeVelocity, applicationPoint, jacDiagInv);
                
                _depth = contactPoint.distance * contactPoint.normal;
            }
        };
        
        ///
        bool isColliding()const{return _isColliding;}
        
        /++
        +/
        DynamicEntity!(N) entity(){return _dynamicEntity;};
        
        /++
        +/
        CollisionConstraint!(N) collisionConstraint;
        
        /++
        +/
        V3 depth()const{
            return _depth;
        };
        
        /++
        +/
        N dynamicFriction()const{
            return _dynamicFriction;
        }
        
        /++
        +/
        N staticFriction()const{
            return _staticFriction;
        }
        
        FrictionConstraint!(N)[2] frictionConstraints;
    }//public

    private{
        DynamicEntity!(N) _dynamicEntity;
        
        V3 _depth;
        
        bool _isColliding;
        
        N _staticFriction;
        N _dynamicFriction;
        
        void updateFrictionConstraints(
            in StaticEntity!N staticEntity,
            in V3 relativeVelocity,
            in V3 applicationPoint,
            in N jacDiagInv,
        ){
            _staticFriction = (_dynamicEntity.material.staticFriction * staticEntity.material.staticFriction)^^N(0.5);
            _dynamicFriction = (_dynamicEntity.material.dynamicFriction * staticEntity.material.dynamicFriction)^^N(0.5);

            V3[2] frictionAxes;
            if(relativeVelocity.vectorProduct(staticEntity.normal).norm > 0){
                frictionAxes[0] = relativeVelocity.vectorProduct(staticEntity.normal).normalized;
                frictionAxes[1] = frictionAxes[0].vectorProduct(staticEntity.normal);
            }else{
                frictionAxes[0] = V3(0, 0, 0);
                frictionAxes[1] = V3(0, 0, 0);
            }

            foreach (int index, frictionAxis; frictionAxes) {
                frictionConstraints[index] = FrictionConstraint!N(
                    relativeVelocity,
                    jacDiagInv,
                    applicationPoint,
                    frictionAxis,
                );
            }
        }
    }//private
}//struct CollisionConstraintPair

