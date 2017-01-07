module fragments.constraintsolver;

import armos.math;
import fragments.constraints;
import fragments.constraintpairs;
import fragments.entity;

/++
+/
class ConstraintSolver(NumericType){
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    alias M33 = Matrix!(N, 3, 3);
    
    public{
        /++
        ++/
        void unitTime(in N t){_unitTime = t;}
        
        /++
        ++/
        void solve(
            ref DynamicEntity!N[]           dynamicEntities,
            ref LinkConstraintPair!N[]      linkConstraintPairs,
            ref LinearImpulseConstraint!N[] linearImpulseConstraints,
        ){
            _iterations.iterate(
                dynamicEntities,
                linkConstraintPairs,
                linearImpulseConstraints
            );
            
            postProcess(
                linkConstraintPairs,
                dynamicEntities
            );
        }
    }//public

    private{
        N _unitTime;
        int _iterations = 10;
    }//private
}//class ConstraintSolver

private void iterate(N)(
    in int iterations, 
    ref DynamicEntity!N[] collidingEntities, 
    ref LinkConstraintPair!N[]      linkConstraintPairs,
    ref LinearImpulseConstraint!N[] linearImpulseConstraints,
){
    alias V3 = Vector!(N, 3);
    alias M33 = Matrix!(N, 3, 3);
    //iteration
    linearImpulseConstraints.updateDeltaVelocities(iterations);
    for (int i = 0; i < iterations; i++) {
        linkConstraintPairs.updateDeltaVelocities;
    }
    collidingEntities.updateDeltaVelocities;
}

private void updateDeltaVelocities(N, V3 = Vector!(N, 3))(ref LinearImpulseConstraint!N[] linearImpulseConstraints, int iterations){
    foreach (ref linearImpulseConstraint; linearImpulseConstraints) {
        import std.conv;
        auto entity = linearImpulseConstraint.entity;
        V3[2] deltaVelocities = linearImpulseConstraint.deltaVelocities(entity);
        entity.deltaLinearVelocity  = entity.deltaLinearVelocity + deltaVelocities[0]/iterations.to!N;
        entity.deltaAngularVelocity = entity.deltaAngularVelocity + deltaVelocities[1]/iterations.to!N;
    }
}

private void updateDeltaVelocities(N, V3 = Vector!(N, 3))(ref LinkConstraintPair!N[] linkConstraintPairs){
    foreach (ref linkConstraintPair; linkConstraintPairs){
        auto entities = linkConstraintPair.dynamicEntities;
        foreach (ref linearConstraint; linkConstraintPair.linearLinkConstraints) {
            V3[2][2] deltaVelocities = linearConstraint.deltaVelocities(entities);
            {
                auto entity = entities[0];
                entity.deltaLinearVelocity  = entity.deltaLinearVelocity + deltaVelocities[0][0];
                entity.deltaAngularVelocity = entity.deltaAngularVelocity + deltaVelocities[0][1];
            }

            {
                auto entity = entities[1];
                entity.deltaLinearVelocity  = entity.deltaLinearVelocity - deltaVelocities[1][0];
                entity.deltaAngularVelocity = entity.deltaAngularVelocity - deltaVelocities[1][1];
            }
        }

        foreach (ref angularConstraint; linkConstraintPair.angularLinkConstraints) {
            V3[2][2] deltaVelocities = angularConstraint.deltaVelocities(entities);
            {
                auto entity = entities[0];
                entity.deltaLinearVelocity  = entity.deltaLinearVelocity + deltaVelocities[0][0];
                entity.deltaAngularVelocity = entity.deltaAngularVelocity + deltaVelocities[0][1];
            }
        
            {
                auto entity = entities[1];
                entity.deltaLinearVelocity  = entity.deltaLinearVelocity - deltaVelocities[1][0];
                entity.deltaAngularVelocity = entity.deltaAngularVelocity - deltaVelocities[1][1];
            }
        }
    }
}

private void updateDeltaVelocities(N, V3 = Vector!(N, 3))(ref DynamicEntity!N[] collidingEntities){
    foreach (entity; collidingEntities) {
        foreach (ref collisionConstraintPair; entity.collisionConstraintPairs) {
            if(collisionConstraintPair.isColliding){
                {
                    V3[2] deltaVelocities = collisionConstraintPair.collisionConstraint.deltaVelocities(entity);
                    entity.deltaLinearVelocity  = entity.deltaLinearVelocity + deltaVelocities[0];
                    entity.deltaAngularVelocity = entity.deltaAngularVelocity + deltaVelocities[1];
                }
                
                import std.math;
                immutable maxFriction = fabs(collisionConstraintPair.collisionConstraint.currentImpulse) * collisionConstraintPair.dynamicFriction;
                foreach (ref frictionConstraint; collisionConstraintPair.frictionConstraints) {
                    frictionConstraint.lowerLimit = -maxFriction;
                    frictionConstraint.upperLimit = maxFriction;
                }
                
                
                //friction
                {
                    foreach (int index, ref frictionConstraint; collisionConstraintPair.frictionConstraints) {
                        V3[2] deltaVelocities = frictionConstraint.deltaVelocities(entity);
                        entity.deltaLinearVelocity  = entity.deltaLinearVelocity + deltaVelocities[0];
                        entity.deltaAngularVelocity = entity.deltaAngularVelocity + deltaVelocities[1];
                    }
                }
            }
        }
    }
}

private void postProcess(N)(
    ref LinkConstraintPair!N[]      linkConstraintPairs,
    ref DynamicEntity!N[]           dynamicEntities,
){
    import std.algorithm.iteration:each;
    dynamicEntities.each!(entity => entity.updateVelocitiesFromDelta);
    dynamicEntities.each!(entity => entity.initDeltaVelocity);
    
    dynamicEntities.updateBias;
}

private void updateBias(N)(
    ref DynamicEntity!N[] dynamicEntities, 
){
    import fragments.contactpoint;
    import std.algorithm.iteration:filter;
    import std.array:array;
    auto collidingEntities = dynamicEntities.filter!(entity => entity.isColliding).array;
    
    foreach (ref collidingEntity; collidingEntities){
        alias V3 = Vector!(N, 3);
        collidingEntity.bias = V3.zero;
        foreach (ref collisionConstraintPair; collidingEntity.collisionConstraintPairs) {
            if(collisionConstraintPair.isColliding){
                immutable depth = collisionConstraintPair.depth;
                if(collidingEntity.bias.norm < depth.norm){
                    collidingEntity.bias = depth;
                }
            }
        }
        // import std.stdio;
        // collidingEntity.bias.writeln;
        collidingEntity.position = collidingEntity.position + collidingEntity.bias;
    }
}

private void updateVelocitiesFromDelta(N)(DynamicEntity!(N) entity){
    entity.linearVelocity = entity.linearVelocity + entity.deltaLinearVelocity;
    entity.angularVelocity = entity.angularVelocity + entity.deltaAngularVelocity;
}

private void initDeltaVelocity(N)(DynamicEntity!(N) entity){
    alias V3 = Vector!(N, 3);
    entity.deltaLinearVelocity = V3.zero;
    entity.deltaAngularVelocity = V3.zero;
}
