module fragments.geometryhelper;

import fragments.contactpoint:ContactPoint;
import fragments.entity:StaticEntity;
import armos.math;

bool detectMostCloselyContactPoint(N, V3 = Vector!(N, 3), StaticEntities)(
    in V3 rayBeginGlobal,
    in V3 rayEndGlobal,
    in StaticEntities staticEntities,
    ref ContactPoint!N[] contactPoints, 
    N margin
){
    bool isDetected = false;
    
    ContactPoint!N[] points;
    foreach (staticEntity; staticEntities) {
        isDetected = isDetected || detectContactPoint(
            rayBeginGlobal,
            rayEndGlobal,
            staticEntity,
            points, 
            margin
        );
    }
    
    if(isDetected) contactPoints ~= points.closestContactPoint;
   
    return isDetected;
}

unittest{
    alias N = double;
    alias V3 = Vector!(N, 3);
    
    import fragments.material;
    auto material = new Material!N;
    
    import fragments.polygon;
    Polygon!N[] polygons;
    auto exceptedPolygon = new Polygon!N(
        [
            V3(1, -4, 1), 
            V3(1, 4, 1), 
            V3(1, 0, -2), 
        ],
        material
    );

    polygons ~= exceptedPolygon;
    polygons ~= new Polygon!N(
        [
            V3(2, -4, 1), 
            V3(2, 4, 1), 
            V3(2, 0, -2), 
        ],
        material
    );
    
    {
        V3 begin = V3(0, 0, 0);
        V3 end = V3(10, 0, 0);
        
        ContactPoint!N[] contactPoints;
        detectMostCloselyContactPoint(begin, end, polygons, contactPoints, 0);
        assert(contactPoints[0].staticEntity == exceptedPolygon);
    }
    {
        V3 begin = V3(0, 0, 0);
        V3 end = V3(-10, 0, 0);
        
        ContactPoint!N[] contactPoints;
        detectMostCloselyContactPoint(begin, end, polygons, contactPoints, 0);
        assert(contactPoints.length == 0);
    }
}

ContactPoint!N closestContactPoint(N)(in ContactPoint!N[] points){
    int closelyIndex = 0;
    N closelyDistance = N.max;
    foreach (int index, point; points) {
        if( point.distance < closelyDistance ){
            closelyIndex = index;
            closelyDistance = point.distance;
        }
    }
    return points[closelyIndex];
}

bool detectContactPoint(N, V3 = Vector!(N, 3))(
    in V3 rayBeginGlobal,
    in V3 rayEndGlobal,
    in StaticEntity!N staticEntity,
    ref ContactPoint!N[] points, 
    in N margin, 
    in V3 applicationPoint = null
){
    bool isSuccess = false;
    
    immutable p0 = V3.zero;
    immutable p1 = staticEntity.vertices[1] -staticEntity.vertices[0];
    immutable p2 = staticEntity.vertices[2] -staticEntity.vertices[0];
    
    immutable pBegin = rayBeginGlobal - staticEntity.vertices[0];
    immutable pEnd = rayEndGlobal - staticEntity.vertices[0];
    immutable pNormal= staticEntity.normal;
    
    immutable isCollidingLineToPlane = ( (pBegin.dotProduct(pNormal)) * (pEnd.dotProduct(pNormal)) <= N(0) );
    if( isCollidingLineToPlane ){
        import std.math;
        immutable d1 = pNormal.dotProduct(pBegin);
        immutable d2 = pNormal.dotProduct(pEnd);
        immutable a = fabs(d1)/(fabs(d1)+fabs(d2));
        immutable pContact = (N(1)-a)*pBegin + a*pEnd;
        
        immutable isBuried = (d2 <= 0 && 0 <= d1);
        // immutable isBuried = (d2 < 0);
        immutable isIncludedInPolygon =
        ( ( p1 - p0 ).vectorProduct(pContact-p0).dotProduct(pNormal) >= -margin )&&
        ( ( p2 - p1 ).vectorProduct(pContact-p1).dotProduct(pNormal) >= -margin )&&
        ( ( p0 - p2 ).vectorProduct(pContact-p2).dotProduct(pNormal) >= -margin );
        
        if(isBuried && isIncludedInPolygon){
            const contactPoint = ContactPoint!N(
                pContact + staticEntity.vertices[0],
                pNormal, 
                -d2,
                !isNaN(applicationPoint[0])?applicationPoint:rayEndGlobal,
                staticEntity
            );
            
            points ~= contactPoint;
            isSuccess = true;
        }
    }
    
    return isSuccess;
}
unittest{
    alias N = double;
    alias V3 = Vector!(N, 3);
    
    import fragments.material;
    auto material = new Material!N;
    
    import fragments.polygon;
    Polygon!N[] polygons;
    auto polygon= new Polygon!N(
        [
            V3(1, -4, 1), 
            V3(1, 4, 1), 
            V3(1, 0, -2), 
        ],
        material
    );
    
    {
        V3 begin = V3(0, 0, 0);
        V3 end = V3(10, 0, 0);
        
        ContactPoint!N[] contactPoints;
        detectContactPoint(begin, end, polygon, contactPoints, 0);
        
        assert(contactPoints.length == 1);
        assert(contactPoints[0].distance == 9);
        assert(contactPoints[0].coordination == V3(1, 0, 0));
    }

    {
        V3 begin = V3(0, 0, 0);
        V3 end = V3(10, 0, 20);
        
        ContactPoint!N[] contactPoints;
        detectContactPoint(begin, end, polygon, contactPoints, 0);
        
        assert(contactPoints.length == 0);
    }
}
