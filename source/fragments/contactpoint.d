module fragments.contactpoint;

import armos.math;
import fragments.entity;

/++
++/
struct ContactPoint(NumericType) {
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    public{
        ///
        this(
            in V3 coordination,
            in V3 normal,
            in N distance,
            in V3 applicationPoint, 
            in StaticEntity!N staticEntity
        ){
            _coordination = coordination;
            _normal = normal;
            _distance = distance;
            _applicationPoint = applicationPoint;
            _staticEntity = staticEntity;
        }
        
        ///
        V3 coordination()const{return _coordination;}
        
        ///
        V3 normal()const{return _normal;}
        
        ///
        N distance()const{return _distance;}
        
        ///
        V3 applicationPoint()const{return _applicationPoint;}
        
        ///
        const(StaticEntity!N) staticEntity()const{return _staticEntity;}
    }//public

    private{
        ///
        V3 _coordination;
        
        ///
        V3 _normal;
        
        ///めり込み量
        N _distance;
        
        ///
        V3 _applicationPoint;
        
        ///
        const(StaticEntity!N) _staticEntity;
    }//private
}//struct ContactPoint
