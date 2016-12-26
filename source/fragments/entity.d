module fragments.entity;

import armos.math;
import fragments.boundingbox;
import fragments.contactpoint;
import fragments.material;
import fragments.constraintpairs.collisionconstraintpair;
import fragments.integrator;
import fragments.mapconstraintdetector;
/++
++/
interface Entity(NumericType) {
    alias N = NumericType;
    alias M33 = Matrix!(N, 3, 3);
    
    public{
        ///
        N massInv()const;
        
        ///
        M33 inertiaGlobalInv()const;
        
        ///
        BoundingBox!(N) boundingBox()const;
        
        ///
        const( Material!(N) ) material()const;
    }//public

    private{
    }//private
}//interface Entity

/++
++/
interface StaticEntity(NumericType) : Entity!(NumericType){
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    
    public{
        ///
        V3[3] vertices()const;
        
        ///
        V3 normal()const;
        
        ///
        BoundingBox!(N) boundingBox()const;
        
        ///
        const( Material!(N) ) material()const;
    }//public

    private{
    }//private
}//interface StaticEntity

/++
++/
interface DynamicEntity(NumericType) : Entity!(NumericType){
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    alias Q = Quaternion!(N);
    alias M33 = Matrix!(N, 3, 3);
    
    public{
        ///
        N mass()const;
        
        ///
        void mass(in N);
        
        ///
        M33 inertia()const;
        
        ///
        void inertia(in M33);
        
        ///
        M33 inertiaGlobal()const;
        
        ///
        V3 position()const;
        
        ///
        void position(in V3);
        
        ///
        V3 positionPre()const;
        
        ///
        V3 linearVelocity()const;
        
        ///
        void linearVelocity(in V3);
        
        ///
        V3 linearVelocityPre()const;
        
        ///
        Q orientation()const;
        
        ///
        void orientation(in Q);
        
        ///
        Q orientationPre()const;
        
        ///
        V3 angularVelocity()const;
        
        ///
        void angularVelocity(in V3);
        
        ///
        V3 angularVelocityPre()const;
        
        ///
        BoundingBox!(N) boundingBox()const;
        
        ///
        const( Material!(N) ) material()const;
            
        ///
        void updateStatus(Integrator!N integrator, MapConstraintDetector!N mapConstraintDetector);

        V3 deltaLinearVelocity()const;
        
        void deltaLinearVelocity(in V3);
        
        V3 deltaAngularVelocity()const;
        
        void deltaAngularVelocity(in V3);
        
        V3 bias()const;
        
        void bias(in V3);
        
        bool isColliding()const;
        
        CollisionConstraintPair!N[] collisionConstraintPairs();
        
        void updateCollisionConstraintPairs(in StaticEntity!N[] staticEntities);
    }//public
}//interface DynamicEntity

template DynamicEntityProperties(NumericType){
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    alias Q = Quaternion!(N);
    alias M33 = Matrix!(N, 3, 3);
    
    public{
        ///
        N mass()const{return _mass;}

        ///
        void mass(in N m){
            _mass = m;
            _massInv = N(1)/m;
        }
        
        ///
        M33 inertia()const{return _inertia;};
        
        ///
        void inertia(in M33 inertia){
            _inertia  = inertia;
            _inertiaGlobal = _orientation.matrix33*_inertia*_orientation.matrix33.inverse;
            _inertiaGlobalInv = _inertiaGlobal.inverse;
        };
        
        ///
        M33 inertiaGlobal()const{return _inertiaGlobal;};
        
        ///
        N massInv()const{
            return _massInv;
        }
        
        M33 inertiaGlobalInv()const{
            return _inertiaGlobalInv;
        }
        
        ///
        V3 position()const{return _position;};
        
        ///
        void position(in V3 position){_position = position;};
        
        ///
        V3 positionPre()const{return _positionPre;};
        
        ///
        V3 linearVelocity()const{return _linearVelocity;};
        
        ///
        void linearVelocity(in V3 linearVelocity){_linearVelocity = linearVelocity;};
        
        ///
        V3 linearVelocityPre()const{return _linearVelocityPre;};
        
        ///
        Q orientation()const{return _orientation;};
        
        ///
        void orientation(in Q orientation){_orientation = orientation;};
        
        ///
        Q orientationPre()const{return _orientationPre;};
        
        ///
        V3 angularVelocity()const{return _angularVelocity;};
        
        ///
        void angularVelocity(in V3 angularVelocity){_angularVelocity = angularVelocity;};
        
        ///
        V3 angularVelocityPre()const{return _angularVelocityPre;};
        
        ///
        BoundingBox!(N) boundingBox()const{return _boundingBox;}
        
        ///
        const( Material!(N) ) material()const{
            return _material;
        }
        
        V3 deltaLinearVelocity()const{
            return _deltaLinearVelocity;
        }
        
        void deltaLinearVelocity(in V3 v){
            _deltaLinearVelocity = v;
        };
        
        V3 deltaAngularVelocity()const{
            return _deltaAngularVelocity;
        }
        
        void deltaAngularVelocity(in V3 v){
            _deltaAngularVelocity = v;
        }
        
        V3 bias()const{
            return _bias;
        }
        
        void bias(in V3 b){
            _bias = b;
        }
        
        import fragments.integrator;
        import fragments.mapconstraintdetector;
        void updateStatus(Integrator!N integrator, MapConstraintDetector!N mapConstraintDetector){
            // pre?
            V3 positionAppliedBias = _position;
            Q orientationAppliedBias = _orientation;

            integrator.integrate(this);
            
            _inertiaGlobal = _orientation.matrix33*_inertiaGlobal*_orientation.matrix33.inverse;
            _inertiaGlobalInv = _inertiaGlobal.inverse;
            
            //updatePrevalue
            if(isColliding){
                _boundingBox = BoundingBox!(N)(_position, _positionPre, _margin);
                updateCollisionConstraintPairs(mapConstraintDetector.detectedStaticEntities(this));
                
                if(isColliding){
                    
                }else{
                    _positionPre = positionAppliedBias;
                    _orientationPre = orientationAppliedBias;
                    
                    _linearVelocityPre = _linearVelocity;
                    _angularVelocityPre = _angularVelocity;
                }
            }else{
                _positionPre = positionAppliedBias;
                _orientationPre = orientationAppliedBias;
                
                _linearVelocityPre = _linearVelocity;
                _angularVelocityPre = _angularVelocity;

            }
            _boundingBox = BoundingBox!(N)(_position, _positionPre, _margin);
        }
        
        bool isColliding()const{
            return _isColliding;
        };
        
        CollisionConstraintPair!N[] collisionConstraintPairs(){
            return _collisionConstraintPairs;
        };
        
    }//public
    
    private{
        N   _mass;
        N   _massInv;
        V3  _position    = V3.zero;
        V3  _positionPre = V3.zero;
        V3  _linearVelocity    = V3.zero;
        V3  _linearVelocityPre = V3.zero;
        Q   _orientation    = Q.unit;
        Q   _orientationPre = Q.unit;
        V3  _angularVelocity    = V3.zero;
        V3  _angularVelocityPre = V3.zero;
        M33 _inertia;
        M33 _inertiaGlobal;
        M33 _inertiaGlobalInv;
        V3  _margin = V3.zero;
        V3  _deltaLinearVelocity  = V3.zero;
        V3  _deltaAngularVelocity = V3.zero;
        V3  _bias = V3.zero;
        
        BoundingBox!(N) _boundingBox;
        const( Material!(N) ) _material;
        
        bool _isColliding;
        CollisionConstraintPair!N[] _collisionConstraintPairs;
    }//private
}
