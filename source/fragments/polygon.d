module fragments.polygon;

import armos.math;
import fragments.entity;
import fragments.boundingbox;
import fragments.material;
/++
++/
class Polygon(NumericType) : StaticEntity!(NumericType){
    alias N   = NumericType;
    alias V3  = Vector!(N, 3);
    alias M33 = Matrix!(N, 3, 3);
    public{
        ///
        this(in V3[3] v, in Material!(N) m, V3 clearance = V3.zero){
            _vertices = v;
            
            auto tmpStart = V3();
            auto tmpEnd = V3();
            import std.math;
            for (int dim = 0; dim < 3; dim++) {
                tmpStart[dim] = fmin(_vertices[0][dim], fmin(_vertices[1][dim], _vertices[2][dim]));
                tmpEnd[dim] = fmax(_vertices[0][dim], fmax(_vertices[1][dim], _vertices[2][dim]));
            }
            _boundingBox = BoundingBox!N(tmpStart-clearance, tmpEnd+clearance);
            
            _normal = ( _vertices[1]-_vertices[2] ).vectorProduct( _vertices[2]-_vertices[0] ).normalized;
            
            _material = m;
        }
        
        ///
        N massInv()const{
            return N(0);
        }
        
        M33 inertiaGlobalInv()const{
            return M33.zero;
        }
        
        ///
        V3[3] vertices()const{return _vertices;};
        
        V3 normal()const{return _normal;};
        
        ///
        BoundingBox!(N) boundingBox()const{return _boundingBox;};
        
        ///
        const(Material!(N)) material()const{
            return _material;
        }
    }//public

    private{
        Vector!(N, 3)[3] _vertices;
        const BoundingBox!(N) _boundingBox;
        const(V3) _normal;
        const(Material!(N)) _material;
    }//private
}//class Polygon
unittest{
    alias V = Vector3d;
    V[3] vertices = [V(0, -1, 0), V(1, 1, 0), V(1, 0, 2)];
    import fragments.material;
    auto material = new Material!double;
    Polygon!double p = new Polygon!double(vertices, material);
    assert( p.boundingBox.start == V(0, -1, 0) );
    assert( p.boundingBox.end == V(1, 1, 2) );
}
