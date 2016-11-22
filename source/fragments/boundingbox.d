module fragments.boundingbox;
import std.math;
import armos.math;

/++
++/
struct BoundingBox(NumericType) {
    alias N = NumericType;
    alias V3 = Vector!(N, 3);
    public{
        ///
        this(in V3 s, in V3 e, in V3 offset = V3.zero)
        in{
            assert(!isNaN(s[0]));
            assert(!isNaN(s[1]));
            assert(!isNaN(s[2]));
            
            assert(!isNaN(e[0]));
            assert(!isNaN(e[1]));
            assert(!isNaN(e[2]));
        }body{
            _start = V3(fmin(s[0], e[0]), fmin(s[1], e[1]), fmin(s[2], e[2])) - offset;
            _end = V3(fmax(s[0], e[0]), fmax(s[1], e[1]), fmax(s[2], e[2])) + offset;
            _center = ( _start, _end )*N(0.5);
        }
        
        ///
        V3 start()const{return _start;};
        
        ///
        V3 end()const{return _end;};
        
        ///
        V3 center()const{return _center;};
        
        ///
        bool opBinary(string op)(in BoundingBox!(N) b)const if(op == "&"){
            immutable diff1 = this.end - b.start;
            immutable diff2 = b.end - this.start;
            immutable diff3 = diff1 * diff2;
            bool isIncluding = true;
            foreach (elem; diff3.elements) {
                isIncluding = isIncluding && N(0) < elem;
            }
            return isIncluding;
        }
        unittest{
            alias Vec = Vector3d;
            {
                auto b1 = BoundingBox!(double)(Vec(1, 1, 1), Vec(3, 3, 3));
                auto b2 = BoundingBox!(double)(Vec(2, 2, 2), Vec(4, 4, 4));
                assert(b1 & b2);
            }
            {
                auto b1 = BoundingBox!(double)(Vec(1, 1, 1), Vec(3, 3, 3));
                auto b2 = BoundingBox!(double)(Vec(4, 2, 2), Vec(4, 4, 4));
                assert(!( b1 & b2 ));
            }
        }
    }//public

    private{
        V3 _start;
        V3 _end;
        V3 _center;
    }//private
}//struct BoundingBox
