module fragments.boundingbox;
import armos;

/++
++/
struct BoundingBox(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	public{
		///
		this(in V3 s, in V3 e, in V3 offset = V3.zero){
			import std.math;
			_start = V3(fmin(s[0], e[0]), fmin(s[1], e[1]), fmin(s[2], e[2])) - offset;
			_end = V3(fmax(s[0], e[0]), fmax(s[1], e[1]), fmax(s[2], e[2])) + offset;
			_center = ( _start, _end )*0.5;
		}
		
		///
		V3 start()const{return _start;};
		
		///
		V3 end()const{return _end;};
		
		///
		V3 center()const{return _center;};
		
		///
		bool opBinary(string op)(BoundingBox!(N) b)const if(op == "&"){
			import std.math;
			if(
				b.start[0] < this.end[0] && this.start[0] < b.end[0] &&
				b.start[1] < this.end[1] && this.start[1] < b.end[1] &&
				b.start[2] < this.end[2] && this.start[2] < b.end[2]
			){
				return true;
			}else{
				return false;
			}
		}
		unittest{
			alias Vec = ar.Vector3d;
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
