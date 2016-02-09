module fragments.boundingbox;
import armos;

/++
++/
struct BoundingBox {
	public{
		///
		this(in ar.Vector3d s, in ar.Vector3d e){
			_start = s;
			_end = e;
			_center = ( s+e )*0.5;
		}
		
		///
		ar.Vector3d start()const{return _start;};
		
		///
		ar.Vector3d end()const{return _end;};
		
		///
		ar.Vector3d center()const{return _center;};
		
		///
		bool opBinary(string op)(BoundingBox b)const if(op == "&"){
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
				auto b1 = BoundingBox(Vec(1, 1, 1), Vec(3, 3, 3));
				auto b2 = BoundingBox(Vec(2, 2, 2), Vec(4, 4, 4));
				assert(b1 & b2);
			}
			{
				auto b1 = BoundingBox(Vec(1, 1, 1), Vec(3, 3, 3));
				auto b2 = BoundingBox(Vec(4, 2, 2), Vec(4, 4, 4));
				assert(!( b1 & b2 ));
			}
		}
	}//public

	private{
		ar.Vector3d _start;
		ar.Vector3d _end;
		ar.Vector3d _center;
	}//private
}//struct BoundingBox
