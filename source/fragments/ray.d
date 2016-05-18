module fragments.ray;

import armos;

/++
++/
struct Ray(NumericType){
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	private{
		this(in V3 begin, in V3 end){
			_begin = begin;
			_end = end;
			_length = (begin - end).norm;
			_normal = (begin - end).normalized;
		}
		
		V3 begin()const{return _begin;}
		V3 end()const{return _end;}
		V3 normal()const{return _normal;}
		N length()const{return _length;}
	}
	
	private{
		V3 _begin;
		V3 _end;
		V3 _normal;
		N _length;
	}
}
unittest{
	alias N = double;
	alias V3 = ar.Vector!(N, 3);
	
	immutable ray = Ray!N(V3(1, 2, 3), V3(4, 5, 6));
	assert(ray.begin == V3(1, 2, 3));
	assert(ray.end == V3(4, 5, 6));
}

