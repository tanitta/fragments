module fragments.ray;

import armos;

/++
++/
struct Ray(NumericType){
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	private{
		this(V3 begin, V3 end){
			_begin = begin;
			_end = end;
			_length = (begin - end).norm;
			_normal = (begin - end).normalised;
		}
		
		V3 begin()const{return _begin;}
		V3 end()const{return _end;}
		V3 normalised()const{return _normalized;}
		N length()const{return _length;}
	}
	
	private{
		V3 _begin;
		V3 _end;
		V3 _normalized;
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

