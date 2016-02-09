module fragments.polygon;
import armos;
import fragments.entity;
import fragments.boundingbox;
/++
++/
class Polygon : StaticEntity{
	public{
		///
		this(in ar.Vector3d[3] v){
			_vertices = v;
			auto tmpStart = ar.Vector3d();
			auto tmpEnd = ar.Vector3d();
			import std.math;
			for (int dim = 0; dim < 3; dim++) {
				tmpStart[dim] = fmin(_vertices[0][dim], fmin(_vertices[1][dim], _vertices[2][dim]));
				tmpEnd[dim] = fmax(_vertices[0][dim], fmax(_vertices[1][dim], _vertices[2][dim]));
			}
			_boundingBox = BoundingBox(tmpStart, tmpEnd);
		}
		
		///
		ar.Vector3d[3] vertices()const{return _vertices;};
		
		///
		BoundingBox boundingBox()const{return _boundingBox;};
	}//public

	private{
		const( ar.Vector3d[3] ) _vertices;
		const BoundingBox _boundingBox;
	}//private
}//class Polygon
unittest{
	alias V = ar.Vector3d;
	V[3] vertices = [V(0, -1, 0), V(1, 1, 0), V(1, 0, 2)];
	Polygon p = new Polygon(vertices);

	assert( p.boundingBox.start == V(0, -1, 0) );
	assert( p.boundingBox.end == V(1, 1, 2) );
}
