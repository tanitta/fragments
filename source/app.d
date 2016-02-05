import armos, std.stdio, std.math;

/++
++/
struct ContactPoint {
	public{
		ar.Vector3d coordination;
		ar.Vector3d normal;
		double distance;
	}//public

	private{
	}//private
}//struct ContactPoint

/++
++/
struct BoundingBox {
	public{
		this(in ar.Vector3d s, in ar.Vector3d e){
			_start = s;
			_end = e;
		}
		
		ar.Vector3d start()const{return _start;};
		ar.Vector3d end()const{return _end;};
		
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
		const ar.Vector3d _start;
		const ar.Vector3d _end;
	}//private
}//struct BoundingBox

/++
++/
interface Entity {
	public{
		BoundingBox boundingBox()const;
	}//public

	private{
	}//private
}//interface Entity

/++
++/
interface StaticEntity : Entity{
	public{
		const( ar.Vector3d[3] ) vertices()const;
	}//public

	private{
	}//private
}//interface StaticEntity

/++
++/
class Polygon : StaticEntity{
	public{
		this(in ar.Vector3d[3] v){
			_vertices = v;
			auto tmpStart = ar.Vector3d();
			auto tmpEnd = ar.Vector3d();
			for (int dim = 0; dim < 3; dim++) {
				tmpStart[dim] = fmin(_vertices[0][dim], fmin(_vertices[1][dim], _vertices[2][dim]));
				tmpEnd[dim] = fmax(_vertices[0][dim], fmax(_vertices[1][dim], _vertices[2][dim]));
			}
			_boundingBox = BoundingBox(tmpStart, tmpEnd);
		}
		
		const( ar.Vector3d[3] ) vertices()const{return _vertices;};
		
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

/++
++/
interface DynamicEntity : Entity{
	public{
		double mass()const;
		void mass(in double);
		ContactPoint[] contactPoints(in StaticEntity staticEntity)const;
	}//public
}//interface DynamicEntity

/++
++/
class Chip : DynamicEntity{
	public{
		double mass()const{return _mass;}
		void mass(in double m){_mass = m;}
		
		ContactPoint[] contactPoints(in StaticEntity staticEntity)const{
			ContactPoint[] points;
			return points;
		};
		
		BoundingBox boundingBox()const{return _boundingBox;};
	}//public

	private{
		double _mass;
		BoundingBox _boundingBox;
	}//private
}//class Chip

/++
++/
class Model {
	public{
	}//public

	private{
	}//private
}//class Model

/++
++/
class World {
	public{
	}//public

	private{
	}//private
}//class World

class TestApp : ar.BaseApp{
	float c = 0;
	float h = 1000;
	float d = 1000;
	auto camera = new ar.Camera;
	auto model = new ar.Model;

	string filename = "Land.x";
	ar.Gui gui;
	
	void setup(){
		ar.blendMode(ar.BlendMode.Alpha);
		ar.enableDepthTest;
		camera.target= ar.Vector3f(0, 0, 0);
		model.load(filename);
		gui = ( new ar.Gui )
		.add(
			(new ar.List)
			.add(new ar.Partition(" "))
			.add(new ar.Label(filename))
			.add(new ar.Partition)
			.add(new ar.Slider!float("rotate", c, -180, 180))
			.add(new ar.Slider!float("height", h, 0, 1000))
			.add(new ar.Slider!float("distance", d, 0, 1000))
			.add(new ar.Partition)
		);
	}
	
	void draw(){
		camera.position = ar.Vector3f(0, h, -d);
		camera.begin;
			ar.pushMatrix;
			ar.rotate(c, 0, 1, 0);
			ar.setColor(255, 255, 255);
			model.drawFill;
			ar.popMatrix;
		camera.end;
		
		gui.draw;
	}
}

void main(){
	// ar.run(new TestApp);
}
