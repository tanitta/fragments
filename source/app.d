import armos, std.stdio, std.math;
import fragments.entity;
import fragments.square;
import fragments.engine;

/++
++/
class Chip(NumericType){
	alias N = NumericType;
	alias Q = ar.Quaternion!(N);
	alias V3 = ar.Vector!(N, 3);
	alias V4 = ar.Vector!(N, 4);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		fragments.square.Square!(N) entity;
		
		this(){
			entity = new fragments.square.Square!(N);
			entity.mass = N(25);
			entity.inertia = M33(
				[0.8, 0, 0],
				[0, 1.5, 0],
				[0, 0, 0.8]
			);
		}
		
		void position(ar.Vector!(N, 3) p){
			entity.position = p;
		}
		
		void orientation(Q p = Q.unit){
			entity.orientation = p;
		}
		
		void draw()const{
			ar.pushMatrix;
				ar.translate(entity.position);
				ar.multMatrix(entity.orientation.matrix);
				ar.drawAxis(1.0);
			ar.popMatrix;
		}
		
		void addForce(in N unitTime, in V3 force,in V3 position = V3.zero){
			entity.linearVelocity = entity.linearVelocity + force / entity.mass * unitTime ;
			entity.angularVelocity = entity.angularVelocity + entity.inertia.inverse*( (position-entity.position).vectorProduct(force) ) * unitTime;
			entity.inertia.print;
			import std.stdio;
			entity.inertia.determinant.writeln;
		}
	}//public
	private{
	}//private
}//class Chip

/++
++/
class Land(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	
	public{
		this(){
			_model = new ar.Model;
		}

		void load(string filepath){
			clear();
			_model.load(filepath);
			import fragments.polygon;
			
			foreach (mesh; _model.meshes) {
				for (int i = 0; i < mesh.numIndices; i+=3) {
					V3[3] vertices;
					for (int j = 0; j < 3; j++) {
						int indicesIndex = mesh.indices[i+j];
						vertices[j] = V3(mesh.vertices[indicesIndex].x, mesh.vertices[indicesIndex].y, mesh.vertices[indicesIndex].z);
					}
					_staticEntities ~= new fragments.polygon.Polygon!(N)(vertices, V3(0.1, 0.1, 0.1));
				}
			}
			import std.stdio;
			"staticEntities".writeln;
			_staticEntities.length.writeln;
		}
		
		void clear(){
			_staticEntities = [];
		}
		
		void draw(){
			_model.drawFill;
			ar.pushStyle;
			ar.setLineWidth = 2;
			ar.setColor(255, 128, 0);
			foreach (staticEntity; _staticEntities) {
				with( staticEntity.boundingBox ){
					ar.drawLine(start[0], start[1], start[2], end[0], start[1], start[2]);
					ar.drawLine(start[0], start[1], start[2], start[0], end[1], start[2]);
					ar.drawLine(start[0], start[1], start[2], start[0], start[1], end[2]);
					
					ar.drawLine(start[0], end[1], start[2], end[0], end[1], start[2]);
					ar.drawLine(start[0], end[1], start[2], start[0], end[1], end[2]);
					
					ar.drawLine(end[0], end[1], end[2], start[0], end[1], end[2]);
					ar.drawLine(end[0], end[1], end[2], end[0], start[1], end[2]);
					ar.drawLine(end[0], end[1], end[2], end[0], end[1], start[2]);
					
					ar.drawLine(end[0], start[1], end[2], start[0], start[1], end[2]);
					ar.drawLine(end[0], start[1], end[2], end[0], start[1], start[2]);
				}
			}
			ar.popStyle;
		}
		
		fragments.entity.StaticEntity!(N)[] staticEntities(){
			return _staticEntities;
		}
	}//public

	private{
		fragments.entity.StaticEntity!(N)[] _staticEntities;
		ar.Model _model;
	}//private
}//class Land

/++
++/
class TestApp : ar.BaseApp{
	alias N = double;
	alias V3 = ar.Vector!(N, 3);
	float c = 0;
	float h = 50;
	float d = 10;
	auto camera = new ar.Camera;
	
	N _unitTime;
	// auto integrator = new fragments.integrator.Integrator!(N)(0.0);
	fragments.engine.Engine!(N) engine;
	
	DynamicEntity!(N)[] dynamicEntities;
	
	Land!(N) land;
	Chip!(N) chip;
	auto _linearVelocity = V3();
	auto _angularVelocity = V3();
	
	string filename = "Land.x";
	ar.Gui gui;
	
	void setup(){
		ar.blendMode(ar.BlendMode.Alpha);
		ar.enableDepthTest;
		camera.target= ar.Vector3f(0, 45, 0);
		
		_unitTime = 0.1;
		
		//Land
		land = new Land!(N);
		land.load(filename);
		//Chip
		chip = new Chip!(N);
		chip.position = ar.Vector3d(0, 45, 0);
		chip.orientation;
		chip.addForce(_unitTime, ar.Vector3d(20, 10, 0), ar.Vector3d(0, 45, 0.1));
		dynamicEntities ~= chip.entity;
		
		//engine
		engine = new fragments.engine.Engine!(N);
		
		//Gui
		gui = ( new ar.Gui )
		.add(
			(new ar.List)
			.add(new ar.Partition)
			.add(new ar.Partition)
			.add(new ar.Label("Fragments"))
			.add(new ar.Partition)
			.add(new ar.Slider!N("UnitTime", _unitTime, 0, 1.0))
			.add(new ar.Label("linearVelocity"))
			.add(new ar.Slider!N("x", _linearVelocity[0], -1.0, 1.0))
			.add(new ar.Slider!N("y", _linearVelocity[1], -1.0, 1.0))
			.add(new ar.Slider!N("z", _linearVelocity[2], -1.0, 1.0))
			.add(new ar.Label("angularVelocity"))
			.add(new ar.Slider!N("x", _angularVelocity[0], -1.0, 1.0))
			.add(new ar.Slider!N("y", _angularVelocity[1], -1.0, 1.0))
			.add(new ar.Slider!N("z", _angularVelocity[2], -1.0, 1.0))
			
			.add(new ar.Partition)
			.add(new ar.Partition)
			.add(new ar.Label(filename))
			.add(new ar.Partition)
			.add(new ar.Slider!float("rotate", c, -180, 180))
			.add(new ar.Slider!float("height", h, 0, 100))
			.add(new ar.Slider!float("distance", d, 0, 100))
		);
	}
	
	void update(){
		engine.unitTime = _unitTime;
		engine.update(dynamicEntities);
	}
	
	void draw(){
		camera.position = ar.Vector3f(0, h, -d);
		camera.begin;
			ar.pushMatrix;
			ar.rotate(c, 0, 1, 0);
			ar.setColor(255, 255, 255);
			land.draw;
			chip.draw;
			ar.popMatrix;
		camera.end;
		
		drawDebug;
	}
	
	void drawDebug(){
		gui.draw;
		( ar.fpsUseRate*100 ).writeln;
	}
}

void main(){
	ar.run(new TestApp);
}
