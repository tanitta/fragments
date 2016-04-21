import armos, std.stdio, std.math;
import fragments.entity;
import fragments.square;
import fragments.engine;
import fragments.boundingbox;
import fragments.constraintpair;


void drawBoundingBox(B)(B boundingBox){
	with( boundingBox ){
		drawBoxFrame(start, end);
	}
}

void drawBoxFrame(V3)(V3 begin, V3 end){
	ar.drawLine(begin[0], begin[1], begin[2], end[0], begin[1], begin[2]);
	ar.drawLine(begin[0], begin[1], begin[2], begin[0], end[1], begin[2]);
	ar.drawLine(begin[0], begin[1], begin[2], begin[0], begin[1], end[2]);

	ar.drawLine(begin[0], end[1], begin[2], end[0], end[1], begin[2]);
	ar.drawLine(begin[0], end[1], begin[2], begin[0], end[1], end[2]);

	ar.drawLine(end[0], end[1], end[2], begin[0], end[1], end[2]);
	ar.drawLine(end[0], end[1], end[2], end[0], begin[1], end[2]);
	ar.drawLine(end[0], end[1], end[2], end[0], end[1], begin[2]);

	ar.drawLine(end[0], begin[1], end[2], begin[0], begin[1], end[2]);
	ar.drawLine(end[0], begin[1], end[2], end[0], begin[1], begin[2]);

	ar.drawLine(begin[0], end[1], end[2], begin[0], begin[1], end[2]);
	ar.drawLine(end[0], end[1], begin[2], end[0], begin[1], begin[2]);
}

/++
+/
class Chip(NumericType){
	alias N = NumericType;
	alias Q = ar.Quaternion!(N);
	alias V3 = ar.Vector!(N, 3);
	alias V4 = ar.Vector!(N, 4);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		fragments.square.Square!(N) entity;
		
		/++
		+/
		this(){
			import fragments.material;
			auto material = new Material!(N);
			
			entity = new fragments.square.Square!(N)(material, 0.3);
			entity.mass = N(25);
			entity.inertia = M33(
				[0.8, 0, 0],
				[0, 1.5, 0],
				[0, 0, 0.8]
			);
		}
		
		/++
		+/
		void position(ar.Vector!(N, 3) p){
			entity.position = p;
		}
		
		/++
		+/
		V3 position()const{
			return entity.position;
		}
		
		/++
		+/
		void orientation(Q p = Q.unit){
			entity.orientation = p;
		}
		
		Q orientation()const{
			return entity.orientation;
		}
		
		/++
		+/
		void draw()const{
			ar.pushMatrix;
				ar.translate(entity.position);
				
				ar.setColor(64, 64, 255);
				ar.drawLine(V3.zero, -entity.linearVelocity*0.33);
				ar.setColor(255, 255, 255);
				
				ar.multMatrix(entity.orientation.matrix44);
				
				ar.drawAxis(1.0);
				
				ar.pushStyle;{
					ar.setColor(64, 64, 64);
					ar.setLineWidth = 2;
					drawBoxFrame(V3(-0.3, -0.02, -0.3), V3(0.3, 0.02, 0.3));
				}ar.popStyle;
				
			ar.popMatrix;
			entity.boundingBox.drawBoundingBox;
		}
		
		/++
		+/
		void addForce(in N unitTime, in V3 force,in V3 position = V3.zero){
			entity.linearVelocity = entity.linearVelocity + force / entity.mass * unitTime ;
			entity.angularVelocity = entity.angularVelocity + entity.inertia.inverse*( (position-entity.position).vectorProduct(force) ) * unitTime;
		}
	}//public
	private{
	}//private
}//class Chip

/++
+/
class Model(N) {
	public{
		void addChip(Chip!N chip){
			_chips ~= chip;
		}
		
		void draw()const{
			import std.algorithm;
			_chips.each!(c => c.draw);
		}
		
		Chip!N[] chips(){return _chips;};
	}//public

	private{
		Chip!N[] _chips;
	}//private
}//class Model

/++
+/
class Land(NumericType) {
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	
	public{
		
		/++
		+/
		this(){
			_model = new ar.Model();
		}

		/++
		+/
		void load(string filepath){
			clear();
			_model.load(filepath);
			import fragments.polygon;
			import fragments.material;
			
			auto material = new Material!(N)(0.01);
			foreach (mesh; _model.meshes) {
				for (int i = 0; i < mesh.numIndices; i+=3) {
					V3[3] vertices;
					for (int j = 0; j < 3; j++) {
						int indicesIndex = mesh.indices[i+j];
						vertices[j] = V3(mesh.vertices[indicesIndex].x, mesh.vertices[indicesIndex].y, mesh.vertices[indicesIndex].z);
					}
					_staticEntities ~= new fragments.polygon.Polygon!(N)(vertices, material, V3(0.1, 0.1, 0.1));
				}
			}
			
			writeln("staticEntities", _staticEntities.length);
		}
		
		/++
		+/
		void clear(){
			_staticEntities = [];
		}
		
		/++
		+/
		void draw(){
			_model.drawFill;
			ar.pushStyle;
			ar.setLineWidth = 2;
			ar.setColor(255, 128, 0);
			foreach (staticEntity; _staticEntities) {
				// staticEntity.boundingBox.drawBoundingBox;
			}
			ar.popStyle;
		}
		
		/++
		+/
		fragments.entity.StaticEntity!(N)[] staticEntities(){
			return _staticEntities;
		}
	}//public

	private{
		fragments.entity.StaticEntity!(N)[] _staticEntities;
		ar.Model _model;
		// fragments.Material!(N)[] _materials;
	}//private
}//class Land


/++
++/
class TestApp : ar.BaseApp{
	alias N = double;
	alias V3 = ar.Vector!(N, 3);
	alias Q = ar.Quaternion!(N);
	float c = 0;
	float h = 10;
	float d = 10;
	float fpsUseRate = 0;
	auto camera = new ar.Camera;
	
	N _unitTime;
	
	fragments.engine.Engine!(N) engine;
	
	DynamicEntity!(N)[] _dynamicEntities;
	LinkConstraintPair!(N)[] _linkConstraintPair;
	
	Land!(N) _land;
	// Chip!(N) chip;
	Model!N _model;
	auto _linearVelocity = V3().zero;
	auto _angularVelocity = V3().zero;
	
	string filename = "Land.x";
	ar.Gui gui;
	
	void setup(){
		ar.blendMode(ar.BlendMode.Alpha);
		ar.enableDepthTest;
		camera.target= ar.Vector3f(0, 45, 0);
		
		_unitTime = 0.033;
		
		//Land
		_land = new Land!(N);
		_land.load(filename);
		
		setupModel;
		
		//Engine
		engine = new fragments.engine.Engine!(N);
		engine.staticEntities = _land.staticEntities;
		
		setupGui;
	}
	
	void setupModel(){
		_model = new Model!N;
		
		import std.random;
		// for (int i = 0; i < 1024; i++) {
		// 	auto chip = new Chip!(N);
		// 	chip.position = V3(uniform(-5.0, 5.0), 45+uniform(0.0, 5.0), uniform(-5.0, 5.0));
		// 	chip.orientation = Q.unit;
		// 	chip.addForce(_unitTime, ar.Vector3d(uniform(-10.0, 10.0) ,uniform(-100.0, 0.0), uniform(-10.0, 10.0)), chip.position + ar.Vector3d(uniform(-1.0, 1.0), uniform(-1.0, 1.0), uniform(-1.0, 1.0)));
		// 	_model.addChip(chip);
		// }
		
		// {
		// 	auto chip = new Chip!(N);
		// 	chip.position = V3(-2, 45, 0);
		// 	chip.orientation = Q.unit;
		// 	chip.addForce(_unitTime, ar.Vector3d(0, -100, 0), ar.Vector3d(-2.1, 45, 0.1));
		// 	_model.addChip(chip);
		// }
		
		{
			auto chip = new Chip!(N);
			chip.position = V3(300, 80, 2);
			chip.orientation = Q.unit;
			// chip.addForce(_unitTime, ar.Vector3d(300000, 20000, 0), ar.Vector3d(3, 80, 2));
			// chip.addForce(_unitTime, ar.Vector3d(1000, 0, 0), ar.Vector3d(3, 80, 2.1));
			// chip.addForce(_unitTime, ar.Vector3d(-10, 0, 0), ar.Vector3d(0, 45, -1));
			_model.addChip(chip);
		}
		
		// {
		// 	auto chip = new Chip!(N);
		// 	chip.position = V3(200, 50, 0);
		// 	chip.orientation = Q.unit;
		// 	chip.addForce(_unitTime, V3(210*120*5, 0, 0), V3(200, 50.0, 0));
		// 	_model.addChip(chip);
		// }
		
		import std.algorithm : map;
		import std.array : array;
		import std.conv;
		_dynamicEntities = _model.chips.map!(c => c.entity.to!(DynamicEntity!N)).array;
	}

	void setupGui(){
		gui = ( new ar.Gui )
		.add(
			(new ar.List)
			.add(new ar.Partition)
			.add(new ar.Partition)
			.add(new ar.Label("Fragments"))
			.add(new ar.Partition)
			.add(new ar.Slider!N("UnitTime", _unitTime, 0, 1.0))
			.add(new ar.MovingGraph!float("fpsUseRate", fpsUseRate, 0.0, 100.0))
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
			.add(new ar.Slider!float("rotate", c, -360, 360))
			.add(new ar.Slider!float("height", h, -10, 10))
			.add(new ar.Slider!float("distance", d, 0, 100))
		);
	}
	
	void update(){
		engine.unitTime = _unitTime;
		engine.update(_dynamicEntities, _linkConstraintPair);
		
		// writeln("speed", _model.chips[0].entity.linearVelocity.norm*3.6, "km/h");
	}
	
	void draw(){
		// camera.position = ar.Vector3f(0, h, -d);
		camera.target = cast(ar.Vector3f)(_model.chips[0].position);
		camera.position = cast(ar.Vector3f)( _model.chips[0].position+V3(-d*sin(c/360.0*PI), h, -d*cos(c/360.0*PI)) );
		camera.begin;
			ar.pushMatrix;
			// ar.rotate(c, 0, 1, 0);
			ar.setColor(255, 255, 255);
			_land.draw;
			ar.pushStyle;
			ar.setLineWidth = 2;
			// engine.draw;
			ar.popStyle;
			_model.draw;
			ar.popMatrix;
		camera.end;
		//
		drawDebug;
	}
	
	void drawDebug(){
		gui.draw;
		fpsUseRate = ar.fpsUseRate*100.0;
		// fpsUseRate.writeln;
	}
	
	void keyPressed(int str){
		// engine.unitTime = _unitTime;
		// engine.update(_dynamicEntities, _constraintPair);
	}
}

void main(){
	ar.run(new TestApp);
}
