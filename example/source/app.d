import armos, std.stdio, std.math;
import fragments.entity;
import fragments.square;
import fragments.engine;
import fragments.boundingbox;
import fragments.constraintpair;
import fragments.joint;

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
			auto material = new Material!(N)(0.5);
			
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
				
				ar.color(64, 64, 255);
				ar.drawLine(V3.zero, -entity.linearVelocity*0.33);
				ar.color(255, 255, 255);
				
				ar.multMatrix(entity.orientation.matrix44);
				
				ar.drawAxis(1.0);
				
				ar.pushStyle;{
					ar.color(64, 64, 64);
					ar.lineWidth = 2;
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
		/++
		++/
		void add(Chip!N chip){
			_chips ~= chip;
		}
		
		/++
		++/
		void add(LinkConstraintPair!N link){
			_links ~= link;
		}
		
		/++
		++/
		void draw()const{
			import std.algorithm;
			_chips.each!(c => c.draw);
		}
		
		/++
		++/
		Chip!N[] chips(){return _chips;};
		
		/++
		++/
		LinkConstraintPair!N[] links(){return _links;};
	}//public

	private{
		Chip!N[] _chips;
		LinkConstraintPair!N[] _links;
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
			
			auto material = new Material!(N)(0.5);
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
	LinearImpulseConstraint!N[] _linearImpulseConstraints;
	
	Land!(N) _land;
	Model!N _model;
	auto _linearVelocity = V3().zero;
	auto _angularVelocity = V3().zero;
	
	string filename = "stadium/stadium.x";
	ar.Gui gui;
	
	void setup(){
		ar.blendMode(ar.BlendMode.Alpha);
		ar.enableDepthTest;
		camera.target= ar.Vector3f(0, 45, 0);
		
		_unitTime = 1.0/30.0;
		
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
		
		// import std.random;
		// for (int i = 0; i < 128; i++) {
		// 	auto chip = new Chip!(N);
		// 	chip.position = V3(uniform(-5.0, 5.0), 45+uniform(0.0, 5.0), uniform(-5.0, 5.0));
		// 	chip.orientation = Q.unit;
		// 	chip.addForce(
		// 		_unitTime,
		// 		V3(
		// 			uniform(-1.0, 1.0),
		// 			uniform(0.0, 1.0),
		// 			uniform(-1.0, 1.0)
		// 		)*210.0*10,
		// 		chip.position + V3(uniform(-1.0, 1.0), uniform(-1.0, 1.0), uniform(-1.0, 1.0)));
		// 	_model.addChip(chip);
		// }
		{
			auto chip = new Chip!(N);
			chip.position = V3(0, 10, 0);
			chip.orientation = Q.unit;
			chip.addForce(
				_unitTime,
				V3(0, 0, 0)*210.0, 
				chip.position
			);
			_model.add(chip);
		}
		{
			auto chip = new Chip!(N);
			chip.position = V3(0, 10, 1.0);
			chip.orientation = Q.unit;
			chip.addForce(
				_unitTime,
				V3(0, 0, 0)*210.0, 
				chip.position
			);
			_model.add(chip);
		}
		
		{
			auto link = BallJoint!N(
				_model.chips[0].entity, 
				_model.chips[1].entity, 
				V3(0, 0, 0.3), 
				V3(0, 0, -0.3), 
			);
			_model.add(link);
		}
		
		import std.algorithm : map;
		import std.array : array;
		import std.conv;
		_dynamicEntities = _model.chips.map!(c => c.entity.to!(DynamicEntity!N)).array;
		
		import std.array:array,join;
		_linearImpulseConstraints = _dynamicEntities.map!(entity => LinearImpulseConstraint!N(entity, V3(0, -9.8*entity.mass*_unitTime, 0))).array;
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
		engine.update(_dynamicEntities, _model.links,_linearImpulseConstraints);
	}
	
	void draw(){
		camera.target = cast(ar.Vector3f)(_model.chips[0].position);
		camera.position = cast(ar.Vector3f)( _model.chips[0].position+V3(-d*sin(c/360.0*PI), h, -d*cos(c/360.0*PI)) );
		camera.begin;
			ar.pushMatrix;
			ar.color(255, 255, 255);
			_land.draw;
			ar.pushStyle;
			ar.lineWidth = 2;
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
	}
	
	void keyPressed(int c){
		V3 force = V3.zero;
		switch (c) {
			case 262:
				force = V3(-1, 0, 0);
			break;
			
			case 263:
				force = V3(1, 0, 0);
			break;
			
			case 264:
				force = V3(0, 0, -1);
			break;
			
			case 265:
				force = V3(0, 0, 1);
			break;
			default:
			break;
		}
		auto chip = _model.chips[0];
		chip.addForce(
			_unitTime,
			force*210.0*20.0, 
			chip.position
		);
	}
}

void main(){
