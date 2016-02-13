import armos, std.stdio, std.math;
import fragments.integrator;
import fragments.entity;
import fragments.square;

/++
++/
class Chip(NumericType){
	alias N = NumericType;
	alias Q = ar.Quaternion!(N);
	public{
		auto entity = new fragments.square.Square!(N);
		
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
				"--".writeln;
				entity.orientation.matrix.print;
				"--".writeln;
				ar.drawAxis(1.0);
			ar.popMatrix;
		}
	}//public

	private{
	}//private
}//class Chip
/++
++/
class TestApp : ar.BaseApp{
	float c = 0;
	float h = 50;
	float d = 10;
	auto camera = new ar.Camera;
	auto model = new ar.Model;
	
	double _unitTime;
	auto integrator = new fragments.integrator.Integrator!(double)(0.0);
	DynamicEntity!(double)[] array;
	auto chip = new Chip!(double);
	auto _linearVelocity = ar.Vector3d();
	auto _angularVelocity = ar.Vector3d();
	
	string filename = "Land.x";
	ar.Gui gui;
	
	void setup(){
		ar.blendMode(ar.BlendMode.Alpha);
		ar.enableDepthTest;
		camera.target= ar.Vector3f(0, 45, 0);
		model.load(filename);
		
		
		_unitTime = 0.0;
		chip.position = ar.Vector3d(0, 45, 0);
		chip.orientation;
		
		array ~= chip.entity;
		
		gui = ( new ar.Gui )
		.add(
			(new ar.List)
			.add(new ar.Partition(" "))
			.add(new ar.Label(filename))
			.add(new ar.Partition)
			.add(new ar.Slider!float("rotate", c, -180, 180))
			.add(new ar.Slider!float("height", h, 0, 100))
			.add(new ar.Slider!float("distance", d, 0, 100))
			.add(new ar.Partition)
			.add(new ar.Partition)
			.add(new ar.Label("Fragments"))
			.add(new ar.Partition)
			.add(new ar.Slider!double("UnitTime", _unitTime, 0, 1.0))
			.add(new ar.Label("linearVelocity"))
			.add(new ar.Slider!double("x", _linearVelocity[0], -1.0, 1.0))
			.add(new ar.Slider!double("y", _linearVelocity[1], -1.0, 1.0))
			.add(new ar.Slider!double("z", _linearVelocity[2], -1.0, 1.0))
			.add(new ar.Label("angularVelocity"))
			.add(new ar.Slider!double("x", _angularVelocity[0], -1.0, 1.0))
			.add(new ar.Slider!double("y", _angularVelocity[1], -1.0, 1.0))
			.add(new ar.Slider!double("z", _angularVelocity[2], -1.0, 1.0))
		);
	}
	
	void update(){
		chip.entity.linearVelocity = _linearVelocity;
		chip.entity.angularVelocity = _angularVelocity;

		integrator.unitTime = _unitTime;
		integrator.integrate(array);
	}
	
	void draw(){
		camera.position = ar.Vector3f(0, h, -d);
		camera.begin;
			ar.pushMatrix;
			ar.rotate(c, 0, 1, 0);
			ar.setColor(255, 255, 255);
			model.drawFill;
			chip.draw;
			ar.popMatrix;
		camera.end;
		
		gui.draw;
	}
}

void main(){
	ar.run(new TestApp);
}
