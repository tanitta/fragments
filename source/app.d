import armos, std.stdio, std.math;

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
