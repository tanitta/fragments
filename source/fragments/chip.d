module fragments.chip;
import armos;
import fragments.entity;
import fragments.contactpoint;
import fragments.boundingbox;

/++
++/
class Chip : DynamicEntity{
	public{
		///
		double mass()const{return _mass;}
		
		///
		void mass(in double m){_mass = m;}
		
		///
		ContactPoint[] contactPoints(in StaticEntity staticEntity)const{
			ContactPoint[] points;
			return points;
		};
		
		///
		BoundingBox boundingBox()const{return _boundingBox;};
	}//public

	private{
		double _mass;
		BoundingBox _boundingBox;
	}//private
}//class Chip
