module fragments.entity;
import armos;
import fragments.boundingbox;
import fragments.contactpoint;
/++
++/
interface Entity {
	public{
		///
		BoundingBox boundingBox()const;
	}//public

	private{
	}//private
}//interface Entity

/++
++/
interface StaticEntity : Entity{
	public{
		///
		ar.Vector3d[3] vertices()const;
		
		BoundingBox boundingBox()const;
	}//public

	private{
	}//private
}//interface StaticEntity

/++
++/
interface DynamicEntity : Entity{
	public{
		///
		double mass()const;
		
		///
		void mass(in double);
		
		///
		ContactPoint[] contactPoints(in StaticEntity staticEntity)const;
		
		BoundingBox boundingBox()const;
	}//public
}//interface DynamicEntity
