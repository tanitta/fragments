module fragments.entity;
import armos;
import fragments.boundingbox;
import fragments.contactpoint;
/++
++/
interface Entity(NumericType) {
	alias N = NumericType;
	
	public{
		///
		BoundingBox!(N) boundingBox()const;
	}//public

	private{
	}//private
}//interface Entity

/++
++/
interface StaticEntity(NumericType) : Entity!(NumericType){
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	
	public{
		///
		V3[3] vertices()const;
		
		BoundingBox!(N) boundingBox()const;
	}//public

	private{
	}//private
}//interface StaticEntity

/++
++/
interface DynamicEntity(NumericType) : Entity!(NumericType){
	alias N = NumericType;
	alias V3 = ar.Vector!(N, 3);
	alias Q = ar.Quaternion!(N);
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		///
		N mass()const;
		
		///
		void mass(in N);
		
		///
		M33 inertia()const;
		
		///
		void inertia(in M33);
		
		///
		V3 position()const;
		
		///
		void position(in V3);
		
		///
		V3 linearVelocity()const;
		
		///
		void linearVelocity(in V3);
		
		///
		Q orientation()const;
		
		///
		void orientation(in Q);
		
		///
		V3 angularVelocity()const;
		
		///
		void angularVelocity(in V3);
		
		///
		BoundingBox!(N) boundingBox()const;
		
		///
		ContactPoint!(N)[] contactPoints(in StaticEntity!(N) staticEntity)const;
		
	}//public
}//interface DynamicEntity
