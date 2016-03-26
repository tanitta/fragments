module fragments.entity;
import armos;
import fragments.boundingbox;
import fragments.contactpoint;
import fragments.material;
/++
++/
interface Entity(NumericType) {
	alias N = NumericType;
	alias M33 = ar.Matrix!(N, 3, 3);
	
	public{
		///
		N massInv()const;
		
		///
		M33 inertiaGlobalInv()const;
		
		///
		BoundingBox!(N) boundingBox()const;
		
		///
		const( Material!(N) ) material()const;
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
		
		///
		V3 normal()const;
		
		///
		BoundingBox!(N) boundingBox()const;
		
		///
		const( Material!(N) ) material()const;
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
		M33 inertiaGlobal()const;
		
		///
		V3 position()const;
		
		///
		void position(in V3);
		
		///
		V3 positionPre()const;
		
		///
		V3 linearVelocity()const;
		
		///
		void linearVelocity(in V3);
		
		///
		Q orientation()const;
		
		///
		void orientation(in Q);
		
		///
		Q orientationPre()const;
		
		///
		V3 angularVelocity()const;
		
		///
		void angularVelocity(in V3);
		
		///
		BoundingBox!(N) boundingBox()const;
		
		///
		const( Material!(N) ) material()const;
		
		///
		void updateProperties();
		
		///
		ContactPoint!(N)[] contactPoints(in StaticEntity!(N) staticEntity)const;
		
		V3 deltaLinearVelocity()const;
		
		void deltaLinearVelocity(in V3);
		
		V3 deltaAngularVelocity()const;
		
		void deltaAngularVelocity(in V3);
	}//public
}//interface DynamicEntity
