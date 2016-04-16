module fragments.material;

/++
+/
class Material(NumericType){
	alias N = NumericType;
	
	public{
		this(
			in N staticFriction = N(0.02),
			in N dynamicFriction = N(0.05),
		){
			_staticFriction = staticFriction;
			_dynamicFriction = dynamicFriction;
		}
		
		N staticFriction()const{
			return _staticFriction;
		}
		
		N dynamicFriction()const{
			return _dynamicFriction;
		}
	}//public

	private{
		N _staticFriction;
		N _dynamicFriction;
	}//private
}//class Material
