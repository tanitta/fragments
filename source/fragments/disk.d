module fragments.disk;

import fragments.material;
/++
+/
class Disk(NumericType) : DynamicEntity!(NumericType){
    alias N = NumericType;
    alias V3 = ar.Vector!(N, 3);
    alias Q = ar.Quaternion!(N);
    mixin DynamicEntityProperties!(N);
    
    public{
        this(in Material!N m, in N radius = N(1.0)){
            _material = m;
            
        }
    }//public

    private{
        Material!N _material;
    }//private
}//class Disk
