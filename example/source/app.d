import ar = armos, std.stdio, std.math;
import fragments.entity;
import fragments.square;
import fragments.engine;
import fragments.boundingbox;
import fragments.constraints;
import fragments.constraintpairs;
import fragments.joint;

/++
+/
class Config {
    public{
        typeof(this) hasGravity(in bool b){
            _hasGravity = b;
            return this;
        };

        bool hasGravity()const{
            return _hasGravity;
        };
    }//public

    private{
        bool _hasGravity;
    }//private
}//class Config

void drawBoundingBox(B)(B boundingBox){
    with( boundingBox ){
        ar.graphics.boxFramePrimitive((start+end)*0.5, end-start).drawWireFrame;
    }
}

/++
+/
class Chip(NumericType){
    alias N = NumericType;
    alias Q = ar.math.Quaternion!(N);
    alias V3 = ar.math.Vector!(N, 3);
    alias V4 = ar.math.Vector!(N, 4);
    alias M33 = ar.math.Matrix!(N, 3, 3);
    
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
        void position(ar.math.Vector!(N, 3) p){
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
            entity.angularVelocity.norm.writeln;
            ar.graphics.pushMatrix;
                ar.graphics.translate(entity.position);
                
                ar.graphics.color(64, 64, 255);
                ar.graphics.drawLine(V3.zero, -entity.linearVelocity*0.33);
                ar.graphics.color(255, 255, 255);
                
                import std.conv;
                ar.graphics.multModelMatrix(entity.orientation.matrix44);
                
                ar.graphics.drawAxis(1.0);
                
                ar.graphics.pushStyle;{
                    ar.graphics.color(64, 64, 64);
                    ar.graphics.lineWidth = 2;
                    // drawBoxFrame(V3(-0.3, -0.02, -0.3), V3(0.3, 0.02, 0.3));
                    // ar.graphics.boxFramePrimitive(V3(0, 0, 0), V3(0.6, 0.04, 0.6)).drawWireFrame;
                    ar.graphics.boxPrimitive(V3(0, 0, 0), V3(0.6, 0.04, 0.6)).drawFill;
                }ar.graphics.popStyle;
                
            ar.graphics.popMatrix;
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
    alias V3 = ar.math.Vector!(N, 3);
    
    public{
        
        /++
        +/
        this(){
            _model = new ar.graphics.Model();
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
                        vertices[j] = V3(mesh.vertices[indicesIndex][0], mesh.vertices[indicesIndex][1], mesh.vertices[indicesIndex][2]);
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
        ar.graphics.Model _model;
    }//private
}//class Land


/++
++/
class TestApp : ar.app.BaseApp{
    alias N = float;
    alias V3 = ar.math.Vector!(N, 3);
    alias Q = ar.math.Quaternion!(N);
    
    float c = 0;
    float h = 10;
    float d = 10;
    float fpsUseRate = 0;
    auto camera = new ar.graphics.DefaultCamera;
    
    N _unitTime;
    
    fragments.engine.Engine!(N) engine;
    
    DynamicEntity!(N)[] _dynamicEntities;
    LinearImpulseConstraint!N[] _linearImpulseConstraints;
    
    Land!(N) _land;
    Model!N _model;
    auto _linearVelocity = V3().zero;
    auto _angularVelocity = V3().zero;
    
    string filename = "stadium/stadium.x";
    ar.utils.Gui gui;
    
    override void setup(){
        // ar.graphics.blendMode(ar.graphics.BlendMode.Alpha);
        ar.graphics.enableDepthTest;
        ar.graphics.samples = 2;
        camera.target= ar.math.Vector3f(0, 45, 0);
        
        _unitTime = 1.0/300.0;
        
        //Land
        _land = new Land!(N);
        _land.load(filename);
        
        setupModel;
        
        //Engine
        _config = (new Config).hasGravity(false);
        engine = new fragments.engine.Engine!(N);
        engine.staticEntities = _land.staticEntities;
        
        setupGui;
    }
    
    void setupModel(){
        _model = new Model!N;
        
        // import std.random;
        // for (int i = 0; i < 128; i++) {
        //     auto chip = new Chip!(N);
        //     chip.position = V3(uniform(-5.0, 5.0), 45+uniform(0.0, 5.0), uniform(-5.0, 5.0));
        //     chip.orientation = Q.unit;
        //     chip.addForce(
        //         _unitTime,
        //         V3(
        //             uniform(-1.0, 1.0),
        //             uniform(0.0, 1.0),
        //             uniform(-1.0, 1.0)
        //         )*210.0*10,
        //         chip.position + V3(uniform(-1.0, 1.0), uniform(-1.0, 1.0), uniform(-1.0, 1.0)));
        //     _model.add(chip);
        // }
        
        {
            auto chip = new Chip!(N);
            chip.position = V3(0, 20, 0);
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
            chip.position = V3(0, 20, 1.0);
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
            chip.position = V3(0, 20, 2.0);
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
            chip.position = V3(0, 20, 3.0);
            chip.orientation = Q.unit;
            chip.addForce(
                _unitTime,
                V3(0, 0, 0)*210.0, 
                chip.position
            );
            _model.add(chip);
        }
        
        // {
        //     auto link = FixedJoint!N(
        //         _model.chips[0].entity, 
        //         _model.chips[1].entity, 
        //         V3(0, 0, 0.3), 
        //         V3(0, 0, -0.3), 
        //     );
        //     _model.add(link);
        // }
        //
        // {
        //     auto link = FixedJoint!N(
        //         _model.chips[1].entity, 
        //         _model.chips[2].entity, 
        //         V3(0, 0, 0.3), 
        //         V3(0, 0, -0.3), 
        //     );
        //     _model.add(link);
        // }
        //
        // {
        //     auto link = FixedJoint!N(
        //         _model.chips[0].entity, 
        //         _model.chips[1].entity, 
        //         V3(0, 0, 0.3), 
        //         V3(0, 0, -0.3), 
        //     );
        //     _model.add(link);
        // }
        {
            auto link = FixedJoint!N(
                _model.chips[0].entity, 
                _model.chips[1].entity, 
                V3(0, 0, 0.3), 
                V3(0, 0, -0.3), 
            );
            _model.add(link);
        }
        
        {
            auto link = BallJoint!N(
                _model.chips[1].entity, 
                _model.chips[2].entity, 
                V3(0, 0, 0.3), 
                V3(0, 0, -0.3), 
            );
            _model.add(link);
        }
        
        {
            auto link = FixedJoint!N(
                _model.chips[2].entity, 
                _model.chips[3].entity, 
                V3(0, 0, 0.3), 
                V3(0, 0, -0.3), 
            );
            _model.add(link);
        }
        
        
        import std.algorithm : map;
        import std.array : array;
        import std.conv;
        _dynamicEntities = _model.chips.map!(c => c.entity.to!(DynamicEntity!N)).array;
        
    }

    void setupGui(){
        gui = ( new ar.utils.Gui )
        .add(
            (new ar.utils.List)
            .add(new ar.utils.Partition)
            .add(new ar.utils.Partition)
            .add(new ar.utils.Label("Fragments"))
            .add(new ar.utils.Partition)
            .add(new ar.utils.Slider!N("UnitTime", _unitTime, 0, 1.0))
            .add(new ar.utils.MovingGraph!float("fpsUseRate", fpsUseRate, 0.0, 100.0))
            .add(new ar.utils.Label("linearVelocity"))
            .add(new ar.utils.Slider!N("x", _linearVelocity[0], -1.0, 1.0))
            .add(new ar.utils.Slider!N("y", _linearVelocity[1], -1.0, 1.0))
            .add(new ar.utils.Slider!N("z", _linearVelocity[2], -1.0, 1.0))
            .add(new ar.utils.Label("angularVelocity"))
            .add(new ar.utils.Slider!N("x", _angularVelocity[0], -1.0, 1.0))
            .add(new ar.utils.Slider!N("y", _angularVelocity[1], -1.0, 1.0))
            .add(new ar.utils.Slider!N("z", _angularVelocity[2], -1.0, 1.0))
            
            .add(new ar.utils.Partition)
            .add(new ar.utils.Partition)
            .add(new ar.utils.Label(filename))
            .add(new ar.utils.Partition)
            .add(new ar.utils.Slider!float("rotate", c, -360, 360))
            .add(new ar.utils.Slider!float("height", h, -10, 10))
            .add(new ar.utils.Slider!float("distance", d, 0, 100))
        );
    }
    
    override void update(){
        {
            V3 force = V3.zero;
            if(hasHeldKey(ar.utils.KeyType.W)){
                force += V3(0, 0, 1);
            }
            if(hasHeldKey(ar.utils.KeyType.S)){
                force += V3(0, 0, -1);
            }
            if(hasHeldKey(ar.utils.KeyType.A)){
                force += V3(1, 0, 0);
            }
            if(hasHeldKey(ar.utils.KeyType.D)){
                force += V3(-1, 0, 0);
            }
            if(hasHeldKey(ar.utils.KeyType.Q)){
                force += V3(0, 1, 0);
            }
            if(hasHeldKey(ar.utils.KeyType.E)){
                force += V3(0, -1, 0);
            }
            auto chip = _model.chips[0];
            chip.addForce(
                _unitTime,
                force*210.0*200.0, 
                chip.position
            );
        }

        //gravity
        if(_config.hasGravity){
            import std.algorithm : map;
            import std.array : array;
            _linearImpulseConstraints = _dynamicEntities.map!(entity => LinearImpulseConstraint!N(entity, V3(0, -9.8*entity.mass*_unitTime, 0)))
                                                        .array;
        }else{
            _linearImpulseConstraints = [];
        }

        engine.unitTime = _unitTime;
        engine.update(_dynamicEntities, _model.links,_linearImpulseConstraints);
    }
    
    override void draw(){
        camera.target = cast(ar.math.Vector3f)(_model.chips[0].position);
        camera.position = cast(ar.math.Vector3f)( _model.chips[0].position+V3(-d*sin(c/360.0*PI), h, -d*cos(c/360.0*PI)) );
        
        camera.begin;
            ar.graphics.pushMatrix;
            ar.graphics.color(255, 255, 255);
            _land.draw;
            ar.graphics.pushStyle;
            ar.graphics.lineWidth = 2;
            ar.graphics.popStyle;
            _model.draw;
            ar.graphics.popMatrix;
        camera.end;
        //
        // drawDebug;
    }
    
    void drawDebug(){
        gui.draw;
        fpsUseRate = ar.app.fpsUseRate*100.0;
    }

    override void keyReleased(ar.utils.KeyType k){
        if(k == ar.utils.KeyType.Num1){
            _config.hasGravity.writeln;
            _config.hasGravity = !_config.hasGravity;
        }
        // ar.utils.KeyType.

    }
    
    // override void keyPressed(ar.utils.KeyType k){
    //     V3 force = V3.zero;
    //     switch (k) {
    //         case ar.utils.KeyType.A:
    //             force = V3(-1, 0, 0);
    //         break;
    //        
    //         case ar.utils.KeyType.D:
    //             force = V3(1, 0, 0);
    //         break;
    //        
    //         case ar.utils.KeyType.W:
    //             force = V3(0, 0, -1);
    //         break;
    //        
    //         case ar.utils.KeyType.S:
    //             force = V3(0, 0, 1);
    //         break;
    //         default:
    //         break;
    //     }
    //     auto chip = _model.chips[0];
    //     chip.addForce(
    //         _unitTime,
    //         force*210.0*200.0, 
    //         chip.position
    //     );
    // }
    private{
        Config _config;
    }
}

void main(){
    version(unittest){
        
    }else{
        ar.app.run(new TestApp);
    }
}
