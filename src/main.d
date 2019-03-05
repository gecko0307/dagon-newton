import std.stdio;
import std.conv;

import dagon;
import bindbc.newton;

extern(C) nothrow @nogc void newtonBodyForceCallback(const NewtonBody* nbody, dFloat timestep, int threadIndex)
{
	NewtonRigidBody b = cast(NewtonRigidBody)NewtonBodyGetUserData(nbody);
    if (b)
    {
        NewtonBodyGetMass(nbody, &b.mass, &b.inertia[0], &b.inertia[1], &b.inertia[2]);
        Vector3f gravityForce = b.gravity * b.mass;
        NewtonBodyAddForce(nbody, gravityForce.arrayof.ptr);
        NewtonBodyAddForce(nbody, b.force.arrayof.ptr);
        NewtonBodyAddTorque(nbody, b.torque.arrayof.ptr);
        b.force = Vector3f(0.0f, 0.0f, 0.0f);
        b.torque = Vector3f(0.0f, 0.0f, 0.0f);
    }
}

class NewtonRigidBody: Owner
{
    NewtonBody* newtonBody;
    float mass;
    Vector3f inertia;
    Vector3f gravity = Vector3f(0.0f, -9.8f, 0.0f);
    Vector3f force = Vector3f(0.0f, 0.0f, 0.0f);
    Vector3f torque = Vector3f(0.0f, 0.0f, 0.0f);
    Vector4f position = Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
    Quaternionf rotation = Quaternionf.identity;
    Matrix4x4f transformation = Matrix4x4f.identity;

    this(Vector3f extents, float mass, NewtonWorld* world, Owner o)
    {
        super(o);
        
        // TODO: user-defined shapes
        NewtonCollision* collision = NewtonCreateBox(world, extents.x, extents.y, extents.z, 0, null);
        newtonBody = NewtonCreateDynamicBody(world, collision, transformation.arrayof.ptr);
        NewtonBodySetUserData(newtonBody, cast(void*)this);
        NewtonDestroyCollision(collision);
        // TODO: compute inertia from shape
        this.mass = mass;
        NewtonBodySetMassMatrix(newtonBody, mass, mass, mass, mass);
        NewtonBodySetForceAndTorqueCallback(newtonBody, &newtonBodyForceCallback);
    }
    
    void update(double dt)
    {
        NewtonBodyGetPosition(newtonBody, position.arrayof.ptr);        
        NewtonBodyGetMatrix(newtonBody, transformation.arrayof.ptr);        
        rotation = Quaternionf.fromMatrix(transformation);
    }
}

class NewtonBodyController: EntityController
{
    NewtonRigidBody rbody;
    
    this(Entity e, NewtonRigidBody b)
    {
        super(e);
        rbody = b;
        
        Quaternionf rot = e.rotation;
        if (e.useRotationAngles)
            rot *= rotationQuaternion!float(Axis.x, degtorad(e.angles.x)) *
                   rotationQuaternion!float(Axis.y, degtorad(e.angles.y)) * 
                   rotationQuaternion!float(Axis.z, degtorad(e.angles.z));
        rbody.transformation = 
            translationMatrix(e.position) *
            rot.toMatrix4x4;
        
        NewtonBodySetMatrix(rbody.newtonBody, rbody.transformation.arrayof.ptr);
    }

    override void update(double dt)
    {
        rbody.update(dt);
        entity.position = rbody.position.xyz;
        entity.transformation = rbody.transformation * scaleMatrix(entity.scaling);
        entity.invTransformation = entity.transformation.inverse;
        entity.rotation = rbody.rotation;
    }
}

class TestScene: Scene
{
    FontAsset aFontDroidSans14;
    OBJAsset aCubeMesh;
    TextureAsset aGridTexture;
    
    TextLine text;

    NewtonWorld* world;
    NewtonBodyController[] cubeBodyControllers;
    size_t numCubes = 1;

    this(SceneManager smngr)
    {
        super(smngr);
    }
    
    ~this()
    {
        NewtonMaterialDestroyAllGroupID(world);
        NewtonDestroyAllBodies(world);
        NewtonDestroy(world);
        
        Delete(cubeBodyControllers);
    }

    override void onAssetsRequest()
    {
        aFontDroidSans14 = addFontAsset("data/font/DroidSans.ttf", 14);
        aCubeMesh = addOBJAsset("data/cube.obj");
        aGridTexture = addTextureAsset("data/grid.png");
    }

    override void onAllocate()
    {
        super.onAllocate();
        
        world = NewtonCreate();
        
        version(X86)
            NewtonLoadPlugins(world, "plugins/x86");
        else
            NewtonLoadPlugins(world, "plugins/x64");
        void* p = NewtonGetPreferedPlugin(world);
        writeln("Selected plugin: ", NewtonGetPluginString(world, p).to!string);
        
        view = New!Freeview(eventManager, assetManager);
        
        mainSun = createLightSun(Quaternionf.identity, environment.sunColor, environment.sunEnergy);
        mainSun.shadow = true;
        environment.sunEnergy = 50.0f;
        environment.setDayTime(9, 00, 00);
        
        auto rRayleighShader = New!RayleighShader(assetManager);
        auto rayleighSkyMaterial = createMaterial(rRayleighShader);
        rayleighSkyMaterial.depthWrite = false;
        rayleighSkyMaterial.culling = false;
        auto eSky = createSky(rayleighSkyMaterial);
        
        renderer.hdr.tonemapper = Tonemapper.ACES;
        renderer.hdr.exposure = 0.3f;
        renderer.ssao.enabled = true;
        renderer.ssao.power = 10.0;
        renderer.glow.enabled = true;
        renderer.glow.radius = 8;
        renderer.glow.brightness = 0.5;
        renderer.glow.minLuminanceThreshold = 0.0;
        renderer.glow.maxLuminanceThreshold = 5.0;
        renderer.antiAliasing.enabled = true;
        
        auto matCube = createMaterial();
        matCube.diffuse = Color4f(1.0, 0.5, 0.3, 1.0);
        matCube.roughness = 0.4f;

        cubeBodyControllers = New!(NewtonBodyController[])(numCubes);
        foreach(i; 0..cubeBodyControllers.length)
        {
            auto eCube = createEntity3D();
            eCube.drawable = aCubeMesh.mesh;
            eCube.material = matCube;
            eCube.position = Vector3f(0, i * 1.5, 0);
            auto b = New!NewtonRigidBody(Vector3f(1, 1, 1), 1.0f, world, assetManager);
            cubeBodyControllers[i] = New!NewtonBodyController(eCube, b);
            eCube.controller = cubeBodyControllers[i];
        }
        
        auto eFloor = createEntity3D();
        eFloor.position = Vector3f(0, -0.5, 0);
        auto b = New!NewtonRigidBody(Vector3f(50, 1, 50), 0.0f, world, assetManager);
        auto planeBodyController = New!NewtonBodyController(eFloor, b);
        eFloor.controller = planeBodyController;
        
        auto matPlane = createMaterial();
        matPlane.diffuse = aGridTexture.texture;
        matPlane.textureScale = Vector2f(5, 5);
        matPlane.roughness = 0.9f;
        auto ePlane = createEntity3D();
        ePlane.drawable = New!ShapePlane(50, 50, 10, assetManager);
        ePlane.material = matPlane;
        
        NewtonInvalidateCache(world);
        
        text = New!TextLine(aFontDroidSans14.font, "0", assetManager);
        text.color = Color4f(1.0f, 1.0f, 1.0f, 0.7f);
        auto eText = createEntity2D();
        eText.drawable = text;
        eText.position = Vector3f(16.0f, 30.0f, 0.0f);
    }
    
    override void onKeyDown(int key)
    {
        if (key == KEY_ESCAPE)
            exitApplication();
        else if (key == KEY_P)
        {
            writeln(cubeBodyControllers[0].entity.position);
        }
    }
    
    char[100] txt;
    
    override void onLogicsUpdate(double dt)
    {
        if (eventManager.keyPressed[KEY_UP])
            foreach(i; 0..cubeBodyControllers.length)
                cubeBodyControllers[i].rbody.force.y = 20.0f;
        if (eventManager.keyPressed[KEY_LEFT])
            foreach(i; 0..cubeBodyControllers.length)
                cubeBodyControllers[i].rbody.force.x = -20.0f;
        if (eventManager.keyPressed[KEY_RIGHT])
            foreach(i; 0..cubeBodyControllers.length)
                cubeBodyControllers[i].rbody.force.x = 20.0f;
    
        NewtonUpdate(world, dt);
        
        uint n = sprintf(txt.ptr, "FPS: %u", eventManager.fps);
        string s = cast(string)txt[0..n];
        text.setText(s);
    }
}

class MyApplication: SceneApplication
{
    this(string[] args)
    {
        super("Dagon + Newton Game Dynamics", args);

        TestScene test = New!TestScene(sceneManager);
        sceneManager.addScene(test, "TestScene");
        sceneManager.goToScene("TestScene");
    }
}

import loader = bindbc.loader.sharedlib;

void main(string[] args)
{
    NewtonSupport sup = loadNewton();
    foreach(info; loader.errors)
    {
        writeln(info.error.to!string, " ", info.message.to!string);
    }
    
    MyApplication app = New!MyApplication(args);
    app.run();
    Delete(app);
}
