import std.stdio;
import std.conv;

import dagon;
import newton;

class TestScene: Scene
{
    FontAsset aFontDroidSans14;
    OBJAsset aCubeMesh;
    TextureAsset aGridTexture;
    
    TextLine text;

    NewtonPhysicsWorld world;
    NewtonBodyController[] cubeBodyControllers;
    size_t numCubes = 10;

    this(SceneManager smngr)
    {
        super(smngr);
    }
    
    ~this()
    {
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
        
        world = New!NewtonPhysicsWorld(assetManager);
        
        version(X86)
            world.loadPlugins("plugins/x86");
        else
            world.loadPlugins("plugins/x64");
        
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
        
        auto box = New!NewtonBoxShape(Vector3f(1, 1, 1), world, assetManager);

        cubeBodyControllers = New!(NewtonBodyController[])(numCubes);
        foreach(i; 0..cubeBodyControllers.length)
        {
            auto eCube = createEntity3D();
            eCube.drawable = aCubeMesh.mesh;
            eCube.material = matCube;
            eCube.position = Vector3f(0, i * 1.5, 0);
            auto b = world.createDynamicBody(box, 1.0f);
            cubeBodyControllers[i] = New!NewtonBodyController(eCube, b);
            eCube.controller = cubeBodyControllers[i];
        }
        
        auto boxFloor = New!NewtonBoxShape(Vector3f(50, 1, 50), world, assetManager);
        
        auto eFloor = createEntity3D();
        eFloor.position = Vector3f(0, -0.5, 0);
        auto b = world.createStaticBody(boxFloor);
        auto planeBodyController = New!NewtonBodyController(eFloor, b);
        eFloor.controller = planeBodyController;
        
        auto matPlane = createMaterial();
        matPlane.diffuse = aGridTexture.texture;
        matPlane.textureScale = Vector2f(5, 5);
        matPlane.roughness = 0.9f;
        auto ePlane = createEntity3D();
        ePlane.drawable = New!ShapePlane(50, 50, 10, assetManager);
        ePlane.material = matPlane;
        
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

        world.update(dt);
        
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
