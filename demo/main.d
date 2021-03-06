module main;

import std.stdio;
import std.conv;
import std.math;
import std.random;

import dagon;
import dagon.ext.ftfont;
import dagon.ext.newton;

class TestScene: Scene
{
    Game game;
    FontAsset aFontDroidSans14;
    TextureAsset aEnvmap;
    OBJAsset aCubeMesh;
    OBJAsset aLevel;
    TextureAsset aGridTexture;
    ImageAsset aHeightmap;
    TextureAsset aGrass;
    TextureAsset aGrassNormal;

    Camera camera;
    FirstPersonViewComponent fpview;
    Light sun;
    Color4f sunColor = Color4f(1.0f, 0.7f, 0.5f, 1.0f);
    float sunPitch = -20.0f;
    float sunTurn = 0.0f;

    NewtonPhysicsWorld world;
    NewtonBodyComponent[] cubeBodyControllers;
    size_t numCubes = 50;
    NewtonBodyComponent[] sphereBodyControllers;
    size_t numSpheres = 20;

    Entity eCharacter;
    NewtonCharacterComponent character;

    TextLine text;

    this(Game game)
    {
        super(game);
        this.game = game;
    }

    ~this()
    {
        if (cubeBodyControllers.length)
            Delete(cubeBodyControllers);
        if (sphereBodyControllers.length)
            Delete(sphereBodyControllers);
    }

    override void beforeLoad()
    {
        aFontDroidSans14 = this.addFontAsset("data/font/DroidSans.ttf", 14);
        aCubeMesh = addOBJAsset("data/cube.obj");
        aLevel = addOBJAsset("data/level.obj");
        aGridTexture = addTextureAsset("data/grid.png");
        aEnvmap = addTextureAsset("data/Zion_7_Sunsetpeek_Ref.hdr");
        aHeightmap = addImageAsset("data/terrain/heightmap.png");
        aGrass = addTextureAsset("data/terrain/grass-albedo.png");
        aGrassNormal = addTextureAsset("data/terrain/grass-normal.png");
    }

    override void afterLoad()
    {
        world = New!NewtonPhysicsWorld(assetManager);

        world.loadPlugins("./");

        camera = addCamera();
        fpview = New!FirstPersonViewComponent(eventManager, camera);
        game.renderer.activeCamera = camera;

        environment.backgroundColor = Color4f(0.9f, 0.8f, 1.0f, 1.0f);
        auto envCubemap = addCubemap(1024);
        envCubemap.fromEquirectangularMap(aEnvmap.texture);
        environment.ambientMap = envCubemap;
        environment.ambientEnergy = 0.5f;

        game.deferredRenderer.ssaoEnabled = true;
        game.deferredRenderer.ssaoPower = 4.0f;
        game.deferredRenderer.ssaoRadius = 0.25f;
        game.deferredRenderer.ssaoDenoise = 1.0f;
        game.postProcessingRenderer.tonemapper = Tonemapper.Filmic;
        game.postProcessingRenderer.fxaaEnabled = true;
        game.postProcessingRenderer.motionBlurEnabled = true;
        game.postProcessingRenderer.glowEnabled = true;
        game.postProcessingRenderer.glowThreshold = 1.0f;
        game.postProcessingRenderer.glowIntensity = 0.3f;
        game.postProcessingRenderer.glowRadius = 7;
        
        sun = addLight(LightType.Sun);
        sun.position.y = 50.0f;
        sun.shadowEnabled = true;
        sun.energy = 10.0f;
        sun.scatteringEnabled = false;
        sun.scattering = 0.35f;
        sun.mediumDensity = 0.15f;
        sun.scatteringUseShadow = true;
        sun.color = sunColor;
        sun.rotation =
            rotationQuaternion!float(Axis.y, degtorad(sunTurn)) *
            rotationQuaternion!float(Axis.x, degtorad(sunPitch));

        auto light1 = addLight(LightType.AreaSphere);
        light1.castShadow = false;
        light1.position = Vector3f(4, 6.5, -4);
        light1.color = Color4f(1.0f, 0.5f, 0.0f, 1.0f);
        light1.energy = 20.0f;
        light1.radius = 0.4f;
        light1.volumeRadius = 10.0f;
        light1.specular = 0.0f;
        
        auto light2 = addLight(LightType.AreaSphere);
        light2.castShadow = false;
        light2.position = Vector3f(-10, 2.5, -4);
        light2.color = Color4f(1.0f, 0.5f, 0.0f, 1.0f);
        light2.energy = 15.0f;
        light2.radius = 0.2f;
        light2.volumeRadius = 10.0f;
        light2.specular = 0.0f;
        
        auto light3 = addLight(LightType.AreaSphere);
        light3.castShadow = false;
        light3.position = Vector3f(-14, 2.5, 11);
        light3.color = Color4f(1.0f, 0.5f, 0.0f, 1.0f);
        light3.energy = 10.0f;
        light3.radius = 0.2f;
        light3.volumeRadius = 10.0f;
        light3.specular = 0.0f;

        auto eSky = addEntity();
        auto psync = New!PositionSync(eventManager, eSky, camera);
        eSky.drawable = New!ShapeBox(Vector3f(1.0f, 1.0f, 1.0f), assetManager);
        eSky.scaling = Vector3f(100.0f, 100.0f, 100.0f);
        eSky.layer = EntityLayer.Background;
        eSky.material = New!Material(assetManager);
        eSky.material.depthWrite = false;
        eSky.material.culling = false;
        eSky.material.diffuse = envCubemap;
        
        auto matPink = New!Material(assetManager);
        matPink.diffuse = Color4f(1.0, 0.36, 0.478, 1.0);
        matPink.roughness = 0.2f;

        auto matOrange = New!Material(assetManager);
        matOrange.diffuse = Color4f(1.0, 0.5, 0.3, 1.0);
        matOrange.roughness = 0.2f;
        
        auto matYellow = New!Material(assetManager);
        matYellow.diffuse = Color4f(1.0, 0.87, 0.36, 1.0);
        matYellow.roughness = 0.2f;
        
        auto matGreen = New!Material(assetManager);
        matGreen.diffuse = Color4f(0.517, 1.0, 0.36, 1.0);
        matGreen.roughness = 0.2f;
        
        auto materials = [matPink, matOrange, matYellow, matGreen];

        auto box = New!NewtonBoxShape(Vector3f(1, 1, 1), world);

        cubeBodyControllers = New!(NewtonBodyComponent[])(numCubes);
        foreach(i; 0..cubeBodyControllers.length)
        {
            auto eCube = addEntity();
            eCube.drawable = aCubeMesh.mesh;
            eCube.material = materials[uniform(0, $)];
            eCube.position = Vector3f(3, i * 1.5, 5);
            auto b = world.createDynamicBody(box, 500.0f);
            cubeBodyControllers[i] = New!NewtonBodyComponent(eventManager, eCube, b);
        }
        
        auto sSphere = New!ShapeSphere(1, 24, 16, false, assetManager);
        auto sphere = New!NewtonSphereShape(1, world);
        
        sphereBodyControllers = New!(NewtonBodyComponent[])(numSpheres);
        foreach(i; 0..sphereBodyControllers.length)
        {
            auto eSphere = addEntity();
            eSphere.drawable = sSphere;
            eSphere.material = materials[uniform(0, $)];
            eSphere.position = Vector3f(-3, i * 1.5, 5);
            auto b = world.createDynamicBody(sphere, 500.0f);
            sphereBodyControllers[i] = New!NewtonBodyComponent(eventManager, eSphere, b);
        }

        eCharacter = addEntity();
        eCharacter.position = Vector3f(0, 2, 20);
        character = New!NewtonCharacterComponent(eventManager, eCharacter, 1.8f, 80.0f, world);
        
        auto levelShape = New!NewtonMeshShape(aLevel.mesh, world);
        auto eLevel = addEntity();
        eLevel.drawable = aLevel.mesh;
        eLevel.turn(45);
        auto matLevel = New!Material(assetManager);
        matLevel.roughness = 0.3f;
        eLevel.material = matLevel;
        auto levelBody = world.createStaticBody(levelShape);
        auto levelBodyController = New!NewtonBodyComponent(eventManager, eLevel, levelBody);
        
        auto heightmap = New!ImageHeightmap(aHeightmap.image, 1.0f, assetManager);
        auto terrain = New!Terrain(128, 64, heightmap, assetManager);
        Vector3f terrainScale = Vector3f(1.0f, 5.0f, 1.0f);
        auto heightmapShape = New!NewtonHeightmapShape(heightmap, 128, 128, terrainScale, world);
        auto terrainBody = world.createStaticBody(heightmapShape);
        auto eTerrain = addEntity();
        eTerrain.position = Vector3f(-64, -4, -64);
        auto terrainBodyController = New!NewtonBodyComponent(eventManager, eTerrain, terrainBody);
        auto eTerrainVisual = addEntity(eTerrain);
        eTerrainVisual.dynamic = false;
        eTerrainVisual.solid = true;
        eTerrainVisual.material = addMaterial();
        eTerrainVisual.material.diffuse = aGrass.texture;
        eTerrainVisual.material.normal = aGrassNormal.texture;
        eTerrainVisual.material.textureScale = Vector2f(50, 50);
        eTerrainVisual.material.roughness = 0.8f;
        eTerrainVisual.drawable = terrain;
        eTerrainVisual.scaling = terrainScale;

        text = New!TextLine(aFontDroidSans14.font, "0", assetManager);
        text.color = Color4f(1.0f, 1.0f, 1.0f, 0.7f);
        auto eText = addEntityHUD();
        eText.drawable = text;
        eText.position = Vector3f(16.0f, 30.0f, 0.0f);
        
        eventManager.showCursor(true);
        fpview.active = false;
    }

    override void onKeyDown(int key)
    {
        if (key == KEY_ESCAPE)
            application.exit();
    }
    
    override void onMouseButtonUp(int button)
    {
        fpview.active = !fpview.active;
        eventManager.showCursor(!fpview.active);
        fpview.prevMouseX = eventManager.mouseX;
        fpview.prevMouseY = eventManager.mouseY;
    }

    override void onUpdate(Time t)
    {
        updateCharacter();
        world.update(t.delta);
        camera.position = character.eyePoint;
        updateText();
    }
    
    void updateCharacter()
    {
        float speed = 6.0f;
        if (inputManager.getButton("left")) character.move(camera.right, -speed);
        if (inputManager.getButton("right")) character.move(camera.right, speed);
        if (inputManager.getButton("forward")) character.move(camera.direction, -speed);
        if (inputManager.getButton("back")) character.move(camera.direction, speed);
        if (inputManager.getButton("jump")) character.jump(1.0f);
        character.updateVelocity();
    }
    
    char[100] txt;
    void updateText()
    {
        uint n = sprintf(txt.ptr, "FPS: %u", cast(int)(1.0 / eventManager.deltaTime));
        string s = cast(string)txt[0..n];
        text.setText(s);
    }
}

class TestGame: Game
{
    this(uint w, uint h, bool fullscreen, string title, string[] args)
    {
        super(w, h, fullscreen, title, args);
        currentScene = New!TestScene(this);
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

    TestGame game = New!TestGame(1280, 720, false, "Dagon + Newton Game Dynamics", args);
    game.run();
    Delete(game);

    writeln("Allocated memory: ", allocatedMemory());
}
