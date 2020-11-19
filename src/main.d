import std.stdio;
import std.conv;
import std.math;

import dagon;
import dagon.ext.ftfont;
import dagon.ext.newton;

class TestScene: Scene, NewtonRaycaster
{
    Game game;
    FontAsset aFontDroidSans14;
    OBJAsset aCubeMesh;
    TextureAsset aGridTexture;

    TextLine text;

    Camera camera;
    FreeviewComponent freeview;
    Light sun;
    Color4f sunColor = Color4f(1.0f, 0.7f, 0.5f, 1.0f);
    float sunPitch = -45.0f;
    float sunTurn = 0.0f;

    NewtonPhysicsWorld world;
    NewtonBodyComponent[] cubeBodyControllers;
    size_t numCubes = 100;

    Entity eCharacter;
    NewtonBodyComponent bCharacterController;
    float groundHeight = -5.0f;

    this(Game game)
    {
        super(game);
        this.game = game;
    }

    ~this()
    {
        if (cubeBodyControllers.length)
            Delete(cubeBodyControllers);
    }

    override void beforeLoad()
    {
        aFontDroidSans14 = this.addFontAsset("data/font/DroidSans.ttf", 14);
        aCubeMesh = addOBJAsset("data/cube.obj");
        aGridTexture = addTextureAsset("data/grid.png");
    }

    override void afterLoad()
    {
        world = New!NewtonPhysicsWorld(assetManager);

        world.loadPlugins("./");

        camera = addCamera();
        freeview = New!FreeviewComponent(eventManager, camera);
        game.renderer.activeCamera = camera;

        environment.backgroundColor = Color4f(0.9f, 0.8f, 1.0f, 1.0f);
        environment.ambientColor = environment.backgroundColor;
        environment.ambientEnergy = 0.5f;

        game.deferredRenderer.ssaoEnabled = true;
        game.postProcessingRenderer.glowEnabled = true;
        game.postProcessingRenderer.fxaaEnabled = true;

        sun = addLight(LightType.Sun);
        sun.position.y = 50.0f;
        sun.shadowEnabled = true;
        sun.energy = 20.0f;
        sun.scatteringEnabled = false;
        sun.color = sunColor;
        sun.rotation =
            rotationQuaternion!float(Axis.y, degtorad(sunTurn)) *
            rotationQuaternion!float(Axis.x, degtorad(sunPitch));

        auto eSky = addEntity();
        auto psync = New!PositionSync(eventManager, eSky, camera);
        eSky.drawable = New!ShapeBox(Vector3f(1.0f, 1.0f, 1.0f), assetManager);
        eSky.scaling = Vector3f(100.0f, 100.0f, 100.0f);
        eSky.layer = EntityLayer.Background;
        eSky.material = New!Material(assetManager);
        eSky.material.depthWrite = false;
        eSky.material.culling = false;
        eSky.material.diffuse = environment.backgroundColor;
        eSky.material.shader = New!SkyShader(assetManager);

        auto matCube = New!Material(assetManager);
        matCube.diffuse = Color4f(1.0, 0.5, 0.3, 1.0);
        matCube.roughness = 0.4f;

        auto matBall = New!Material(assetManager);
        matBall.diffuse = Color4f(0.7, 0.1, 0.1, 1.0);
        matBall.roughness = 0.3f;

        auto box = New!NewtonBoxShape(Vector3f(1, 1, 1), world);

        cubeBodyControllers = New!(NewtonBodyComponent[])(numCubes);
        foreach(i; 0..cubeBodyControllers.length)
        {
            auto eCube = addEntity();
            eCube.drawable = aCubeMesh.mesh;
            eCube.material = matCube;
            eCube.position = Vector3f(0, i * 1.5, 0);
            auto b = world.createDynamicBody(box, 80.0f);
            cubeBodyControllers[i] = New!NewtonBodyComponent(eventManager, eCube, b);
        }

        auto sphere = New!NewtonSphereShape(0.5f, world);
        eCharacter = addEntity();
        eCharacter.drawable = New!ShapeSphere(0.5f, 24, 16, false, assetManager);
        eCharacter.material = matBall;
        eCharacter.position = Vector3f(5, 1, 0);
        auto bCharacter = world.createDynamicBody(sphere, 80.0f);
        bCharacter.groupId = world.kinematicGroupId;
        bCharacter.raycastable = false;
        bCharacter.enableRotation = false;
        bCharacterController = New!NewtonBodyComponent(eventManager, eCharacter, bCharacter);
        bCharacter.createUpVectorConstraint(Vector3f(0.0f, 1.0f, 0.0f));

        auto boxFloor = New!NewtonBoxShape(Vector3f(50, 1, 50), world);

        auto eFloor = addEntity();
        eFloor.position = Vector3f(0, -0.5, 0);
        auto b = world.createStaticBody(boxFloor);
        auto planeBodyController = New!NewtonBodyComponent(eventManager, eFloor, b);

        auto matPlane = New!Material(assetManager);
        matPlane.diffuse = aGridTexture.texture;
        matPlane.textureScale = Vector2f(5, 5);
        matPlane.roughness = 0.9f;
        auto ePlane = addEntity();
        ePlane.drawable = New!ShapePlane(50, 50, 10, assetManager);
        ePlane.material = matPlane;

        text = New!TextLine(aFontDroidSans14.font, "0", assetManager);
        text.color = Color4f(1.0f, 1.0f, 1.0f, 0.7f);
        auto eText = addEntityHUD();
        eText.drawable = text;
        eText.position = Vector3f(16.0f, 30.0f, 0.0f);
    }

    override void onKeyDown(int key)
    {
        if (key == KEY_ESCAPE)
            application.exit();
    }

    char[100] txt;

    override void onUpdate(Time t)
    {
        Vector3f rayStart = eCharacter.position;
        Vector3f rayEnd = eCharacter.position;
        rayEnd.y = -5.0f;
        groundHeight = -5.0f;
        world.raycast(rayStart, rayEnd, this);

        Vector3f targetVelocity = Vector3f(0, 0, 0);
        float speed = 6.0f;
        if (eventManager.keyPressed[KEY_A]) targetVelocity += camera.right * -speed;
        if (eventManager.keyPressed[KEY_D]) targetVelocity += camera.right * speed;
        if (eventManager.keyPressed[KEY_W]) targetVelocity += camera.direction * -speed;
        if (eventManager.keyPressed[KEY_S]) targetVelocity += camera.direction * speed;
        if (eventManager.keyPressed[KEY_SPACE]) jump(15.0f);

        Vector3f velocityChange = targetVelocity - bCharacterController.rbody.velocity;
        velocityChange.y = 0.0f;
        bCharacterController.rbody.velocity = bCharacterController.rbody.velocity + velocityChange;

        world.update(t.delta);

        freeview.setTargetSmooth(eCharacter.position, 1.0f);

        uint n = sprintf(txt.ptr, "FPS: %u", cast(int)(1.0 / eventManager.deltaTime));
        string s = cast(string)txt[0..n];
        text.setText(s);
    }

    void onRayHit(NewtonRigidBody nbody, Vector3f hitPoint, Vector3f hitNormal)
    {
        if (nbody.raycastable && hitPoint.y > groundHeight)
            groundHeight = hitPoint.y;
    }

    void jump(float height)
    {
        if (abs(eCharacter.position.y - groundHeight) <= (0.5f + 0.001f))
        {
            float jumpSpeed = sqrt(2.0f * height);
            Vector3f v = bCharacterController.rbody.velocity;
            v.y = jumpSpeed;
            bCharacterController.rbody.velocity = v;
        }
    }
}

class TestGame: Game
{
    this(uint w, uint h, bool fullscreen, string title, string[] args)
    {
        super(w, h, fullscreen, title, args);

        currentScene = New!TestScene(this);

        deferredRenderer.setViewport(0, 0, eventManager.windowWidth, eventManager.windowHeight);
        postProcessingRenderer.setViewport(0, 0, eventManager.windowWidth, eventManager.windowHeight);
        presentRenderer.setViewport(0, 0, eventManager.windowWidth, eventManager.windowHeight);
        hudRenderer.setViewport(0, 0, width, height);
    }

    override void onResize(int width, int height)
    {
        deferredRenderer.setViewport(0, 0, width, height);
        postProcessingRenderer.setViewport(0, 0, width, height);
        presentRenderer.setViewport(0, 0, width, height);
        hudRenderer.setViewport(0, 0, width, height);
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
