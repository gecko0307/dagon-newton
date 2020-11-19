module newton;

import std.stdio;
import std.string;
import std.conv;
import dlib.core.ownership;
import dlib.core.memory;
import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.transformation;
import dlib.math.quaternion;
import dlib.math.utils;
import dagon.core.event;
import dagon.core.time;
import dagon.graphics.entity;
public import bindbc.newton;

extern(C) nothrow @nogc void newtonBodyForceCallback(
    const NewtonBody* nbody,
    dFloat timestep,
    int threadIndex)
{
    NewtonRigidBody b = cast(NewtonRigidBody)NewtonBodyGetUserData(nbody);
    if (b)
    {
        Vector3f gravityForce = b.gravity * b.mass;
        NewtonBodyAddForce(nbody, gravityForce.arrayof.ptr);
        NewtonBodyAddForce(nbody, b.force.arrayof.ptr);
        NewtonBodyAddTorque(nbody, b.torque.arrayof.ptr);
        b.force = Vector3f(0.0f, 0.0f, 0.0f);
        b.torque = Vector3f(0.0f, 0.0f, 0.0f);
    }
}

extern(C) dFloat newtonWorldRayFilterCallback(
    const NewtonBody* nbody,
    const NewtonCollision* shapeHit,
    const dFloat* hitContact,
    const dFloat* hitNormal,
    dLong collisionID,
    void* userData,
    dFloat intersectParam)
{
    NewtonRaycaster raycaster = cast(NewtonRaycaster)userData;
    NewtonRigidBody b = cast(NewtonRigidBody)NewtonBodyGetUserData(nbody);
    if (raycaster && b)
    {
        Vector3f p = Vector3f(hitContact[0], hitContact[1], hitContact[2]);
        Vector3f n = Vector3f(hitNormal[0], hitNormal[1], hitNormal[2]);
        raycaster.onRayHit(b, p, n);
    }
    return 1.0f;
}

extern(C) uint newtonWorldRayPrefilterCallback(
    const NewtonBody* nbody,
    const NewtonCollision* collision,
    void* userData)
{
    return 1;
}

interface NewtonRaycaster
{
    void onRayHit(NewtonRigidBody nbody, Vector3f hitPoint, Vector3f hitNormal);
}

class NewtonPhysicsWorld: Owner
{
    NewtonWorld* newtonWorld;
    int defaultGroupId;
    int kinematicGroupId;

    this(Owner o)
    {
        super(o);
        newtonWorld = NewtonCreate();
        defaultGroupId = NewtonMaterialGetDefaultGroupID(newtonWorld);
        kinematicGroupId = createGroupId();
        NewtonMaterialSetDefaultElasticity(newtonWorld, defaultGroupId, kinematicGroupId, 0.0f);
    }
    
    int createGroupId()
    {
        return NewtonMaterialCreateGroupID(newtonWorld);
    }

    void loadPlugins(string dir)
    {
        NewtonLoadPlugins(newtonWorld, dir.toStringz);
        void* p = NewtonGetPreferedPlugin(newtonWorld);
        writeln("Selected plugin: ", NewtonGetPluginString(newtonWorld, p).to!string);
    }

    void update(double dt)
    {
        NewtonUpdate(newtonWorld, dt);
    }

    NewtonRigidBody createDynamicBody(NewtonCollisionShape shape, float mass)
    {
        NewtonRigidBody b = New!NewtonRigidBody(shape, mass, this, this);
        // TODO: store a list of bodies
        return b;
    }

    NewtonRigidBody createStaticBody(NewtonCollisionShape shape)
    {
        return createDynamicBody(shape, 0.0f);
    }

    void raycast(Vector3f pstart, Vector3f pend, NewtonRaycaster raycaster)
    {
        NewtonWorldRayCast(newtonWorld, pstart.arrayof.ptr, pend.arrayof.ptr, &newtonWorldRayFilterCallback, cast(void*)raycaster, &newtonWorldRayPrefilterCallback, 0);
    }

    ~this()
    {
        NewtonMaterialDestroyAllGroupID(newtonWorld);
        NewtonDestroyAllBodies(newtonWorld);
        NewtonDestroy(newtonWorld);
    }
}

abstract class NewtonCollisionShape: Owner
{
    NewtonPhysicsWorld world;
    NewtonCollision* newtonCollision;

    this(NewtonPhysicsWorld world)
    {
        super(world);
        this.world = world;
    }

    ~this()
    {
        if (newtonCollision)
            NewtonDestroyCollision(newtonCollision);
    }

    // Override me
    Vector3f inertia(float mass)
    {
        return Vector3f(mass, mass, mass);
    }
}

class NewtonBoxShape: NewtonCollisionShape
{
    Vector3f halfSize;

    this(Vector3f extents, NewtonPhysicsWorld world)
    {
        super(world);
        newtonCollision = NewtonCreateBox(world.newtonWorld, extents.x, extents.y, extents.z, 0, null);
        halfSize = extents * 0.5f;
    }

    override Vector3f inertia(float mass)
    {
        float x2 = halfSize.x * halfSize.x;
        float y2 = halfSize.y * halfSize.y;
        float z2 = halfSize.z * halfSize.z;
        return Vector3f(
            (y2 + z2)/3 * mass,
            (x2 + z2)/3 * mass,
            (x2 + y2)/3 * mass
        );
    }
}

class NewtonSphereShape: NewtonCollisionShape
{
    float radius;

    this(float radius, NewtonPhysicsWorld world)
    {
        super(world);
        this.radius = radius;
        newtonCollision = NewtonCreateSphere(world.newtonWorld, radius, 0, null);
    }

    override Vector3f inertia(float mass)
    {
        float v = 0.4f * mass * radius * radius;
        return Vector3f(v, v, v);
    }
}

class NewtonRigidBody: Owner
{
    NewtonPhysicsWorld world;
    NewtonBody* newtonBody;
    int materialGroupId;
    float mass;
    Vector3f inertia;
    Vector3f gravity = Vector3f(0.0f, -9.8f, 0.0f);
    Vector3f force = Vector3f(0.0f, 0.0f, 0.0f);
    Vector3f torque = Vector3f(0.0f, 0.0f, 0.0f);
    Vector4f position = Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
    Quaternionf rotation = Quaternionf.identity;
    Matrix4x4f transformation = Matrix4x4f.identity;
    bool enableRotation = true;
    bool raycastable = true;

    bool isRaycastable() { return raycastable; }

    this(NewtonCollisionShape shape, float mass, NewtonPhysicsWorld world, Owner o)
    {
        super(o);

        this.world = world;

        newtonBody = NewtonCreateDynamicBody(world.newtonWorld, shape.newtonCollision, transformation.arrayof.ptr);
        NewtonBodySetUserData(newtonBody, cast(void*)this);
        this.groupId = world.defaultGroupId;
        this.mass = mass;
        this.inertia = shape.inertia(mass);
        NewtonBodySetMassMatrix(newtonBody, mass, inertia.x, inertia.y, inertia.z);
        NewtonBodySetForceAndTorqueCallback(newtonBody, &newtonBodyForceCallback);
    }

    void update(double dt)
    {
        NewtonBodyGetPosition(newtonBody, position.arrayof.ptr);
        NewtonBodyGetMatrix(newtonBody, transformation.arrayof.ptr);
        if (enableRotation)
        {
            rotation = Quaternionf.fromMatrix(transformation);
        }
        else
        {
            rotation = Quaternionf.identity;
            transformation = translationMatrix(position.xyz);
            NewtonBodySetMatrix(newtonBody, transformation.arrayof.ptr);
        }
    }
    
    void groupId(int id) @property
    {
        NewtonBodySetMaterialGroupID(newtonBody, id);
        materialGroupId = id;
    }
    
    int groupId() @property
    {
        return materialGroupId;
    }

    void addForce(Vector3f f)
    {
        force += f;
    }

    void addTorque(Vector3f t)
    {
        torque += t;
    }

    void createUpVectorConstraint(Vector3f up)
    {
        NewtonJoint* joint = NewtonConstraintCreateUpVector(world.newtonWorld, up.arrayof.ptr, newtonBody);
    }

    void velocity(Vector3f v) @property
    {
        NewtonBodySetVelocity(newtonBody, v.arrayof.ptr);
    }

    Vector3f velocity() @property
    {
        Vector3f v;
        NewtonBodyGetVelocity(newtonBody, v.arrayof.ptr);
        return v;
    }
}

class NewtonBodyComponent: EntityComponent
{
    NewtonRigidBody rbody;
    Matrix4x4f prevTransformation;

    this(EventManager em, Entity e, NewtonRigidBody b)
    {
        super(em, e);
        rbody = b;

        Quaternionf rot = e.rotation;
        rbody.transformation =
            translationMatrix(e.position) *
            rot.toMatrix4x4;

        NewtonBodySetMatrix(rbody.newtonBody, rbody.transformation.arrayof.ptr);

        prevTransformation = Matrix4x4f.identity;
    }

    override void update(Time t)
    {
        rbody.update(t.delta);

        entity.prevTransformation = prevTransformation;

        entity.position = rbody.position.xyz;
        entity.transformation = rbody.transformation * scaleMatrix(entity.scaling);
        entity.invTransformation = entity.transformation.inverse;
        entity.rotation = rbody.rotation;

        entity.absoluteTransformation = entity.transformation;
        entity.invAbsoluteTransformation = entity.invTransformation;
        entity.prevAbsoluteTransformation = entity.prevTransformation;

        prevTransformation = entity.transformation;
    }
}
