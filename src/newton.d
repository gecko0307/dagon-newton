module newton;

import std.stdio;
import std.string;
import std.conv;
import dlib.core.memory;
import dlib.math.vector;
import dlib.math.matrix;
import dlib.math.transformation;
import dlib.math.quaternion;
import dlib.math.utils;
import dagon.core.ownership;
import dagon.logics.entity;
import dagon.logics.controller;
public import bindbc.newton;

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

class NewtonPhysicsWorld: Owner
{
    NewtonWorld* newtonWorld;
    
    this(Owner o)
    {
        super(o);
        newtonWorld = NewtonCreate();
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
    
    NewtonRigidBody createBody(Vector3f extents, float mass)
    {
        NewtonRigidBody b = New!NewtonRigidBody(extents, mass, this, this);
        // TODO: store a list of bodies
        return b;
    }
    
    ~this()
    {
        NewtonMaterialDestroyAllGroupID(newtonWorld);
        NewtonDestroyAllBodies(newtonWorld);
        NewtonDestroy(newtonWorld);
    }
}

class NewtonRigidBody: Owner
{
    NewtonPhysicsWorld world;
    NewtonBody* newtonBody;
    float mass;
    Vector3f inertia;
    Vector3f gravity = Vector3f(0.0f, -9.8f, 0.0f);
    Vector3f force = Vector3f(0.0f, 0.0f, 0.0f);
    Vector3f torque = Vector3f(0.0f, 0.0f, 0.0f);
    Vector4f position = Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
    Quaternionf rotation = Quaternionf.identity;
    Matrix4x4f transformation = Matrix4x4f.identity;

    this(Vector3f extents, float mass, NewtonPhysicsWorld world, Owner o)
    {
        super(o);
        
        this.world = world;
        
        // TODO: user-defined shapes
        NewtonCollision* collision = NewtonCreateBox(world.newtonWorld, extents.x, extents.y, extents.z, 0, null);
        newtonBody = NewtonCreateDynamicBody(world.newtonWorld, collision, transformation.arrayof.ptr);
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
    
    void addForce(Vector3f f)
    {
        force += f;
    }
    
    void addTorque(Vector3f t)
    {
        torque += t;
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
