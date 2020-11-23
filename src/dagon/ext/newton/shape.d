/*
Copyright (c) 2019-2020 Timur Gafarov

Boost Software License - Version 1.0 - August 17th, 2003
Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement, including
the above license grant, this restriction and the following disclaimer,
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

module dagon.ext.newton.shape;

import dlib.core.ownership;
import dlib.core.memory;
import dlib.math.vector;
import dlib.math.matrix;
import bindbc.newton;
import dagon.graphics.mesh;
import dagon.ext.newton.world;

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
    
    void setTransformation(Matrix4x4f m)
    {
        if (newtonCollision)
            NewtonCollisionSetMatrix(newtonCollision, m.arrayof.ptr);
    }
}

class NewtonBoxShape: NewtonCollisionShape
{
    Vector3f halfSize;

    this(Vector3f extents, NewtonPhysicsWorld world)
    {
        super(world);
        newtonCollision = NewtonCreateBox(world.newtonWorld, extents.x, extents.y, extents.z, 0, null);
        NewtonCollisionSetUserData(newtonCollision, cast(void*)this);
        halfSize = extents * 0.5f;
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
        NewtonCollisionSetUserData(newtonCollision, cast(void*)this);
    }
}

class NewtonMeshShape: NewtonCollisionShape
{
    this(Mesh mesh, NewtonPhysicsWorld world)
    {
        super(world);
        NewtonMesh* nmesh = NewtonMeshCreate(world.newtonWorld);
        NewtonMeshBeginBuild(nmesh);
        foreach(face; mesh.indices)
        foreach(i; face)
        {
            Vector3f p = mesh.vertices[i];
            Vector3f n = mesh.normals[i];
            NewtonMeshAddPoint(nmesh, p.x, p.y, p.z);
            NewtonMeshAddNormal(nmesh, n.x, n.y, n.z);
        }
        NewtonMeshEndBuild(nmesh);
        
        newtonCollision = NewtonCreateTreeCollisionFromMesh(world.newtonWorld, nmesh, 0);
        NewtonCollisionSetUserData(newtonCollision, cast(void*)this);
        
        NewtonMeshDestroy(nmesh);
    }
}

class NewtonCompoundShape: NewtonCollisionShape
{
    this(NewtonCollisionShape[] shapes, NewtonPhysicsWorld world)
    {
        super(world);
        newtonCollision = NewtonCreateCompoundCollision(world.newtonWorld, 0);
        NewtonCompoundCollisionBeginAddRemove(newtonCollision);
        foreach(shape; shapes)
        {
            NewtonCompoundCollisionAddSubCollision(newtonCollision, shape.newtonCollision);
        }
        NewtonCompoundCollisionEndAddRemove(newtonCollision);
    }
}
