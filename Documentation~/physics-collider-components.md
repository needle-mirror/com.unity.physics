# The *PhysicsCollider* component

This is the most important component for the simulation of physics. By adding this component to an entity, you declare that this body will participate in the physics simulation and collision queries (though you also need transform components: `LocalTransform` for dynamic bodies, `LocalTransform`, and/or `LocalToWorld` for static bodies). This component decides what the collision geometry "looks" like to the physics simulation. It is analogous to a mesh in a rendering system.

For performance reasons, you should try to avoid using a mesh during physics – use specialized primitive types when possible, which greatly simplifies the process of determining collisions between two Colliders. For example, if a collision geometry can be represented by a sphere, you can write collision tests which only need to consider the sphere center and radius; with a mesh wrapping the same sphere, you would need to consider every triangle in the mesh.

The most important property of a `PhysicsCollider` is a `BlobAssetReference` to the Collider data used by the physics simulation (which is in a format optimized for collision queries). Each of the Collider types implement an `ICollider` interface, and for each Collider type, there is a static `Create()` function, which takes parameters specific to the shape. For example, a `SphereCollider` is built from a position and a radius, while a `ConvexCollider` is built from a point cloud.

>[!NOTE]
>It is possible to create a Collider which has a null `BlobAssetReference`. Unity Physics still simulates the body, but it cannot collide with anything. This can be useful in some particular scenarios – for example you can connect such a body to another body using a joint, and feed the simulated positions into your skinning system.

## Collider types
Various collider types are supported in Unity Physics. The collider type of every `PhysicsCollider` is defined by its `Collider` blob and can be obtained through the `Collider.ColliderType` property.

| Type            | Name               | Description                                                                                                                           |
|-----------------|--------------------|---------------------------------------------------------------------------------------------------------------------------------------|
| Convex          | `ConvexCollider`   | A collider in the shape of an arbitrary convex hull created from a set of 3D points.                                                  |
| Sphere          | `SphereCollider`   | A collider in the shape of a sphere defined by a radius.                                                                              |
| Capsule         | `CapsuleCollider`  | A collider in the shape of a capsule formed by an inner line segment and a radius.                                                    |
| Triangle / Quad | `PolygonCollider`  | A flat convex collider with either 3 or 4 coplanar vertices forming either a triangle or a quad, respectively.                        |
| Box             | `BoxCollider`      | A collider in the shape of a box represented by a local center position and a 3D size parameter, defining the three box side lengths. |
| Cylinder        | `CylinderCollider` | A collider in the shape of a cylinder represented by a local position and orientation, and a cylinder height and radius.              |
| Mesh            | `MeshCollider`     | A collider representing a mesh comprised of triangles and quads.                                                                      |
| Compound        | `CompoundCollider` | A collider containing instances of other colliders.                                                                                   |
| Terrain         | `TerrainCollider`  | A collider representing a terrain described by a uniform grid of height samples.                                                      |

## Scaling colliders
All collider types can be uniformly scaled at run-time when they belong to a dynamic rigid body. This can be achieved by modifying the `Scale` member in the entity's `LocalTransform` component.

Besides uniform scaling, additional non-uniform and type specific scaling can be achieved through use of the individual collider types' geometric parameters. 
For example, a box could be scaled by modifying its width, length and height, or a cylinder could be changed in height or radius, etc.
Only primitive types (Sphere, Box, Capsule and Cylinder) support such non-uniform scaling. Mesh collider types (Mesh, Convex and Terrain) and compound colliders can not be modified.

For a demonstration of both uniform and non-uniform scaling at runtime, please refer to the [Unity Physics Samples](https://github.com/Unity-Technologies/EntityComponentSystemSamples/). 

## The Collision filter

Each Collider also has a `CollisionFilter` which allows you to control what objects are permitted to collide with each other. The properties on this object allow you to categorize objects in relation to what types they collide with. For example, you might want to mark certain Colliders as "transparent" so that when performing a raycast test to determine if two characters can see each other, they are able to "see through" Colliders which have the transparent bit set.

The default values for collision filter ensure that every object collides with every other object. By configuring the filter in particular ways, you are able to opt-out of select collisions, depending on what you want from gamecode.

## Modifying PhysicsCollider

It is possible to modify PhysicsCollider's `BlobAssetReference<Collider>` value. While it is completely safe to do so in jobs that declare write access to `PhysicsCollider` component, you need to be extra careful while modifying it otherwise. For example, you could access the collider using `EntityManager.GetComponentData<PhysicsCollider>(entity)` and change the collision filter of the blob. In order for this change to be picked up by the engine, you also need to write back to original `PhysicsCollider` of that entity. Below is a code snippet demonstrating how to properly change physics collider using both approaches.

```csharp
// Change the filter to CollisionFilter.Zero using Burst-compiled job
// This is the recommended way of changing the PhysicsCollider, as it is Burst compatible.
// As long as components are accessed by reference instead of by value, the change will
// be picked up by the physics engine, and will work as intended.

using UnityEngine;
using Unity.Entities;
using Unity.Burst;
using Unity.Physics;

public partial class ChangeColliderSystem : ISystem
{
    [BurstCompile]
    public partial struct ChangeColliderJob : IJobEntity
    {
        [WithAll(typeof(ChangeColliderFilterJob))]
        public void Execute(ref PhysicsCollider collider)
        {
            collider.Value.Value.SetCollisionFilter(CollisionFilter.Zero);
        }
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        state.Dependency = new ChangeColliderJob().Schedule(state.Dependency);
    }
}

```

```csharp
// Changing the PhysicsCollider using EntityManager.Get/SetComponentData.
// This is not the recommended way, since it isn't Burst friendly, but if needed
// this is how it needs to happen
for (int i = 0; i < entities.Length; i++)
{
    var entity = entities[i];
    var collider = EntityManager.GetComponentData<PhysicsCollider>(entity);

    collider.Value.Value.SetCollisionFilter(CollisionFilter.Zero);

    // IMPORTANT: collider has been changed, but the PhysicsCollider component data hasn't
    // been written back to, and if left at this state, the change might not get caught by
    // the physics engine, so it is necessary to proceed with the write back using SetComponentData
    EntityManager.SetComponentData(entity, collider);
}
```

# Dynamic bodies

By itself, a world containing entities with `PhysicsCollider` components won't actually _do_ anything. This is because the bodies you declare are all treated as static – they cannot move, and from the perspective of collision, they have infinite mass. In order to make our simulations more interesting, you need to add the ability for body transforms to change.

Adding a `PhysicsVelocity` component makes the physics simulation aware that the Collider can have some linear and angular speed and that the Collider should move. You can change the values of this component yourself if you wish to control how a Collider is moving, but during the physics simulation, Unity Physics also computes a new value for velocity (from gravity and forces from contacts/joints) and updates the component.

# Mass

Suppose you have a scene with two physics Colliders, one of which has a velocity pointing towards the static Collider. When you enter Play mode, the moving Collider moves right through the static one. You haven't changed the collision filter, or made one a trigger, so what happened? This is a case of an unstoppable force meeting an immovable object. As explained before, the Collider without a velocity cannot move. When the two collide, you would expect forces to be applied between them.

The problem is that, even though one Collider is moving, the simulation does not know how _heavy_ it is, so does not know how it would respond to collisions. In this case, Unity Physics treats the moving Collider as if it had an _infinite_ mass, so it will just push every object out of the way.

This kind of behaviour is useful in some scenarios. Suppose you had an elevator object in-game. It makes sense that it should follow your desired path _exactly_ – it should move no matter how many characters are in the lift and it should not get stuck on "snags" inside a bumpy elevator shaft. This behaviour is sometimes called "kinematic" or "keyframed" in other physics simulations.

To inform the simulation of masses, you can use the `PhysicsMass` component. This tells Unity Physics how a Collider reacts to an impulse. It stores the mass and [inertia tensor](https://en.wikipedia.org/wiki/Moment_of_inertia) for that entity as well as a transform describing the orientation of the inertia tensor and center of mass.

>[!NOTE]
>Some of these values are stored as inverses, which speeds up many of the internal physics calculations. It also allows you to specify infinite values, by setting the relevant component to zero.

While you can provide these values yourself, it is not necessary in many cases; the `ICollider` for a Collider interface has a `MassProperties` property, where appropriate values are calculated for you automatically. You might find it more useful to use the calculated `MassProperties` property as a starting point, and then scale them – for example, by multiplying the mass by ten for an extra-heavy gameplay object.

# Other components

Some additional components allow you to change the simulation behavior:

| Component              | Description                                                                                                                                                                                                                                                           |
|------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `PhysicsStep`          | Allows you to override the default simulation parameters for the whole scene, for example, by changing the direction of gravity or increasing the solver iteration count, making it more rigid (at the cost of performance).                                          |
| `PhysicsDamping`       | Allows you to add a per-Collider "slow-down" factor. Every step, a Collider with this component will have its velocities scaled down. This could slow down objects, making them more stable, or be used as a cheap approximation of aerodynamic drag.                 |
| `PhysicsGravityFactor` | Allows you to scale the amount of gravity applied to an individual Collider. Some objects look more realistic if they appear to fall faster. Other objects (for example, hot air balloons) appear to fall _up_, which can be emulated with a negative gravity factor. |
