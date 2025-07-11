# The *PhysicsCollider* component

The `PhysicsCollider` component is the most important component for simulation with Unity Physics. By adding this component to an entity, you declare this entity as a rigid body which will participate in the physics simulation and in collision queries, such as ray casts and collider casts. This component determines what the body's collision geometry "looks" like in the physics simulation. It is analogous to a mesh in a rendering system. For placement of a rigid body it also needs transform components: a `LocalTransform` for dynamic bodies, and a `LocalTransform` and/or `LocalToWorld` for static bodies.

The most important property of a `PhysicsCollider` is a [`BlobAssetReference`](xref:Unity.Entities.BlobAssetReference`1) to the Collider data used by the physics simulation (which is in a format optimized for collision queries). If you aren't familiar with blob assets, we recommend you review the [available documentation](xref:blobassets). Each of the Collider types implement an `ICollider` interface, and for each Collider type, there is a static `Create()` function, which takes parameters specific to the shape. For example, a `SphereCollider` is built from a position and a radius, while a `ConvexCollider` is built from a point cloud.

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
| Compound        | `CompoundCollider` | A collider regrouping a set of other colliders. See the [Compound Collider](concepts-compound.md) section for more details.           |
| Terrain         | `TerrainCollider`  | A collider representing a terrain described by a uniform grid of height samples.                                                      |

>[!NOTE]
> For performance reasons, you should try to avoid using mesh colliders. Instead, use primitive types whenever possible, as this greatly reduces the calculations required in determining collisions between colliders. For example, if a collision geometry can be represented by a sphere, collision calculations only need to consider the sphere's center and its radius. With a mesh collider, however, every triangle in the mesh needs to be considered.

## Working with collider data in the editor

Using the Unity Editor's Inspector to display runtime data can be a powerful tool to examine a `PhysicsCollider` component and its collider blob. To show this data, have the subscene open for editing and have a GameObject selected. Navigate to the top right corner of the Inspector and click on the circle icon (see Figure 1) and choose the 'Runtime' option to display the entity components that are baked from this GameObject.

![Inspector Navigation](images/inspector-runtime-view.png)<br/>_Figure 1: How to navigate to the Runtime View in the Inspector._

Figure 2 shows an example of the Inspector dynamically updating in Runtime View while the `CollisionFilter` in a collider is modified each time a specified counter value is reached.

![Inspector Runtime Example](images/runtime-view-collisionfilter-change.gif)<br/>_Figure 2: An example of the Inspector updating in Runtime View._

# The collision filter

Each Collider also has a `CollisionFilter` which allows you to control what objects are permitted to collide with each other. The properties on this object allow you to categorize objects in relation to what types they collide with. For example, you might want to mark certain Colliders as "transparent" so that when performing a raycast test to determine if two characters can see each other, they are able to "see through" Colliders which are flagged as "transparent".

The default values in the colliders' collision filters ensure that every object collides with every other object. By configuring the filter in particular ways, you are able to opt-out of select collisions, depending on what you want to achieve in your game.

The filter contains two bitfield properties, `CollisionFilter.BelongsTo` and `CollisionFilter.CollidesWith` which are described in more detail below. Both these properties can be modified using standard bit manipulation. For example, to set the n'th bit in a property you can use `property |= 1 << n`.

The `BelongsTo` bitfield allows the collider to be assigned to one or more categories, such as "transparent" or "water", where each bit represents a single categorie.

The `CollidesWith` bitfield uses the same concept of categories, and together with the `BelongsTo` bitfield allows you to control what categories this collider will or will not collide with. Setting the bit of a given category in the `CollidesWith` bitfield will enable collision between this collider and the specified category. Analogously, if the bit is not set, collision between this collider and the specified category will be disabled. Collision with more than one category can be enabled by setting multiple bits accordingly.

Note that for a collision to take place, the filter in both colliders need to specify that there should be collision with the other collider. For example, say you have a collider A and a collider B, and the filter in collider A enables collision with collider B, but the filter in collider B disables collision with collider A. In this case, the two colliders will not collide.

Finally, the filter's `GroupIndex` property lets you override the `CollidesWith` and `BelongsTo` behavior described above. When this value is non-zero, the `CollidesWith` and `BelongsTo` properties are not checked at all. If the `GroupIndex` of both objects is equal and positive, then the object will always collide. If the value in both objects is equal and negative, then the objects will never collide. By default, this value is set to zero, and thus its overriding function disabled.

## Authoring collision filters

Collision filters can be modified at runtime as described above, and also specified at edit-time by using authoring components. When using the [custom physics authoring](custom-samples-physics-components.md) components, the available collision filter categories can be assigned names in your project by creating a `Physics Category Names` asset under _Assets -> Create -> Unity Physics -> Physics Category Names_. Up to 32 categories can be defined, corresponding to the bits in the collision filter's bitfields described above.
![Physics Category Names Asset](images/physics-category-names-U-inspector.png)<br/>_Figure 3: A custom authoring's `Physics Category Names` asset, defined in a project (left). An example of the `CollisionFilter.BelongsTo` field in a `Physics Shape` when the collider is set to collide with Everything (right)._

When the [built-in physics authoring components](built-in-components.md) are used, the collision filter's bitfield categories are set based on the game object's [Layer](xref:LayerBasedCollision) and the [include](xref:Collider-includeLayers) and [exclude](Collider-excludeLayers) layer masks specified in the game object's [Rigidbody](xref:class-Rigidbody) or [Collider](xref:CollidersOverview) components (see "Layer Overrides" section in the Inspector).

# Collider sharing and unique colliders

During the rigid body baking process, colliders are automatically shared by multiple rigid bodies, if they all use the exact same geometry shape and size parameters, and have the same collision filter and material settings. This automatic collider sharing is the default behavior in Unity Physics. The `PhysicsCollider` components of such rigid bodies will all refer to the exact same collider `BlobAssetReference` via their `PhysicsCollider.Value` fields.
Especially when mesh-based colliders are involved, this type of data sharing can have significant memory and performance benefits.

A consequence of this data sharing is that if a collider is modified, this change is applied to all the other identical and shared colliders. Sometimes this is beneficial and expected, but in cases when it is not, you can always make a shared collider unique to ensure the change is applied only to this collider instance. How to prevent a collider from being shared by making it unique so that it can be modified exclusively is described in the following section.

## Making colliders unique during authoring

There are multiple ways to make a collider unique, depending on how you are authoring your `PhysicsCollider`. If you are using the `Physics Shape` custom authoring component, then enable its `Force Unique` checkbox (see Figure 4).

![Force Unique Checkbox](images/force-unique-checkbox.png)<br/>_Figure 4: The Force Unique checkbox in the `Physics Shape` custom authoring component._

If you are using a built-in collider type, then to make it unique, add the `Force Unique Collider` authoring component (see Figure 5) to the same game object.

![Force Unique Collider Component](images/force-unique-component.png)<br/>_Figure 5: The Force Unique Collider component is used with a built-in collider to mark it as unique._

When using the `Force Unique Collider` component, the uniqueness will be applied to all colliders on a game object where this component is present. If a `Physics Shape` is used on this game object, but it does not have the `Force Unique` checkbox enabled, the `Force Unique Collider` component will take priority.

Also when forcing colliders unique within game objects in Prefabs with any of the above described methods, all Prefab Instances will contain unique colliders as expected.

>[!NOTE]
> The force unique feature is not yet supported for Compound Colliders. Compound Colliders, created during baking, will always be shared if possible.

## Making colliders unique at runtime

You can also take an authored collider and make it unique at runtime if ever you need to modify it. This lets you benefit from automatic collider sharing by default, and gives you the freedom to make only those colliders unique that you need to be unique for runtime modifications at a specific point in time.

For this case, you can check if a collider is already unique using its `IsUnique()` function, and if it is not, simply call its `MakeUnique()` function to disable collider sharing for this collider at this moment. An example usage of this method is in the code snippet below, where we want to change the restitution of one instance of some potentially shared collider.

```csharp
// Make the physics collider unique if it isn't already.
if (!physicsCollider.IsUnique)
{
    physicsCollider.MakeUnique(entity, EntityManager);
}

// Modify the restitution of the collider
physicsCollider.Value.Value.SetRestituion(0.5f);
```

# Making colliders move

By itself, a world containing entities only with `PhysicsCollider` components won't actually _do_ anything. This is because the rigid bodies you are declaring in this way are all treated as static – they cannot move. In order to make our simulations more interesting, you need to add the ability for rigid body positions and orientations to change.

Adding a `PhysicsVelocity` component to a rigid body entity makes the physics simulation aware that the rigid body can move. It can have a linear and angular velocity and its collider can therefore move.

## Mass properties and dynamic collisions

Suppose you have a scene with two rigid bodies with colliders, one of which has a rigid body with a `PhysicsVelocity` component and it is moving towards the other, static Collider. When you enter Play Mode, the moving Collider moves right through the static one. You haven't changed the collision filter, or made one of the colliders a trigger to explain the absence of collisions. When the two collide, you would expect forces to be applied between them.

So, what happened? This is a case of an unstoppable force meeting an immovable object.

The problem is that, even though one of the two colliders is moving, the simulation does not know how _heavy_ it is. So the simulation does not know how the moving collider would respond to a collision with the static one. In this case, Unity Physics treats the moving collider as if it had an _infinite_ mass, so it will not feel any reaction forces from collisions. It will instead either push every other dynamic object out of the way or move straight through static objects, as is the case in this scenario.

This kind of behaviour is useful in some cases. Suppose you had an elevator object in-game. It makes sense that it should follow your desired motion path _exactly_ - it should move no matter how many characters are in the lift and it should not get stuck on "snags" inside a bumpy elevator shaft. This behaviour is called "kinematic" or "keyframed" in physics simulations and can be modeled by only attaching a `PhysicsVelocity` component alongside the `PhysicsCollider` component to the rigid body entity and updating its motion path and velocity data manually.

For a realistic, automatically calculated physics-based collision with the moving collider, however, you need to inform the simulation system of its mass. To this end, you can add a `PhysicsMass` component to the rigid body entity and with it specify the body's mass properties, that is, its scalar mass, [moment of inertia](https://en.wikipedia.org/wiki/Moment_of_inertia) and center of mass. This provides Unity Physics with the information required to calculate how the body reacts to added impulses, and in this case, to a collision impulse.

>[!NOTE]
>Some of the mass properties are stored as inverses, which speeds up many of the internal physics calculations. It also allows you to specify infinite mass, by setting the relevant values to zero.

While you can provide the mass properties yourself, it is not necessary in most cases. Every `Collider` blob has a `MassProperties` property which is calculated automatically based on the collider geometry. You might find it more useful to use the calculated `MassProperties` property as a starting point, and then scale them – for example, by multiplying the mass by ten for an extra-heavy gameplay object.

# Modifying colliders

Colliders can be modified via the `PhysicsCollider` component's `BlobAssetReference<Collider>` value. Caution should be taken when modifying this collider blob for two reasons. First, because you are modifying data which is potentially shared, and second, because the timing of this operation will matter to the physics simulation. It is generally safe to modify collider data in jobs that declare write access to the `PhysicsCollider` component, but it should be avoided when write-access is not specified. Additionally, this modification should be timed so that it happens before or after the physics simulation, or else you may encounter issues.

To make a change in a collider you need to access the corresponding `PhysicsCollider` component using any of the available methods, e.g., `EntityManager.GetComponentData<PhysicsCollider>(entity)`. Then, you can change the data within the collider blob directly, while remaining aware of the collider blob potentially being shared across multiple `PhysicsCollider` components, or after ensuring it is unique as explained in the previous section.

The following example code snippet demonstrates how to change a collider's collision filter.

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

## Modifying collider geometry
There are several ways in which the geometry of colliders can be modified.

All colliders can be uniformly scaled at runtime. This can be achieved by modifying the `Scale` member in the corresponding rigid body entity's `LocalTransform` component as described in the section on [rigid bodies](concepts-data.md).

Besides purely uniform scaling, additional geometric modifications can be achieved in two ways.
1. Through a collider's type-specific geometric parameters.
2. Through the generic [Collider.BakeTransform()](xref:Unity.Physics.Collider.BakeTransform(AffineTransform)) method.

Both approaches are described in more detail below. For a demonstration of these approaches, please refer to the [Unity Physics Samples](https://github.com/Unity-Technologies/EntityComponentSystemSamples/).

### Type-specific geometry modifications
You can modify the geometry of any primitive collider through their type-specific geometric parameters. These collider types offer a specialized `Geometry` property (e.g., the [BoxCollider.Geometry](xref:Unity.Physics.BoxCollider.Geometry) property of type [BoxGeometry](xref:Unity.Physics.BoxGeometry)) which can be readily changed. As such, a [BoxCollider](xref:Unity.Physics.BoxCollider) can be modified by scalings its width, length and height, or a [CylinderCollider](xref:Unity.Physics.CylinderCollider) can be modified in height or radius, etc.

![Specific Geometry Modification Example](images/collider-modify-geometry.gif)<br/>_Figure 6: The type-specific geometry parameters of a box are modified at runtime using the `PhysicsCollider` component view in the Inspector panel._

Other collider types, such as mesh-based colliders or compound colliders, don't offer any type-specific geometric parameters, but they can be modified using the generic method described below.

### Generic geometry modifications
A generic way for modifying the geometry of colliders is offered by the [Collider.BakeTransform()](xref:Unity.Physics.Collider.BakeTransform(Unity.Mathematics.AffineTransform)) method. This method bakes any given affine transformation into the underlying geometry of the collider. An affine transformation can contain translation, rotation, uniform or non-uniform scale and even shear, and when applied to a collider, the collider's geometry is accordingly translated, rotated, scaled and sheared in the process.

![Collider.BakeTransform Example](images/collider-bake-transform.gif)<br/>_Figure 7: An example showing various different colliders (in blue), including mesh-based collider types, being modified using the `Collider.BakeTransform()` method. Given an affine transformation to be baked into a collider, this generic method directly modifies the type-specific geometric parameters of the affected collider accordingly, as we can see in the inspector panel (right side) for the selected `CylinderCollider`. See the "Modify Runtime - Collider Geometry" scene in the [Unity Physics Samples](https://github.com/Unity-Technologies/EntityComponentSystemSamples/)._

As opposed to the type-specific geometry modifications, this approach works for both primitive and non-primitive collider types. Depending on the collider type, The resultant geometric modification will either always be exact (as is the case for the mesh-based collider types `MeshCollider` and `ConvexCollider`) or can be approximative (for primitive types). As such, the point clouds defining the mesh-based colliders are simply transformed in accordance with the provided affine transformation, while primitives are transformed as best as possible, which in certain cases can also lead to an exact transformation. An example for an exact modification of a primitive type would be applying an affine transformation which contains a non-uniform scale but no rotation to a box collider, which will simply scale the box' side lengths accordingly and exactly.

### Modifying the geometry of dynamic rigid bodies

When a collider's geometry is modified with either of the two approaches described above, its corresponding rigid body entity's mass distribution and center of mass, defined through the `PhysicsMass` component, will remain the same. This will make dynamic rigid bodies not entirely behave in accordance with their new shape. For example, such bodies would collide with their new shape, but the resultant collision responses could cause motion patterns that seem unrealistic.

In order to correct this situation, you can manually update the mass properties in the `PhysicsMass` component by using the information in the collider's [Collider.MassProperties](xref:Unity.Physics.Collider.MassProperties) property. The latter is  automatically adjusted to the collider's new shape each time you modify its geometry.

# Runtime collider creation
It is possible to create colliders manually and during runtime through the various collider types' static `Collider.Create()` functions. These functions produce collider blobs which can be assigned to a `PhysicsCollider` component's `Value` field. The `PhysicsCollider` component in turn has to be added to some entity in order to become part of the physics simulation.

Creating a `PhysicsCollider` component at runtime can be a time-consuming process since adding this component to an entity incurs a structural change. Therefore, this operation should be done with care.
To avoid structural changes, it is possible to create a `PhysicsCollider` component with a null `BlobAssetReference` ahead of time and to assign another, non-null collider blob at a later moment during runtime.

>[!NOTE]
> When a new `BlobAssetReference<Collider>` is manually created, it is up to you to keep track of the reference and dispose of it when the asset is no longer needed. To do this, you could add the `BlobAssetReference` to a NativeList for later disposal.

The following code snippet demonstrates an example of how to create a `MeshCollider` from a `UnityEngine.Mesh` in a job. Note that you cannot directly use `UnityEngine.Mesh` in a job, because it is a managed component; instead, you must pass either the `MeshData` or `MeshDataArray` into the job. Manual management of the BlobAssetReference lifetime is required in this situation.

```csharp
struct CreateFromMeshDataJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<UnityEngine.Mesh.MeshData> MeshData;
    [WriteOnly] public NativeArray<BlobAssetReference<Collider>> ColliderBlobReference;

    public void Execute(int i)
    {
        ColliderBlobReference[i] = MeshCollider.Create(MeshData[i], CollisionFilter.Default, Material.Default);
    }
}

public void MeshCollider_CreateFromJob(UnityEngine.Mesh engineMesh)
{
    int num = 3; // We are 3 creating colliders of the same mesh for example purposes only.
    var colliderBlobReferenceTracking = new NativeArray<BlobAssetReference<Collider>>(num, Allocator.Persistent);

    using var meshData = new NativeArray<UnityEngine.Mesh.MeshData>(num, Allocator.TempJob);

    for (int i = 0; i < num; i++)
    {
        var meshDataArray = UnityEngine.Mesh.AcquireReadOnlyMeshData(engineMesh);
        meshData[i] = meshDataArray[0];
    }

    new CreateFromMeshDataJob()
    {
        MeshData = meshData,
        ColliderBlobReference = colliderBlobReferenceTracking
    }.Run(num);

    for (int i = 0; i < num; i++)
    {
        var physicsCollider = new PhysicsCollider
        {
            Value = colliderBlobReferenceTracking[i]
        };
        // do stuff
    }

    // Make sure to dispose of the collider blobs that are tracked in
    // colliderBlobReferenceTracking once the blobs are no longer required.
    ...
}
```

For a more complex demonstration of runtime collider creation, refer to the [Unity Physics Samples Project](https://github.com/Unity-Technologies/EntityComponentSystemSamples/tree/master/PhysicsSamples); specifically the Modify/Runtime Collider Modification section in a demo called "Runtime Collider Creation". This demo focuses on using `UnityEngine.Mesh` to create several `MeshCollider`s at runtime as a reaction to a trigger event.

