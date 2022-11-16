# Collision queries

*Collision queries* (also known as *spatial queries*) are one of the most important features of any physics engine and often drive a significant amount of game logic. Unity Physics has a powerful collision query system which supports queries like ray casting, linear casting and closest point estimation. These queries support options like efficient collision filtering and user data retrieval.

Collision queries use physics simulation data (read-only access), more precisely, an internal structure called the broadphase. The broadphase is built by a job scheduled in [PhysicsInitializeGroup](physics-pipeline.md#physics-pipeline). Collision queries can be performed any time before or after `PhysicsInitializeGroup`.
By default, the broadphase will not get updated until the next physics step's `PhysicsInitializeGroup`, so all collision queries will return results according to that state (effectively the state after the previous simulation step).
To update the broadphase before the next simulation step (at some performance cost), check `Synchronize Collision World` in the Physics Step component or set `PhysicsStep.SynchronizeCollisionWorld` to a positive value. In that case, it is also advised to perform collision queries after `PhysicsSimulationGroup` or before the next `PhysicsInitializeGroup`.

## Local Vs Global

Queries can be performed against individual colliders or against an entire collision world. When performed against an entire world, a query acceleration structure – a bounding volume tree in the case of Unity Physics – is used for efficiency.

You can choose to create your own collision worlds which are entirely independent of the physics world. However, if you are performing queries against the same physics world you are simulating, which is common, then you can take advantage of the fact that the broad phase had already been built.

## Query Types

| Query Type        | Input                                            | Description                                                                                                       |
| ----------------- | ------------------------------------------------ |-------------------------------------------------------------------------------------------------------------------|
| Ray cast          | Start, End, Filter                               | Finds all (or closest) intersections for an oriented line segment.                                                |
| Collider cast     | Collider, Start, End, Collider scale             | Finds all (or closest)  intersection for the given shape swept along the given line segment in the given context. |
| Collider Distance | Collider, Position, Max distance, Collider scale | Finds the closest point between the given shape and any others within a specified maximum radius.                 |
| Point Distance    | Point, Filter, Max distance, Collider scale      | Finds the closest point to any shapes within a given maximum radius of the specified point.                       |
| Overlap query     | AABB, Filter                                     | Find all the bodies with bounding boxes overlapping a given area.                                                 |

Convenience functions are provided to return any hit, the closest hit or all hits. These convenience functions use [collectors](#collectors) to interpret their results.

 >**Note:** Unity Physics also provides direct access to all underlying query algorithms, e.g. ray sphere intersection, so that you can
 use these algorithms directly if desired without allocating memory for the collision world and colliders.

### Query outputs

Many queries produce similar outputs or hits as they are referred to in the code. These hits have a subset of the following fields

| Type             | Description                                                                                                                                              |
| ---------------- |----------------------------------------------------------------------------------------------------------------------------------------------------------|
| `Fraction`       | The proportion along the ray or cast in the given direction to the point of intersection.                                                                |
| `Position`       | The point of intersection on the surface, in world space.                                                                                                |
| `SurfaceNormal`  | The normal to the surface at the point of intersection, in world space.                                                                                  |
| `RigidBodyIndex` | The index of the rigid body in the collision world if a world query was performed.                                                                       |
| `Entity`         | The Entity of the rigid body in the collision world if a world query was performed.                                                                      |
| `ColliderKey`    | Internal information about which part of a composite shape (e.g. mesh) was hit – i.e. a reference to which triangle or quad in the mesh collider was hit. |
| `Material`       | Material information of the hit collider, or part of a composite shape defined by ColliderKey.                                                           |

### Ray cast

Ray cast queries use start and end points as their input and produce a set of hit results.

Lower level ray cast routines against primitives have a slightly different input. They do not compute the surface intersection point explicitly for efficiency reasons. Instead, given a ray of origin, `O` and displacement (a vector defining direction & distance), `D`, they return a hit fraction, `f`, which can be used later to compute the hit position if needed using `O + (D * f)`. See [Ray](xref:Unity.Physics.Ray) in API documentation for more details.

>**Note:** Rays starting inside primitives (spheres, capsules, box and convex shapes) confirm a hit at the starting point, but do not report another intersection as the ray leaves the volume.

### Collider cast

Collider casts sweep a collider along a line segment stopping at the first point of contact with another collider. These queries can be significantly more expensive than performing a ray cast. In the image below you can see the results (magenta) of casting a collider (orange) against other colliders in the world (yellow).

![collider_cast](images/collider_cast_queries.gif)

Collider cast inputs are the following:
- `Collider`: A reference to the collider to cast.
- `Orientation`: The orientation of the collider.
- `Start`: The initial position of the collider.
- `End`: The final position the collider should be swept to.
- `QueryColliderScale`: The uniform scale to apply to the casted collider.

### Distance query

Distance queries – or closest point queries – are often used to determine proximity to surfaces. The image below shows the results (magenta points) of a distance query between the query collider (orange) and the rest of the collision world (yellow).

![Closest_Points](images/closest_points_all_hits.gif)

You can see that not all queries return a result for all shapes. This is because the query has specified a maximum range which helps control the computational cost of the query.

There are two types of distance queries: **Point Distance Queries** and **Collider Distance Queries**. Their inputs are the following:
Point Distance Queries:
 * `Position`: The origin point of the query.
 * `MaxDistance`: The maximum distance the query should check away from the point.
 * `CollisionFilter`: Filter to determine what objects should be ignored by the query.

Collider Distance Queries:
 * `Collider`: A reference to the collider to query.
 * `Transform`: The Input collider's transform in the calling Collider's local space.
 * `MaxDistance`: Filter to determine what objects should be ignored by the query.
 * `Scale`: The uniform scale to apply to the casted collider.

See [PointDistanceInput](xref:Unity.Physics.PointDistanceInput) and [ColliderDistanceInput](xref:Unity.Physics.ColliderDistanceInput) in API documentation for more details about the input structures.

### Overlap query

Overlap queries are performed by calling `OverlapAabb` directly on the `CollisionWorld`. Given an `Aabb` and a `CollisionFilter` this query returns a list of indices into the bodies in the `CollisionWorld`.

## Collectors

Collectors provide a code driven way for you to intercept results as they are generated from any of the queries. When intercepting a result, you can control the efficiency of a query by exiting early for example.

Unity Physics provides 3 implementations of the collector interface which return the closest hit found, all hits found or exit early if any hit is found.

## Filtering

Filtering is the preferred data driven method for controlling which results queries will perform. Unity Physics default collision filter provides for a simple but flexible way to control which bodies are involved in spatial queries or collision detection.

The default collision filter is designed around a concept of collision layers and has 3 important members:

| Member           | Type | Purpose                                                                                                                                                                                             |
| ---------------- | ---- |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `BelongsTo`      | uint | A bit mask describing which layers this collider belongs to.                                                                                                                                        |
| `CollidesWith`   | uint | A bit mask describing which layers this collider can collide with.                                                                                                                                  |
| `GroupIndex`     | int  | An override for the bit mask checks. If the value in both objects is equal and positive, the objects always collide. If the value in both objects is equal and negative, the objects never collide. |

When determining if two colliders should collide or a query should be performed, Unity Physics checks the `BelongsTo` bits of one against the `CollidesWith` bits of the other. Both objects must want to collide with each other for the collision to happen.

```csharp
public static bool IsCollisionEnabled(CollisionFilter filterA, CollisionFilter filterB)
{
    if (filterA.GroupIndex > 0 && filterA.GroupIndex == filterB.GroupIndex)
    {
        return true;
    }
    if (filterA.GroupIndex < 0 && filterA.GroupIndex == filterB.GroupIndex)
    {
        return false;
    }
    return
        (filterA.BelongsTo & filterB.CollidesWith) != 0 &&
        (filterB.BelongsTo & filterA.CollidesWith) != 0;
}
```

Currently the Editor view of the **Collision Filter** just exposes the **Belongs To** and **Collides With** masks as a set of layers. Think of each layer listed in those drop downs as if an `OR` operation is performed on them, as in `BelongsTo = (1u << belongLayer1) | (1u << belongLayer2)`. Similarly for the **Collides With** for the mask.

The `groupIndex` is not currently exposed in the Editor, but do use at runtime if you have the need. For example, you can use `-1` for a few objects that you don't want colliding with each other, but also don't want to change the general settings for layers.

## Code examples

This section lists some simple examples for various queries. The examples for Ray and Collider casts require the following namespaces:
```csharp
    using Unity.Entities;
    using Unity.Mathematics;
    using Unity.Physics;
```

### Ray casts

 ```csharp
    public Entity Raycast(float3 RayFrom, float3 RayTo)
    {
        // Set up Entity Query to get PhysicsWorldSingleton
        // If doing this in SystemBase or ISystem, call GetSingleton<PhysicsWorldSingleton>()/SystemAPI.GetSingleton<PhysicsWorldSingleton>() directly.
        EntityQueryBuilder builder = new EntityQueryBuilder(Allocator.Temp).WithAll<PhysicsWorldSingleton>();

        EntityQuery singletonQuery = World.DefaultGameObjectInjectionWorld.EntityManager.CreateEntityQuery(builder);
        var collisionWorld = singletonQuery.GetSingleton<PhysicsWorldSingleton>().CollisionWorld;
        singletonQuery.Dispose();

        RaycastInput input = new RaycastInput()
        {
            Start = RayFrom,
            End = RayTo,
            Filter = new CollisionFilter()
            {
                BelongsTo = ~0u,
                CollidesWith = ~0u, // all 1s, so all layers, collide with everything
                GroupIndex = 0
            }
        };

        RaycastHit hit = new RaycastHit();
        bool haveHit = collisionWorld.CastRay(input, out hit);
        if (haveHit)
        {
            return hit.Entity;
        }
        return Entity.Null;
    }
```              

That will return the closest hit Entity along the desired ray. You can inspect the results of `RaycastHit` for more information such as hit position and normal etc.

### Collider casts

Collider casts are very similar to the ray casts, however you need to make a Collider (or borrow from an existing `PhysicsCollider` on a body). For more information on Collider creation process, see the [Interacting with Bodies](interacting-with-bodies.md) section.

```csharp
  public unsafe Entity SphereCast(float3 RayFrom, float3 RayTo, float radius)
    {
        // Set up Entity Query to get PhysicsWorldSingleton
        // If doing this in SystemBase or ISystem, call GetSingleton<PhysicsWorldSingleton>()/SystemAPI.GetSingleton<PhysicsWorldSingleton>() directly.
        EntityQueryBuilder builder = new EntityQueryBuilder(Allocator.Temp).WithAll<PhysicsWorldSingleton>();

        EntityQuery singletonQuery = World.DefaultGameObjectInjectionWorld.EntityManager.CreateEntityQuery(builder);
        var collisionWorld = singletonQuery.GetSingleton<PhysicsWorldSingleton>().CollisionWorld;
        singletonQuery.Dispose();

        var filter = new CollisionFilter()
        {
            BelongsTo = ~0u,
            CollidesWith = ~0u, // all 1s, so all layers, collide with everything
            GroupIndex = 0
        };

        SphereGeometry sphereGeometry = new SphereGeometry() { Center = float3.zero, Radius = radius };
        BlobAssetReference<Collider> sphereCollider = SphereCollider.Create(sphereGeometry, filter);

        ColliderCastInput input = new ColliderCastInput()
        {
            Collider = (Collider*)sphereCollider.GetUnsafePtr(),
            Orientation = quaternion.identity,
            Start = RayFrom,
            End = RayTo
        };

        ColliderCastHit hit = new ColliderCastHit();
        bool haveHit = collisionWorld.CastCollider(input, out hit);
        if (haveHit)
        {
            return hit.Entity;
        }

        sphereCollider.Dispose();

        return Entity.Null;
    }
```

## Cast Performance

The above code all calls into Unity Physics through normal C#. That is fine, will work, but is not necessarily the most optimal solution. To get really good performance from the casts and other queries you should do them from within a Burst compiled job. That way the code within the Unity Physics for the cast can also make use of Burst. If you are already in a Burst job, just call as normal, otherwise you will need to create a simple Job to do it for you. Since it is a threaded job, try to batch a few together too and wait some time later for results instead of straight away.

Jobs can only be created in classes that inherit from `SystemBase`. Here are the necessary namespaces for the following examples:

```csharp
    using Unity.Burst;
    using Unity.Collections;
    using Unity.Entities;
    using Unity.Jobs;
    using Unity.Mathematics;
    using Unity.Physics;
```

```csharp
    [BurstCompile]
    public struct RaycastJob : IJobParallelFor
    {
        [ReadOnly] public CollisionWorld world;
        [ReadOnly] public NativeArray<RaycastInput> inputs;
        public NativeArray<RaycastHit> results;

        public unsafe void Execute(int index)
        {
            RaycastHit hit;
            world.CastRay(inputs[index], out hit);
            results[index] = hit;
        }
    }

    public static JobHandle ScheduleBatchRayCast(CollisionWorld world,
        NativeArray<RaycastInput> inputs, NativeArray<RaycastHit> results)
    {
        JobHandle rcj = new RaycastJob
        {
            inputs = inputs,
            results = results,
            world = world

        }.Schedule(inputs.Length, 4);
        return rcj;
    }
```

If the Scene is complex it may be worth converting main thread calls to threaded calls, even just for one ray. For example, here's how to call the above job for one ray and wait:

```csharp
    public static void SingleRayCast(CollisionWorld world, RaycastInput input,
        ref RaycastHit result)
    {
        var rayCommands = new NativeArray<RaycastInput>(1, Allocator.TempJob);
        var rayResults = new NativeArray<RaycastHit>(1, Allocator.TempJob);
        rayCommands[0] = input;
        var handle = ScheduleBatchRayCast(world, rayCommands, rayResults);
        handle.Complete();
        result = rayResults[0];
        rayCommands.Dispose();
        rayResults.Dispose();
    }
```

The code will be similar for Collider casts and Overlap queries as well.

## Using Collider keys

Collider keys correspond to the internal primitives interleaved with the bounding volume data. They do not correspond 1:1 with the triangles of a `Unity.Mesh` when a mesh collider is created from one. You can get the triangle data for a given ray hit above, using `Collider->GetChild(key, out childleaf)`, but most of the information you need should already be in the hit return struct (position, normal, etc).
