Interacting with Bodies
=========

Interacting with Physics Bodies can be done using scripts. For example to alter velocity of a body, you can query for its `PhysicsVelocity` and just set new values as you want. Let's make an example that just attracts all bodies to a single point.
>**Pre-requisite:** The examples provided in this section assume that you have previously created a scene as per the instructions provided in the [Getting Started](getting_started.md) section.

# Attracting bodies to a single point

First you need to make a `ComponentSystem` that is going to iterate over all bodies with `PhysicsVelocity` on update. Then you need to check the distance to the `Translation` of the body and see if it is close enough to be affected.
>**Note:** it's not necessary to iterate over all the bodies to get a subset to be affected, but for simplicity it is shown like this for now.

```csharp
using Unity.Entities;
using Unity.Transforms;
using Unity.Physics;
using Unity.Mathematics;
using UnityEngine;

public class AttractSystem : ComponentSystem
{
    public float3 center;
    public float maxDistanceSqrd;
    public float strength;

    protected override unsafe void OnUpdate()
    {
        Entities.ForEach(
            ( ref PhysicsVelocity velocity,
              ref Translation position,
              ref Rotation rotation) =>
            {
                float3 diff = center - position.Value;
                float distSqrd = math.lengthsq(diff);
                if (distSqrd < maxDistanceSqrd)
                {
                    // Alter linear velocity
                    velocity.Linear += strength * (diff / math.sqrt(distSqrd));
                }
            });
    }
};
```

You can alter the velocity as you want. If you want the bodies to go directly to the `center` point above, you can just set `velocity.Linear` to be exactly the difference in position. That would get the body there in 1 second, assuming it does not hit anything or is not affected by gravity while en route. If you want to get there in one step, you also need to know the physics delta time step using `Time.fixedDeltaTime`, and divide the linear velocity by that.

Note that the above example is not setting the position directly, but is just altering velocity to get the body to where you want it to be. That way it can still interact with all other objects in the scene correctly, rather than just teleporting to a given position and hoping for the best.

## Creating an attractor

To add an attractor to your scene, the simplest (if not pure ECS way) is to just add a MonoBehaviour that alters the system's values.

1. Create a new empty GameObject and position it near the Sphere (for example just above it).
2. Attach the following script to it:

    ```csharp
    using Unity.Entities;
    using UnityEngine;
    
    public class AttractComponent : MonoBehaviour
    {
        public float maxDistance = 3;
        public float strength = 1;
    
        void Update()
        {
            var vortex = World.Active.GetOrCreateSystem<AttractSystem>();
            vortex.center = transform.position;
            vortex.maxDistanceSqrd = maxDistance * maxDistance;
            vortex.strength = strength;
        }
    }
    ```

3. Enter Play mode and see what happens.

The Sphere should orbit the point in space where you placed the new GameObject. If it doesn't, try putting the GameObject closer to the sphere or increase the **Max Distance** and **Strength** values.

>**Tip:** If you'd like to modify the position of the attractor in the editor, you can migrate the Cube, Sphere and a GameObject containing the **Physics Step** component into a subscene and remove the **Convert To Entity** components from all of them – and then make sure Live Link is enabled.
>![Attractor](images/attractor.gif)

### How it should actually be done with DOTS

The correct DOTS oriented way would be to use the `IConvertGameObjectToEntity` interface on the `AttractComponent` MonoBehaviour we made. This can be done by implementing `IConvertGameObjectToEntity.Convert` which would create an `AttractData` component of the `ComponentData` type that is attached to the Entity representing the point of attraction. Then modify the `AttractSystem` to iterate over all the `AttractData` components instead.

The most efficient way to get all the bodies close to that point is to either use the `Unity.Physics.CollisionWorld.OverlapAabb` method or, for more accuracy, `CalculateDistance` (with its `MaxDistance` set to the `maxDistance` defined in the above example). For more information, see the [Collision queries](collision_queries.md) section.

## Impulses

Now you have seen how to alter velocity in code, but it can be tricky to work out what velocity values to set in order to get a desired outcome. A common thing you want to do is applying an impulse at a given point on the body and having it react – for example, shooting the object with a gun.

Unity Physics provides a few `Unity.Physics.Extensions.ComponentExtensions` methods to do the math for you, for example `ApplyImpulse()`. Here's its current implementation:

```csharp
    public static void ApplyImpulse(ref PhysicsVelocity pv, PhysicsMass pm,
        Translation t, Rotation r, float3 impulse, float3 point)
        {
            // Linear
            pv.Linear += impulse;

            // Angular
            {
                // Calculate point impulse
                var worldFromEntity = new RigidTransform(r.Value, t.Value);
                var worldFromMotion = math.mul(worldFromEntity, pm.Transform);
                float3 angularImpulseWorldSpace = math.cross(point - worldFromMotion.pos, impulse);
                float3 angularImpulseInertiaSpace = math.rotate(math.inverse(worldFromMotion.rot), angularImpulseWorldSpace);

                pv.Angular += angularImpulseInertiaSpace * pm.InverseInertia;
            }
        }
```
Favour the form of calls that take the raw Component Data (such as `PhysicsVelocity`) rather than ones that query for them. Do this to encourage the code to be more efficient and work over arrays of the data in the efficient DOTS style.
> **Note:** More methods will be provided over time, but they will all work off the same `PhysicsVelocity` and `PhysicsMass` components you have today, so feel free to implement your own as you need.

# Creating bodies in code

Now that you know how to create bodies in the editor and how to alter their properties in code, let's see how to create them dynamically in code.

## Starting from a Prefab

If you have a Prefab setup with the body, then the `SpawnRandomPhysicsBodies` script used in some of the [Unity Physics Samples](https://github.com/Unity-Technologies/EntityComponentSystemSamples/blob/master/UnityPhysicsSamples/Documentation/samples.md) is a good place to start. Once you use `ConvertGameObjectHierarchy` on the prefab, you get an Entity that you can Instantiate as many times as you like, setting the `Translation` and `Rotation` on it.

In theory this should be sufficient, but if you want it to be really fast you can also let Unity Physics know they all share the same Collider. The `PhysicsCollider`, especially in the case of Convex Hull or Mesh shape colliders, can be expensive to create when first seen, so if you know it is the same as another body and it can be shared, just set the `PhysicsCollider` to say so.

1. Save your Sphere GameObject (previously created) as a Prefab: drag the GameObject from the Hierarchy into the project window.
2. Add a GameObject with the following `SpawnRandomPhysicsBodies` script attached to it:

    ```csharp
    using System;
    using Unity.Physics;
    using Unity.Entities;
    using Unity.Mathematics;
    using Unity.Collections;
    using UnityEngine;
    using Unity.Transforms;
    using Collider = Unity.Physics.Collider;
    ```
	
    ```csharp
    [Serializable]
    public class SpawnRandomPhysicsBodies : MonoBehaviour
    {
        public GameObject prefab;
        public float3 range;
        public int count;
    
        void OnEnable()
        {
            if (this.enabled)
            {
                // Create entity prefab from the game object hierarchy once
                Entity sourceEntity = GameObjectConversionUtility.ConvertGameObjectHierarchy(prefab, BasePhysicsDemo.DefaultWorld);
                var entityManager = BasePhysicsDemo.DefaultWorld.EntityManager;
    
                var positions = new NativeArray<float3>(count, Allocator.Temp);
                var rotations = new NativeArray<quaternion>(count, Allocator.Temp);
                RandomPointsOnCircle(transform.position, range, ref positions, ref rotations);
    
                BlobAssetReference<Collider> sourceCollider = entityManager.GetComponentData<PhysicsCollider>(sourceEntity).Value;
                for (int i = 0; i < count; i++)
                {
                    var instance = entityManager.Instantiate(sourceEntity);
                    entityManager.SetComponentData(instance, new Translation { Value = positions[i] });
                    entityManager.SetComponentData(instance, new Rotation { Value = rotations[i] });
                    entityManager.SetComponentData(instance, new PhysicsCollider { Value = sourceCollider });
                }
    
                positions.Dispose();
                rotations.Dispose();
            }
        }
    
        public static void RandomPointsOnCircle(float3 center, float3 range, ref NativeArray<float3> positions, ref NativeArray<quaternion> rotations)
        {
            var count = positions.Length;
            // Initialize the seed of the random number generator
            Unity.Mathematics.Random random = new Unity.Mathematics.Random();
            random.InitState(10);
            for (int i = 0; i < count; i++)
            {
                positions[i] = center + random.NextFloat3(-range, range);
                rotations[i] = random.NextQuaternionRotation();
            }
        }
    }
    ```

3. Set the **Count** to **1000**.
4. Set the **Range** to **(2,2,2)**.
5. Enter Play mode.

You have now just created your first Unity Physics runtime effect!

## Creating bodies from scratch

The above example used a pre-existing Prefab you set up in Editor, so it glosses over some of what you needed to add, but was close.

Here is code similar to `CreateBody` from the `BasicPhysicsDemos` in the [Unity Physics Samples](https://github.com/Unity-Technologies/EntityComponentSystemSamples/blob/master/UnityPhysicsSamples/Documentation/samples.md). The Colliders are stored in highly optimized structures, so some of the interaction with them is currently via 'unsafe' raw pointers and you have to bear with that for now here. You can add the example method below to your script to create a Physics Body from scratch.

First, you need these namespaces for your script:

```csharp
using Unity.Entities;
using Unity.Transforms;
using Unity.Physics;
using Unity.Mathematics;
using Unity.Rendering;
using Collider = Unity.Physics.Collider;
```

Here's the method for creating bodies:

```csharp
public unsafe Entity CreateBody(RenderMesh displayMesh, float3 position, quaternion orientation,
        BlobAssetReference<Collider> collider,
        float3 linearVelocity, float3 angularVelocity, float mass, bool isDynamic)
{
    EntityManager entityManager = EntityManager;
    ComponentType[] componentTypes = new ComponentType[isDynamic ? 7 : 4];

    componentTypes[0] = typeof(RenderMesh);
    componentTypes[1] = typeof(Translation);
    componentTypes[2] = typeof(Rotation);
    componentTypes[3] = typeof(PhysicsCollider);
    if (isDynamic)
    {
        componentTypes[4] = typeof(PhysicsVelocity);
        componentTypes[5] = typeof(PhysicsMass);
        componentTypes[6] = typeof(PhysicsDamping);
    }
    Entity entity = entityManager.CreateEntity(componentTypes);

    entityManager.SetSharedComponentData(entity, displayMesh);


    // Check if the Entity has a Translation and Rotation components and execute the appropriate method
    if (entityManager.HasComponent(entity, typeof(Translation)))
       {
           entityManager.SetComponentData(entity, new Translation { Value = position });=
       }
       else
       {
           entityManager.AddComponentData(entity, new Translation { Value = position });
       }
    if (entityManager.HasComponent(entity, typeof(Rotation)))
       {
           entityManager.SetComponentData(entity, new Rotation { Value = orientation });
       }
       else
       {
           entityManager.AddComponentData(entity, new Rotation { Value = orientation });
       }
    entityManager.SetComponentData(entity, new PhysicsCollider { Value = collider });

    if (isDynamic)
    {
        Collider* colliderPtr = (Collider*)collider.GetUnsafePtr();
        entityManager.SetComponentData(entity, PhysicsMass.CreateDynamic(colliderPtr->MassProperties, mass));
        // Calculate the angular velocity in local space from rotation and world angular velocity
        float3 angularVelocityLocal = math.mul(math.inverse(colliderPtr->MassProperties.MassDistribution.Transform.rot), angularVelocity);
        entityManager.SetComponentData(entity, new PhysicsVelocity()
        {
            Linear = linearVelocity,
            Angular = angularVelocityLocal
        });
        entityManager.SetComponentData(entity, new PhysicsDamping()
        {
            Linear = 0.01f,
            Angular = 0.05f
        });
    }

    return entity;
}
```

Getting your `RenderMesh` (etc.) is up to you, and you can create a Collider through the `Create()` function on each of the different Collider types, like this:

```csharp
  public Entity CreateDynamicSphere(RenderMesh displayMesh, float radius, float3 position, quaternion orientation)
    {
        // Sphere with default filter and material. Add to Create() call if you want non default:
        BlobAssetReference<Unity.Physics.Collider> spCollider = Unity.Physics.SphereCollider.Create(float3.zero, radius);
        return CreateBody(displayMesh, position, orientation, spCollider, float3.zero, float3.zero, 1.0f, true);
    };
```

# Next steps

Now that you know how to create bodies, it would be a good time to learn about [Collision queries](collision_queries.md) and filtering. At that stage you should be all set for at least intermediate used of Unity Physics and can progress onto the more advanced topics such as modifying the simulation as it is running via callbacks and getting a deeper understanding of how the code works.
