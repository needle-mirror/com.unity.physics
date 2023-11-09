# Creating bodies in code

Now that you know how to create rigid bodies in the editor and how to alter their properties in code, let's see how to create them dynamically in code.

## Starting from a Prefab

If we take the Sphere prefab that we used before we can make it spawn multiple times once we enter the runtime mode by baking it with `ConfigAuthoring` component. 

1. Create and attach the `ConfigAuthoring` script to a new empty gameObject.
2. Add the Sphere prefab to the gameObject field and set the number of times we want to spawn.
3. Create the baker with an IComponentData both named `ConfigBaker` and `Config` to pass 2 parameters, one for converting the Prefab/GameObject into an Entity and the counter which sets the amount of Spheres to be spawn.
4. Create the `SphereSpawningSystem` that uses ISystem. See the code below.
5. Enter Play mode.

```csharp
using UnityEngine;
using Unity.Entities;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Transforms;

public struct Config : IComponentData
{
    public Entity SpherePrefab;
    public int SphereCount;
}

public class ConfigAuthoring : MonoBehaviour
{
    public GameObject SpherePrefab;
    public int SphereCount;
}

public class ConfigBaker : Baker<ConfigAuthoring>
{
    public override void Bake(ConfigAuthoring authoring)
    {
        AddComponent(new Config
        {
            SpherePrefab = GetEntity(authoring.SpherePrefab),
            SphereCount = authoring.SphereCount,
        });
    }
}

public partial struct SphereSpawningSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var config = SystemAPI.GetSingleton<Config>();

        var ecbSingleton = SystemAPI.GetSingleton<BeginSimulationEntityCommandBufferSystem.Singleton>();
        var ecb = ecbSingleton.CreateCommandBuffer(state.WorldUnmanaged);

        for (int i = 0; i < config.SphereCount; i++)
        {
            var entity = ecb.Instantiate(config.SpherePrefab);
            ecb.SetComponent(entity, new LocalTransform
            {
                Position = new float3(0, i * 2, 0),
                Rotation = quaternion.identity,
                Scale = 1
            });
        }

        // This system should only run once at startup. So it disables itself after one update.
        state.Enabled = false;
    }
}
```

You have now just created your first Unity Physics runtime effect!

Another option is to use the `SpawnRandomObjectsAuthoring` script used in some of the [Unity Physics Samples](https://github.com/Unity-Technologies/EntityComponentSystemSamples/tree/master/PhysicsSamples). By baking GameObjects with this component, you can instantiate several copies of a baked prefab and set the `LocalTransform` values for them.

In theory this should be sufficient, but if you want it to be really fast you can also let Unity Physics know they all share the same Collider. The `PhysicsCollider`, especially in the case of Convex Hull or Mesh shape colliders, can be expensive to create when first seen, so if you know it is the same as another body and it can be shared, just set the `PhysicsCollider` to say so.

1. Save your Sphere GameObject (previously created) as a Prefab: drag the GameObject from the Hierarchy into the project window.
2. Add a GameObject with the `SpawnRandomObjectsAuthoring` script attached to it.
3. Set the **Count** to **1000**.
4. Set the **Range** to **(5,5,5)**.
5. Enter Play mode.

>![Spawn-Spheres](images/spawn-spheres.gif)

## Creating bodies from scratch

The above example used a pre-existing Prefab you set up in Editor, so it glosses over some of what you needed to add, but was close.

Here is code similar to `CreateBody` from the `BasicPhysicsDemos` in the [Unity Physics Samples](https://github.com/Unity-Technologies/EntityComponentSystemSamples/tree/master/PhysicsSamples). The Colliders are stored in highly optimized structures, so some of the interaction with them is currently via 'unsafe' raw pointers and you have to bear with that for now here. You can add the example method below to your script to create a Physics Body from scratch.

Here's the method for creating bodies:

```csharp
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Rendering;
using Unity.Transforms;
using Collider = Unity.Physics.Collider;
using Unity.Collections.LowLevel.Unsafe;

public partial class SphereSpawningSystem : SystemBase
{
    protected override void OnUpdate()
    {
        var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        var renderer = sphere.GetComponent<MeshRenderer>();
        var mesh = sphere.GetComponent<MeshFilter>().mesh;

        var renderMeshDescription = new RenderMeshDescription(renderer);
        var renderMeshArray = new RenderMeshArray(new[] { renderer.material }, new[] { mesh });
        var materialMeshInfo = MaterialMeshInfo.FromRenderMeshArrayIndices(0, 0);
        Object.DestroyImmediate(sphere);

        var entity = CreateDynamicSphere(EntityManager, renderMeshArray, materialMeshInfo, renderMeshDescription, 1, 0.5f, new float3(0, 5, 0), quaternion.identity);
        EntityManager.SetName(entity, "Sphere");

        Enabled = false;
    }

    private Entity CreateDynamicSphere(EntityManager entityManager, RenderMeshArray renderMeshArray, MaterialMeshInfo materialMeshInfo, RenderMeshDescription renderMeshDescription, int uniformScale,  float radius, float3 position, quaternion orientation)
    {
        SphereGeometry sphereGeometry = new SphereGeometry
        {
            Center = float3.zero,
            Radius = radius
        };

        // Sphere with default filter and material. Add to Create() call if you want non default:
        BlobAssetReference<Unity.Physics.Collider> sphereCollider = Unity.Physics.SphereCollider.Create(sphereGeometry, CollisionFilter.Default);
        return CreateBody(entityManager, renderMeshArray, materialMeshInfo, renderMeshDescription, position, orientation, uniformScale, sphereCollider, float3.zero, float3.zero, 1.0f, true);
    }

    private unsafe Entity CreateBody(EntityManager entityManager, RenderMeshArray renderMeshArray, MaterialMeshInfo materialMeshInfo, RenderMeshDescription renderMeshDescription,  float3 position,
        quaternion orientation, float uniformScale, BlobAssetReference<Collider> collider, float3 linearVelocity,
        float3 angularVelocity, float mass, bool isDynamic)
    {
        ComponentType[] componentTypes = new ComponentType[isDynamic ? 8 : 4];

        componentTypes[0] = typeof(LocalTransform);
        componentTypes[1] = typeof(LocalToWorld);
        componentTypes[2] = typeof(PhysicsCollider);
        componentTypes[3] = typeof(PhysicsWorldIndex);
        if (isDynamic)
        {
            componentTypes[4] = typeof(PhysicsVelocity);
            componentTypes[5] = typeof(PhysicsMass);
            componentTypes[6] = typeof(PhysicsDamping);
            componentTypes[7] = typeof(PhysicsGravityFactor);
        }
        Entity entity = entityManager.CreateEntity(componentTypes);

        RenderMeshUtility.AddComponents(entity, entityManager, renderMeshDescription, renderMeshArray, materialMeshInfo);

        entityManager.SetComponentData(entity, new RenderBounds
        {
            Value = new AABB
            {
                Center = new float3(0, 0, 0),
                Extents = new float3(0.5f, 0.5f, 0.5f)
            }
        });

        entityManager.SetComponentData(entity, new LocalTransform
        {
            Position = position,
            Rotation = orientation,
            Scale = uniformScale
        });

        if (uniformScale != 1.0f)
        {
            entityManager.AddComponentData(entity, new PostTransformScale
            {
                Value = new float3x3
                {
                    c0 = new float3(1, 1, 1) * uniformScale,
                    c1 = new float3(1, 1, 1) * uniformScale,
                    c2 = new float3(1, 1, 1) * uniformScale
                }
            });
        }

        entityManager.SetComponentData(entity, new PhysicsCollider
        {
            Value = collider
        });

        //By default Physics simulation happens in PhysicsWorldIndex 0.
        EntityManager.AddSharedComponentManaged(entity, new PhysicsWorldIndex
        {
            Value = 0
        });

        if (!isDynamic) return entity;

        Collider* colliderPtr = (Collider*)collider.GetUnsafePtr();
        entityManager.SetComponentData(entity, PhysicsMass.CreateDynamic(colliderPtr->MassProperties, mass));
        /*entityManager.SetComponentData(entity, new PhysicsMass
        {
            Transform = new RigidTransform
            {
                pos = new float3(0, 0, 0),
                rot = quaternion.identity
            },
            InverseMass = 1,
            InverseInertia = new float3(10, 10, 10),
            AngularExpansionFactor = 0
        });*/

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

        entityManager.SetComponentData(entity, new PhysicsGravityFactor
        {
            Value = 1
        });

        return entity;
    }
}


```
> [!NOTE]
> The bodies created will not be part of any specific sub scene despite of being entities.
