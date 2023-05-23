---
uid: simulation-modification

---

# Overriding intermediate simulation results

You can usually consider the physics simulation as a monolithic process whose inputs are components described in [core components](core-components.md) and outputs are updated `LocalTransform` and `PhysicsVelocity` components. However, internally, the Physics step is actually broken down into smaller subsections, the output of each becomes the input to the next. Unity Physics currently gives you the ability to read and modify this data, if it is necessary for your gameplay use cases.

You can do that in two ways:
- Directly modify 'PhysicsWorld'
- Modify simulation results at specific points

Regardless of the way you wish to use, to modify intermediate simulation results, your system needs to update after [PhysicsInitializeGroup](physics-pipeline.md) and before [PhysicsSimulationGroup](physics-pipeline.md) (your system needs `[UpdateAfter(typeof(PhysicsInitializeGroup))] [UpdateBefore(PhysicsSimulationGroup)]`), or it needs to update in [PhysicsSimulationGroup](physics-pipeline.md).

# Directly modifying 'PhysicsWorld'

After [PhysicsInitializeGroup](physics-pipeline.md) has finished, you are free to modify the resulting `PhysicsWorld`. Note that for modification purposes, you should retrieve the world using (`SystemBase|SystemAPI|EntityQuery`).GetSingletonRW<`PhysicsWorldSingleton`>(), instead of just (`SystemBase|SystemAPI|EntityQuery`).GetSingleton<`PhysicsWorldSingleton`>(), since the latter is the read-only version.
This is a suitable place to modify simulation data to implement additional simulation features (driving the car, custom friction...).
There are useful methods in the `PhysicsWorldExtensions` class which enable this, such as: `ApplyImpulse(), GetEffectiveMass(), GetCenterOfMass(), SetAngularVelocity()` etc.
In the code below, we will apply an impulse to the RigidBody with index 3

```csharp
// After the PhysicsInitialzeGroup has finished, PhysicsWorld will be created.
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(PhysicsInitializeGroup))]
[UpdateBefore(typeof(PhysicsSimulationGroup))]
public partial struct ApplyImpulseSystem : ISystem
{
    [BurstCompile]
    public partial struct ApplyImpulseJob : IJob
    {
        PhysicsWorld World;
        public void Execute()
        {
            // values randomly selected
            World.ApplyImpulse(3, new float3(1, 0, 0), new float3(1, 1, 1));
        }
    }
    
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {        
        PhysicsWorldSingleton worldSingleton = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>(); 
        // Note that it is neccessary to call GetSingletonRW (not just GetSingleton), since you are writing to the world.
        state.Dependency = new ApplyImpulseJob
        {
            World = worldSingleton.PhysicsWorld
        }.Schedule(state.Dependency);

        // If you want to modify world immediately, complete dependency
        state.CompleteDependency();
        worldSingleton.PhysicsWorld.ApplyImpulse(3, new float3(1, 0, 0), new float3(1, 1, 1));
    }
}
```

# Modifying simulation results

[PhysicsSimulationGroup](physics-pipeline.md) consists of four subgroups (`PhysicsCreateBodyPairsGroup`, `PhysicsCreateContactsGroup`, `PhysicsCreateJacobiansGroup`, `PhysicsSolveAndIntegrateGroup`).
After a subgroup is finished (and before the next one starts), it is possible to modify its output. In previous versions, this was done using simulation callbacks, which are now removed.
To modify simulation results, you need to:
- Create a system
- Set its update attribute to be `[UpdateInGroup(typeof(PhysicsSimulationGroup))]`, and between two consecutive subgroups (for example, `[UpdateAfter(typeof(PhysicsSimulationGroup))` `[UpdateBefore(typeof(PhysicsCreateContactsGroup))]`)
- Implement one of the specialised jobs (see below for examples)
- Schedule the specialised job

Specialised jobs require [SimulationSingleton](physics-singletons.md) and `PhysicsWorld` for scheduling. Retrieve those using `(SystemBase|SystemAPI|EntityQuery)`.GetSingletonRW<>().

### Modification point 1 – After _PhysicsCreateBodyPairsGroup_ and _PhysicsCreateContactsGroup_

**Status** – At this point, the simulation has generated the list of _potentially_ interacting objects, based on joints and the overlaps at the _broadphase_ level. It has also performed some sorting, making the pairs of interacting objects suitable for multithreading.

**What you can do** – You can schedule an implementation of `IBodyPairsJob` to get access to all of these body pairs. If for some reason you wish to disable an interaction, you can do it during this job.

>**Note:** from both a performance and debugging perspective, it is preferable to disable pairs using a collision filter.

**Example:**
```csharp
[UpdateInGroup(typeof(PhysicsSimulationGroup))]
[UpdateAfter(typeof(PhysicsCreateBodyPairsGroup))]
[UpdateBefore(typeof(PhysicsCreateContactsGroup))]
public partial struct DisableDynamicDynamicPairsSystem : ISystem
{
    [BurstCompile]
    struct DisableDynamicDynamicPairsJob : IBodyPairsJob
    {
        public int NumDynamicBodies;

        public unsafe void Execute(ref ModifiableBodyPair pair)
        {
            // Disable the pair if it's dynamic-dynamic
            bool isDynamicDynamic = pair.BodyIndexA < NumDynamicBodies && pair.BodyIndexB < NumDynamicBodies;
            if (isDynamicDynamic)
            {
                pair.Disable();
            }
        }
    }
    
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {        
        PhysicsWorldSingleton worldSingleton = SystemAPI.GetSingletonRW<PhysicsWorldSingleton>();
        SimulationSingleton simulationSingleton = SystemAPI.GetSingletonRW<SimulationSingleton>();

        state.Dependency = new DisableDynamicDynamicPairsJob
        {
            NumDynamicBodies = worldSingleton.PhysicsWorld.NumDynamicBodies
        }.Schedule(simulationSingleton, ref worldSingleton.PhysicsWorld,
        state.Dependency);
    }
}
```

### Modification point 2 – After _PhysicsCreateContactsGroup_ and before _PhysicsCreateJacobiansGroup_

**Status** – At this point, the simulation has performed the low-level collision information between every pair of bodies by inspecting their colliders.

**What you can do** – At this point you can schedule an implementation of `IContactsJob` to access that information. The data is in the format of a `ModifiableContactHeader` containing some shared properties, followed by a number (`ModifiableContactHeader.NumContacts`) of `ModifiableContactPoint` contact points. You can modify or disable the contacts, for example you can change the separating normal or the contact positions and distances.

**Example:**
```csharp
[UpdateInGroup(typeof(PhysicsSimulationGroup))]
[UpdateAfter(typeof(PhysicsCreateContactsGroup))]
[UpdateBefore(typeof(PhysicsCreateJacobiansGroup))]
[BurstCompile]
public partial struct EnableSurfaceVelocitySystem : ISystem
{
    // This sets some data on the contact, to get propagated to the jacobian
    // for processing in our jacobian modifier job. This is necessary because some flags require extra data to
    // be allocated along with the jacobian (e.g., SurfaceVelocity data typically does not exist).
    [BurstCompile]
    public partial struct EnableSurfaceVelocityJob : IContactsJob
    {
        public void Execute(ref ModifiableContactHeader manifold, ref ModifiableContactPoint contact)
        {
            manifold.JacobianFlags |= JacobianFlags.EnableSurfaceVelocity;
        }
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        state.Dependency = new EnableSurfaceVelocityJob()
            .Schedule(SystemAPI.GetSingletonRW<SimulationSingleton>(),
            ref SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().PhysicsWorld, 
            state.Dependency);
    }
}
```

### Modification point 3 – After _PhysicsCreateJacobiansGroup_ and before _PhysicsSolveAndIntegrateGroup_

**Status** – Now, the joint data from your Joint components and contact data from the step has been converted to data which is suitable for consumption by the constraint solver. It represents the same as it did before, but has been transformed into a form which optimizes the simulation's ability to solve the constraints accurately – so, the data appears to be significantly more abstract and harder to manipulate.

**What you can do** – You can schedule an implementation of `IJacobiansJob` to get access to each of the jacobian datas. Since the data is harder to manage, Unity Physics provides some accessors to perform many reasonable modifications. For these, it is best to consult the sample scene.

**Example:**
```csharp
[BurstCompile]
[UpdateInGroup(typeof(PhysicsSimulationGroup))]
[UpdateAfter(typeof(PhysicsCreateJacobiansGroup))]
[UpdateBefore(typeof(PhysicsSolveAndIntegrateGroup))]
public partial struct SetFrictionToZeroSystem : ISystem
{   
    [BurstCompile]
    public partial struct SetFrictionToZeroJob : IJacobiansJob 
    // Note: there are much more performant ways of setting friction to zero, this is just for demonstration purposes
    {
        // Don't do anything for triggers
        public void Execute(ref ModifiableJacobianHeader h, ref ModifiableTriggerJacobian j) {}
        
        [BurstCompile]
        public void Execute(ref ModifiableJacobianHeader jacHeader, ref ModifiableContactJacobian contactJacobian)
        {
            var friction0 = contactJacobian.Friction0;
            friction0.AngularA = 0.0f;
            friction0.AngularB = 0.0f;
            contactJacobian.Friction0 = friction0;

            var friction1 = contactJacobian.Friction1;
            friction1.AngularA = 0.0f;
            friction1.AngularB = 0.0f;
            contactJacobian.Friction1 = friction1;

            var angularFriction = contactJacobian.AngularFriction;
            angularFriction.AngularA = 0.0f;
            angularFriction.AngularB = 0.0f;
            contactJacobian.AngularFriction = angularFriction;
        }
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        state.Dependency = new SetFrictionToZeroJob()
            .Schedule(SystemAPI.GetSingletonRW<SimulationSingleton>(), 
            ref SystemAPI.GetSingletonRW<PhysicsWorldSingleton>().PhysicsWorld,
            state.Dependency);
    }    
}
```

## Fine grained control of modification logics applied

Typically, not all intermediate data is modified the same way. Examples above show how single modification logic is applied with very simple filtering, like disabling only dynamic vs dynamic body collisions. What is typically needed is to have much better control over which subset of intermediate simulation data to modify. On top of that very often is needed to apply different modifications on different subsets of intermediate simulation data.

To achieve both of those check [Custom Physics Body Tags](tag-physics-body.md) section.
