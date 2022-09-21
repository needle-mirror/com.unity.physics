---
uid: interacting-with-physics
---

# Physics Pipeline

All physics jobs (initializing simulation data, stepping, and exporting data to ECS) are executed inside `PhysicsSystemGroup`. It is a `ComponentSystemGroup`, and is a subgroup of `FixedStepSimulationSystemGroup`. Therefore, multiple physics steps can occur during one frame. 

`PhysicsSystemGroup` consists of two other subgroups, each responsible for a single phase in physics step, and `ExportPhysicsWorld` system, which schedules jobs that convert physics simulation data to ECS data (`Translation`, `Rotation` and `PhysicsVelocity`).

Subgroups are:
| System group | Description |
|---|---|
| `PhysicsInitializeGroup` |  Schedules the jobs that create physics simulation data from ECS data for the current simulation step. |
| `PhysicsSimulationGroup` |  Schedules the jobs that execute physics simulation pipeline. |

`PhysicsSimulationGroup` is further divided into four other subgroups that are responsible for different stages of simulation, and between which it is possible to modify physics simulation data.
Those are:
| System group | Description |
|---|---|
| `PhysicsCreateBodyPairsGroup` |  Schedules the jobs that find pairs of bodies whose AABB's overlap. |
| `PhysicsCreateContactsGroup` |  Schedules the jobs that create contacts based on overlapping pairs. |
| `PhysicsCreateJacobiansGroup` |  Schedules the jobs that create jacobians based on created contacts.. |
| `PhysicsSolveAndIntegrateGroup` |  Schedules the jobs that solve jacobians. |

The running order is illustrated in the image below:

![PhysicsPipeline](images/PhysicsPipeline.png)

-----

# Physics data types

Unity Physics references 2 _types_ of data at runtime: ECS components and physics simulation data.

ECS components are structures that implement the `IComponentData` interface and their data is permanently stored in chunks. The key ECS components for physics are listed in [Physics Body Data](getting_started.md#physics-body-data) section (e.g. `PhysicsVelocity`).

Physics simulation data is the runtime-only data stored in `PhysicsWorld`, formed from ECS components and restructured in a way that is more convenient for real-time physics simulation (this is done by `PhysicsInitializeGroup`). All collision queries are performed on physics simulation data.

For reading physics data (for queries or some other case) you need to make sure to set up proper update order attributes on your system. `[UpdateBefore|After|InGroup(typeof(PhysicsSystemGroup))]` are fine for reading purposes. If `[UpdateBefore|After(typeof(PhysicsSystemGroup))]` is selected, it is also needed to add a `[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]`. If `[UpdateInGroup(typeof(PhysicsSystemGroup))]` is selected, then it is possible provide more granularity by selecting update order between the two subgroups and a physics system in that group (`PhysicsInitializeGroup`, `PhysicsSimulationGroup` and `ExportPhysicsWorld`). It is also possible to select `[UpdateIn(typeof(PhysicsSimulationGroup))]`, and if done so, even more granularity can be provided by selecting update order between four subgroups (`PhysicsCreateBodyPairsGroup`, `PhysicsCreateContactsGroup`, `PhysicsCreateJacobiansGroup`, `PhysicsSolveAndIntegrateGroup`). Usage of these subgroups, as well as modifying physics data is covered in [Simulation modification](simulation_modification.md#overriding-intermediate-simulation-results)).

All jobs from `PhysicsSystemGroup` work only on physics simulation data, so physics ECS data should not be changed while they are running. In order to write the simulation results to ECS components, `PhysicsInitializeGroup` and `ExportPhysicsWorld` systems expect the chunk layouts for rigid bodies to be the same at both ends of the physics pipeline. Therefore, it is not safe to make structural changes between these two systems (for example, do not add/remove entities with physics components, or add/remove components from physics entities). Modifying physics ECS data while in physics pipeline will have no effect, as the modifications will be overwritten by `ExportPhysicsWorld`. An integrity error will be thrown if modifying or adding/removing ECS data on physics entities during physics pipeline is tried.

In short, ECS components of physics entities should only be changed before or after `PhysicsSystemGroup`.

# Physics Singletons

## PhysicsWorldSingleton

`PhysicsWorldSingleton` stores a `PhysicsWorld` created by the physics pipeline. Getting this singleton is the supported way of retrieving physics simulation data - the `PhysicsWorld`. Call (`SystemBase|SystemAPI|EntityQuery`).GetSingleton<`PhysicsWorldSingleton`>() to retrieve it if read-only access to physics simulation data is sufficient, or (`SystemBase|SystemAPI|EntityQuery`).GetSingletonRW<`PhysicsWorldSingleton`>() if read-write access is required.
In addition to returning a `PhysicsWorldSingleton`, these calls will also ensure that the jobs using the retrieved singleton will not run into a race condition. In older versions, to ensure this, a call to `AddInputDependency()`, `AddInputDependencyToComplete()` or `RegisterPhysicsSystemsReadOnly()|ReadWrite()` was required. There is now no need to do any of this, and these calls were removed.
Retrieving the `PhysicsWorld` is also possible by getting the `BuildPhysicsWorld` system from the world, and then accessing `PhysicsWorldData.PhysicsWorld`, but this is **STRONGLY** discouraged, as there are no thread-safety guarantees in that case.

### Usage example

```csharp

// ISystem version
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(PhysicsSystemGroup))] // Make sure that the running order of systems is correct
[BurstCompile]
public partial struct CastRayISystemExample : ISystem
{
    public partial struct CastRayJob : IJob
    {
        public PhysicsWorldSingleton World;
        
        [BurstCompile]
        public void Execute()
        {
            World.CastRay(...);
        }
    }

    [BurstCompile]
    public void OnCreate(ref SystemState state){}

    [BurstCompile]
    public void OnDestroy(ref SystemState state){}
    
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        PhysicsWorldSingleton physicsWorldSingleton = SystemAPI.GetSingleton<PhysicsWorldSingleton>();

        // Version that schedules a job
        state.Dependency = new CastRayJob
        {
            World = physicsWorldSingleton
        }.Schedule(state.Dependency);

        // Main thread version - be sure to complete the dependency
        state.CompleteDependency();
        physicsWorldSingleton.CastRay(...);

        // It is also possible to access the PhysicsWorld
        PhysicsWorld world = physicsWorldSingleton.PhysicsWorld;
    }
}

// SystemBase version
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(PhysicsSystemGroup))] // Make sure that the running order of systems is correct
public partial class CastRaySystemBaseExample : SystemBase
{
    public partial struct CastRayJob : IJob
    {
        public PhysicsWorldSingleton World;
        
        [BurstCompile]
        public void Execute()
        {
            World.CastRay(...);
        }
    }

    protected override void OnUpdate()
    {
        PhysicsWorldSingleton physicsWorldSingleton = GetSingleton<PhysicsWorldSingleton>();

        // Version that schedules a job
        state.Dependency = new CastRayJob
        {
            World = physicsWorldSingleton
        }.Schedule(state.Dependency);

        // Main thread version - be sure to complete the dependency
        CompleteDependency();
        physicsWorldSingleton.CastRay(...);

        // It is also possible to access the PhysicsWorld
        PhysicsWorld world = physicsWorldSingleton.PhysicsWorld;
    }
}
```

## SimulationSingleton

`SimulationSingleton` holds a pointer to a simulation that is currently running. Similar to `PhysicsWorldSingleton`, call (`SystemBase|SystemAPI|EntityQuery`).GetSingleton<`SimulationSingleton`>() to retrieve it in case of read-only access to simulation, or (`SystemBase|SystemAPI|EntityQuery`).GetSingletonRW<`SimulationSingleton`>() in case of read-write access to simulation.
This singleton is mostly used while scheduling jobs between simulation subgroups, as well as for scheduling trigger and collision event jobs.
See [Simulation modification](simulation_modification.md#modifying-simulation-results) and [Events](simulation_results.md#events).


[Back to Index](index.md)
