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
public partial struct CastRayISystemExample : ISystem
{
    [BurstCompile]
    public partial struct CastRayJob : IJob
    {
        public PhysicsWorldSingleton World;
        
        public void Execute()
        {
            World.CastRay(...);
        }
    }
    
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
See [Simulation modification](simulation-modification.md#modifying-simulation-results) and [Events](simulation-results.md#events).
