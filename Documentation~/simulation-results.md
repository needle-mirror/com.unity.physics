# Simulation results

After completion of [PhysicsSimulationGroup](physics-pipeline.md), the simulation step is complete.
The results of the simulation are:
- New positions, rotations and velocities of dynamic bodies - stored inside a `PhysicsWorld`.
- Collision and trigger events - stored internally in `Simulation` streams.

After `PhysicsSimulationGroup` finishes, the final system of the `PhysicsSystemGroup` can update - [ExportPhysicsWorld](physics-pipeline.md).
Its role is copy the updated body positions, rotations and velocities from `PhysicsWorld` ([physics simulation data](physics-data-types.md)) to ECS data.

# Events

In addition to updated positions, rotations and velocities of bodies, the results of simulation also include events.
Events are stored in streams stored inside the `Simulation` struct.
You can access them directly from `Simulation` by calling (`SystemBase|SystemAPI|EntityQuery`).GetSingleton<`SimulationSingleton`>().AsSimulation().(`CollisionEvents|TriggerEvents`) and iterate through them.
The other approach is to use specialised jobs.
To schedule these jobs, you will need [SimulationSingleton](physics-singletons.md).
Get it using (`SystemBase|SystemAPI|EntityQuery`).GetSingleton<`SimulationSingleton`>(). (See below)

>**Note:** Events are valid after the `PhysicsSimulationGroup` has finished, and up until it starts in the next frame. Using them while `PhysicsSimulationGroup` is updating will lead to undefined behaviour.

## Collision events

Collision events are raised from colliders that opted in to this behaviour.
For each collision that involves those colliders, a collision event will be raised.
To access them directly, [see above](#events).
To access them through a specialised job, implement and schedule `ICollisionEventsJob`; see example below.
See example below.

**Example:**
```csharp
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateBefore(typeof(PhysicsSimulationGroup))] // We are updating before `PhysicsSimulationGroup` - this means that we will get the events of the previous frame
public partial struct GetNumCollisionEventsSystem : ISystem
{
    [BurstCompile]
    public partial struct CountNumCollisionEvents : ICollisionEventsJob
    {
        public NativeReference<int> NumCollisionEvents;
        public void Execute(CollisionEvent collisionEvent)
        {
            NumCollisionEvents.Value++;
        }
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        NativeReference<int> numCollisionEvents = new NativeReference<int>(0, Allocator.TempJob);
        
        state.Dependency = new CountNumCollisionEvents
        {
            NumCollisionEvents = numCollisionEvents
        }.Schedule(SystemAPI.GetSingleton<SimulationSingleton>());

        // ...
    }
}

```

## Trigger events

Trigger events are raised from colliders that opted in this behaviour.
These colliders will not collide with anything, but will instead raise a trigger event once a collision should have happened.
To access them directly, [see above](#events).
To access them through a specialised job, implement and schedule `ITriggerEventsJob`.
See example below.

**Example:**
```csharp
[UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
[UpdateAfter(typeof(PhysicsSimulationGroup))] // We are updating after `PhysicsSimulationGroup` - this means that we will get the events of the current frame.
public partial struct GetNumTriggerEventsSystem : ISystem
{
    [BurstCompile]
    public partial struct CountNumTriggerEvents : ITriggerEventsJob
    {
        public NativeReference<int> NumTriggerEvents;
        public void Execute(TriggerEvent collisionEvent)
        {
            NumTriggerEvents.Value++;
        }
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        NativeReference<int> numTriggerEvents = new NativeReference<int>(0, Allocator.TempJob);
        
        state.Dependency = new CountNumTriggerEvents
        {
            NumTriggerEvents = numTriggerEvents
        }.Schedule(SystemAPI.GetSingleton<SimulationSingleton>());

        // ...
    }
}
```
