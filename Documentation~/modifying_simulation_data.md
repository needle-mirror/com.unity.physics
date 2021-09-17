Modifying simulation data
=========

# What is physics data?

Unity Physics references 2 “types” of data at runtime: ECS components and physics simulation data.

ECS components are structures that implement `IComponentData` interface and their data is permanently stored in chunks. The key ECS components for physics are listed in [Physics Body Data](getting_started.md#physics-body-data) section (`PhysicsVelocity` is an example).

Physics simulation data is the runtime-only data in `PhysicsWorld`, formed from ECS components and restructured in a way that is more convenient for real-time physics simulation. All collision queries are performed on physics simulation data.

# Important systems in Unity Physics

All systems that perform physics simulation (briefly described in [Simulation basics](getting_started.md#simulation-basics)) are updated inside the `FixedStepSimulationSystemGroup`. You can read more about the system group update order and fixed time stepping in the [System update order in Entities documentation](https://docs.unity3d.com/Packages/com.unity.entities@0.18/manual/system_update_order.html) and on the [Unity forum](https://forum.unity.com/threads/fixed-timestep-features-coming-to-entities-dots-physics-packages.899687/).

The most important physics systems are: 
| System | Description |
|---|---|
| `BuildPhysicsWorld` | Schedules the jobs that create physics simulation data from ECS data for the current simulation step. `OnUpdate()` resets simulation data before spawning any jobs, so it is important to notify this system about any extra jobs that might be reading simulation data. This can be done by passing a job handle to `AddInputDependencyToComplete()`. Before resetting simulation data, `BuildPhysicsWorld` will complete all input dependencies, allowing those jobs to finish working on valid data. |
| `StepPhysicsWorld` | Schedules the jobs that execute the physics simulation pipeline and fire callbacks. Works on physics simulation data (read/write). |
| `ExportPhysicsWorld` | Schedules the jobs that copy the result of physics step from physics simulation data to ECS components. |
| `EndFramePhysicsSystem` | Marker for the end of physics frame, created for users’ convenience. Declares read/write access to physics simulation data in order to gather all dependencies on that data and notify `BuildPhysicsWorld` of them. |

All jobs from `BuildPhysicsWorld`, `StepPhysicsWorld` and `ExportPhysicsWorld` work only on physics simulation data, so physics ECS data should not be changed while they are running (after the first job spawned in `BuildPhysicsWorld` starts until the last job spawned in `ExportPhysicsWorld` ends). In order to write the simulation results to ECS components, `BuildPhysicsWorld` and `ExportPhysicsWorld` systems expect the chunk layouts for rigid bodies to be the same at both ends of the physics pipeline. Therefore, it is not safe to make structural changes between these two systems (for example, do not add/remove entities with physics components, or add/remove components from physics entities).
In short, ECS components of physics entities should only be changed before `BuildPhysicsWorld` or after `ExportPhysicsWorld`.

Physics simulation data is being actively changed from `BuildPhysicsWorld.OnUpdate()` (after input dependencies are completed) until the last job scheduled by `StepPhysicsWorld` ends. If you wish to affect the simulation  data directly, your system should update between `BuildPhysicsWorld` and `StepPhysicsWorld` or you should use callbacks (described in the [Modifying simulation behavior](simulation_modification.md)). 

You probably noticed that `BuildPhysicsWorld` is the only mentioned system that has `AddInputDependencyToComplete()` – that’s because it writes physics simulation data in `OnUpdate()` and not just in jobs. Please refer to the [Entities package documentation](https://docs.unity3d.com/Packages/com.unity.entities@latest) for details about job dependencies and system’s `OnUpdate()` method. `EndFramePhysicsSystem` is there to help with these dependencies – jobs scheduled by any system that updates before `EndFramePhysicsSystem` and uses physics simulation data will be registered with `BuildPhysicsWorld` automatically.
`StepPhysicsWorld` is also making changes in `OnUpdate()`, but only to collision event data, so the only limitation there is not to schedule jobs implementing `ICollisionEventsJob` between `BuildPhysicsWorld` and `StepPhysicsWorld`.

`Unity.Physics.Systems.PhysicsRuntimeExtensions` defines 2 `SystemBase` extension methods that should be called in system’s `OnStartRunning()` by any system that wants to just read (`RegisterPhysicsRuntimeSystemReadOnly()`) or read and write (`RegisterPhysicsRuntimeSystemReadWrite()`) physics simulation data. Calling those methods provides automatic dependency handling for all jobs spawned in the system’s `OnUpdate()`, with details explained in [How to create my own system and spawn jobs?](#how-to-create-my-own-system-and-schedule-jobs). Of course, all systems described above call one of those extension methods. 

Systems that schedule jobs implementing `ICollisionEventsJob` or `ITriggerEventsJob` should also register for physics simulation data read (or read/write, depending on what they do exactly). This is not obvious since those interfaces don’t require `PhysicsWorld`, but collision and trigger event streams are considered parts of the physics simulation data. 

# How to create my own system and schedule jobs?

The following 2 examples should illustrate the usage of 2 `PhysicsRuntimeExtensions` methods and help understand what is important to take care of when writing systems that need to read or write physics simulation data. 

Let’s say we want to set `RigidBody.CustomTags` from one system and read it from another system afterwards, but only when appropriate scripts are added to some object in the editor (`PhysicsWritingScript` and `PhysicsReadingScript`). The first system is obviously writing physics simulation data (`RigidBody` is in `PhysicsWorld`), so it will be updated between `BuildPhysicsWorld` and `StepPhysicsWorld`, while the other system is reading the data and will be updated between `StepPhysicsWorld` and `ExportPhysicsWorld` (could also be before `EndFramePhysicsSystem`). 

We will make `Monobehavior` conversion add appropriate ECS components to some Entities, to “trigger” `OnUpdate()` calls for systems. Note that scripts should be in separate files. 

```csharp
    public struct WritingPhysicsData : IComponentData {} 

    public class PhysicsWritingScript : MonoBehaviour, IConvertGameObjectToEntity 
    { 
        public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) 
        { 
            dstManager.AddComponentData(entity, new WritingPhysicsData()); 
        } 
    }


    public struct UserReadingPhysicsData : IComponentData {} 

    public class UserPhysicsReadingScript : MonoBehaviour, IConvertGameObjectToEntity 
    { 
        public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) 
        { 
            dstManager.AddComponentData(entity, new UserReadingPhysicsData()); 
        } 
    } 
```

As already mentioned, the writing system will query for `WritingPhysicsData` ECS component existence and if any Entity has it, it will loop through all rigid bodies and set `CustomTags` to 1. From scheduling perspective, 3 things are impotant:
1. Setting `UpdateAfter` and `UpdateBefore` class attributes to position system’s `OnUpdate()` between `BuildPhysicsWorld.OnUpdate()` and `StepPhysicsWorld.OnUpdate()`.
2. Calling `this.RegisterPhysicsRuntimeSystemReadWrite()` in `OnStartRunning()`, to register read/write dependency on physics simulation data. It’s good practise to call `base.OnStartRunning()` first, to ensure that any functionality from base class is preserved. 
3. Using the `Dependency` property to schedule the job that will do the work. Based on the ECS component declarations and usage in jobs from other systems (in this case it’s about the `PhysicsSystemRuntimeData` component), `Dependency` will contain the right job handles that need to finish before `WritePhysicsTagsJob` can start, so it needs to be passed into `Schedule()`. `Dependency` also needs to contain the handle of the last job scheduled in the system when `OnUpdate()` returns, so the result of `Schedule()` call is assigned back to it. 

```csharp
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))] 
    [UpdateAfter(typeof(BuildPhysicsWorld))] 
    [UpdateBefore(typeof(StepPhysicsWorld))] 
    public partial class WritePhysicsTagsSystem : SystemBase 
    { 
        protected override void OnCreate() 
        { 
            base.OnCreate(); 
            RequireForUpdate(GetEntityQuery(new EntityQueryDesc 
            { 
                All = new ComponentType[] { typeof(WritingPhysicsData) } 
            })); 
        } 

        protected override void OnStartRunning() 
        { 
            base.OnStartRunning(); 
            this.RegisterPhysicsRuntimeSystemReadWrite(); 
        } 

        protected override void OnUpdate() 
        { 
            Dependency = new WritePhysicsTagsJob 
            { 
                PhysicsWorld = World.GetOrCreateSystem<BuildPhysicsWorld>().PhysicsWorld 
            }.Schedule(Dependency); 
        } 

        struct WritePhysicsTagsJob : IJob 
        { 
            public PhysicsWorld PhysicsWorld; 

            public void Execute() 
            { 
                var bodies = PhysicsWorld.Bodies; 
                for (int i = 0; i < bodies.Length; i++) 
                { 
                    // Default tags are 0, write 1 to each of them 
                    var body = bodies[i]; 
                    body.CustomTags = 1; 
                    bodies[i] = body; 
                } 
            } 
        } 
    } 
```

The reading system is very similar to the previous one, with  `this.RegisterPhysicsRuntimeSystemReadOnly()` call in `OnStartRunning()` (read-only access to physics simulation data), different `OnUpdate()` sequence (between `StepPhysicsWorld` and `ExportPhysicsWorld`, but could have also been before `EndFramePhysicsSystem`) and `ReadOnly` attribute on `ReadPhysicsTagsJob.PhysicsWorld` property.

```csharp
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))] 
    [UpdateAfter(typeof(StepPhysicsWorld))] 
    [UpdateBefore(typeof(ExportPhysicsWorld))] 
    public partial class ReadPhysicsTagsSystem : SystemBase 
    { 
        protected override void OnCreate() 
        { 
            base.OnCreate(); 
            RequireForUpdate(GetEntityQuery(new EntityQueryDesc 
            { 
                All = new ComponentType[] { typeof(ReadingPhysicsData) } 
            })); 
        } 

        protected override void OnStartRunning() 
        { 
            base.OnStartRunning(); 
            this.RegisterPhysicsRuntimeSystemReadOnly(); 
        } 

        protected override void OnUpdate() 
        { 
            Dependency = new ReadPhysicsTagsJob 
            { 
                PhysicsWorld = World.GetOrCreateSystem<BuildPhysicsWorld>().PhysicsWorld 
            }.Schedule(Dependency); 
        } 

        struct ReadPhysicsTagsJob : IJob 
        { 
            [ReadOnly] public PhysicsWorld PhysicsWorld; 

            public void Execute() 
            { 
                var bodies = PhysicsWorld.Bodies; 
                for (int i = 0; i < bodies.Length; i++) 
                { 
                    // Default tags are 0, 1 should be written in WritingPhysicsTagsSystem to each of them 
                    var body = bodies[i]; 
                    Assertions.Assert.AreEqual(1, body.CustomTags, "CustomTags should be 1 on all bodies!"); 
                } 
            } 
        } 
    } 
```

This is a good place to illustrate one specific thing about `BuildPhysicsWorld`, mentioned in [Important systems in Unity Physics](#important-systems-in-unity-physics). If we wanted to update `ReadPhysicsTagsSystem` after `EndFramePhysicsSystem`, aside from changing `UpdateAfter` and `UpdateBefore` class attributes we would need to make sure that `BuildPhysicsWorld.OnUpdate()` doesn’t reset the `PhysicsWorld` while we’re reading it. So, we would need to save a reference to `BuildPhysicsWorld` in our `OnCreate()` and call `m_BuildPhysicsWorld.AddInputDependencyToComplete(Dependency)` at the end of our `OnUpdate()`. This illustrates the value of `EndFramePhysicsSystem`, as it automatically handles the dependencies of systems dealing with physics simulation data before it and passes them on to `BuildPhysicsWorld`.
The situation is similar with any system that performs data manipulation in `OnUpdate()` like `BuildPhysicsWorld`, since ECS automatic dependency handling does not handle that. 
