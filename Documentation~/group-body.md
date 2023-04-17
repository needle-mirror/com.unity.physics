# Multiple worlds - simulating groups of bodies separately

It is possible to simulate groups of bodies totally separately from each other, in separate physics worlds. Each body in each world is represented by a separate Entity and each Entity must have all components that are needed for physics simulation in its world. `PhysicsWorldIndex.Value` denotes the index of physics world that the Entity belongs to (0 is the default value and represents the main world). Note that Entities for different physics worlds will be stored in separate ECS chunks, due to different values of the shared component.
To enable custom simulation of bodies in non-default worlds, you need to:
- Assign the respective bodies to different worlds
- Create a system group which derives from `CustomPhysicsSystemGroup`
- Make an empty constructor that is calling the constructor from `CustomPhysicsSystemGroup`, providing it with world index value and a flag indicating whether to share static bodies between worlds.
- Optionally override `Pre/PostGroupUpdateCallback()` or `AddExistingSystemsToUpdate` (covered below).

## Assigning bodies to different worlds
By default, bodies are assigned to the main physics world (world index 0).
In order to assign a body to a different world, add a `PhysicsWorldIndexAuthoring` component to the GameObject you want to modify alongside its `Rigidbody` component and then set its `World Index` property to the desired value. 

## CustomPhysicsSystemGroup

When overriding `CustomPhysicsSysemGroup` with your own group, you can freely specify the update order of that group via `[UpdateBefore/After/InGroup]` attributes.
In the background, that group will enable all physics systems to be run in multiple groups - the main one which is processing world index 0 entities, and a custom one which is processing the specified world index entities. In the background, it will create `PhysicsSystemGroup` as part of your new group. In addition to enabling only base systems to be ran in that group, all user systems defined in `PhysicsInitializeGroup`, `PhysicsSimulationGroup`, `BeforePhysicsSystemGroup` and `AfterPhysicsSystemGroup` will be running in the new group as well.
`PhysicsWorldSingleton` and `SimulationSingleton` retrieval will return the singletons of the currently ran group.
Example:

```csharp
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    public partial class UserPhysicsGroup : CustomPhysicsSystemGroup
    {
        public UserPhysicsGroup() : base(1, true) {} // our physics group will simulate entities with world index 1, and share static entities with main world        
    }

    [UpdateInGroup(PhysicsInitializeGroup)] // As the system updates inside [PhysicsSinitializeGroup], it will automatically be ran inside UserPhysicsGroup as well
    public partial class CheckWorldIndexSystem : SystemBase
    {
        protected override void OnUpdate()
        {
            var physicsWorldSingleton = GetSingleton<PhysicsWorldSingleton>();
            if (physicsWorldSingleton.PhysicsWorldIndex.Value == 0)
            {
                // ...
                // When ran as part of main physics group, this code path will be hit.
            }
            else
            {
                // ...
                // When ran as part of UserPhysicsGroup, this code path will be hit.
            }
        }
    }
```

### `AddExistingSystemsToUpdate()`
- If your system is not in one one of the mentioned groups above that automatically get ran in multiple groups, you will need to add it on your own by overriding `AddExistingSystemsToUpdate()`.

### `BeforePhysicsSystemGroup` and `AfterPhysicsSystemGroup`
- If you have systems that have [UpdateBefore(typeof(`PhysicsSystemGroup`))] or [UpdateAfter(typeof(`PhysicsSystemGroup`))] and need them in the group that operates on non-default index entities, calling `AddExistingSystemsToUpdate()` will not work in those cases.
- In those cases, you should replace those attributes with these ones: [UpdateInGroup(typeof(`BeforePhysicsSystemGroup`))] and [UpdateInGroup(typeof(`AfterPhysicsSystemGroup`))].
- The systems should be ran properly in multiple groups when that is done.
- These groups provide no behaviour difference to using [UpdateBefore(typeof(`PhysicsSystemGroup`))] or [UpdateAfter(typeof(`PhysicsSystemGroup`))], and are not necessary if not using multiple worlds feature.

### `PreGroupUpdateCallback()` and `PostGroupUpdateCallback()`

These callbacks are called by `CustomPhysicsGroup` before/after the nested physics systems update. Sometimes, it is not enough for the systems just to be ran in two different groups in order to work properly. This mainly refers to state, for example, if your system has a `NativeArray`, it might need to be saved and switched by another array in a different group before it is ran. `PreGroupUpdateCallback()` is the place to switch state. Similarly, when the custom group finishes updating, you need to restore the main group state (in this example, an array). `PostGroupUpdateCallback()` is the place to do that.
Internally, physics operates this way - in `PreGroupUpdateCallback()` - `PhysicsWorldData` is saved and replaced by another storage, and a proper `PhysicsWorldSingleton` is set up. Same applies for `Simulation` and `SimulationSingleton`. Finally, in `PostGroupUpdateCallback()` the main world data and singletons are restored.

If overriding these methods, it is necessary to call `base.Pre/PostGroupUpdateCallback()`, so the physics properly switches state.

### Limitations

The main physics world and all other physics worlds use the same `PhysicsStep` singleton. 
 
# Next steps

Now that you know how to create bodies, it would be a good time to learn about [Collision queries](collision-queries.md) and filtering. At that stage you should be all set for at least intermediate use of Unity Physics and can progress onto the more advanced topics such as understanding the [Physics pipeline](physics-pipeline.md#physics-pipeline) and modifying simulation as it is running ([directly](simulation-modification.md#directly-modifying-physicsworld) or via [slotting in systems between physics pipeline stages](simulation-modification.md)) and getting a deeper understanding of how the code works.
