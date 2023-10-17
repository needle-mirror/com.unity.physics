# What's new in Unity Physics 1.0

This is a summary of the changes in Unity Physics version 1.0.

For a full list of changes, see the [Changelog](xref:unity-physics-changelog). For information on how to upgrade to version 1.0, see the [Upgrade guide](upgrade-guide.md).

## Added

* **Motors**: For all 4 motor types, added a new field for the maximum impulse that the motor constraint can exert.
* **Licence warning pop-up**: adding the `com.havok.physics` package in your project without the Unity Pro, Enterprise or Unity Industrial Collection license will prevent you from simulating Havok Physics in the editor and builds.
* `[CreateBefore(typeof(BroadphaseSystem))]` for `PhysicsSimulationPickerSystem` within `UnityPhysicsSimulationSystems.cs` script file.
* Reference to com.unity.render-pipelines.universal version 10.7
* New shaders in the sampler that are SRP batcher and universal render pipeline compliant.
* New struct - `Unity.Physics.Math.ScaledMTransform`: Provides the same utility as `Unity.Physics.Math.MTransform` but supports uniform scale.
* Operator which converts a `float4` into a `Unity.Physics.Plane`.
* `PhysicsComponentExtensions.ApplyScale(in this PhysicsMass pm, in Scale scale)` - an extension method which scales up the `PhysicsMass` component.
* Multiple extension methods have received a version which takes a `Scale` argument, you can see the full list at the [Changelog](xref:unity-physics-changelog).
* `bool OverlapAabb(OverlapAabbInput input, ref NativeList<int> allHits)` has been added to PhysicsWorld.
* `SimulationSingleton` IComponentData is added.
* `PhysicsWorldSingleton` IComponentData is added. It implements `ICollidable` and has access to the stored `PhysicsWorld` and its utility methods.
* `NativeReference<int> HaveStaticBodiesChanged` get property is added to `BuildPhysicsWorld`.
* Multiple new system groups are introduced: like the `PhysicsSystemGroup` , see the full list at the [Changelog](xref:unity-physics-changelog).
* Impulse events to allow users to break joints.
* Supports the following types of motors: rotational, linear velocity, rotational, angular velocity.
* `CustomPhysicsSystemGroup` and `CustomPhysicsSystemGroupBase` for providing multiple worlds support.
* `CustomPhysicsProxyDriver` IComponentData, with it's authoring (`CustomPhysicsProxyAuthoring`) and a system (`SyncCustomPhysicsProxySystem`), which enable you to drive an entity from one world by an entity from another, using kinematic velocities.


## Updated

* Naming path for Unity Physics Authoring components are now under `Components > Entities > Physics`.
* Use of the new idiomatic `foreach` statement inside of the physics package replacing the old `Entities.ForEach` API.
* All materials in the samples to be universal render pipeline compliant.
* Restored many Gizmo/Mesh methods from DisplayCollidersSystem.cs, but placed in a Utility file instead.
* Changed allocator label to `Allocator.Temp` internally when building `CollisionWorld` from `CollisionWorldProxy`.
* Physics Debug Display: Performance improvements when drawing colliders (faces, edges, AABBs), broadphase, mass properties and contacts.
* Physics Debug Display: Drawing collider faces for Mesh and Convex Hull types use different rendering methods.
* Physics Debug Display: The original Collider Edge drawing code that uses Gizmos has been moved to class 'DisplayGizmoColliderEdges' and to class 'AppendMeshColliders'.
* Using built-in resources for the reference mesh used by cube and icosahedron.
* Resources/ (used by Debug Draw) has been renamed DebugDisplayResources/ and now loads assets differently.
* Removed use of the obsolete AlwaysUpdateSystem attribute. The new RequireMatchingQueriesForUpdate attribute has been added where appropriate.
* `ColliderCastInput` now has a property `QueryColliderScale`. It defaults to `1.0f`, and represents the scale of the passed-in input collider.
* `ColliderCastInput` constructor has changed to take in uniform scale of the query collider as the last parameter. It defaults to `1.0f`.
* The following methods have a uniform scale argument added as the last argument (defaults to `1.0f`), and their arguments are reordered. The old versions are deprecated, see the full list at [Changelog](xref:unity-physics-changelog).
* `BuildPhysicsWorld` is now a `struct` instead of a `class`, and implements `ISystem` instead of `SystemBase`.
* `ExportPhysicsWorld` is now a `struct` instead of a `class`, and implements `ISystem` instead of `SystemBase`.
* Multiple `PhysicsWorldBuilder.` signatures have changed, see [Changelog](xref:unity-physics-changelog).
* `PhysicsWorldData.HaveStaticBodiesChanged` is now a `NativeReference<int>` instead of `NativeArray<int>`.
* `PhysicsWorldData.PhysicsWorldComponentHandles` struct is added. It contains component handles to data types needed to create a `PhysicsWorld`.
* `PhysicsWorldData` also has an  `Update(ref SystemState)` method which just calls `Update(ref SystemState)` on `PhysicsWorldComponentHandles`.
* Multiple `PhysicsWorldExporter.` signatures have changed, see [Changelog](xref:unity-physics-changelog).
* The Joint Constraints container changed from FixedList128 to a custom internal container. Users will get a FixedList512 returned when retrieving constraints.
* Replaced obsolete EntityQueryBuilder APIs with current ones.
* `ISystem` implementations with public data converted to using components for data access.


## Removed
* Dependency on `com.unity.jobs` package has been removed.
* Dependency on `com.unity.test-framework.performance` package has been removed.
* Removed `ICollider.CalculateAabb(RigidTransform transform)`. All `ICollider` implementations will still be able to call `CalculateAabb(RigidTransform transform, float uniformScale = 1)`, except `RigidBody`, `PhysicsWorld` and `CollisionWorld`, where these methods are deprecated.
* Removed `CollisionWorldProxy`. Use `PhysicsWorldSingleton` to achieve the same functionality.
* Simulation callback mechanism has been removed. Find removed APIs at [Changelog](xref:unity-physics-changelog).
* Removed `PhysicsWorld` getter from `BuildPhysicsWorld`. It is still possible to get a `PhysicsWorld` reference through `BuildPhysicsWorld.PhysicsData.PhysicsWorld` but it isn't recommended since it can cause race conditions.
* Removed `StepPhysicsWorld` system.
* Removed `EndFramePhysicsSystem` system.
* Removed `BuildPhysicsWorld.AddInputDependencyToComplete()` from public API.
* Removed `BuildPhysicsWorld.AddInputDependency()` method.
* Removed `BuildPhysicsWorld.GetOutputDependency()` method.
* Removed `ExportPhysicsWorld.AddInputDependency()` method.
* Removed `ExportPhysicsWorld.GetOutputDependency()` method.
* Removed `static class PhysicsRuntimeExtensions`, as a consequence, some extension methods are removed as well, see [Changelog](xref:unity-physics-changelog).
* Removed `PhysicsWorldExporter.SharedData` struct.
* Removed `PhysicsWorldExporter.ScheduleCollisionWorldProxy()` method.
* Removed `PhysicsWorldExporter.ScheduleCollisionWorldCopy()` method.
* Removed `PhysicsWorldExporter.CopyCollisionWorldImmediate()` method.
* Removed `PhysicsWorldStepper` class.


## Further information

* [Upgrade guide](upgrade-guide.md)
* [Changelog](xref:unity-physics-changelog)
