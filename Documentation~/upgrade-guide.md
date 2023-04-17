# Upgrading from Physics 0.51 to 1.0

To upgrade from Physics 0.51 to 1.0, you firstly need to upgrade your Entities package following the com.unity.entities [package upgrade guide](https://docs.unity3d.com/Packages/com.unity.entities@1.0/manual/index.html).

## Physics pipeline reworked

The physics pipeline has been reworked. 
* `PhysicsSystemGroup` is introduced. It is a `ComponentSystemGroup` that covers all physics jobs. It consists of `PhysicsInitializeGroup`, `PhysicsSimulationGroup`, and `ExportPhysicsWorld`. `PhysicsSimulationGroup` further consists of `PhysicsCreateBodyPairsGroup`, `PhysicsCreateContactsGroup`, `PhysicsCreateJacobiansGroup`, `PhysicsSolveAndIntegrateGroup` which run in that order. See [documentation](interacting-with-physics.md) for details. 
* `StepPhysicsWorld` and `EndFramePhysicsSystem` systems have been removed, `BuildPhysicsWorld` has been moved to `PhysicsInitializeGroup`:
  * If you had Update `(Before|After)StepPhysicsWorld`, replace it with: `[UpdateInGroup(typeof(PhysicsSystemGroup))][Update(After|Before)(typeof(PhysicsSimulationGroup))]`. 
  * If you had `Update(Before|After)BuildPhysicsWorld`, replace it with: `[UpdateBefore(typeof(PhysicsSystemGroup))]` or `[UpdateInGroup(typeof(PhysicsSystemGroup))][UpdateAfter(typeof(PhysicsInitializeGroup))]`.
  * If you had `Update(Before|After)ExportPhysicsWorld` replace it with: `[UpdateInGroup(typeof(PhysicsSystemGroup))][UpdateBefore(typeof(ExportPhysicsWorld))]` or `[UpdateAfter(typeof(PhysicsSystemGroup))]`. 
  * If you had `[Update(Before|After)EndFramePhysicsSystem]` replace it with: `[UpdateAfter(typeof(PhysicsSystemGroup))]`. 
  * If you had combination of those (e.g. `[UpdateAfter(typeof(BuildPhysicsWorld))][UpdateBefore(typeof(StepPhysicsWorld))]`) take a look at the diagram in [documentation](physics-pipeline.md). 
* All new systems are unmanaged, which means that they are more efficient, and their `OnUpdate()` is Burst friendly. You shouldn't call `World.GetOrCreateSystem<AnyPhysicsSystem>()` as of this release and should be using singletons (see below).

## Physics Sample custom components
`PhysicsBody`, `PhysicsShape` have been moved to a different package and are not included in the Unity Physice base package.
If you want to keep using those components, please follow this [documentation](custom-samples-physics-components.md) to import them back to your project.

## Retrieval of the Physics world has changed
Retrieval of `PhysicsWorld` is achieved differently. Previously, it was necessary to get it directly from `BuildPhysicsWorld` system. Now, `PhysicsWorld` is retrieved by calling `(SystemAPI|SystemBase|EntityQuery).GetSingleton().PhysicsWorld` in case read-only access is required, and by calling `(SystemAPI|SystemBase|EntityQuery).GetSingletonRW().PhysicsWorld` in case of a read-write access. It is still possible to get the world from `BuildPhysicsWorld`, but is not recommended, as it can cause race conditions. This is only affecting the `PhysicsWorld` managed by the engine. Users still can create and manage their own `PhysicsWorld`. Check out [documentation](physics-singletons.md) for more information.

## Retrieval of the simulation has changed
Retrieval of `Simulation` is achieved differently. Previously, it was neccessary to get it directly from `StepPhysicsWorld` system. Now, `Simulation` is retrieved by calling `(SystemAPI|SystemBase|EntityQuery).GetSingleton().AsSimulation()` in case read-only access is required, and by calling `(SystemAPI|SystemBase|EntityQuery).GetSingletonRW().AsSimulation()` in case of read-write access. Check out [documentation](simulation-results.md) for more information.

## Physics systems dependencies sorting

The dependencies between physics systems now get sorted automatically as long as `GetSingleton<>()` approach is used for retrieving `PhysicsWorld` and `Simulation`. There is no need to call `RegisterPhysicsSystems(ReadOnly|ReadWrite)`, `AddInputDependency()` or `AddInputDependencyToComplete()` and these functions were removed.

## Physics jobs schedule method argument change
`ITriggerEventsJob`, `ICollisionEventsJob`, `IBodyPairsJob`, `IContactsJob` and `IJacobiansJob` no longer take `ISimulation` as an argument for `Schedule()` method, but instead take `SimulationSingleton`. Use `GetSingleton<SimulationSingleton>()` for `ITriggerEventsJob` and `ICollisionEventsJob`, `GetSingletonRW<SimulationSingleton>()` for `IBodyPairsJob`, `IContactsJob` and `IJacobiansJob`. All of these jobs can be now scheduled in Burst friendly way.

## Removed callbacks between simulation stages
Callbacks between simulation stages have been removed. To get the same functionality, you now need to:
* Create a system
* Make it `[UpdateInGroup(typeof(PhysicsSimulationGroup))]` and make it `[UpdateBefore]` and `[UpdateAfter]` one of 4 `PhysicsSimulationGroup` subgroups.
* In `OnUpdate()` of the system, recreate the functionality of a callback by scheduling one of the specialised jobs: `IBodyPairsJob`, `IContactsJob`, `IJacobiansJob`.

## Uniform scale support
Uniform scale is now supported. - `Scale` component is now taken into account when creating physics bodies. The component doesn't get created by `Baking` (previously known as `Conversion`) in the Editor. Scale set in Editor gets baked into the collider geometry. If you want to dynamically scale bodies, add this component to physics body entities. - You might get problems if you were creating `RigidBody` struct instances directly, since the scale will be initialized to zero. Set it to `1.0f` to return to previous behaviour. - `ColliderCast` and `ColliderDistance` queries now support uniform scale for colliders that you are querying with. `ColliderDistanceInput` and `ColliderCastInput` therefore have a new field that enables you to set it. Same as `RigidBody`, you might get problems since the scale will be initialized to zero. Set it to `1.0f` to return to previous behaviour. - Positive and negative values of scale are supported.



## Reworked multiple worlds support
Multiple worlds support has been reworked. To support this use case previously, it was necessary to create a physics pipeline on your own, by using helpers such as `PhysicsWorldData`, `PhysicsWorldStepper` and `PhysicsWorldExporter`. Now it is possible to instantiate a `CustomPhysicsSystemGroup` with a proper world index, which will run the physics simulation on non-default world index bodies. Check out the documentation for more information.

See [documentation](simulation-modification.md) for details and examples.

## DefaultSpringFrequency and DefaultSpringDamping

The `Constraint.DefaultSpringFrequency` and `Constraint.DefaultSpringDamping` values have been changed. The original defaults were setup to match the default `fixedDeltaTime` and therefore assumed a 50hz simulation timestep. The current default simulation step is now 60hz and so the default spring parameters have been changed to match this assumption. This change may affect more complex Joint setups that are close to being overconstrained, but generally it should not break the original intent of the setup.

## PhysicsStep.ThreadCountHint

`PhysicsStep.ThreadCountHint` now been removed, so if you had it set to a value less or equal to 0 (meaning you wanted single threaded simulation with small number of jobs), you now need to set the new field `PhysicsStep.MultiThreaded` to false. Otherwise, it will be set to true, meaning you'll get a default multi threaded simulation as if `PhysicsStep.ThreadCountHint` is a positive number.

## Integrity checks
Integrity checks can now be enabled and disabled by toggling the new `DOTS/Physics/Enable Integrity Checks` menu item. Integrity checks should be enabled when checking simulation quality and behaviour. Integrity checks should be disabled when measuring performance. When enabled, Integrity checks will be included in a in Development build of a standalone executable, but are always excluded in release builds.

## Max ray length
An extra check was added to verify that the data provided to the `Start` & `End` properties of `RayCastInput` & `ColliderCastInput` does not generate a cast length that is too long. The maximum length allowed is half of `float.MaxValue`

## Partial SystemBase-derived
Added partial keyword to all SystemBase-derived classes

## Changed signatures
* `PhysicsWorldBuilder.SchedulePhysicsWorldBuild(SystemBase system, ref PhysicsWorldData physicsData, in JobHandle inputDep, float timeStep, bool isBroadphaseBuildMultiThreaded, float3 gravity, uint lastSystemVersion )` signature has changed to `PhysicsWorldBuilder.SchedulePhysicsWorldBuild(ref SystemState systemState, ref PhysicsWorldData physicsData, in JobHandle inputDep, float timeStep, bool isBroadphaseBuildMultiThreaded, float3 gravity, uint lastSystemVersion )`.
* `PhysicsWorldBuilder.SchedulePhysicsWorldBuild(SystemBase system, ref PhysicsWorld world, ref NativeArray<int> haveStaticBodiesChanged, ref PhysicsWorld world, ref NativeReference<int> haveStaticBodiesChanged, in PhysicsWorldData.PhysicsWorldComponentHandles componentHandles, in JobHandle inputDep, float timeStep, bool isBroadphaseBuildMultiThreaded, float3 gravity, uint lastSystemVersion, EntityQuery dynamicEntityGroup, EntityQuery staticEntityGroup, EntityQuery jointEntityGroup)` signature has changed to `PhysicsWorldBuilder.SchedulePhysicsWorldBuild(ref PhysicsWorld world, ref NativeReference<int> haveStaticBodiesChanged, in PhysicsWorldData.PhysicsWorldComponentHandles componentHandles, in JobHandle inputDep, float timeStep, bool isBroadphaseBuild,MultiThreadedfloat3 gravity, uint lastSystemVersion, EntityQuery dynamicEntityGroup, EntityQuery staticEntityGroup, EntityQuery jointEntityGroup)`.
* `PhysicsWorldBuilder.ScheduleBroadphaseBVHBuild(ref PhysicsWorld world, ref NativeArray<int> haveStaticBodiesChanged, in JobHandle inputDep, float timeStep, bool isBroadphaseBuildMultiThreaded, float3 gravity)` signature has changed to `PhysicsWorldBuilder.ScheduleBroadphaseBVHBuild(ref PhysicsWorld world, NativeReference<int> haveStaticBodiesChanged, in JobHandle inputDep, float timeStep, bool isBroadphaseBuildMultiThreaded, float3 gravity)`.
* `PhysicsWorldBuilder.BuildPhysicsWorldImmediate(SystemBase system, ref PhysicsWorldData data, float timeStep, float3 gravity, uint lastSystemVersion)` signature has changed to `PhysicsWorldBuilder.BuildPhysicsWorldImmediate(ref SystemState systemState, ref PhysicsWorldData data, float timeStep, float3 gravity, uint lastSystemVersion)`.
* `PhysicsWorldBuilder.BuildPhysicsWorldImmediate(SystemBase system, ref PhysicsWorld world, ref NativeArray<int> haveStaticBodiesChanged, float timeStep, float3 gravity, uint lastSystemVersion, EntityQuery dynamicEntityGroup, EntityQuery staticEntityGroup, EntityQuery jointEntityGroup)` signature has changed to `PhysicsWorldBuilder.BuildPhysicsWorldImmediate(ref PhysicsWorld world, NativeReference<int> haveStaticBodiesChanged, in PhysicsWorldData.PhysicsWorldComponentHandles, float timeStep, float3 gravity, uint lastSystemVersion, EntityQuery dynamicEntityGroup, EntityQuery staticEntityGroup, EntityQuery jointEntityGroup)`.
* `PhysicsWorldExporter.SchedulePhysicsWorldExport(SystemBase system, in PhysicsWorld world, in JobHandle inputDep, EntityQuery dynamicEntities)` signature has changed to `PhysicsWorldExporter.SchedulePhysicsWorldExport(ref SystemState systemState, ref ExportPhysicsWorldTypeHandles componentTypeHandles, in PhysicsWorld world, in JobHandle inputDep, EntityQuery dynamicEntities)`.
* `PhysicsWorldExporter.ExportPhysicsWorldImmediate(SystemBase system, in PhysicsWorld world, EntityQuery dynamicEntities)` signature has changed to `PhysicsWorldExporter.ExportPhysicsWorldImmediate(ref SystemState systemState, ref ExportPhysicsWorldTypeHandles componentTypeHandles, in PhysicsWorld world, EntityQuery dynamicEntities)`.

## Deprecated
The following methods have a uniform scale argument added as the last argument (defaults to `1.0f`), and their arguments are reordered. The old versions are deprecated:
* `AppendMeshColliders.GetMeshes.AppendSphere(SphereCollider* sphere, RigidTransform worldFromCollider, ref List results)` has been deprecated. Use `AppendSphere(ref List results, SphereCollider* sphere, RigidTransform worldFromCollider, float uniformScale = 1)`.
* `AppendMeshColliders.GetMeshes.AppendCapsule(CapsuleCollider* capsule, RigidTransform worldFromCollider, ref List results)` has been deprecated. Use `AppendCapsule(ref List results, CapsuleCollider* capsule, RigidTransform worldFromCollider, float uniformScale = 1)`.
* `AppendMeshColliders.GetMeshes.AppendMesh(MeshCollider* meshCollider, RigidTransform worldFromCollider, ref List results)` has been deprecated. Use `AppendMesh(ref List results, MeshCollider* meshCollider, RigidTransform worldFromCollider, float uniformScale = 1)`.
* `AppendMeshColliders.GetMeshes.AppendCompound(CompoundCollider* compoundCollider, RigidTransform worldFromCollider, ref List results)` has been deprecated. Use `AppendCompound(ref List results, CompoundCollider* compoundCollider, RigidTransform worldFromCollider, float uniformScale = 1)`.
* `AppendMeshColliders.GetMeshes.AppendTerrain(TerrainCollider* terrainCollider, RigidTransform worldFromCollider, ref List results)` has been deprecated. Use `AppendTerrain(ref List results, TerrainCollider* terrainCollider, RigidTransform worldFromCollider, float uniformScale = 1)`.
* `AppendMeshColliders.GetMeshes.AppendCollider(Collider* collider, RigidTransform worldFromCollider, ref List results)` has been deprecated. Use `AppendCollider(ref List results, Collider* collider, RigidTransform worldFromCollider, float uniformScale = 1)`.
* `ColliderDistanceInput.ColliderDistanceInput(BlobAssetReference collider, RigidTransform transform, float maxDistance)` has been deprecated. Use `ColliderDistanceInput(BlobAssetReference collider, float maxDistance, RigidTransform transform, float uniformScale = 1)`.
* `Collider.GetLeafCollider(Collider* root, RigidTransform rootTransform, ColliderKey key, out ChildCollider leaf)` has been deprecated. Use Use `GetLeafCollider(out ChildCollider leaf, Collider* root, ColliderKey key, RigidTransform rootTransform, float rootUniformScale = 1)` instead.
* `Math.TransformAabb(RigidTransform transform, Aabb aabb)` hase been deprecated. Use `Math.TransformAabb(Aabb aabb, RigidTransform transform, float uniformScale = 1)` instead.
* `Math.TransformAabb(MTransform transform, Aabb aabb)` hase been deprecated. Use `Math.TransformAabb(Aabb aabb, MTransform transform, float uniformScale = 1)` instead.

## Removed
* Removed `ICollider.CalculateAabb(RigidTransform transform)`. All ICollider implementations will still be able to call `CalculateAabb(RigidTransform transform, float uniformScale = 1)`, except `RigidBody`, `PhysicsWorld` and `CollisionWorld`, where these methods are deprecated.
* Removed `[CollisionWorldProxy]`. Use `PhysicsWorldSingleton` to achieve the same functionality.
* Simulation callback mechanism has been removed. As a consequence, the following APIs are removed as well:
  * `class SimulationCallbacks` is removed.
  * `enum SimulationCallbacks.Phase is removed`. - callback delegate : `public delegate JobHandle Callback(ref ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)` has been removed.
* Removed `PhysicsWorld` getter from `BuildPhysicsWorld`. It is still possible to get a `PhysicsWorld` reference through `BuildPhysicsWorld`.`PhysicsData.PhysicsWorld` but it isn't recommended since it can cause race conditions.
* Removed `StepPhysicsWorld` system. 
* Removed `EndFramePhysicsSystem` system. 
* Removed `BuildPhysicsWorld.AddInputDependencyToComplete()` from public API. 
* Removed `BuildPhysicsWorld.AddInputDependency()` method. 
* Removed `BuildPhysicsWorld.GetOutputDependency()` method. 
* Removed `ExportPhysicsWorld.AddInputDependency()` method. 
* Removed `ExportPhysicsWorld.GetOutputDependency()` method. 
  * Removed static class `PhysicsRuntimeExtenstions`, as a consequence, the following extension methods are removed as well: 
  * public static void `RegisterPhysicsRuntimeSystemReadOnly(this SystemBase system)` 
  * public static void `RegisterPhysicsRuntimeSystemReadWrite(this SystemBase system) `
  * public static void `RegisterPhysicsRuntimeSystemReadOnly<T>(this SystemBase system) where T : unmanaged, IComponentData `
  * public static void `RegisterPhysicsRuntimeSystemReadWrite<T>(this SystemBase system) where T : unmanaged, IComponentData `
* Removed `PhysicsWorldExporter.SharedData` struct. 
* Removed `PhysicsWorldExporter.ScheduleCollisionWorldProxy()` method. 
* Removed `PhysicsWorldExporter.ScheduleCollisionWorldCopy()` method. 
* Removed `PhysicsWorldExporter.CopyCollisionWorldImmediate()` method. 
* Removed `PhysicsWorldStepper` class.



RigidTransform argument removed:
* `CollisionWorld.CalculateAabb(RigidTransform transform)` has been deprecated. `Use CollisionWorld.CalculateAabb()` without a parameter.
* `RigidBody.CalculateAabb(RigidTransform transform)` has been deprecated. Use `RigidBody.CalculateAabb()` without a parameter.
* `PhysicsWorld.CalculateAabb(RigidTransform transform)` has been deprecated. Use `PhysicsWorld.CalculateAabb()` without a parameter.

## Other

* PhysicsWorldData.HaveStaticBodiesChanged is now a NativeReference<int> instead of NativeArray<int>.
* `Attributes.cs` script has been removed since the `com.unity.properties` package is part of the editor as a module.
* Use of `TransformAspect.WorldPosition`, T`ransformAspect.WorldRotation`, `TransformAspect.WorldScale` when using `Transform_V2` instead of `TransformAspect.Position`, `TransformAspect.Rotation`, `TransformAspect.Scale`. 
* `BaseShapeBakingSystem` and `BuildCompoundCollidersBakingSystem` have been modified to use `IJobEntity` instead of `Entities.ForEach()`.
* Replaced `PhysicsTransformAspect` with `TransformAspect`

