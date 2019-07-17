## [0.2.0-preview] - 2019-07-18

### Upgrade guide

* If you created per-body custom data using `PhysicsShape.CustomFlags` then you should instead do it using `PhysicsBody.CustomTags`.
* Some public API points were _removed_, either because they were not intended to be public or because they introduce other problems (see Changes below).
    * Some of these API points may later be reintroduced on a case-by-case basis to enable customization for advanced use cases. Please provide feedback on the forums if these removals have affected current use cases so we can prioritize them.
* Some public API points were _replaced_ with another one in a way that cannot be handled by the script updater, so they must be manually fixed in your own code (see Changes below). 
* All public types in test assemblies were never intended to be public and have been made internal.
* Some properties on `PhysicsMaterialTemplate` and `PhysicsShape` now return `PhysicsCategoryTags` instead of `int`. Use its `Value` property if you need to get/set a raw `int` value (see Changes below).

### Changes

* Run-Time API
    * Added first draft of a new `TerrainCollider` struct. Terrain geometry is defined by a height field. It requires less memory than an equivalent mesh and is faster to query. It also offers a fast, low-quality option for collision detection.
        * Added `ColliderType.Terrain`.
        * Added `CollisionType.Terrain`.
        * Added `DistanceQueries.ConvexTerrain<T>()`.
        * Added `DistanceQueries.PointTerrain<T>()`.
    * Shapes and bodies how have their own separate custom data.
        * Added `Material.CustomTags` (for shapes).
        * Replaced `PhysicsCustomData` with `PhysicsCustomTags` (for bodies).
        * Replaced `ContactHeader.BodyCustomDatas` with `ContactHeader.BodyCustomTags`.
        * Replaced `CustomDataPair` with `CustomTagsPair`.
        * Replaced `RigidBody.CustomData` with `RigidBody.CustomTags`.
    * Removed coefficient of restitution concept from Jacobians. All restitution calculations are approximated and applied before the solve, so restitution changes at this point in the simulation have no effect.
        * `ModifiableContactJacobian.CoefficientOfRestitution` is now obsolete.
    * `ModifiableContactJacobian.FrictionEffectiveMassOffDiag` is now obsolete. It was not possible to make any meaningful changes to it without fully understanding friction solving internals.
    * Removed max impulse concept from Jacobians. Solver design implies impulses are pretty unpredictable, making it difficult to choose maximum impulse value in practice.
        * `JacobianFlags.EnableMaxImpulse` is now obsolete.
            * Underlying values of `JacobianFlags` have been changed.
            * Added `JacobianFlags.UserFlag2`.
        * `Material.EnableMaxImpulse` is now obsolete.
        * `MaterialFlags.EnableMaxImpulse` is now obsolete.
        * `ModifiableJacobianHeader.HasMaxImpulse` is now obsolete.
        * `ModifiableJacobianHeader.MaxImpulse` is now obsolete.
    * Removed the following members:
        * `CollisionFilter.CategoryBits`
        * `CollisionFilter.MaskBits`
    * Removed the following types from public API and made them internal:
        * `AngularLimit1DJacobian`
        * `AngularLimit2DJacobian`
        * `AngularLimit3DJacobian`
        * `BaseContactJacobian`
        * `BoundingVolumeHierarchy.BuildBranchesJob`
        * `BoundingVolumeHierarchy.BuildFirstNLevelsJob`
        * `BoundingVolumeHierarchy.FinalizeTreeJob`
        * `Broadphase`
        * `ColliderCastQueries`
        * `CollisionEvent`
        * `CollisionEvents`
        * `ContactHeader`
        * `ContactJacobian`
        * `ConvexConvexDistanceQueries`
        * `ConvexHull`
        * `ConvexHullBuilder`
        * `ConvexHullBuilderExtensions`
        * `DistanceQueries`
        * `ElementPool<T>` (Unity.Collections)
        * `Integrator`
        * `IPoolElement` (Unity.Collections)
        * `JacobianHeader`
        * `JacobianIterator`
        * `JacobianUtilities`
        * `LinearLimitJacobian`
        * `ManifoldQueries`
        * `Mesh`
        * `MotionExpansion`
        * `NarrowPhase`
        * `OverlapQueries`
        * `QueryWrappers`
        * `RaycastQueries`
        * `Scheduler`
        * `Simulation.Context`
        * `Solver`
        * `StaticLayerChangeInfo`
        * `TriggerEvent`
        * `TriggerEvents`
        * `TriggerJacobian`
    * Removed the following members from public API and made them internal:
        * Aabb.CreateFromPoints(float3x4)
        * `BoundingVolumeHierarchy.BuildBranch()`
        * `BoundingVolumeHierarchy.BuildCombinedCollisionFilter()`
        * `BoundingVolumeHierarchy.BuildFirstNLevels()`
        * `BoundingVolumeHierarchy.CheckIntegrity()`
        * All explicit `ChildCollider` constructors
        * `ColliderKeyPath(ColliderKey, uint)`
        * `ColliderKeyPath.Empty`
        * `ColliderKeyPath.GetLeafKey()`
        * `ColliderKeyPath.PopChildKey()`
        * `ColliderKeyPath.PushChildKey()`
        * `CollisionWorld.Broadphase`
        * `CompoundCollider.BoundingVolumeHierarchy`
        * `Constraint.ConstrainedAxis1D`
        * `Constraint.Dimension`
        * `Constraint.FreeAxis2D`
        * `ConvexCollider.ConvexHull`
        * `MeshCollider.Mesh`
        * `MotionVelocity.CalculateExpansion()`
        * `SimulationCallbacks.Any()`
        * `SimulationCallbacks.Execute()`
    * `ChildCollider.TransformFromChild` is now a readonly property instead of a field.
    * Removed `BuildPhysicsWorld.m_StaticLayerChangeInfo`.
    * Added `FourTranposedAabbs.DistanceFromPointSquared()` signatures passing scale parameter.
    * Changed `ISimulation` interface (i.e. `Simulation` class).
        * Added `ISimulation.FinalJobJandle`.
        * Added `ISimulation.FinalSimulationJobJandle`.
        * Added simpler `ISimulation.ScheduleStepJobs()` signature and marked previous one obsolete.
    * Added `RigidBody.HasCollider`.
    * `SimplexSolver.Solve()` now takes an optional bool to specify whether it should respect minimum delta time.
    * `SurfaceVelocity.ExtraFrictionDv` has been removed and replaced with more usable `LinearVelocity` and `AngularVelocity` members.
* Authoring/Conversion API
    * Added `CustomBodyTagNames`.
    * Renamed `CustomMaterialTagNames.FlagNames` to `CustomMaterialTagNames.TagNames`.
    * Renamed `CustomFlagNames` to `CustomPhysicsMaterialTagNames`.
    * Renamed `CustomPhysicsMaterialTagNames.FlagNames` to `CustomPhysicsMaterialTagNames.TagNames`.
    * Added `PhysicsCategoryTags`, `CustomBodyTags`, and `CustomMaterialTags` authoring structs.
    * The following properties now return `PhysicsCategoryTags` instead of `int`:
        * `PhysicsMaterialTemplate.BelongsTo`
        * `PhysicsMaterialTemplate.CollidesWith`
        * `PhysicsShape.BelongsTo`
        * `PhysicsShape.CollidesWith`
    * The following members on `PhysicsMaterialTemplate` are now obsolete:
        * `GetBelongsTo()`
        * `SetBelongsTo()`
        * `GetCollidesWith()`
        * `SetCollidesWith()`
    * The following members on `PhysicsShape` are now obsolete:
        * `GetBelongsTo()`
        * `SetBelongsTo()`
        * `GetCollidesWith()`
        * `SetCollidesWith()`
    * Added `PhysicsMaterialTemplate.CustomTags`.
        * `CustomFlags`, `GetCustomFlag()` and `SetCustomFlag()` are now obsolete.
    * Added `PhysicsShape.CustomTags`.
        * `CustomFlags`, `GetCustomFlag()` and `SetCustomFlag()` are now obsolete.
    * Added `PhysicsBody.CustomTags`.
    * Renamed `PhysicsShape.OverrideCustomFlags` to `PhysicsShape.OverrideCustomTags`.
    * Renamed `PhysicsShape.CustomFlags` to `PhysicsShape.CustomTags`.
    * Renamed `PhysicsShape.GetCustomFlag()` to `PhysicsShape.GetCustomTag()`.
    * Renamed `PhysicsShape.SetCustomFlag()` to `PhysicsShape.SetCustomTag()`.
    * Overload of `PhysicsShape.GetCapsuleProperties()` is now obsolete.
    * Overload of `PhysicsShape.GetPlaneProperties()` is now obsolete.
    * Removed `PhysicsBody.m_OverrideDefaultMassDistribution` (backing field never intended to be public).
    * `PhysicsShape.GetBoxProperties()` now returns underlying serialized data instead of reorienting/resizing when aligned to local axes.
    * `BaseShapeConversionSystem.GetCustomFlags()` is now obsolete.
    * Parameterless constructors have been made private for the following types because they should not be used (use instead ScriptableObjectCreateInstance<T>() or GameObject.AddComponent<T>() as applicable):
        * `CustomPhysicsMaterialTagNames`
        * `PhysicsCategoryNames`
        * `PhysicsMaterialTemplate`
        * `PhysicsBody`
        * `PhysicsShape`
    * The following types have been made sealed:
        * `LegacyBoxColliderConversionSystem`
        * `LegacyCapsuleColliderConversionSystem`
        * `LegacySphereColliderConversionSystem`
        * `LegacyMeshColliderConversionSystem`
        * `LegacyRigidbodyConversionSystem`
        * `PhysicsBodyConversionSystem`
        * `PhysicsShapeConversionSystem`
    * `DisplayContactsJob` has been deprecated in favor of protected `DisplayContactsSystem.DisplayContactsJob`.
    * `FinishDisplayContactsJob` has been deprecated in favor of protected `DisplayContactsSystem.FinishDisplayContactsJob`.
    * `DisplayJointsJob` has been deprecated in favor of protected `DisplayJointsSystem.DisplayJointsJob`.
    * `FinishDisplayTriggerEventsJob` has been deprecated in favor of protected `DisplayTriggerEventsSystem.FinishDisplayTriggerEventsJob`.
    * The following types have been deprecated and will be made internal in a future release:
        * `DisplayBodyColliders.DrawComponent`
        * `DisplayCollisionEventsSystem.FinishDisplayCollisionEventsJob`
* Run-Time Behavior
    * Collision events between infinite mass bodies (kinematic-kinematic and kinematic-static) are now raised. The reported impulse will be 0.
    * The default value of `Unity.Physics.PhysicsStep.ThreadCountHint` has been increased from 4 to 8.
    * `EndFramePhysicsSystem` no longer waits for `HandlesToWaitFor`, instead it produces a `FinalJobHandle` which is a combination of those jobs plus the built-in physics jobs. Subsequent systems that depend on all physics jobs being complete can use that as an input dependency.
* Authoring/Conversion Behavior
    * `PhysicsCustomData` is now converted from `PhysicsBody.CustomTags` instead of using the flags common to all leaf shapes.
    * `PhysicsShape.CustomTags` is now converted into `Material.CustomTags`.
    * If a shape conversion system throws an exception when producing a `PhysicsCollider`, then it is simply skipped and logs an error message, instead of causing the entire conversion process to fail.
    * `PhysicsShape` Inspector now displays suggestions of alternative primitive shape types if a simpler shape would yield the same collision result as the current configuration.
    * `PhysicsShape` Inspector now displays error messages if a custom mesh or discovered mesh is not compatible with run-time conversion.

### Fixes

* Draw Collider Edges now supports spheres, capsules, cylinders and compound colliders.
* Fixed bug causing editor to get caught in infinite loop when adding `PhysicsShape` component to a GameObject with deactivated children with `MeshFilter` components. 
* Fixed bug resulting in the creation of `PhysicMaterial` objects in sub-scenes when converting legacy colliders.
* Fixed bug when scaling down friction on bounce in Jacobian building. Once a body was declared to bounce (apply restitution), all subsequent body Jacobians had their friction scaled down to 25%.
* Fixed bug resulting in the broadphase for static bodies possibly not being updated when adding or moving a static body, causing queries and collisions to miss.
* Fixed bug with restitution impulse during penetration recovery being too big due to wrong units used.
* Fixed bug with energy gain coming from restitution impulse with high restitution values.
* Fixed Jacobian structures being allocated at non 4 byte aligned addresses, which caused crashes on Android
* Removed Solver & Scheduler asserts from Joints between two static bodies #383.
* Fixed bug preventing the conversion of classic `BoxCollider` components with small sizes.
* Fixed bug where `PhysicsShape.ConvexRadius` setter was clamping already serialized value instead of newly assigned value.
* Fixed bug where `PhysicsShape` orientation, size, and convex radius values might undergo changes during conversion resulting in identical volumes; only objects inheriting non-uniform scale now exhibit this behavior. 
* Fixed bug causing minor drawing error for cylinder `PhysicsShape` with non-zero convex radius.
* Fixed crash when trying to run-time convert a `MeshCollider` or `PhysicsShape` with a non-readable mesh assigned. Conversion system now logs an exception instead.
* Fixed crash when constructing a `MeshCollider` with no input triangles. A valid (empty) mesh is still created.
* Fixed bugs building to IL2CPP resulting in unresolved external symbols in `BvhLeafProcessor`, `ConvexCompoundLeafProcessor`, and `ConvexMeshLeafProcessor`.
* Fixed bug causing physics IJob's to not be burst compiled (ICollisionEventsJob, ITriggerEventsJob, IBodyPairsJob, IContactsJob, IJacobiansJob)



## [0.1.0-preview] - 2019-05-31

### Upgrade guide

* Any run-time code that traversed the transform hierarchy of physics objects to find other entities must instead store references to entity IDs of interest through a different mechanism. 
* Baking of non-uniform scale for parametric `PhysicsShape` types has changed to more predictably approximate skewed render meshes. Double check the results on any non-uniformly scaled objects in your projects.
* Angular Velocity is currently set in Motion Space. The Motion Space orientation of dynamic bodies may have changed with the changes to the conversion pipeline. If dynamic bodies are now rotating differently, check the values of `Initial Angular Velocity` in the `PhysicsBody` component, or values set to `Angular` in the `PhysicsVelocity` component.
* Convex `PhysicsShape` objects with no custom mesh assigned now include points from enabled mesh renderers on their children. Double check any convex objects in your projects.
* The `RaycastInput` and `ColliderCastInput` structs have changed. See below for details.

### Changes

* Run-Time API
    * Renamed `CollisionFilter.CategoryBits` to `CollisionFilter.BelongsTo`.
    * Renamed `CollisionFilter.MaskBits` to `CollisionFilter.CollidesWith`. 
    * `RaycastInput` and `ColliderCastInput` have changed:
        * At the input level, start and end positions are now specified rather then an initial position and a displacement. 
        * `Start` replaces `Position`.
        * `End` replaces `Direction` which had been confusing as it was actually a displacement vector to a second point on the ray. 
        * `Ray` has been made internal as a lower level interface and updated to be less ambiguous.
    * Added job interfaces for easier reading of simulation events, instead of having to work directly with block streams.
        * `ICollisionEventsJob` calls `Execute()` for every collision event produced by the solver.
        * `ITriggerEventsJob` calls `Execute()` for every trigger event produced by the solver.
        * These events now also include the Entity's involved, not just the rigid body indices.
    * Added job interfaces for easier modification of simulation data, instead of having to work directly with block streams.
        * `IBodyPairsJob` calls `Execute()` for every body pair produced by the broad phase.
        * `IContactsJob` calls `Execute()` for every contact manifold produced by the narrowphase.
        * `IJacobiansJob` calls `Execute()` for every contact jacobian before it is solved.
* Authoring/Conversion API
    * Renamed `SecondPassLegacyRigidbodyConversionSystem` to `LegacyRigidbodyConversionSystem`.
    * Deprecated `FirstPassPhysicsBodyConversionSystem`.
    * Renamed `SecondPassPhysicsBodyConversionSystem` to `PhysicsBodyConversionSystem`.
    * Deprecated `FirstPassLegacyRigidbodyConversionSystem`.
    * Deprecated overload of `PhysicsShape.GetCapsuleProperties()` returning raw points.
    * Deprecated overload of `PhysicsShape.GetPlaneProperties()` returning raw points.
    * Renamed `PhysicsShape.FitToGeometry()` to `PhysicsShape.FitToEnabledRenderMeshes()`. New method accounts for enabled `MeshRenderer` components on child objects with the same physics body and leaf shape.
    * `PhysicsShape.GetConvexHullProperties()` now includes points from enabled `MeshRenderer` components on child objects with the same physics body and leaf shape when no custom mesh has been assigned.
* Run-Time Behavior
    * Added a fixed angle constraint to prismatic joints so they no longer rotate freely.
    * Any Entity with a `PhysicsVelocity` component will be added to the `PhysicsWorld` and integrated, even if it has no `PhysicsCollider`.
* Authoring/Conversion Behavior
    * Physics data on deactivated objects in the hierarchy are no longer converted.
    * All physics objects (i.e. bodies) are now un-parented (required since all simulation happens in world space).
    * Legacy `Collider` components are no longer converted if they are disabled.
    * Converted `PhysicsShape` objects with non-uniform scale now bake more predictable results when their `Transform` is skewed.
    * All menu paths for custom assets and authoring components have changed from 'DOTS Physics' to 'DOTS/Physics'.

### Fixes

* Fixed trigger events not being raised between kinematic-vs-kinematic or kinematic-vs-static body pairs.
* Fixed crash in BuildPhysicsWorld when creating a dynamic body without a `PhysicsMass` component
* Cylinder/sphere GameObjects no longer appear in first frame when draw colliders is enabled on `PhysicsDebugDisplay`.
* Fix bugs in `BoundingVolumeHierarchy.Build` which now produces `BoundingVolumeHierarchy` of greater quality. This will affect performance of BVH queries, and most notably it will significantly improve performance of `DynamicVsDynamicFindOverlappingPairsJob`
* Fixed incorrect 3DOF angular constraint solving with non-identity joint orientation
* Fixed bug where converted `SphereCollider` would apply incorrect center offset when in a hierarchy with non-uniform scale.
* Fixed bug where converted `PhysicsShape` would not become part of a compound collider at run-time if the first `PhysicsBody` ancestor found higher up the hierarchy was disabled.
* Fixed bug where leaves of compound shapes in a hierarchy might be added to the wrong entity if it had disabled `UnityEngine.Collider` components.
* Fixed bug causing leaves of compound shapes on prefabs to convert into individual static colliders.
* Fixed restitution response to be more bouncy for convex objects with high restitution values
* Fixed bug where a converted GameObject in a hierarchy would have the wrong translation and rotation at run-time.
* Fixed bug where objects with `StaticOptimizeEntity` would not be converted into physics world.
* Preview for Mesh `PhysicsShape` no longer culls back faces.
* Inspector control for convex radius on `PhysisShape` now appears when shape type is convex hull.

### Known Issues

* Attempting to fit a non-uniformly scaled `PhysicsShape` to its render meshes may produce unexpected results.
* Physics objects loaded from sub-scenes may have the wrong transformations applied on Android.
* Dragging a control label to modify the orientation of a `PhysicsShape` sometimes produces small changes in its size.
* If gizmos are enabled in the Game tab, the 'Physics Debug Display' viewers are incorrectly rendered. Debug viewers render correctly in the Scene tab. 



## [0.0.2-preview] - 2019-04-08

### Upgrade guide

* Any assembly definitions referencing `Unity.Physics.Authoring` assembly by name must be updated to instead reference `Unity.Physics.Hybrid`.

### Changes

* Renamed `Unity.Physics.Authoring` assembly to `Unity.Physics.Hybrid`. (All of its types still exist in the `Unity.Physics.Authoring` namespace.)
* Radius of cylinder `PhysicsShape` is no longer non-uniformly scaled when converted.

### Fixes

* Fixed exception when converting a box `PhysicsShape` with negative scale.
* Fixed incorrect orientation when fitting capsule, cylinder, or plane `PhysicsShape` to render mesh.
* Fixed convex radius being too large when switching `PhysicsShape` from plane to box or cylinder.
* Fixed calculation of minimum half-angle between faces in convex hulls.
* Fixed collision/trigger event iterators skipping some events when iterating.
* Fixed memory leak when enabling "Draw colliders" in the Physics Debug Display.

### Known Issues

* Physics systems are tied to (variable) rendering frame rate when using automatic world bootstrapping. See `FixedTimestep` examples in ECS Samples project for currently available approaches.
* IL2CPP player targets have not yet been fully validated.
* Some tests might occasionally fail due to JobTempAlloc memory leak warnings.
* Swapping `PhysicsShape` component between different shape types may produce non-intuitive results when nested in hierarchies with non-uniform scale.
* Some `PhysicsShape` configurations do not bake properly when nested in hierarchies with non-uniform scale.
* `PhysicsShape` does not yet visualize convex hull shapes at edit-time.
* Drag values on classic `Rigidbody` components are not currently converted correctly.



## [0.0.1-preview] - 2019-03-12

* Initial package version
