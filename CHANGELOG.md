## [0.2.5-preview.1] - 2019-12-05

### Fixes

* Fixed a bug that could cause compound colliders to not update with Live Link when moving around a mesh collider in the compound's hierarchy.
* Fixed a performance regression that occurred in certain situations when using Live Link

## [0.2.5-preview] - 2019-12-04

### Upgrade guide

* By default, `PhysicsColliders` that share the same set of inputs and that originate from the same sub-scene should reference the same data at run-time. This change not only reduces memory pressure, but also speeds up conversion. If you were altering `PhysicsCollider` data at run-time, you need to enable the `ForceUnique` setting on the respective `PhysicsColliderAuthoring` component. This setting guarantees the object will always convert into a unique instance.
* Any usages of `BlockStream` should be replaced with the `NativeStream` type from the com.unity.collections package.

### Changes

* Run-Time API
    * Removed `BlockStream` and migrated all usages to `NativeSteam`.
    * Access to `JointData.Constraints` has changed to an indexer. This means that to change the values of a `Constraint`, a copy should be made first. E.g.,
        ```c#
        var c = JointData.Constraints[index];
        c.Min = 0;
        JointData.Constraints[index] = c;
        ```
    * Added `maxVelocity` parameter to `SimplexSolver.Solve()` to clamp the solving results to the maximum value.
    * Added `SurfaceConstraintInfo.IsTooSteep` to indicate that a particular surface constraint has a slope bigger than the slope character controller supports.
    * `MeshCollider.Create()` now takes grouped triangle indices (NativeArray<int3>) instead of a flat list of indices (NativeArray<int>) as input.
    * Removed the following expired members/types:
        * `BoxCollider.ConvexRadius` (renamed to `BevelRadius`)
        * `CyllinderCollider.ConvexRadius` (renamed to `BevelRadius`)
        * Collider factory methods passing nullable types and numeric primitives:
            * `BoxCollider.Create()`
            * `CyllinderCollider.Create()`
            * `CapsuleCollider.Create()`
            * `ConvexCollider.Create()`
            * `MeshCollider.Create()`
            * `PolygonCollider.CreateQuad()`
            * `PolygonCollider.CreateTriangle()`
            * `SphereCollider.Create()`
            * `TerrainCollider.Create()`
        * `ComponentExtensions` members passing Entity (use components or variants passing component data instead):
            * `ApplyAngularImpulse()`
            * `ApplyImpulse()`
            * `ApplyLinearImpulse()`
            * `GetAngularVelocity()`
            * `GetCenterOfMass()`
            * `GetCollisionFilter()`
            * `GetEffectiveMass()`
            * `GetLinearVelocity()`
            * `GetMass()`
            * `GetPosition()`
            * `GetRotation()`
            * `GetVelocities()`
            * `SetAngularVelocity()`
            * `SetLinearVelocity()`
            * `SetVelocities()`
        * `CustomDataPair`
        * `ISimulation.ScheduleStepJobs()`
        * `JacobianFlags.EnableEnableMaxImpulse`
        * `MaterialFlags.EnableMaxImpulse`
        * `ModifiableContactHeader.BodyCustomDatas`
        * `ModifiableJacobianHeader.HasMaxImpulse`
        * `ModifiableJacobianHeader.MaxImpulse`
        * `ModifiableContactJacobian.CoefficientOfRestitution`
        * `ModifiableContactJacobian.FrictionEffectiveMassOffDiag`
        * `PhysicsCustomData`
        * `SimplexSolver.c_SimplexSolverEpsilon`
        * `SimplexSolver.Solve()` without `minDeltaTime` parameter

* Authoring/Conversion API
    * Added `PhysicsShapeAuthoring.ForceUnique`.
    * Added the following conversion systems:
        * `BeginColliderConversionSystem`
        * `BuildCompoundCollidersConversionSystem`
        * `EndColliderConversionSystem`
    * `PhysicsShapeAuthoring.GetMeshProperties()` now populates a `NativeList<int3>` for indices, instead of a `NativeList<int>`.
    * The following public members have been made protected:
        * `DisplayCollisionEventsSystem.FinishDisplayCollisionEventsJob`
        * `DisplayTriggerEventsSystem.FinishDisplayTriggerEventsJob`
    * Removed the following expired members/types:
        * `BaseShapeConversionSystem<T>.GetCustomFlags()`
        * `DisplayCollidersSystem.DrawComponent`
        * `DisplayCollidersSystem.DrawComponent.DisplayResult`
        * `DisplayContactsJob`
        * `DisplayJointsJob`
        * `FinishDisplayContactsJob`
        * `PhysicsShapeAuthoring` members:
            * `SetConvexHull()` passing only a mesh
            * `GetMesh()` replaced with `GetMeshProperties`
            * `ConvexRadius` replaced with `BevelRadius`
            * `GetBoxProperties()` returning `void`
            * `SetBox()` passing a box center, size and orientation. Pass `BoxGeometry` instead.
            * `GetCapsuleProperties()` returning `void`
            * `SetCapsule()` passing a capsule center, height, radius and orientation. Pass `CapsuleGeometry` instead.
            * `GetCylinderProperties()` returning `void`
            * `SetCylinder()` passing a cylinder center, height, radius and orientation. Pass `CylinderGeometry` instead.
            * `GetSphereProperties()` returning `void`
            * `SetSphere()` passing a sphere center, radius and orientation. Pass `SphereGeometry` instead.
        * The following components have been removed:
            * `PhysicsBody` (renamed to `PhysicsBodyAuthoring`)
            * `PhysicsShape` (renamed to `PhysicsShapeAuthoring`)
            * `PhysicsStep` (renamed to `PhysicsStepAuthoring`)
            * `PhysicsDebugDisplay` (renamed to `PhysicsDebugDisplayAuthoring`)

* Run-Time Behavior
    * `CompoundCollider.Create()` is now compatible with Burst.
    * `CompoundCollider.Create()` now correctly shares memory among repeated instances in the list of children.

* Authoring/Conversion Behavior
    * If mesh and convex `PhysicsShapeAuthoring` components have not explicitly opted in to `ForceUnique`, they may share the same `PhysicsCollider` data at run-time, if their inputs are the same.
    * Classic `MeshCollider` instances with the same inputs will always share the same data at run-time when converted in a sub-scene.
    * Further improved performance of collider conversion for all types.

### Fixes

* `JointData.Version` was not being incremented with changes to its properties.
* Fixed the issue of uninitialized array when scheduling collision event jobs with no dynamic bodies in the scene.
* Fixed the issue of `CollisionEvent.CalculateDetails()` reporting 0 contact points in some cases.
* Fixed the issue of joints with enabled collisions being solved after contacts for the same body pair.
* Fixed exception and leak caused by trying to display a convex hull preview for a physics shape with no render mesh assigned.
* Fixed bug causing incremental changes to a compound collider to accumulate ghost colliders when using Live Link.
* Fixed an issue where kinematic (i.e. infinite mass dynamic) bodies did not write to the collision event stream correctly.

### Known Issues

* Compound collider mass properties are not correctly updated while editing using Live Link.


## [0.2.4-preview] - 2019-09-19

### Changes

* Updated dependency on `com.unity.entities` to version `0.1.1-preview`. If you need to stay on entities version `0.0.12-preview.33` then you can use the previous version of this package, `0.2.3-preview`, which is feature equivalent:



## [0.2.3-preview] - 2019-09-19

### Upgrade guide

* Implicitly static shapes (i.e. those without a `PhysicsBodyAuthoring` or `Rigidbody`) in a hierarchy under a GameObject with `StaticOptimizeEntity` are now converted into a single compound `PhysicsCollider` on the entity with the `Static` tag. If your queries or contact events need to know about data associated with the entities from which these leaf shapes were created, you need to explicitly add `PhysicsBodyAuthoring` components with static motion type, in order to prevent them from becoming part of the compound.

### Changes

* Run-Time API
    * Deprecated `Broadphase.ScheduleBuildJobs()` and provided a new implementation that takes gravity as input.
    * Deprecated `CollisionWorld.ScheduleUpdateDynamicLayer()` and provided a new implementation that takes gravity as input.
    * Deprecated `PhysicsWorld.CollisionTolerance` and moved it to `CollisionWorld`.
    * Deprecated `ManifoldQueries.BodyBody()` and provided a new implementation that takes two bodies and two velocities.
    * Added `CollisionEvent.CalculateDetails()` which provides extra information about the collision event:
        * Estimated impulse
        * Estimated impact position
        * Array of contact point positions
    * Removed `CollisionEvent.AccumulatedImpulses` and provided `CollisionEvent.CalculateDetails()` which gives a more reliable impulse value.

* Authoring/Conversion API
    * Removed the following expired ScriptableObject:
        * `CustomFlagNames`
    * Removed the following expired members:
        * `PhysicsShapeAuthoring.GetBelongsTo()`
        * `PhysicsShapeAuthoring.SetBelongsTo()`
        * `PhysicsShapeAuthoring.GetCollidesWith()`
        * `PhysicsShapeAuthoring.SetCollidesWith()`
        * `PhysicsShapeAuthoring.OverrideCustomFlags`
        * `PhysicsShapeAuthoring.CustomFlags`
        * `PhysicsShapeAuthoring.GetCustomFlag()`
        * `PhysicsShapeAuthoring.SetCustomFlag()`
        * `PhysicsMaterialTemplate.GetBelongsTo()`
        * `PhysicsMaterialTemplate.SetBelongsTo()`
        * `PhysicsMaterialTemplate.GetCollidesWith()`
        * `PhysicsMaterialTemplate.SetCollidesWith()`
        * `PhysicsMaterialTemplate.CustomFlags`
        * `PhysicsMaterialTemplate.GetCustomFlag()`
        * `PhysicsMaterialTemplate.SetCustomFlag()`

* Run-Time Behavior
    * Gravity is now applied at the beginning of the step, as opposed to previously being applied at the end during integration.

* Authoring/Conversion Behavior
    * Implicitly static shapes in a hierarchy under a GameObject with `StaticOptimizeEntity` are now converted into a single compound `PhysicsCollider`.

### Fixes

* Fixed issues preventing compatibility with DOTS Runtime.
* Fixed occasional tunneling of boxes through other boxes and mesh triangles.
* Fixed incorrect AABB sweep direction during collisions with composite colliders, potentially allowing tunneling.
* Fixed obsolete `MeshCollider.Create()` creating an empty mesh.
* Fixed obsolete `ConvexCollider.Create()` resulting in infinite recursion.
* Fixed simplex solver bug causing too high output velocities in 3D solve case.
* Fixed bug causing custom meshes to be ignored on convex shapes when the shape's transform was bound to a skinned mesh.
* Fixed Burst incompatibilities in the following types:
    * `BoxGeometry`
    * `CapsuleGeometry`
    * `CollisionFilter`
    * `ConvexHullGenerationParameters`
    * `CylinderGeometry`
    * `Material`
    * `SphereGeometry`
* Improved performance of `MeshConnectivityBuilder.WeldVertices()`.
* Fixed a potential assert when joints are created with a static and dynamic body (in that order).

### Known Issues



## [0.2.2-preview] - 2019-09-06

### Fixes

* Added internal API extensions to work around an API updater issue with Unity 2019.1 to provide a better upgrading experience.



## [0.2.1-preview] - 2019-09-06

### Upgrade guide

* A few changes have been made to convex hulls that require double checking convex `PhysicsShapeAuthoring` components:
    * Default parameters for generating convex hulls have been tweaked, which could result in minor differences.
    * Bevel radius now applies a shrink to the shape rather than an expansion (as with primitive shape types).
* Mesh `PhysicsShapeAuthoring` objects with no custom mesh assigned now include points from enabled mesh renderers on their children (like convex shapes). Double check any mesh shapes in your projects.
* Due to a bug in version 0.2.0, any box colliders added to uniformly scaled objects had their scale baked into the box size parameter when initially added and/or when fit to render geometry. Double check box colliders on any uniformly scaled objects and update them as needed (usually by just re-fitting them to the render geometry).
* The serialization layout of `PhysicsShapeAuthoring` has changed. Values previously saved in the `m_ConvexRadius` field will be migrated to `m_ConvexHullGenerationParameters.m_BevelRadius`, and a `m_ConvexRadius_Deprecated` field will then store a negative value to indicate the old data have been migrated. Because this happens automatically when objects are deserialized, prefab instances may mark this field dirty even if the prefab has already been migrated. Double check prefab overrides for Bevel Radius on your prefab instances.

### Changes

* Run-Time API
    * Added the following new members:
        * `BodyIndexPair.IsValid`
        * `Math.Dotxyz1()` using double
        * `Plane.Projection()`
        * `Plane.SignedDistanceToPoint()`
    * Added the following structs:
        * `BoxGeometry`
        * `CapsuleGeometry`
        * `CylinderGeometry`
        * `SphereGeometry`
        * `ConvexHullGenerationParameters`
    * `Constraint` now implements `IEquatable<Constraint>` to avoid boxing allocations.
    * All previous `SphereCollider`, `CapsuleCollider`, `BoxCollider` and `CylinderCollider` properties are now read only. A new `Geometry` property allows reading or writing all the properties at once.
    * `BoxCollider.Create()` now uses `BoxGeometry`. The signature passing nullable types has been deprecated.
    * `CapsuleCollider.Create()` now uses `CapsuleGeometry`. The signature passing nullable types has been deprecated.
    * `ConvexCollider.Create()` now uses `ConvexHullGenerationParameters`. The signature passing nullable types has been deprecated.
    * `CylinderCollider.Create()` now uses `CylinderGeometry`. The signature passing nullable types has been deprecated.
    * `MeshCollider.Create()` now uses native containers. The signature using managed containers has been deprecated.
    * `PolygonCollider.CreateQuad()` signature passing nullable types has been deprecated.
    * `PolygonCollider.CreateTriangle()` signature passing nullable types has been deprecated.
    * `SphereCollider.Create()` now uses `SphereGeometry`. The signature passing nullable types has been deprecated.
    * `TerrainCollider.Create()` signature passing pointer and nullable types has been deprecated.
    * `SimplexSolver.Solve()` taking the `respectMinDeltaTime` has been deprecated. Use the new `SimplexSolver.Solve()` method that takes `minDeltaTime` instead.
    * Renamed `BoxCollider.ConvexRadius` to `BevelRadius`.
    * Renamed `CylinderCollider.ConvexRadius` to `BevelRadius`.
    * Deprecated `SimplexSolver.c_SimplexSolverEpsilon`.
    * Deprecated the following methods in `ComponentExtensions` taking an `Entity` as the first argument.
        * `GetCollisionFilter()`
        * `GetMass()`
        * `GetEffectiveMass()`
        * `GetCenterOfMass()`
        * `GetPosition()`
        * `GetRotation()`
        * `GetVelocities()`
        * `SetVelocities()`
        * `GetLinearVelocity()`
        * `SetLinearVelocity()`
        * `GetAngularVelocity()`
        * `SetAngularVelocity()`
        * `ApplyImpulse()`
        * `ApplyLinearImpulse()`
        * `ApplyAngularImpulse()`
    * Removed the following expired members:
        * `ColliderCastInput.Direction`
        * `ColliderCastInput.Position`
        * `Ray(float3, float3)`
        * `Ray.Direction`
        * `Ray.ReciprocalDirection`
        * `RaycastInput.Direction`
        * `RaycastInput.Position`
* Authoring/Conversion API
    * Renamed `PhysicsBody` to `PhysicsBodyAuthoring`.
    * Renamed `PhysicsShape` to `PhysicsShapeAuthoring`.
    * `PhysicsShapeAuthoring.BevelRadius` now returns the serialized bevel radius data in all cases, instead of returning the shape radius when the type was either sphere or capsule, or a value of 0 for meshes. Its setter now only clamps the value if the shape type is box or cylinder.
    * `PhysicsShapeAuthoring.GetBoxProperties()` now returns `BoxGeometry`. The signature containing out parameters has been deprecated.
    * `PhysicsShapeAuthoring.SetBox()` now uses `BoxGeometry`. The signature passing individual parameters has been deprecated.
    * `PhysicsShapeAuthoring.GetCapsuleProperties()` now returns `CapsuleGeometry`. The signature containing out parameters has been deprecated.
    * `PhysicsShapeAuthoring.SetCapsule()` now uses `CapsuleGeometry`. The signature passing individual parameters has been deprecated.
    * `PhysicsShapeAuthoring.GetCylinderGeometry()` now returns `CylinderGeometry`. The signature containing out parameters has been deprecated.
    * `PhysicsShapeAuthoring.SetCylinder()` now uses `CylinderGeometry`. The signature passing individual parameters has been deprecated.
    * `PhysicsShapeAuthoring.GetSphereProperties()` now returns `SphereGeometry`. The signature containing out parameters has been deprecated.
    * `PhysicsShapeAuthoring.SetSphere()` now uses `SphereGeometry`. The signature passing individual parameters has been deprecated.
    * `PhysicsShapeAuthoring.SetConvexHull()` now uses `ConvexHullGenerationParameters`. The old signature has been deprecated.
    * `PhysicsShapeAuthoring.ConvexRadius` has been deprecated. Instead use `BevelRadius` values returned by geometry structs.
    * `PhysicsShapeAuthoring.GetConvexHullProperties()` now returns points from `SkinnedMeshRenderer` components that are bound to the shape's transform, or to the transforms of its children that are not children of some other shape, when no custom mesh has been assigned.
    * Added `PhysicsShapeAuthoring.SetConvexHull()` signature specifying minimum skinned vertex weight for inclusion.
    * Added `PhysicsShapeAuthoring.ConvexHullGenerationParameters` property.
    * `PhysicsShapeAuthoring.GetMesh()` has been deprecated.
    * Added `PhysicsShapeAuthoring.GetMeshProperties()`. When no custom mesh has been assigned, this will return mesh data from all render geometry in the shape's hierarchy, that are not children of some other shape.
    * `PhysicsShapeAuthoring.FitToEnabledRenderMeshes()` now takes an optional parameter for specifying minimum skinned vertex weight for inclusion.
    * Removed the `OutputStream` field from various deprecated debug drawing jobs. These will be redesigned in a future release, and you are advised to not try to extend them yet.
    * Removed the following expired members:
        * `PhysicsShapeAuthoring.FitToGeometry()`.
        * `PhysicsShapeAuthoring.GetCapsuleProperties()` returning raw points.
        * `PhysicsShapeAuthoring.GetPlaneProperties()` returning raw points.
        * `FirstPassLegacyRigidbodyConversionSystem`.
        * `FirstPassPhysicsBodyConversionSystem`.
        * `SecondPassLegacyRigidbodyConversionSystem`.
        * `SecondPassPhysicsBodyConversionSystem`.
* Run-Time Behavior
    * `BoxCollider.Create()` is now compatible with Burst.
    * `CapsuleCollider.Create()` is now compatible with Burst.
    * `ConvexCollider.Create()` is now compatible with Burst.
    * `CylinderCollider.Create()` is now compatible with Burst.
    * `MeshCollider.Create()` is now compatible with Burst.
    * `SphereCollider.Create()` is now compatible with Burst.
    * `TerrainCollider.Create()` is now compatible with Burst.
* Authoring/Conversion Behavior
    * Converting mesh and convex shapes is now several orders of magnitude faster.
    * Convex meshes are more accurate and less prone to jitter.
    * `PhysicsShapeAuthoring` components set to convex now display a wire frame preview at edit time.
    * `PhysicsShapeAuthoring` components set to cylinder can now specify how many sides the generated hull should have.
    * Inspector controls for physics categories, custom material tags, and custom body tags now have a final option to select and edit the corresponding naming asset.

### Fixes

* Body hierarchies with multiple shape types (e.g., classic collider types and `PhysicsShapeAuthoring`) now produce a single flat `CompoundCollider` tree, instead of a tree with several `CompoundCollider` leaves.
* Fixed issues causing dynamic objects to tunnel through thin static objects (most likely meshes)
* Fixed incorrect behavior of Constraint.Twist() with limitedAxis != 0
* Fixed regression introduced in 0.2.0 causing box shapes on uniformly scaled objects to always convert into a box with size 1 on all sides.
* Fixed exception when calling `Dispose()` on an uninitialized `CollisionWorld`.

### Known Issues

* Wire frame previews for convex `PhysicsShapeAuthoring` components can take a while to generate.
* Wire frame previews for convex `PhysicsShapeAuthoring` components do not currently illustrate effects of bevel radius in the same way as primitives.
* The first time you convert convex or mesh shapes in the Editor after a domain reload, you will notice a delay while the conversion jobs are Burst compiled. all subsequent conversions should be significantly faster until the next domain reload.
* Updated dependency on `com.unity.burst` to version `1.1.2`.



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
