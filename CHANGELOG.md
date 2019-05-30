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
