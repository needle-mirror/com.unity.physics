## [0.0.2] - 2019-04-08

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

## [0.0.1] - 2019-03-12

* Initial package version
