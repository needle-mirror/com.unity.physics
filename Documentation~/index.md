# Unity Physics overview

The Unity Physics package, part of Unity's Data-Oriented Technology Stack (DOTS), provides a deterministic rigid body dynamics system and spatial query system.

See the [Unity Physics Samples](https://github.com/Unity-Technologies/EntityComponentSystemSamples/tree/master/PhysicsSamples) for introductory material, including tutorials, samples, and videos.

![](images/entities-splash-image.png)

## Package installation

To use the Unity Physics package, you must have Unity version 2022.2.0b8 and later installed.

To install the package, open the Package Manager window (**Window &gt; Package Manager**) and perform one of the following options:

* [Add the package by its name](xref:upm-ui-quick)
* [Add the package from its Git URL](xref:upm-ui-giturl)

## Known issues

* Handles Gizmos in the scene window do not follow physics bodies when sub scene is open during play mode time, this results with an incorrect respawning position after simulation. Either converting game objects to prefabs or close the sub scene before running the scene.
* Physics colliders gizmos are not being displayed within a sub scene during the edit mode. Physics colliders are displayed outside sub scenes.
* Compound collider gets created even if children are disabled in Editor's game object hierarchy.
* Mesh collider types contacting results in a wrong behaviour by getting glued to each other as if they were joints.
* Enabling the `Draw Collider` toggle button within the `Physics Debug Display` results in errors being displayed in the console window and breaking the scene.
* Mesh collider simplification: if primitive scale is smaller than 0.018f, selecting entities in the scene view throws error.
* Selecting entities in the scene view throws error “GetAllOverlapping failed”.
* The rotational motor does not behave as expected in some cases (spins unexpectedly or does not move at all).
* Adding a rigidbody to a game object in a sub scene, causes it to disappear from game and scene view.
* Moving the child of a game object in a sub scene results in exceptions, when both parent and child have traditional colliders (e.g. box collider component) but no rigidbodies.
* A GameObject with both non-uniform scale in its `Transform` and its `Physics Body`'s `Smoothing` value of anything other than `None` will throw the following error:
  > InvalidOperationException: Baking error: Attempt to add duplicate component Unity.Transforms.PropagateLocalToWorld for Baker PhysicsBodyAuthoringBaker with authoring component PhysicsBodyAuthoring.  Previous component added by Baker PhysicsBodyAuthoringBaker
  
  This will be fixed before the final 1.0 release. In the meantime, the suggested workaround is to temporarily set `Smoothing` to `None`. This may result in visual choppiness when rendering entities, but they will still be simulated correctly.
  
## Additional resources

* [Getting started](getting-started.md)
* [Upgrade guide](upgrade-guide.md)
* [What's new](whats-new.md)
* [ECS packages](ecs-packages.md)
