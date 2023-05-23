# Unity Physics overview

The Unity Physics package, part of Unity's Data-Oriented Technology Stack (DOTS), provides a deterministic rigid body dynamics system and spatial query system.

See the [Unity Physics Samples](https://github.com/Unity-Technologies/EntityComponentSystemSamples/tree/master/PhysicsSamples) for introductory material, including tutorials, samples, and videos.

![](images/entities-splash-image.png)

## Package installation

To use the Unity Physics package, you must have Unity version 2022.2.0b8 and later installed.

To install the package, open the Package Manager window (**Window &gt; Package Manager**) and perform one of the following options:

* [Add the package by its name](xref:upm-ui-quick) (com.unity.physics)
* [Add the package from its Git URL](xref:upm-ui-giturl)

## Known issues

* Compound collider gets created even if children are disabled in Editor's game object hierarchy.
* Mesh collider types contacting results in a wrong behaviour by getting glued to each other as if they were joints.
* Mesh collider simplification: if primitive scale is smaller than 0.018f, selecting entities in the scene view throws error.
* Moving the child of a game object in a sub scene results in exceptions, when both parent and child have traditional colliders (e.g. box collider component) but no rigidbodies.

## Additional resources

* [Getting started](getting-started.md)
* [Upgrade guide](upgrade-guide.md)
* [What's new](whats-new.md)
* [ECS packages](ecs-packages.md)
