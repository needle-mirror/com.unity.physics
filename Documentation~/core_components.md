In the [getting started](getting_started.md) section, we discussed how to setup and configure bodies and shapes. Under the hood, when we start simulating the scene, several conversion systems (`PhysicsBodyConversionSystem`, `PhysicsShapeConversionSystem` and `PhysicsJointConversionSystem`) read the `PhysicsShape`, `PhysicsBody` and joint scripts (there are several types of joint script), convert them to components and add them to your entities. Various physics systems read from these components to generate input to the simulation. This document discusses each of these components, so you can write systems which process these to apply game effects, or even create simulated bodies from code.

## Collider `PhysicsCollider`

This is the most important component for the simulation of physics. By adding this component to an entity, you declare that this body will participate in the physics simulation and collision queries (though, we also need a Translation and Rotation component). This component decides what the collision geometry "looks" like to the physics simulation. This component is analogous to a mesh in a rendering system. For performance reasons, we try to avoid using a mesh during physics - we use specialized primitive types when possible, which greatly simplifies the process of determining collisions between two colliders. For example, if a collision geometry can be represented by a sphere, we can write collision tests which only need to consider the sphere center and radius; with a mesh wrapping the same sphere, we would need to consider every triangle in the
mesh.

The most important property of a `PhysicsCollider` is a `BlobAssetReference` to the collider data used by the physics simulation (which is in a format optimized for collision queries) - each of our collider types implement an `ICollider` interface, and for each collider type, we have a static `Create()` function, which takes parameters specific to the shape -- for example , a `SphereCollider` is built from a position and a radius, while a `ConvexCollider` is built from a point cloud.

_Note_ that it is possible to create a collider which has a null `BlobAssetReference``. We'll still simulate the body, but it cannot collide with anything. This can be useful in some particular scenarios, for example you can connect such a body to another body using a joint, and feed the simulated positions into your skinning system for example.

### Collider materials

The collider also stores a material, which describes how it reacts when in collision with other objects.

The [restitution](https://en.wikipedia.org/wiki/Coefficient_of_restitution) of a material determines how "bouncy" the material is - how much of a body's velocity is preserved when it collides with another. A value of zero indicates that the object should not bounce at all, while a value of one indicates that it should all the speed of the object should be preserved. _Note_ that due to numerical precision and approximations inside the physics simulation, a body with a restitution of one will eventually come to rest.

The [coefficient of friction](https://en.wikipedia.org/wiki/Friction) of a body relates how "sticky" an object is when another object is sliding along it's surface. This is the ratio between the force pushing down on the surface and the force pushing against the relative velocities between the bodies. A value of zero means that friction would not slow down the body, while higher values indicate that more energy should be lost.

Both friction and restitution have a `CombinePolicy` which determines how the engine should merge two different values. For example, you may want to always use the largest or smallest value in a collision.

In addition to these, a material also has a set of flags, which enable special behaviors during the physics simulation. The two most important of these are:

* `IsTrigger`: if this flag is enabled, the collider is treated as a "detector" rather than as a physical body. This means it cannot receive forces from a collision, instead, it will raise an event to signify that an overlap occurred. For example, you can use this to determine when your player enters a specific region.
* `EnableCollisionEvents`: this is similar to the previous flag, but still allows the body to push other bodies normally. The events that the simulation raises, then can be used to determine how objects are colliding -- this would, for example, allow you to play sound events.

### The Collision Filter

Each collider also has a `CollisionFilter` which allows you to control what objects are permitted to collide with each other. The properties on this object allow you to categorize objects in relation to what types they collide with. For example, you might want to mark certain colliders as "transparent" so that when performing a raycast test to determine if two characters can see each other, they are able to "see through" colliders which have the transparent bit set.

The default values for collision filter ensure that every object collides with every other object. By configuring the filter in particular ways, you are able to opt-out of select collisions, depending on what you want from gamecode.

# Dynamic bodies

By itself, a world containing entities with `PhysicsCollider` components won't actually _do_ anything. This is because the bodys we declared are all treated as static -- they cannot move, and from the perspective of collision, they have infinite mass. In order to make our simulations more interesting, we'll want to add the ability for body transforms to change.

Adding a `PhysicsVelocity` component makes the physics simulation aware that the collider can have some linear and angular speed and that the collider should move. You can change the values of this component yourself if you wish to control how a collider is moving, but during the physics simulation, we also compute a new value for velocity (from gravity and forces from contacts/joints) and update the component.

# Mass

So now, suppose you have a scene with two physics colliders; one of which has a velocity pointing towards the static collider. When you press play, the moving collider moves right through the static one. You haven't changed the collision filter, or made one a trigger, so what happened? This is a case of an unstoppable force meeting an immovable object. As we discussed before, the collider without a velocity cannot move. When the two collide, we would expect forces to be applied between them.

The problem is that, even though one collider is moving, the simulation does not know how _heavy_ it is, so does not know how it would respond to collisions. In this case, we treat the moving collidable as if it had _infinite_ mass, so it will just push every object out of the way.

This kind of behaviour is useful in some scenarios. Suppose you had an elevator object in-game. It makes sense that it should follow your desired path _exactly_ -- it should move no matter how many characters are in the lift and it should not get stuck on "snags" inside a bumpy elevator shaft. This behaviour is sometimes called "kinematic" or "keyframed" in other physics simulations.

To inform the simulation of masses, we use the `PhysicsMass` component. This tells physics how an collider reacts to an impulse. It stores the mass and [inertia tensor](https://en.wikipedia.org/wiki/Moment_of_inertia) for that entity as well as a transform describing the orientation of the inertia tensor and center of mass.

_Note_ some of these values are stored as inverses, which speeds up many of the internal physics calculations. It also allows you to specify infinite values, by setting the relevant component to zero.

While you can provide these values yourself, it is not necessary in many cases; the `ICollider` for a collider has a `MassProperties` property, where appropriate values are calculated for you automatically. You might find it more useful to use the calculated `MassProperties` as a starting point, and then scale them -- for example, by multiplying the mass by ten for an extra-heavy gameplay object.



# Other components

Some additional components allow you to change the simulation behavior:

* `PhysicsStep` allows you to overide the default simulation parameters for the whole scene, for example, by changing the direction of gravity or increasing the solver iteration count, making it more rigid (at the cost of performance)
* `PhysicsDamping` allows you to add a per-collider "slow-down" factor. Every step, a collider with this component will have it's velocities scaled down. This could slow down objects, making them more stable, or be used as a cheap approximation of aerodynamic drag.
* `PhysicsGravityFactor` allows you to scale the amount of gravity applied to an individual collider. Some objects _feel_ better if they appear to fall faster. Some objects (for example, hot air balloons) appear to fall _up_, which can be emulated with a negative gravity factor.


# Joints

A `PhysicsJoint` component is a little different from the others described here. It _links_ two entities together with some sort of constraint, for example, a door hinge. These entities should have physics collider components, and at least one of them should have a velocity -- otherwise the joint would have no effect. During the physics step, we solve the joint as well as the contacts that affect each entity.

The behavior of the joint is described by the `JointData` property; like the collider component, this is a `BlobAssetReference` to a `JointData`. The precise behavior of each joint depends on the type of this data. We have pre-created several types of joint, but do not yet have a complete repetroire of joints -- as such, some of this information is likely to change in the near future. For the joint types which are implemented, there are static creation functions in `Physics.JointData` and, like with shapes, the input parameters vary between different types. For example, the `CreateBallAndSocket` simply needs to know where the joint is located relative to each body, while `CreateLimitedHinge()` additionally needs to know what axis the bodies are permitted to rotate about, and what the minimum and maximum limit for this rotation is.

In addition to specifying the joint data and the entities, an important setting is `EnableCollision` -- this defaults to _off_, which is the recommended setting. If you have two bodies constrained together (such as a door, attached to a car) it is very likely that they overlap each other to some degree. When this happens, you can imagine that the joint pulls the objects together, while the collision detection is pushing them apart. This leads to "fighting" between the joint and collision, resulting in unstable simulation or too many events being raised. When `EnableCollision` is _off_, the physics simulation will not perform collision detection between the two bodies, even if the collider's collision filter would normally say they should collide. _Note_ if you have multiple joints between a pair of bodies, collisions will be enabled if _any_ of the joints have requested collision.

[Back to Index](index.md)
