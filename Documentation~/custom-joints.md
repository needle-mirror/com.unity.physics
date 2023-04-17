# Joints

A `PhysicsJoint` data component is a little different from the others described here. It _links_ two entities together with some sort of constraint, for example, a door hinge. These entities should have `PhysicsCollider` components, and at least one of them should have a `PhysicsVelocity` component – otherwise the joint would have no effect. During the physics step, Unity Physics solves the joint as well as the contacts that affect each entity.

The behavior of the joint is described by the `JointData` property. Like the `PhysicsCollider` component, this is a `BlobAssetReference` to a `JointData`. The precise behavior of each joint depends on the type of this data.

Unity Physics has currently several pre-created types of joints – and the list will extend in the near future. For the current existing joint types, there are static creation functions in `Unity.Physics.JointData` and, like with shapes, the input parameters vary between different types. For example, the `CreateBallAndSocket` method simply needs to know where the joint is located relative to each body, while `CreateLimitedHinge()` additionally needs to know what axis the bodies are permitted to rotate about, and what the minimum and maximum limit for this rotation is.

Here are the currently available Joint types:

| Joint           | Description                                                                                           |
|-----------------|-------------------------------------------------------------------------------------------------------|
| Ball and Socket | Allows motion around an indefinite number of axes. Humans have such joints in the hips and shoulders. |
| Limited Hinge   | Allows limited articulation on one axis. Humans have such joints in the fingers and knees.            |
| Fixed           | Constrains two rigid bodies together, removing their ability to act independent of each other.        |
| Hinge           | Allows free rotation on one axis. Can be used for spinning wheels and carousels.                      |
| Prismatic       | Constrains two bodies to a sliding motion on one axis. Can be used to make various sliding doors.     |
| Ragdoll         | Limits the motion on a few axes. Useful for creating characters.                                      |
| Stiff Spring    | Constrains two bodies to be a certain distance apart from each other.                                 |

In addition to specifying the joint data and the entities, an important setting is `EnableCollision` – this defaults to _off_, which is the recommended setting. If you have two bodies constrained together (such as a door attached to a car) it is very likely that they overlap each other to some degree. When this happens, you can imagine that the joint pulls the objects together, while the collision detection is pushing them apart. This leads to "fighting" between the joint and collision, resulting in unstable simulation or too many events being raised. When `EnableCollision` is _off_, the physics simulation will not perform collision detection between the two bodies, even if the Collider's collision filter would normally say they should collide.

>**Note:** If you have multiple joints between a pair of bodies, collisions will be enabled if _any_ of the joints have requested collision.
