# Physics body data concepts

Unity Physics is based on the [Entity Component System (ECS)](https://docs.unity3d.com/Packages/com.unity.entities@latest). Rigid bodies are represented by component data on the entities within your project. The built-in [`Rigidbody`](xref:Unity.Physics.RigidBody) and [`Collider`](xref:Unity.Physics.Collider) components, and the simplified custom [`Physics Body`](custom-bodies.md) and [`Physics Shape`](custom-shapes.md) component views in the Unity Editor are composed of multiple data components under the hood at runtime. This allows more efficient access and saves space for static bodies which do not require some of the data.

The current set of data components for a rigid body is as follows:

| Component                      | Description                                                                                                                                                                             |
|--------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `PhysicsCollider`              | The shape of the body. Needed for any bodies that can collide.                                                                                                                          |
| `PhysicsColliderKeyEntityPair` | A buffer element used to associate an original Entity with a collider key in a Compound Collider. Only present when the rigid body contains a compound collider.                        |
| `PhysicsCustomTags`            | Custom flags applied to the body. They can be used for certain collision event applications. Assumed to be zero if not present, optional component.                                     |
| `PhysicsDamping`               | The amount of damping to apply to the motion of a dynamic body. Assumed to be zero if not present.                                                                                      |
| `PhysicsGravityFactor`         | The scalar for how much gravity should affect a dynamic body. Assumed to be 1 if not present, optional component.                                                                       |
| `PhysicsMass`                  | The current mass properties (center of mass and inertia) of a dynamic body. Assumed to be infinite mass if not present.                                                                 |
| `PhysicsVelocity`              | The current linear and angular velocities of a dynamic body. Needed for any body that can move.                                                                                         |
| `PhysicsWorldIndex`            | Shared component required on any Entity that is involved in physics simulation (body or joint). Its Value denotes the index of physics world that the Entity belongs to (0 by default). |

All physics bodies require components from `Unity.Transforms` in order to represent their position and orientation in world space. When a GameObject with non-identity scale is baked, the non-identity scale data is baked to a `PostTransformMatrix` component (to preserve the scale of the render mesh at bake time). If a non-uniform scale is present during runtime, then the scale is applied directly to a rigid body. If a GameObject already has scale baked into `PostTransformMatrix`, then the runtime scale will be ignored. Physics ignores any scale of rigid bodies during baking. After a non-uniform scale is baked into `PostTransformMatrix`, the scale is reset to 1.0, relative to this non-uniform scale. This avoids double-scaling within the physics simulation.

Dynamic bodies (i.e., those with `PhysicsVelocity`) require a `LocalTransform` component. Their values are presumed to be in world space. As such, dynamic bodies are unparented during entity baking.
The uniform scale of dynamic bodies can be adjusted at runtime using their `LocalTransform.Scale` value. 

Static bodies (i.e., those with `PhysicsCollider` but without `PhysicsVelocity`) require at least one of either `LocalTransform`, and/or `LocalToWorld`. For static bodies without a `Parent`, physics can read their `LocalTransform` values directly, as they are presumed to be in world space. World space transformations are decomposed from `LocalToWorld` if the body has a `Parent`, using whatever the current value is (which may be based on the results of the transform systems at the end of the previous frame). For best performance and up-to-date results, it is recommended that static bodies do not have a `Parent`.

The [Interacting with bodies](interacting-with-bodies.md) section provides more info on how to interact with Physics Bodies and their data.
