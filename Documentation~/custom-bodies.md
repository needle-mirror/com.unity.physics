# Custom Physics Body Component

By adding a `PhysicsBodyAuthoring` component to a GameObject in the Editor, you can control how it is simulated at run time.

A `PhysicsBodyAuthoring` component has different `MotionTypes`:
* `Static body`: The physics solver doesn't move the rigid body. Unity treats any transforms applied to the body as though it is teleporting.
* `Kinematic body`: The physics solver moves the rigid body according to its velocity, but treats it as though it has infinite mass. It generates a collision response with any rigid bodies that lie in its path of motion, but is not affected by them.
* `Dynamic body`: The physics solver moves the rigid body and handles its collision response with other bodies, based on its physical properties.

The `PhysicsBody` component has the following properties:

| Field                              | Description                                                                                                                                                         |
|------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| MotionType                         | Specifies whether the body should be fully physically simulated (dynamic), moved directly (kinematic), or fixed in place (static)                                   |
| Smoothing                          | Specifies how this body's motion in its graphics representation should be smoothed when the rendering framerate is greater than the fixed step rate used by physics |
| Mass                               | The mass of the body (default is 1.0f)                                                                                                                              |
| Linear Damping                     | This is applied to a body's linear velocity reducing it over time (default is 0.01f)                                                                                |
| Angular Damping                    | This is applied to a body's angular velocity, reducing it over time (default is 0.01f)                                                                              |
| Initial Linear Damping             | The initial linear velocity of the body in world space (default is {0.0f, 0.0f, 0.0f})                                                                              |
| Initial Angular Damping            | This represents the initial rotation speed around each axis in the local motion space of the body i.e. around the center of mass default is {0.0f, 0.0f, 0.0f}      |
| Gravity Factor                     | Scales the amount of gravity to apply to this body, where 1.0f is the default value for -9.8m/s^2                                                                   |
| Advanced                           |                                                                                                                                                                     |
| World Index                        | The index of the physics world this body belongs to. Default physics world has index 0                                                                              |
| Override Default Mass Distribution | Default mass distribution is based on the shapes associated with this body (default is false)                                                                       |
| Custom Tags                        | Specify a custom tag for your body (default is nothing)                                                                                                             |
