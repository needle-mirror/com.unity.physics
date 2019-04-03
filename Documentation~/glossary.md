Glossary
=========

| Term | Description |
|--|---|
|Inertia Tensor| Describes the mass distribtuion of a rigid body. In paricular it effects how the angular velocity of a body can change. See [Moment of inertia](https://en.wikipedia.org/wiki/Moment_of_inertia) |
|Center of mass (COM) | Forces applied to the [center of mass](https://en.wikipedia.org/wiki/Center_of_mass) will push the body in the direction of the force without causing rotation
|Convex shape| A convex shape or volume has the property that a line segment drawn between any two points inside the volume never leaves the volume. We exploit this property to accelerate the performance of collision detection and spatial queries.
| Degree of Freedom (DOF) | A describtion of how a linear system is free to move. In rigid body dynamcis we usualy refer to the 6 degrees of freedom a body has when moving in free space. These are 3 linear and 3 angular degress of freedom. The correspond to the axes you specify when describing a [constraints](joints_and_constraints.md/#Constraint) |
| Convex Radius | To improve the performance and robustness of collision detection we allow contacts to be generated within a small tolerance of the surface of a convex shape. This tolerance is referred to as convex radius |
|Jacobian|Typically describes a constraint used during constraint solving. See [Jaconian]( https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant)|


[Back to Index](index.md)
