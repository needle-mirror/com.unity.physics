# Principal data components

In the [simulation set up](concepts-simulation-set-up.md) section, we've seen an example of how to set up and configure bodies and shapes. Under the hood, when Unity Physics starts simulating the scene the following happens:
1. Several baking systems read the `built-in` ([`Collider`](xref:Unity.Physics.Collider) and [`Rigidbodies`](xref:Unity.Physics.RigidBody)) or `custom` ( [`PhysicsShapeAuthoring`](custom-shapes.md) and [`PhysicsBodyAuthoring`](custom-bodies.md)) authoring components, bake them into data components and add them to your entities.
2. Various physics systems read from these principal data components to generate input to the simulation.

This section describes each of these components in detail.

| Topic                                      | Description                                                      |
|--------------------------------------------|------------------------------------------------------------------|
| [Rigid bodies](concepts-data.md)           | Detailed description of the data representation of rigid bodies. |
| [Colliders](physics-collider-components.md)| Detailed description of the data representation of colliders.    |
| [Joints](custom-joints.md)                 | Detailed description of the data representation of joints.       |
| [Motors](custom-motors.md)                 | Detailed description of the data representation of motors.       |
