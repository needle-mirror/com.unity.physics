# Core components

In the [simulation set up](concepts-simulation-set-up.md) section, we've learnt how to setup and configure bodies and shapes. Under the hood, when Unity Physics starts simulating the scene:
1. Several baking systems (`PhysicsBodyBakingSystem`, `PhysicsShapeBakingSystem` and `PhysicsJointConversionSystem`) read the `PhysicsShapeAuthoring`, `PhysicsBodyAuthoring`, joint and motor scripts, bake them to components and add them to your entities.
2. Various physics systems read from these components to generate input to the simulation.

This document describes each of these components, so we can write systems which process these to apply game effects, or even create simulated bodies from code.

| Topic                                             | Description                                                       |
|---------------------------------------------------|-------------------------------------------------------------------|
| [PhysicsCollider](physics-collider-components.md) | Detailed description of PhysicsCollider component.                |
| [Joints](joint-components.md)                     | Detailed description for the different types of Joints component. |
| [Motors](motor-components.md)                     | Detailed description for the different types of Motors component. |
