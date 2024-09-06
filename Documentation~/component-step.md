# The Physics Step authoring component 

To control Unity Physics settings, you need a `Physics Step` authoring component. As usual when working with **Entities**, a **SubScene** is necessary when adding the `Physics Step` authoring component. Only one instance of this component should be present in a scene as it will apply to the whole physics simulation.

This component allows configuring various aspects of the simulation via the following fields.

| Field                                         | Description |
|-----------------------------------------------|-------------|
| Simulation Type                               | Select between **Unity Physics**, **Havok Physics** or **None** as physics engine. Unity Physics is the default engine.<br>You can only access the Havok Physics features if you have installed the Havok Physics package and you have a license. |
| Gravity                                       | Set the global gravity applied to all dynamic rigid bodies in the physics world.<br>Note that you can use the **PhysicsGravityFactor** ECS data component to modify the gravity applied to individual rigid bodies, as a multiple of the global gravity specified here. |
| Solver Iteration Count                        | Specify the number of solver iterations the physics system performs to correct contact penetrations and joint errors. Higher values provide higher accuracy and stability, but can reduce simulation performance. |
| Multi Threaded                                | Enable multi-threading.<br>If enabled, the physics system maximizes the number of threads used for calculating the simulation results. If disabled, the physics system reduces the number of utilized threads to a minimum, performing most operations in a single thread. |
| Collision Tolerance                           | Set the collision tolerance.<br> The collision tolerance specifies the minimum distance required for contacts between rigid bodies to be created. This value can be increased if undesired collision tunneling is observed in the simulation. |
| Incremental Dynamic Broadphase                | Enable the incremental dynamic broadphase.<br>Enabling this option will update the dynamic broadphase incrementally whenever changes between simulation steps occur, potentially leading to time savings for cases with many dynamic rigid bodies that don't move or otherwise change. |
| Incremental Static Broadphase                | Enable the incremental static broadphase.<br> Enabling this option will update the static broadphase incrementally whenever changes between simulation steps occur, potentially leading to time savings for cases with many static rigid bodies that don't move or otherwise change. |
| Synchronize Collision World                   | Specify whether to update the collision world after the step for more precise raycast, collider and distance query results. |
| Enable Contact Solver Stabilization Heuristic | Enabling the contact solver stabilization heuristic results in better simulation stability when stacking objects with low solver iteration counts, by preventing undesired sliding artifacts. However, it can reduce simulation performance and produce unplausible results in certain physical interactions involving friction forces. |

![collider_cast](images/physics-step.png)<br/>_The Physics Step authoring component provides access to various simulation configuration settings._
