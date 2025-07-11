# The Simulation Pipeline

The Unity Physics simulation pipeline is comprised of the following stages executed in the given order.

1. __Physics World Building:__ The physics system gets the current state from the components on the body entities and builds the underlying _Physics World_, which contains integral parts for the following stages. This must happen first, because the simulation is "stateless" (that is, it does not cache anything frame-to-frame).
The Physics World is made up of two parts:
    * The Collision World, which stores information used for the Collision Detection stages (see below) and for user collider queries such as raycasts or distance queries.
    * The Dynamics World, which stores information used for the Constraint Solver, such as joints and mass properties for dynamic rigid bodies.
In the Physics World Building stage, both these parts are constructed.
2. __Collision Detection Broadphase :__ The physics system runs the broadphase of the collision detection. In this phase the simulation uses high-level collider information of all active rigid bodies in the scene and quickly identifes which pairs of colliders are potentially colliding. Through fast overlap checks of the colliders' bounding volumes (axis-aligned bounding boxes, or AABBs), this phase efficiently tests for potential collisions, quickly discarding all other colliders in the scene.
3. __Collision Detection Narrowphase:__ The physics system runs the narrowphase of the collision detection. Given the pairs of potentially colliding colliders computed in the broadphase (see above), in the narrowphase, the simulation determines whether these pairs do in fact collide through precise intersection tests. If a pair is found to collide, the exact points of contact (the intersections) between the corresponding colliders are calculated.
4. __Constraint Solver:__ Based on the collisions determined in the narrowphase, the physics system calculates a collision response for each colliding dynamic rigid body that takes into account the following information:
    * rigid body mass properties (such as scalar mass and moment of inertia)
    * material friction
    * material restitution (bounciness)
    * points of contact

    The constraint solver proceeds to iteratively correct the interpenetrations of all dynamically controlled colliders (dynamic rigid bodies) by applying contact impulses for the identified contact points. It performs similar corrections for joints, correspondingly restricting the motion of any jointed dynamic rigid body. This stage produces new linear and angular velocities for the affected rigid bodies.
6. __Integration:__ The physics system integrates all dynamic bodies forward in time by moving the dynamic rigid bodies according to their newly calculated linear and angular velocities and the current time step, creating new positions and orientations.
7. __Export:__ Finally, the physics system applies each rigid body's new position and orientation to the Entity that represents that rigid body.

## The Collision World
The Collision World, created during the Physics World Building stage, contains all rigid bodies that have a collider. It also contains dedicated spatial acceleration structures, required for the efficient determination of potentially overlapping collider pairs during the __Collision Detection Broadphase__ stage. This structure, a bounding volume hierarchy (BVH), spatially organizes the bounding volumes of all colliders into a tree for fast collision detection and user collider queries.

Unity Physics uses axis-aligned bounding boxes (AABBs) as rigid body bounding volumes in the BVH for reduced memory consumption and faster overlap tests while obtaining a sufficiently accurate approximation of the space occupied by a rigid body. See the following figure for how this bounding volume type compares to other types.
![](images/bounding-volumes.png)<br/>_Types of bounding volumes: bounding sphere, axis-aligned bounding box (AABB), oriented bounding box (OBB), eight-direction discrete orientation polytope (8-DOP), and convex hull. [Reference source](https://www.researchgate.net/figure/Bounding-volumes-sphere-axis-aligned-bounding-box-AABB-oriented-bounding-box_fig9_272093426)._

The bounding volume hierarchy in the broadphase can either be re-calculated from scratch or updated incrementally. When the broadphase is updated incrementally, only the rigid body collider changes (such as changes in transformation or size), detected between simulation steps, are incorporated into the BVH. This approach can significantly reduce the time consumption for the broadphase update in cases where the majority of colliders does not change in any given simulation step. By default the broadphase is updated from scratch. Incremental broadphase updates can be enabled using the [Physics Step authoring component](component-step.md).

Two independent BVHs are constructed during the Physics World Building stage, one for static and one for dynamic rigid bodies. For static rigid bodies, the BVH (the static broadphase) is only updated when a change to any static rigid body is detected, regardless of whether the static broadphase is updated incrementally or not. The BVH for dynamic rigid bodies is always updated, as these bodies are assumed to move most of the time. While this approach allows scenes with a large number of static rigid bodies, if any of these rigid bodies moves in any given step, a very large time consumption for the static broadphase update can occur during the Physics World Building stage. To prevent a performance hit in such cases, the static broadphase update should be set to incremental as described above.