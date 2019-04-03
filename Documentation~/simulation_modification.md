# Overriding intermediate simulation results

_Note_ hooks for simulation modification are a work in progress. They may be incomplete and the interfaces are very likely to change between releases. The examples project has one scene for each type of modification and it may be best to refer to those in order to see how the current interface behaves and what the API is.

You can usually consider the physics simulation as a monolithic process whose inputs are components described in [core components](core_components.md) and outputs are updated `Translation`, `Rotation` and `PhysicsVelocity` components. However, internally, the Physics update is broken down into smaller subsections, the output of each becomes the input to the next. We have added the ability for you to read and modify this data, if necessary for your gameplay use-cases.

In all cases, you should insert a job into the task dependencies for the physics world step. To make this easy to do without modifying the physics code, we allow you to request a delegate to be called when physics is setting up the jobs For each type of simulation result you wish to read or modify, you should implement the `Physics.SImulationCallbacks.Callback` delegate; this function receives a reference to all previously scheduled physics tasks as well as a reference to the simulation interface. In here, you should schedule any jobs you need and return the combined JobHandles of any jobs you schedule -- this will prevent future physics tasks from processing data before you are done with it.

Then, _every step_ you should inform the physics simulation that you wish your function to be called; you do this by calling `Unity.Physics.Systems.StepPhysicsWorld.EnqueueCallback`, passing the time you wish your function to be called (which affects the kind of data you can access) where it will later be called at an appropriate time. The reason we require you to do this every frame is that you may have some modifications you wish to enable temporarily, based on the presence or properties of some other gameplay component.

# Simulation phases

This section briefly describes the sections of the simulation, what you can access and change.

#### `Unity.Physics.SimulationCallbacks.Phase.PostCreateDispatchPairs`

At this point, the simulation has generated the list of _potentially_ interacting objects, based on joints and the overlaps at the broadphase level. It has also performed some sorting, making the pairs of interacting objects suitable for multithreading.

During this stage, you can use a `BodyPairs.Iterator` from `Simulation.BodyPairs` to see all these pairs. If, for some reason you wish to disable an interaction, the iterator is able to disable a pair before any processing is done.

_Note_ from both a performance and debugging perspective, it is preferable to disable pairs using a collision filter.

#### `Unity.Physics.SimulationCallbacks.Phase.PostCreateContacts`

At this point, we have performed the low-level collision information between every pair of bodies by inspecting their colliders. You can access this through `Simulation.Contacts` where the data is in the format of a `ContactManifold` containing some shared properties, followed by a number (`ContactManifold.NumContacts`) of `ContactPoint` contact points.

Here, you can modify the contacts, changing the separating normal or the contact positions and distances.

It is also possible to _add_ contacts, which might be useful, for example, if you want to deliberately add "invisible walls" to some regions.

#### `Unity.Physics.SimulationCallbacks.Phase.PostCreateContactJacobians`

Now, the joint data from your joint components and contact data from the previous step has been converted to data which is suitable for consumption by the constraint solver. It represents the same as it did before, but has been transformed into a form which optimizes our ability to solve the constraints accurately -- so, the data appears to be significantly more abstract and harder to manipulate.

Here, you can access the `SimulationData.Jacobians.Iterator` and inspect each of the jacobian datas. Since the data is harder to manage, we have provided some accessors to perform many reasonable modifications. For these, it is best to consult the sample scene.

#### `Unity.Physics.SimulationCallbacks.Phase.PostSolveJacobians`

This is nearly the final stage of the physics step. At this point we have solved all the constraints, which provides bodies with new velocities. The last thing to do is to use these velocities to update the positions of all the entities.

This stage gives you an opportunity to modify the velocities before they feed back to the transforms by modifying `PhysicsWorld.MotionVelocities`. _Note_ this allows you to completely override the results of the simulation, so can cause objects to behave non-physically. You might consider using modifying the `PhysicsVelocity` components outside the simulation step, where the simulation will still get a chance to enforce constraints.


[Back to Index](index.md)

