# Overriding intermediate simulation results

> **Note:** Hooks for simulation modification are a work in progress. They may be incomplete and the interfaces are very likely to change between releases. The [Unity Physics Samples](https://github.com/Unity-Technologies/EntityComponentSystemSamples/blob/master/UnityPhysicsSamples/Documentation/samples.md) project has one scene for each type of modification and it may be best to refer to those in order to see how the current interface behaves and what the API is.

You can usually consider the physics simulation as a monolithic process whose inputs are components described in [core components](core_components.md) and outputs are updated `Translation`, `Rotation` and `PhysicsVelocity` components. However, internally, the Physics step is actually broken down into smaller subsections, the output of each becomes the input to the next. Unity Physics currently gives you the ability to read and modify this data, if it is necessary for your gameplay use cases.

This works by injecting jobs into the task dependencies for the physics world step. To make this easy to do without modifying the physics code, Unity Physics allows you to request a delegate to be called when physics is setting up the jobs. For each type of simulation result you wish to read or modify, you should implement the `Physics.SimulationCallbacks.Callback` delegate. This function receives a reference to all previously scheduled physics tasks as well as a reference to the simulation interface. In here, you should schedule any jobs you need and return the combined `JobHandles` of any jobs you schedule – this will prevent future physics tasks from processing data before you are done with it.

Then, before _every step_ you should inform the physics simulation that you wish your function to be called; you do this by calling `Unity.Physics.Systems.StepPhysicsWorld.EnqueueCallback()`, passing the time you wish your function to be called (which affects the kind of data you can access) where it will later be called at an appropriate time. The reason why Unity Physics requires you to do this every frame is that you might have some modifications you wish to enable temporarily, based on the presence or properties of some other gameplay component.

# Simulation phases

This section briefly describes the phases of the simulation and what you can access and change.

### Phase 1 – _PostCreateDispatchPairs_
`Unity.Physics.SimulationCallbacks.Phase.PostCreateDispatchPairs`

**Status** – At this point, the simulation has generated the list of _potentially_ interacting objects, based on joints and the overlaps at the _broadphase_ level. It has also performed some sorting, making the pairs of interacting objects suitable for multithreading.

**What you can do** – You can schedule an implementation of `IBodyPairsJob` to get access to all of these body pairs. If for some reason you wish to disable an interaction, you can do it during this job.

>**Note:** from both a performance and debugging perspective, it is preferable to disable pairs using a collision filter.

### Phase 2 – _PostCreateContacts_
`Unity.Physics.SimulationCallbacks.Phase.PostCreateContacts`

**Status** – At this point, the simulation has performed the low-level collision information between every pair of bodies by inspecting their colliders.

**What you can do** – At this point you can schedule an implementation of `IContactsJob` to access that information. The data is in the format of a `ModifiableContactHeader` containing some shared properties, followed by a number (`ModifiableContactHeader.NumContacts`) of `ModifiableContactPoint` contact points. You can modify or disable the contacts, for example you can change the separating normal or the contact positions and distances.

### Phase 3 – _PostCreateContactJacobians_
`Unity.Physics.SimulationCallbacks.Phase.PostCreateContactJacobians`

**Status** – Now, the joint data from your Joint components and contact data from the previous step has been converted to data which is suitable for consumption by the constraint solver. It represents the same as it did before, but has been transformed into a form which optimizes the simulation's ability to solve the constraints accurately – so, the data appears to be significantly more abstract and harder to manipulate.

**What you can do** – You can schedule an implementation of `IJacobiansJob` to get access to each of the jacobian datas. Since the data is harder to manage, Unity Physics provides some accessors to perform many reasonable modifications. For these, it is best to consult the sample scene.
