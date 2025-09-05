using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace Unity.Physics
{
    /// <summary>
    /// Holds temporary data in a storage that lives as long as simulation lives and is only re-
    /// allocated if necessary.
    /// </summary>
    public struct SimulationContext : IDisposable
    {
        private int m_NumDynamicBodies;
        private NativeArray<Velocity> m_InputVelocities;

        // Solver stabilization data (it's completely ok to be unallocated)
        [NativeDisableContainerSafetyRestriction]
        private NativeArray<Solver.StabilizationMotionData> m_SolverStabilizationMotionData;
        internal NativeArray<Solver.StabilizationMotionData> SolverStabilizationMotionData => m_SolverStabilizationMotionData.GetSubArray(0, m_NumDynamicBodies);

        internal float TimeStep;

        internal NativeArray<Velocity> InputVelocities => m_InputVelocities.GetSubArray(0, m_NumDynamicBodies);

        internal NativeStream CollisionEventDataStream;
        internal NativeStream TriggerEventDataStream;
        internal NativeStream ImpulseEventDataStream;

        /// <summary>   Gets the collision events. </summary>
        ///
        /// <value> The collision events. </value>
        public CollisionEvents CollisionEvents => new CollisionEvents(CollisionEventDataStream, InputVelocities, TimeStep);

        /// <summary>   Gets the trigger events. </summary>
        ///
        /// <value> The trigger events. </value>
        public TriggerEvents TriggerEvents => new TriggerEvents(TriggerEventDataStream);

        /// <summary>   Gets the impulse events. </summary>
        ///
        /// <value> The impulse events. </value>
        public ImpulseEvents ImpulseEvents => new ImpulseEvents(ImpulseEventDataStream);

        private NativeArray<int> WorkItemCount;

        internal bool ReadyForEventScheduling => m_InputVelocities.IsCreated && CollisionEventDataStream.IsCreated && TriggerEventDataStream.IsCreated && ImpulseEventDataStream.IsCreated;

        /// <summary>
        /// Resets the simulation storage
        /// - Reallocates input velocities storage if necessary
        /// - Disposes event streams and allocates new ones with a single work item
        /// NOTE: Reset or ScheduleReset needs to be called before passing the SimulationContext to a
        /// simulation step job. If you don't then you may get initialization errors.
        /// </summary>
        ///
        /// <param name="stepInput">    The step input. </param>
        public void Reset(SimulationStepInput stepInput)
        {
            m_NumDynamicBodies = stepInput.World.NumDynamicBodies;
            if (!m_InputVelocities.IsCreated || m_InputVelocities.Length < m_NumDynamicBodies)
            {
                if (m_InputVelocities.IsCreated)
                {
                    m_InputVelocities.Dispose();
                }
                m_InputVelocities = new NativeArray<Velocity>(m_NumDynamicBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            // Solver stabilization data
            if (stepInput.SolverStabilizationHeuristicSettings.EnableSolverStabilization)
            {
                if (!m_SolverStabilizationMotionData.IsCreated || m_SolverStabilizationMotionData.Length < m_NumDynamicBodies)
                {
                    if (m_SolverStabilizationMotionData.IsCreated)
                    {
                        m_SolverStabilizationMotionData.Dispose();
                    }
                    m_SolverStabilizationMotionData = new NativeArray<Solver.StabilizationMotionData>(m_NumDynamicBodies, Allocator.Persistent, NativeArrayOptions.ClearMemory);
                }
                else if (m_NumDynamicBodies > 0)
                {
                    unsafe
                    {
                        UnsafeUtility.MemClear(m_SolverStabilizationMotionData.GetUnsafePtr(), m_NumDynamicBodies * UnsafeUtility.SizeOf<Solver.StabilizationMotionData>());
                    }
                }
            }

            if (CollisionEventDataStream.IsCreated)
            {
                CollisionEventDataStream.Dispose();
            }
            if (TriggerEventDataStream.IsCreated)
            {
                TriggerEventDataStream.Dispose();
            }
            if (ImpulseEventDataStream.IsCreated)
            {
                ImpulseEventDataStream.Dispose();
            }

            {
                if (!WorkItemCount.IsCreated)
                {
                    WorkItemCount = new NativeArray<int>(1, Allocator.Persistent);
                    WorkItemCount[0] = 1;
                }
                CollisionEventDataStream = new NativeStream(WorkItemCount[0], Allocator.Persistent);
                TriggerEventDataStream = new NativeStream(WorkItemCount[0], Allocator.Persistent);
                ImpulseEventDataStream = new NativeStream(WorkItemCount[0], Allocator.Persistent);
            }
        }

        // TODO: We need to make a public version of ScheduleReset for use with
        // local simulation calling StepImmediate and chaining jobs over a number
        // of steps. This becomes a problem if new bodies are added to the world
        // between simulation steps.
        // A public version could take the form:
        //         public JobHandle ScheduleReset(ref PhysicsWorld world, JobHandle inputDeps = default)
        //         {
        //             return ScheduleReset(ref world, inputDeps, true);
        //         }
        // However, to make that possible we need a why to allocate InputVelocities within a job.
        // The core simulation does not chain jobs across multiple simulation steps and so
        // will not hit this issue.
        internal JobHandle ScheduleReset(SimulationStepInput stepInput, JobHandle inputDeps, bool allocateEventDataStreams)
        {
            m_NumDynamicBodies = stepInput.World.NumDynamicBodies;
            if (!m_InputVelocities.IsCreated || m_InputVelocities.Length < m_NumDynamicBodies)
            {
                // TODO: can we find a way to setup InputVelocities within a job?
                if (m_InputVelocities.IsCreated)
                {
                    m_InputVelocities.Dispose();
                }
                m_InputVelocities = new NativeArray<Velocity>(m_NumDynamicBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            // Solver stabilization data
            if (stepInput.SolverStabilizationHeuristicSettings.EnableSolverStabilization)
            {
                if (!m_SolverStabilizationMotionData.IsCreated || m_SolverStabilizationMotionData.Length < m_NumDynamicBodies)
                {
                    if (m_SolverStabilizationMotionData.IsCreated)
                    {
                        m_SolverStabilizationMotionData.Dispose();
                    }
                    m_SolverStabilizationMotionData = new NativeArray<Solver.StabilizationMotionData>(m_NumDynamicBodies, Allocator.Persistent, NativeArrayOptions.ClearMemory);
                }
                else if (m_NumDynamicBodies > 0)
                {
                    unsafe
                    {
                        UnsafeUtility.MemClear(m_SolverStabilizationMotionData.GetUnsafePtr(), m_NumDynamicBodies * UnsafeUtility.SizeOf<Solver.StabilizationMotionData>());
                    }
                }
            }

            var handle = inputDeps;
            if (CollisionEventDataStream.IsCreated)
            {
                handle = CollisionEventDataStream.Dispose(handle);
            }
            if (TriggerEventDataStream.IsCreated)
            {
                handle = TriggerEventDataStream.Dispose(handle);
            }
            if (ImpulseEventDataStream.IsCreated)
            {
                handle = ImpulseEventDataStream.Dispose(handle);
            }

            if (allocateEventDataStreams)
            {
                if (!WorkItemCount.IsCreated)
                {
                    WorkItemCount = new NativeArray<int>(1, Allocator.Persistent);
                    WorkItemCount[0] = 1;
                }
                handle = NativeStream.ScheduleConstruct(out CollisionEventDataStream, WorkItemCount, handle, Allocator.Persistent);
                handle = NativeStream.ScheduleConstruct(out TriggerEventDataStream, WorkItemCount, handle, Allocator.Persistent);
                handle = NativeStream.ScheduleConstruct(out ImpulseEventDataStream, WorkItemCount, handle, Allocator.Persistent);
            }
            return handle;
        }

        /// <summary>
        /// Disposes the simulation context.
        /// </summary>
        public void Dispose()
        {
            if (m_InputVelocities.IsCreated)
            {
                m_InputVelocities.Dispose();
            }

            if (m_SolverStabilizationMotionData.IsCreated)
            {
                m_SolverStabilizationMotionData.Dispose();
            }

            if (CollisionEventDataStream.IsCreated)
            {
                CollisionEventDataStream.Dispose();
            }

            if (TriggerEventDataStream.IsCreated)
            {
                TriggerEventDataStream.Dispose();
            }

            if (ImpulseEventDataStream.IsCreated)
            {
                ImpulseEventDataStream.Dispose();
            }

            if (WorkItemCount.IsCreated)
            {
                WorkItemCount.Dispose();
            }
        }
    }

    // Temporary data created and destroyed during the step
    internal struct StepContext
    {
        // Built by the scheduler. Groups body pairs into phases in which each
        // body appears at most once, so that the interactions within each phase can be solved
        // in parallel with each other but not with other phases. This is consumed by the
        // ProcessBodyPairsJob, which outputs contact and joint Jacobians.
        public NativeList<DispatchPairSequencer.DispatchPair> PhasedDispatchPairs;

        // Job handle for the scheduler's job that creates the phased dispatch pairs.
        // Results will appear in the SolverSchedulerInfo property upon job completion.
        public JobHandle CreatePhasedDispatchPairsJobHandle;

        // Built by the scheduler. Describes the grouping of phased dispatch pairs for parallel processing
        // of joints and contacts in the solver.
        // Informs how we can schedule the solver jobs and what data locations they read info from.
        public DispatchPairSequencer.SolverSchedulerInfo SolverSchedulerInfo;

        public NativeStream Contacts;
        public NativeStream Jacobians;
    }

    /// <summary>   Steps a physics world. </summary>
    public struct Simulation : ISimulation
    {
        /// <summary>   Gets the simulation type. </summary>
        ///
        /// <value> <see cref="SimulationType.UnityPhysics"/>. </value>
        public SimulationType Type => SimulationType.UnityPhysics;

        /// <summary>   Gets the handle of the final simulation job (not including dispose jobs). </summary>
        ///
        /// <value> The final simulation job handle. </value>
        public JobHandle FinalSimulationJobHandle => m_StepHandles.FinalExecutionHandle;

        /// <summary>   Gets the handle of the final job. </summary>
        ///
        /// <value> The final job handle. </value>
        public JobHandle FinalJobHandle => JobHandle.CombineDependencies(FinalSimulationJobHandle, m_StepHandles.FinalDisposeHandle);

        internal SimulationScheduleStage m_SimulationScheduleStage;

        internal StepContext StepContext;

        /// <summary>
        /// Gets the contacts stream.
        /// This value is only valid after the CreateContactsJob (Narrowphase System), and before BuildJacobiansJob (CreateJacobiansSystem)
        /// </summary>
        /// <value> The contacts stream-->. </value>
        public readonly NativeStream Contacts => StepContext.Contacts;

        /// <summary>   Gets the collision events. </summary>
        ///
        /// <value> The collision events. </value>
        public CollisionEvents CollisionEvents => SimulationContext.CollisionEvents;

        /// <summary>   Gets the trigger events. </summary>
        ///
        /// <value> The trigger events. </value>
        public TriggerEvents TriggerEvents => SimulationContext.TriggerEvents;

        /// <summary>   Gets the impulse events. </summary>
        ///
        /// <value> The impulse events. </value>
        public ImpulseEvents ImpulseEvents => SimulationContext.ImpulseEvents;

        internal SimulationContext SimulationContext;

        internal SimulationJobHandles m_StepHandles;

        internal bool ReadyForEventScheduling => SimulationContext.ReadyForEventScheduling;

        /// <summary>   Creates a new Simulation. </summary>
        ///
        /// <returns>   A Simulation. </returns>
        public static Simulation Create()
        {
            Simulation sim = new Simulation();
            sim.Init();
            return sim;
        }

        /// <summary>
        /// Disposes the simulation.
        /// </summary>
        public void Dispose()
        {
            SimulationContext.Dispose();
        }

        private void Init()
        {
            StepContext = new StepContext();
            SimulationContext = new SimulationContext();
            m_StepHandles = new SimulationJobHandles(new JobHandle());
            m_SimulationScheduleStage = SimulationScheduleStage.Idle;
        }

        /// <summary>
        /// Steps the simulation immediately on a single thread without spawning any jobs.
        /// </summary>
        ///
        /// <param name="input">                The input. </param>
        /// <param name="simulationContext">    [in,out] Context for the simulation. </param>
        public static void StepImmediate(SimulationStepInput input, ref SimulationContext simulationContext)
        {
            SafetyChecks.CheckFiniteAndPositiveAndThrow(input.TimeStep, nameof(input.TimeStep));
            SafetyChecks.CheckInRangeAndThrow(input.NumSolverIterations, new int2(1, int.MaxValue), nameof(input.NumSolverIterations));

            if (input.World.NumDynamicBodies == 0)
            {
                // No need to do anything, since nothing can move
                return;
            }

            // Inform the context of the timeStep: this is a frame timestep
            simulationContext.TimeStep = input.TimeStep;

            // Find all body pairs that overlap in the broadphase
            var dynamicVsDynamicBodyPairs = new NativeStream(1, Allocator.Temp);
            var dynamicVsStaticBodyPairs = new NativeStream(1, Allocator.Temp);
            {
                var dynamicVsDynamicBodyPairsWriter = dynamicVsDynamicBodyPairs.AsWriter();
                var dynamicVsStaticBodyPairsWriter = dynamicVsStaticBodyPairs.AsWriter();
                input.World.CollisionWorld.FindOverlaps(ref dynamicVsDynamicBodyPairsWriter, ref dynamicVsStaticBodyPairsWriter);
            }

            // Create dispatch pairs
            var dispatchPairs = new NativeList<DispatchPairSequencer.DispatchPair>(Allocator.Temp);
            DispatchPairSequencer.CreateDispatchPairs(ref dynamicVsDynamicBodyPairs, ref dynamicVsStaticBodyPairs,
                input.World.NumDynamicBodies, input.World.Joints, ref dispatchPairs);

            var contacts = new NativeStream(1, Allocator.Temp);
            {
                if (input.NumSubsteps <= 1)
                {
                    // Integrate gravity using frame timestep
                    Solver.ApplyGravityAndUpdateInputVelocities(input.World.DynamicsWorld.MotionVelocities,
                        simulationContext.InputVelocities, input.Gravity, input.TimeStep);

                    var contactsWriter = contacts.AsWriter();
                    NarrowPhase.CreateContacts(ref input.World, simulationContext.InputVelocities,
                        dispatchPairs.AsArray(), input.TimeStep, ref contactsWriter); //Frame timestep
                }
                else // Using substeps
                {
                    // Predict linear velocity at the end of a frame under the influence of gravity
                    var copyInputVelocities = new NativeArray<Velocity>(input.World.DynamicsWorld.MotionVelocities.Length, Allocator.Temp);
                    var velocityFromGravity = input.TimeStep * input.Gravity;
                    Solver.UpdateInputVelocities(input.World.DynamicsWorld.MotionVelocities, copyInputVelocities, velocityFromGravity);

                    var contactsWriter = contacts.AsWriter();
                    NarrowPhase.CreateContacts(ref input.World, copyInputVelocities,
                        dispatchPairs.AsArray(), input.TimeStep, ref contactsWriter); //Frame timestep
                    copyInputVelocities.Dispose();

                    // Integrate gravity using substep timestep
                    Solver.ApplyGravityAndUpdateInputVelocities(input.World.DynamicsWorld.MotionVelocities,
                        simulationContext.InputVelocities, input.Gravity, input.SubstepTimeStep);
                }
            }

            // Build Jacobians
            var jacobians = new NativeStream(1, Allocator.Temp);
            {
                var contactsReader = contacts.AsReader();
                var jacobiansWriter = jacobians.AsWriter();
                Solver.BuildJacobians(ref input.World, input.SubstepTimeStep, math.length(input.Gravity), input.NumSubsteps,
                    input.NumSolverIterations, dispatchPairs.AsArray(), ref contactsReader, ref jacobiansWriter);
            }

            var stepInput = new Solver.StepInput()
            {
                Gravity = input.Gravity,
                Timestep = input.SubstepTimeStep,
                InvTimestep = Solver.CalculateInvTimeStep(input.SubstepTimeStep),
                InvNumSolverIterations = 1.0f / input.NumSolverIterations,
                NumSubsteps = input.NumSubsteps,
                NumSolverIterations = input.NumSolverIterations,
                CurrentSubstep = -1,
                CurrentSolverIteration = -1
            };

            // Iterate through substeps
            for (var i = 0; i < input.NumSubsteps; i++)
            {
                stepInput.CurrentSubstep = i;

                var jacobiansReader = jacobians.AsReader();
                if (i > 0) // First substep will be covered by the Jacobians.Build stage
                {
                    Solver.ApplyGravityAndUpdateInputVelocities(input.World.DynamicsWorld.MotionVelocities, simulationContext.InputVelocities,
                        input.Gravity, input.SubstepTimeStep);
                    Solver.Update(0, input.World.DynamicsWorld.MotionDatas, input.World.DynamicsWorld.MotionVelocities,
                        input.World.CollisionWorld.Bodies, ref jacobiansReader, stepInput);
                }

                var collisionEventsWriter = simulationContext.CollisionEventDataStream.AsWriter();
                var triggerEventsWriter = simulationContext.TriggerEventDataStream.AsWriter();
                var impulseEventsWriter = simulationContext.ImpulseEventDataStream.AsWriter();
                Solver.StabilizationData solverStabilizationData = new Solver.StabilizationData(input, simulationContext);

                Solver.SolveJacobians(ref jacobiansReader, input.World.DynamicsWorld.MotionVelocities, stepInput,
                    ref collisionEventsWriter, ref triggerEventsWriter, ref impulseEventsWriter, solverStabilizationData);

                // Integrate motions
                Integrator.Integrate(input.World.DynamicsWorld.MotionDatas, input.World.DynamicsWorld.MotionVelocities,
                    input.SubstepTimeStep);
            }

            // Synchronize the collision world if asked for
            if (input.SynchronizeCollisionWorld)
            {
                input.World.CollisionWorld.UpdateDynamicTree(ref input.World, input.TimeStep, input.Gravity);
            }
        }

        /// <summary>   Schedule broadphase jobs. </summary>
        ///
        /// <param name="input">            The input. </param>
        /// <param name="inputDeps">        The input deps. </param>
        /// <param name="multiThreaded">    (Optional) True if multi threaded. </param>
        ///
        /// <returns>   The SimulationJobHandles. </returns>
        public SimulationJobHandles ScheduleBroadphaseJobs(SimulationStepInput input, JobHandle inputDeps, bool multiThreaded = true)
        {
            return ScheduleBroadphaseJobsInternal(input, inputDeps, multiThreaded, incrementalDynamicBroadphase: false, incrementalStaticBroadphase: false);
        }

        internal SimulationJobHandles ScheduleBroadphaseJobsInternal(SimulationStepInput input, JobHandle inputDeps, bool multiThreaded, bool incrementalDynamicBroadphase, bool incrementalStaticBroadphase)
        {
            SafetyChecks.CheckFiniteAndPositiveAndThrow(input.TimeStep, nameof(input.TimeStep));
            SafetyChecks.CheckInRangeAndThrow(input.NumSolverIterations, new int2(1, int.MaxValue), nameof(input.NumSolverIterations));
            SafetyChecks.CheckSimulationStageAndThrow(m_SimulationScheduleStage, SimulationScheduleStage.Idle);
            m_SimulationScheduleStage = SimulationScheduleStage.PostCreateBodyPairs;

            // Dispose and reallocate input velocity buffer, if dynamic body count has increased.
            // Dispose previous collision, trigger and impulse event data streams.
            // New event streams are reallocated later when the work item count is known.
            JobHandle handle = SimulationContext.ScheduleReset(input, inputDeps, false);
            SimulationContext.TimeStep = input.TimeStep;

            StepContext = new StepContext();

            if (input.World.NumDynamicBodies == 0)
            {
                // No need to do anything, since nothing can move
                m_StepHandles = new SimulationJobHandles(handle);
                return m_StepHandles;
            }

            // Find all body pairs that overlap in the broadphase
            var handles = input.World.CollisionWorld.ScheduleFindOverlapsJobsInternal(
                out NativeStream dynamicVsDynamicBodyPairs, out NativeStream dynamicVsStaticBodyPairs,
                handle, multiThreaded, incrementalDynamicBroadphase, incrementalStaticBroadphase);
            handle = handles.FinalExecutionHandle;
            var disposeHandle = handles.FinalDisposeHandle;
            var postOverlapsHandle = handle;

            // Sort all overlapping and jointed body pairs into phases
            handles = DispatchPairSequencer.ScheduleCreatePhasedDispatchPairsJob(
                ref input.World, ref dynamicVsDynamicBodyPairs, ref dynamicVsStaticBodyPairs, handle,
                ref StepContext.PhasedDispatchPairs, out StepContext.SolverSchedulerInfo, multiThreaded);

            StepContext.CreatePhasedDispatchPairsJobHandle = handles.FinalExecutionHandle;

            m_StepHandles.FinalDisposeHandle = JobHandle.CombineDependencies(handles.FinalDisposeHandle, disposeHandle);
            m_StepHandles.FinalExecutionHandle = multiThreaded ?
                JobHandle.CombineDependencies(handles.FinalExecutionHandle, postOverlapsHandle) : handles.FinalExecutionHandle;

            return m_StepHandles;
        }

        /// <summary>   Schedule narrowphase jobs. </summary>
        ///
        /// <param name="input">            The input. </param>
        /// <param name="inputDeps">        The input deps. </param>
        /// <param name="multiThreaded">    (Optional) True if multi threaded. </param>
        ///
        /// <returns>   The SimulationJobHandles. </returns>
        public SimulationJobHandles ScheduleNarrowphaseJobs(SimulationStepInput input, JobHandle inputDeps, bool multiThreaded = true)
        {
            SafetyChecks.CheckSimulationStageAndThrow(m_SimulationScheduleStage, SimulationScheduleStage.PostCreateBodyPairs);
            m_SimulationScheduleStage = SimulationScheduleStage.PostCreateContacts;

            if (input.World.NumDynamicBodies == 0)
            {
                // No need to do anything, since nothing can move
                m_StepHandles = new SimulationJobHandles(inputDeps);
                return m_StepHandles;
            }

            var disposeHandle = m_StepHandles.FinalDisposeHandle;
            if (input.NumSubsteps <= 1)
            {
                // Integrate gravity so Jacobian Build is using same source data as Jacobian Solve
                var handle = Solver.ScheduleApplyGravityAndUpdateInputVelocitiesJob(ref input.World.DynamicsWorld,
                    SimulationContext.InputVelocities, input.Gravity, input.TimeStep, inputDeps, multiThreaded);

                m_StepHandles = NarrowPhase.ScheduleCreateContactsJobs(ref input.World, input.TimeStep,
                    SimulationContext.InputVelocities, ref StepContext.Contacts, ref StepContext.Jacobians,
                    ref StepContext.PhasedDispatchPairs, handle, ref StepContext.SolverSchedulerInfo, multiThreaded);
            }
            else // Using substeps
            {
                // Predict Linear Velocity at the end of a frame under the influence of gravity
                var copyInputVelocities = new NativeArray<Velocity>(input.World.DynamicsWorld.MotionVelocities.Length, Allocator.TempJob);
                var velocityFromGravity = input.TimeStep * input.Gravity;
                var handle = Solver.ScheduleUpdateInputVelocitiesJob(input.World.DynamicsWorld.MotionVelocities,
                    copyInputVelocities, velocityFromGravity, inputDeps, multiThreaded);

                // Create contacts using the velocity prediction data for the full frame timestep
                m_StepHandles = NarrowPhase.ScheduleCreateContactsJobs(ref input.World, input.TimeStep, copyInputVelocities,
                    ref StepContext.Contacts, ref StepContext.Jacobians, ref StepContext.PhasedDispatchPairs, handle,
                    ref StepContext.SolverSchedulerInfo, multiThreaded);

                var copyHandle = copyInputVelocities.Dispose(m_StepHandles.FinalExecutionHandle);
                m_StepHandles.FinalExecutionHandle = JobHandle.CombineDependencies(copyHandle, m_StepHandles.FinalExecutionHandle);
            }

            m_StepHandles.FinalDisposeHandle = JobHandle.CombineDependencies(disposeHandle, m_StepHandles.FinalDisposeHandle);

            return m_StepHandles;
        }

        /// <summary>   Schedule create jacobians jobs. </summary>
        ///
        /// <param name="input">            The input. </param>
        /// <param name="inputDeps">        The input deps. </param>
        /// <param name="multiThreaded">    (Optional) True if multi threaded. </param>
        ///
        /// <returns>   The SimulationJobHandles. </returns>
        public SimulationJobHandles ScheduleCreateJacobiansJobs(SimulationStepInput input, JobHandle inputDeps, bool multiThreaded = true)
        {
            SafetyChecks.CheckSimulationStageAndThrow(m_SimulationScheduleStage, SimulationScheduleStage.PostCreateContacts);
            m_SimulationScheduleStage = SimulationScheduleStage.PostCreateJacobians;

            if (input.World.NumDynamicBodies == 0)
            {
                // No need to do anything, since nothing can move
                m_StepHandles = new SimulationJobHandles(inputDeps);
                return m_StepHandles;
            }

            if (input.NumSubsteps > 1)
            {
                // Integrate gravity so Jacobian Build is using same source data as Jacobian Solve
                inputDeps = Solver.ScheduleApplyGravityAndUpdateInputVelocitiesJob(ref input.World.DynamicsWorld,
                    SimulationContext.InputVelocities, input.Gravity, input.SubstepTimeStep, inputDeps, multiThreaded);
            }

            // Create/Initialize Jacobians for first substep
            var disposeHandle = m_StepHandles.FinalDisposeHandle;
            m_StepHandles = Solver.ScheduleBuildJacobiansJobs(ref input.World, input.SubstepTimeStep, math.length(input.Gravity),
                input.NumSubsteps, input.NumSolverIterations, inputDeps, ref StepContext.PhasedDispatchPairs,
                ref StepContext.SolverSchedulerInfo, ref StepContext.Contacts, ref StepContext.Jacobians, multiThreaded);
            m_StepHandles.FinalDisposeHandle = JobHandle.CombineDependencies(disposeHandle, m_StepHandles.FinalDisposeHandle);

            return m_StepHandles;
        }

        /// <summary>   Schedule solve and integrate jobs. </summary>
        ///
        /// <param name="input">            The input. </param>
        /// <param name="inputDeps">        The input deps. </param>
        /// <param name="multiThreaded">    (Optional) True if multi threaded. </param>
        ///
        /// <returns>   The SimulationJobHandles. </returns>
        public SimulationJobHandles ScheduleSolveAndIntegrateJobs(SimulationStepInput input, JobHandle inputDeps, bool multiThreaded = true)
        {
            SafetyChecks.CheckSimulationStageAndThrow(m_SimulationScheduleStage, SimulationScheduleStage.PostCreateJacobians);
            m_SimulationScheduleStage = SimulationScheduleStage.Idle;

            if (input.World.NumDynamicBodies == 0)
            {
                // No need to do anything, since nothing can move
                m_StepHandles = new SimulationJobHandles(inputDeps);
                return m_StepHandles;
            }

            // Make sure we know the number of phased dispatch pairs so that we can efficiently schedule the solve jobs
            // in Solver.ScheduleSolveJacobiansJobs() below
            if (multiThreaded)
            {
                StepContext.CreatePhasedDispatchPairsJobHandle.Complete();
            }

            var executionHandle = m_StepHandles.FinalExecutionHandle;
            var disposeHandle = m_StepHandles.FinalDisposeHandle;
            var jobHandle = inputDeps;
            Solver.StabilizationData solverStabilizationData = new Solver.StabilizationData(input, SimulationContext);

            // Initialize the event streams outside of the substep loop. These should only be written to on the last
            // solver iteration of the last substep
            //TODO: Change NativeStream allocations to Allocator.TempJob when leaks are fixed https://github.com/Unity-Technologies/Unity.Physics/issues/7
            if (!multiThreaded)
            {
                SimulationContext.CollisionEventDataStream = new NativeStream(1, Allocator.Persistent);
                SimulationContext.TriggerEventDataStream = new NativeStream(1, Allocator.Persistent);
                SimulationContext.ImpulseEventDataStream = new NativeStream(1, Allocator.Persistent);
            }
            else
            {
                NativeArray<int> workItemList = StepContext.SolverSchedulerInfo.NumWorkItems;
                JobHandle collisionEventStreamHandle = NativeStream.ScheduleConstruct(
                    out SimulationContext.CollisionEventDataStream, workItemList, inputDeps, Allocator.Persistent);
                JobHandle triggerEventStreamHandle = NativeStream.ScheduleConstruct(
                    out SimulationContext.TriggerEventDataStream, workItemList, inputDeps, Allocator.Persistent);
                JobHandle impulseEventStreamHandle = NativeStream.ScheduleConstruct(
                    out SimulationContext.ImpulseEventDataStream, workItemList, inputDeps, Allocator.Persistent);

                var streamJobHandle = JobHandle.CombineDependencies(
                    collisionEventStreamHandle, triggerEventStreamHandle, impulseEventStreamHandle);
                jobHandle = JobHandle.CombineDependencies(jobHandle, streamJobHandle);
            }

            var stepInput = new Solver.StepInput()
            {
                Gravity = input.Gravity,
                Timestep = input.SubstepTimeStep,
                InvTimestep = Solver.CalculateInvTimeStep(input.SubstepTimeStep),
                InvNumSolverIterations = 1.0f / input.NumSolverIterations,
                NumSubsteps = input.NumSubsteps,
                NumSolverIterations = input.NumSolverIterations,
                CurrentSubstep = -1,
                CurrentSolverIteration = -1
            };

            for (var i = 0; i < input.NumSubsteps; i++)
            {
                stepInput.CurrentSubstep = i;

                if (i > 0) // Gravity integration for first substep is covered in the Jacobians.Build stage
                {
                    jobHandle = Solver.ScheduleApplyGravityAndUpdateInputVelocitiesJob(ref input.World.DynamicsWorld, SimulationContext.InputVelocities,
                        input.Gravity, input.SubstepTimeStep,  jobHandle, multiThreaded);

                    jobHandle = Solver.ScheduleUpdateJacobiansJobs(ref input.World, ref StepContext.Jacobians,
                        ref StepContext.SolverSchedulerInfo, stepInput, jobHandle, multiThreaded);
                }

                // Solve Jacobians
                m_StepHandles = Solver.ScheduleSolveJacobiansJobs(ref input.World.DynamicsWorld, stepInput,
                    ref StepContext.Jacobians, ref SimulationContext.CollisionEventDataStream,
                    ref SimulationContext.TriggerEventDataStream, ref SimulationContext.ImpulseEventDataStream,
                    ref StepContext.SolverSchedulerInfo, solverStabilizationData, jobHandle, multiThreaded);

                // Integrate motions (updates MotionDatas, MotionVelocities)
                jobHandle = Integrator.ScheduleIntegrateJobs(ref input.World.DynamicsWorld,
                    input.SubstepTimeStep, m_StepHandles.FinalExecutionHandle, multiThreaded);

                m_StepHandles.FinalDisposeHandle =
                    JobHandle.CombineDependencies(disposeHandle, m_StepHandles.FinalDisposeHandle);

                jobHandle = JobHandle.CombineDependencies(jobHandle, executionHandle, m_StepHandles.FinalExecutionHandle);

                if (stepInput.IsLastSubstep)
                {
                    m_StepHandles.FinalExecutionHandle = jobHandle;
                }
            }

            m_StepHandles.FinalExecutionHandle = JobHandle.CombineDependencies(m_StepHandles.FinalExecutionHandle, jobHandle);

            // Synchronize the collision world
            if (input.SynchronizeCollisionWorld)
            {
                m_StepHandles.FinalExecutionHandle = input.World.CollisionWorld.ScheduleUpdateDynamicTree(
                    ref input.World, input.TimeStep, input.Gravity, m_StepHandles.FinalExecutionHandle, multiThreaded);
            }

            // Different dispose logic for single threaded simulation compared to "standard" threading (multi threaded)
            if (!multiThreaded)
            {
                // Note: In the multithreaded case, StepContext.PhasedDispatchPairs is disposed in Solver.ScheduleBuildJacobiansJobs().
                m_StepHandles.FinalDisposeHandle = StepContext.PhasedDispatchPairs.Dispose(m_StepHandles.FinalExecutionHandle);

                m_StepHandles.FinalDisposeHandle = StepContext.Contacts.Dispose(m_StepHandles.FinalDisposeHandle);
                m_StepHandles.FinalDisposeHandle = StepContext.Jacobians.Dispose(m_StepHandles.FinalDisposeHandle);
                m_StepHandles.FinalDisposeHandle = StepContext.SolverSchedulerInfo.ScheduleDisposeJob(m_StepHandles.FinalDisposeHandle);
            }

            return m_StepHandles;
        }

        /// <summary>
        /// Resets the simulation storage
        /// - Reallocates input velocities storage if necessary
        /// - Disposes event streams and allocates new ones with a single work item
        /// </summary>
        /// <param name="input">    The input. </param>
        public void ResetSimulationContext(SimulationStepInput input)
        {
            SimulationContext.Reset(input);
        }

        /// <summary>   Steps the world immediately. </summary>
        ///
        /// <param name="input">    The input. </param>
        public void Step(SimulationStepInput input)
        {
            StepImmediate(input, ref SimulationContext);
        }

        /// <summary>
        /// Schedule all the jobs for the simulation step. Enqueued callbacks can choose to inject
        /// additional jobs at defined sync points. multiThreaded defines which simulation type will be
        /// called:
        ///     - true will result in default multithreaded simulation
        ///     - false will result in a very small number of jobs (1 per physics step phase) that are
        ///     scheduled sequentially
        /// Behavior doesn't change regardless of the multiThreaded argument provided.
        /// </summary>
        ///
        /// <param name="input">            The input. </param>
        /// <param name="inputDeps">        The input deps. </param>
        /// <param name="multiThreaded">    (Optional) True if multi threaded. </param>
        ///
        /// <returns>   The SimulationJobHandles. </returns>
        public unsafe SimulationJobHandles ScheduleStepJobs(SimulationStepInput input, JobHandle inputDeps, bool multiThreaded = true)
        {
            ScheduleBroadphaseJobs(input, inputDeps, multiThreaded);
            ScheduleNarrowphaseJobs(input, m_StepHandles.FinalExecutionHandle, multiThreaded);
            ScheduleCreateJacobiansJobs(input, m_StepHandles.FinalExecutionHandle, multiThreaded);
            ScheduleSolveAndIntegrateJobs(input, m_StepHandles.FinalExecutionHandle, multiThreaded);

            return m_StepHandles;
        }
    }
}
