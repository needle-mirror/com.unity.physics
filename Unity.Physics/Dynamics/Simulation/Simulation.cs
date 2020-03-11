using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;

namespace Unity.Physics
{
    // Holds temporary data in a storage that lives as long as simulation lives
    // and is only re-allocated if necessary.
    public struct SimulationContext : IDisposable
    {
        // This container is read-only and is only used to get the entity for a given rigid body
        // when reporting collision events. However, if SimulationContext is stored next to SimulationStepInput
        // in a job, aliasing check will complain both have the same bodies array.
        [NativeDisableContainerSafetyRestriction]
        private NativeSlice<RigidBody> m_Bodies;

        internal float TimeStep;

        internal NativeArray<Velocity> InputVelocities;

        internal NativeStream CollisionEventDataStream;
        internal NativeStream TriggerEventDataStream;

        public CollisionEvents CollisionEvents =>
            new CollisionEvents(CollisionEventDataStream, m_Bodies, InputVelocities, TimeStep);
        public TriggerEvents TriggerEvents =>
            new TriggerEvents(TriggerEventDataStream, m_Bodies);

        private NativeArray<int> WorkItemCount;

        // Resets the simulation storage
        // - Reallocates input velocities storage if necessary
        // - Disposes event streams and allocates new ones with a single work item
        // NOTE: Reset or ScheduleReset needs to be called before passing the SimulationContext
        // to a simulation step job. If you don't then you may get initialization errors.
        public void Reset(ref PhysicsWorld world)
        {
            m_Bodies = world.Bodies;

            int numDynamicBodies = world.NumDynamicBodies;
            if (InputVelocities.Length < numDynamicBodies)
            {
                if (InputVelocities.IsCreated)
                {
                    InputVelocities.Dispose();
                }
                InputVelocities = new NativeArray<Velocity>(numDynamicBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            }

            if (CollisionEventDataStream.IsCreated)
            {
                CollisionEventDataStream.Dispose();
            }
            if (TriggerEventDataStream.IsCreated)
            {
                TriggerEventDataStream.Dispose();
            }
            
            {
                if (!WorkItemCount.IsCreated)
                {
                    WorkItemCount = new NativeArray<int>(new int[] { 1 }, Allocator.Persistent);
                }
                CollisionEventDataStream = new NativeStream(WorkItemCount[0], Allocator.Persistent);
                TriggerEventDataStream = new NativeStream(WorkItemCount[0], Allocator.Persistent);
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
        internal JobHandle ScheduleReset(ref PhysicsWorld world, JobHandle inputDeps, bool allocateEventDataStreams)
        {
            m_Bodies = world.Bodies;

            int numDynamicBodies = world.NumDynamicBodies;
            if (InputVelocities.Length < numDynamicBodies)
            {
                // TODO: can we find a way to setup InputVelocities within a job?
                if (InputVelocities.IsCreated)
                {
                    InputVelocities.Dispose();
                }
                InputVelocities = new NativeArray<Velocity>(numDynamicBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
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
            if (allocateEventDataStreams)
            {
                if (!WorkItemCount.IsCreated)
                {
                    WorkItemCount = new NativeArray<int>(new int[] { 1 }, Allocator.Persistent);
                }
                handle = NativeStream.ScheduleConstruct(out CollisionEventDataStream, WorkItemCount, handle, Allocator.Persistent);
                handle = NativeStream.ScheduleConstruct(out TriggerEventDataStream, WorkItemCount, handle, Allocator.Persistent);
            }
            return handle;
        }

        public void Dispose()
        {
            if (InputVelocities.IsCreated)
            {
                InputVelocities.Dispose();
            }

            if (CollisionEventDataStream.IsCreated)
            {
                CollisionEventDataStream.Dispose();
            }

            if (TriggerEventDataStream.IsCreated)
            {
                TriggerEventDataStream.Dispose();
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

        // Built by the scheduler. Describes the grouping of PhasedBodyPairs
        // which informs how we can schedule the solver jobs and where they read info from.
        public DispatchPairSequencer.SolverSchedulerInfo SolverSchedulerInfo;

        public NativeStream Contacts;
        public NativeStream Jacobians;
    }

    // Default simulation implementation
    public class Simulation : ISimulation
    {
        public SimulationType Type => SimulationType.UnityPhysics;
        public JobHandle FinalSimulationJobHandle => m_StepHandles.FinalExecutionHandle;
        public JobHandle FinalJobHandle => JobHandle.CombineDependencies(FinalSimulationJobHandle, m_StepHandles.FinalDisposeHandle);

        internal StepContext StepContext = new StepContext();

        public CollisionEvents CollisionEvents => SimulationContext.CollisionEvents;

        public TriggerEvents TriggerEvents => SimulationContext.TriggerEvents;

        internal SimulationContext SimulationContext = new SimulationContext();

        private readonly DispatchPairSequencer m_Scheduler = new DispatchPairSequencer();
        private SimulationJobHandles m_StepHandles = new SimulationJobHandles(new JobHandle());

        public void Dispose()
        {
            m_Scheduler.Dispose();
            SimulationContext.Dispose();
        }

        // Steps the simulation immediately on a single thread without spawning any jobs.
        public static void StepImmediate(SimulationStepInput input, ref SimulationContext simulationContext)
        {
            if (input.TimeStep < 0)
                throw new ArgumentOutOfRangeException();
            if (input.NumSolverIterations <= 0)
                throw new ArgumentOutOfRangeException();

            if (input.World.NumDynamicBodies == 0)
            {
                // No need to do anything, since nothing can move
                return;
            }

            // Inform the context of the timeStep
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

            // Apply gravity and copy input velocities
            Solver.ApplyGravityAndCopyInputVelocities(input.World.DynamicsWorld.MotionDatas, input.World.DynamicsWorld.MotionVelocities,
                simulationContext.InputVelocities, input.TimeStep * input.Gravity);

            // Narrow phase
            var contacts = new NativeStream(1, Allocator.Temp);
            {
                var contactsWriter = contacts.AsWriter();
                NarrowPhase.CreateContacts(ref input.World, dispatchPairs.AsArray(), input.TimeStep, ref contactsWriter);
            }

            // Build Jacobians
            var jacobians = new NativeStream(1, Allocator.Temp);
            {
                var contactsReader = contacts.AsReader();
                var jacobiansWriter = jacobians.AsWriter();
                Solver.BuildJacobians(ref input.World, input.TimeStep, input.Gravity, input.NumSolverIterations,
                    dispatchPairs.AsArray(), ref contactsReader, ref jacobiansWriter);
            }

            // Solve Jacobians
            {
                var jacobiansReader = jacobians.AsReader();
                var collisionEventsWriter = simulationContext.CollisionEventDataStream.AsWriter();
                var triggerEventsWriter = simulationContext.TriggerEventDataStream.AsWriter();
                Solver.SolveJacobians(ref jacobiansReader, input.World.DynamicsWorld.MotionVelocities, input.TimeStep, input.NumSolverIterations,
                    ref collisionEventsWriter, ref triggerEventsWriter);
            }

            // Integrate motions
            Integrator.Integrate(input.World.DynamicsWorld.MotionDatas, input.World.DynamicsWorld.MotionVelocities, input.TimeStep);

            // Synchronize the collision world if asked for
            if (input.SynchronizeCollisionWorld)
            {
                input.World.CollisionWorld.UpdateDynamicTree(ref input.World, input.TimeStep, input.Gravity);
            }
        }

        public void Step(SimulationStepInput input)
        {
            StepImmediate(input, ref SimulationContext);
        }

        // Schedule all the jobs for the simulation step.
        // Enqueued callbacks can choose to inject additional jobs at defined sync points.
        // threadCountHint defines which simulation type will be called:
        //     - threadCountHint > 0 will result in default multithreaded simulation
        //     - threadCountHint <=0 will result in a very small number of jobs (1 per physics step phase) that are scheduled sequentially
        // Behavior doesn't change regardless of the threadCountHint provided.
        public unsafe SimulationJobHandles ScheduleStepJobs(SimulationStepInput input, SimulationCallbacks callbacksIn, JobHandle inputDeps, int threadCountHint = 0)
        {
            if (input.TimeStep < 0)
                throw new ArgumentOutOfRangeException();
            if (input.NumSolverIterations <= 0)
                throw new ArgumentOutOfRangeException();

            bool singleThreadedSim = (threadCountHint <= 0);

            // Dispose and reallocate input velocity buffer, if dynamic body count has increased.
            // Dispose previous collision and trigger event data streams. 
            // New event streams are reallocated later when the work item count is known.
            JobHandle handle = SimulationContext.ScheduleReset(ref input.World, inputDeps, false);
            SimulationContext.TimeStep = input.TimeStep;

            StepContext = new StepContext();

            if (input.World.NumDynamicBodies == 0)
            {
                // No need to do anything, since nothing can move
                m_StepHandles = new SimulationJobHandles(handle);
                return m_StepHandles;
            }

            SimulationCallbacks callbacks = callbacksIn ?? new SimulationCallbacks();

            // Find all body pairs that overlap in the broadphase
            var handles = input.World.CollisionWorld.ScheduleFindOverlapsJobs(
                out NativeStream dynamicVsDynamicBodyPairs, out NativeStream dynamicVsStaticBodyPairs, handle, threadCountHint);
            handle = handles.FinalExecutionHandle;
            var disposeHandle1 = handles.FinalDisposeHandle;
            var postOverlapsHandle = handle;

            // Sort all overlapping and jointed body pairs into phases
            handles = m_Scheduler.ScheduleCreatePhasedDispatchPairsJob(
                ref input.World, ref dynamicVsDynamicBodyPairs, ref dynamicVsStaticBodyPairs, handle,
                ref StepContext.PhasedDispatchPairs, out StepContext.SolverSchedulerInfo, threadCountHint);
            handle = handles.FinalExecutionHandle;
            var disposeHandle2 = handles.FinalDisposeHandle;

            // Apply gravity and copy input velocities at this point (in parallel with the scheduler, but before the callbacks)
            var applyGravityAndCopyInputVelocitiesHandle = Solver.ScheduleApplyGravityAndCopyInputVelocitiesJob(
                ref input.World.DynamicsWorld, SimulationContext.InputVelocities, input.TimeStep * input.Gravity, singleThreadedSim ? handle : postOverlapsHandle, threadCountHint);

            handle = JobHandle.CombineDependencies(handle, applyGravityAndCopyInputVelocitiesHandle);
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateDispatchPairs, this, ref input.World, handle);

            // Create contact points & joint Jacobians
            handles = NarrowPhase.ScheduleCreateContactsJobs(ref input.World, input.TimeStep,
                ref StepContext.Contacts, ref StepContext.Jacobians, ref StepContext.PhasedDispatchPairs, handle,
                ref StepContext.SolverSchedulerInfo, threadCountHint);
            handle = handles.FinalExecutionHandle;
            var disposeHandle3 = handles.FinalDisposeHandle;
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateContacts, this, ref input.World, handle);

            // Create contact Jacobians
            handles = Solver.ScheduleBuildJacobiansJobs(ref input.World, input.TimeStep, input.Gravity, input.NumSolverIterations,
                handle, ref StepContext.PhasedDispatchPairs, ref StepContext.SolverSchedulerInfo,
                ref StepContext.Contacts, ref StepContext.Jacobians, threadCountHint);
            handle = handles.FinalExecutionHandle;
            var disposeHandle4 = handles.FinalDisposeHandle;
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateContactJacobians, this, ref input.World, handle);

            // Solve all Jacobians
            handles = Solver.ScheduleSolveJacobiansJobs(ref input.World.DynamicsWorld, input.TimeStep, input.NumSolverIterations,
                ref StepContext.Jacobians, ref SimulationContext.CollisionEventDataStream, ref SimulationContext.TriggerEventDataStream,
                ref StepContext.SolverSchedulerInfo, handle, threadCountHint);
            handle = handles.FinalExecutionHandle;
            var disposeHandle5 = handles.FinalDisposeHandle;
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostSolveJacobians, this, ref input.World, handle);

            // Integrate motions
            handle = Integrator.ScheduleIntegrateJobs(ref input.World.DynamicsWorld, input.TimeStep, handle, threadCountHint);

            // Synchronize the collision world
            if (input.SynchronizeCollisionWorld)
            {
                handle = input.World.CollisionWorld.ScheduleUpdateDynamicTree(ref input.World, input.TimeStep, input.Gravity, handle, threadCountHint);  // TODO: timeStep = 0?
            }

            // Return the final simulation handle
            m_StepHandles.FinalExecutionHandle = handle;

            // Different dispose logic for single threaded simulation compared to "standard" threading (multi threaded)
            if (singleThreadedSim)
            {
                handle = dynamicVsDynamicBodyPairs.Dispose(handle);
                handle = dynamicVsStaticBodyPairs.Dispose(handle);
                handle = StepContext.PhasedDispatchPairs.Dispose(handle);
                handle = StepContext.Contacts.Dispose(handle);
                handle = StepContext.Jacobians.Dispose(handle);
                handle = StepContext.SolverSchedulerInfo.ScheduleDisposeJob(handle);

                m_StepHandles.FinalDisposeHandle = handle;
            }
            else
            {
                // Return the final handle, which includes disposing temporary arrays
                JobHandle* deps = stackalloc JobHandle[5]
                {
                    disposeHandle1,
                    disposeHandle2,
                    disposeHandle3,
                    disposeHandle4,
                    disposeHandle5
                };
                m_StepHandles.FinalDisposeHandle = JobHandleUnsafeUtility.CombineDependencies(deps, 5);
            }

            return m_StepHandles;
        }

        [Obsolete("ScheduleStepJobs() has been deprecated. Use the new ScheduleStepJobs method that takes callbacks and threadCountHint as input and returns SimulationStepHandles. (RemovedAfter 2020-05-01)")]
        public void ScheduleStepJobs(SimulationStepInput input, JobHandle inputDeps)
        {
            m_StepHandles = ScheduleStepJobs(input, null, inputDeps, input.ThreadCountHint);
        }
    }
}
