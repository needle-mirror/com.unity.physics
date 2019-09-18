using System;
using System.ComponentModel;
using Unity.Collections;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.Physics
{
    // Implementations of ISimulation
    public enum SimulationType
    {
        NoPhysics,      // A dummy implementation which does nothing
        UnityPhysics,   // Default C# implementation
        HavokPhysics    // Havok implementation (using C++ plugin)
    }

    // Parameters for a simulation step
    public struct SimulationStepInput
    {
        public PhysicsWorld World;
        public float TimeStep;
        public float3 Gravity;
        public int ThreadCountHint;
        public int NumSolverIterations;
        public bool SynchronizeCollisionWorld;  // whether to update the collision world after the step
        public SimulationCallbacks Callbacks;
    }

    // Interface for simulations
    public interface ISimulation : IDisposable
    {
        // The implementation type
        SimulationType Type { get; }

        // Step the simulation (single threaded)
        void Step(SimulationStepInput input);

        // Schedule a set of jobs to step the simulation
        void ScheduleStepJobs(SimulationStepInput input, JobHandle inputDeps);

        // The final scheduled simulation job.
        // Jobs which use the simulation results should depend on this.
        JobHandle FinalSimulationJobHandle { get; }

        // The final scheduled job, including all simulation and cleanup.
        // The end of each step should depend on this.
        JobHandle FinalJobHandle { get; }

        #region Obsolete
        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("ScheduleStepJobs(SimulationStepInput, JobHandle, out JobHandle, out JobHandle) has been deprecated. Use ScheduleStepJobs(SimulationStepInput, JobHandle) instead. (RemovedAfter 2019-10-15)", true)]
        void ScheduleStepJobs(SimulationStepInput input, JobHandle inputDeps, out JobHandle finalSimulationJobHandle, out JobHandle finalJobHandle);
        #endregion
    }

    // Default simulation implementation
    public class Simulation : ISimulation
    {
        public SimulationType Type => SimulationType.UnityPhysics;
        public JobHandle FinalSimulationJobHandle { get; protected set; }
        public JobHandle FinalJobHandle { get; protected set; }

        internal Context m_Context;
        internal LowLevel.CollisionEvents CollisionEvents => new LowLevel.CollisionEvents(m_Context.CollisionEventStream);
        internal LowLevel.TriggerEvents TriggerEvents => new LowLevel.TriggerEvents(m_Context.TriggerEventStream);

        private Storage m_Storage;
        private readonly Scheduler m_Scheduler;

        public Simulation()
        {
            m_Scheduler = new Scheduler();
            m_Context = new Context();
            m_Storage = new Storage();
        }

        public void Dispose()
        {
            m_Scheduler.Dispose();
            m_Storage.Dispose();
            DisposeEventStreams(new JobHandle()).Complete();
        }

        public void Step(SimulationStepInput input)
        {
            // TODO : Using the multithreaded version for now, but should do a proper single threaded version
            ScheduleStepJobs(input, new JobHandle());
            FinalJobHandle.Complete();
        }

        // Schedule all the jobs for the simulation step.
        // Enqueued callbacks can choose to inject additional jobs at defined sync points.
        public unsafe void ScheduleStepJobs(SimulationStepInput input, JobHandle inputDeps)
        {
            if (input.TimeStep < 0)
                throw new ArgumentOutOfRangeException();
            if (input.ThreadCountHint <= 0)
                throw new ArgumentOutOfRangeException();
            if (input.NumSolverIterations <= 0)
                throw new ArgumentOutOfRangeException();

            // Dispose event streams from previous frame
            JobHandle handle = DisposeEventStreams(inputDeps);

            // Allocate storage for input velocities
            m_Storage.InputVelocityCount = input.World.NumDynamicBodies;

            m_Context = new Context
            {
                TimeStep = input.TimeStep,
                InputVelocities = m_Storage.InputVelocities
            };

            if (input.World.NumDynamicBodies == 0)
            {
                // No need to do anything, since nothing can move
                FinalSimulationJobHandle = handle;
                FinalJobHandle = handle;
                return;
            }

            SimulationCallbacks callbacks = input.Callbacks ?? new SimulationCallbacks();

            // Find all body pairs that overlap in the broadphase
            handle = input.World.CollisionWorld.Broadphase.ScheduleFindOverlapsJobs(
                out BlockStream dynamicVsDynamicBodyPairs, out BlockStream dynamicVsStaticBodyPairs, ref m_Context, handle);
            var postOverlapsHandle = handle;

            // Sort all overlapping and jointed body pairs into phases
            handle = m_Scheduler.ScheduleCreatePhasedDispatchPairsJob(
                ref input.World, ref dynamicVsDynamicBodyPairs, ref dynamicVsStaticBodyPairs, ref m_Context, handle);

            // Apply gravity and copy input velocities at this point (in parallel with the scheduler, but before the callbacks)
            var applyGravityAndCopyInputVelocitiesHandle = Solver.ScheduleApplyGravityAndCopyInputVelocitiesJob(
                ref input.World.DynamicsWorld, m_Storage.InputVelocities, input.TimeStep * input.Gravity, postOverlapsHandle);

            handle = JobHandle.CombineDependencies(handle, applyGravityAndCopyInputVelocitiesHandle);
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateDispatchPairs, this, ref input.World, handle);

            // Create contact points & joint Jacobians
            handle = NarrowPhase.ScheduleProcessBodyPairsJobs(ref input.World, input.TimeStep, input.NumSolverIterations, ref m_Context, handle);
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateContacts, this, ref input.World, handle);

            // Create contact Jacobians
            handle = Solver.ScheduleBuildContactJacobiansJobs(ref input.World.DynamicsWorld, input.TimeStep, math.length(input.Gravity), ref m_Context, handle);
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateContactJacobians, this, ref input.World, handle);

            // Solve all Jacobians
            handle = Solver.ScheduleSolveJacobiansJobs(ref input.World.DynamicsWorld, input.TimeStep, input.NumSolverIterations, ref m_Context, handle);
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostSolveJacobians, this, ref input.World, handle);

            // Integrate motions
            handle = Integrator.ScheduleIntegrateJobs(ref input.World.DynamicsWorld, input.TimeStep, handle);

            // Synchronize the collision world
            if (input.SynchronizeCollisionWorld)
            {
                handle = input.World.CollisionWorld.ScheduleUpdateDynamicLayer(ref input.World, input.TimeStep, input.Gravity, input.ThreadCountHint, handle);  // TODO: timeStep = 0?
            }

            // Return the final simulation handle
            FinalSimulationJobHandle = handle;

            // Return the final handle, which includes disposing temporary arrays
            JobHandle* deps = stackalloc JobHandle[11]
            {
                FinalSimulationJobHandle,
                m_Context.DisposeOverlapPairs0,
                m_Context.DisposeOverlapPairs1,
                m_Context.DisposeBroadphasePairs0,
                m_Context.DisposeBroadphasePairs1,
                m_Context.DisposeContacts,
                m_Context.DisposeJacobians,
                m_Context.DisposeJointJacobians,
                m_Context.DisposeSolverSchedulerData,
                m_Context.DisposeProcessBodyPairs,
                m_Context.DisposePhasedDispatchPairs
            };
            FinalJobHandle = JobHandleUnsafeUtility.CombineDependencies(deps, 11);
        }

        // Event streams live until the next step, so they should be disposed at the beginning of it
        private JobHandle DisposeEventStreams(JobHandle inDeps)
        {
            if (m_Context.CollisionEventStream.IsCreated)
            {
                inDeps = m_Context.CollisionEventStream.Dispose(inDeps);
            }
            if (m_Context.TriggerEventStream.IsCreated)
            {
                inDeps = m_Context.TriggerEventStream.Dispose(inDeps);
            }

            return inDeps;
        }

        // Holds temporary data in a storage that lives as long as simulation lives
        // and is only re-allocated if necessary.
        private struct Storage : IDisposable
        {
            private int m_InputVelocityCount;

            private NativeArray<Velocity> m_InputVelocities;

            public NativeSlice<Velocity> InputVelocities => new NativeSlice<Velocity>(m_InputVelocities, 0, m_InputVelocityCount);

            public int InputVelocityCount
            {
                get => m_InputVelocityCount;
                set
                {
                    m_InputVelocityCount = value;
                    if (m_InputVelocities.Length < m_InputVelocityCount)
                    {
                        if (m_InputVelocities.IsCreated)
                        {
                            m_InputVelocities.Dispose();
                        }
                        m_InputVelocities = new NativeArray<Velocity>(m_InputVelocityCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                    }
                }
            }

            public void Dispose()
            {
                m_InputVelocities.Dispose();
            }
        }

        // Temporary data created and destroyed during the step
        internal struct Context
        {
            // Delta time used to step the simulation
            public float TimeStep;

            // Copy of MotionVelocity velocities from the start of the step
            public NativeSlice<Velocity> InputVelocities;

            // Built by the scheduler. Groups body pairs into phases in which each
            // body appears at most once, so that the interactions within each phase can be solved
            // in parallel with each other but not with other phases. This is consumed by the
            // ProcessBodyPairsJob, which outputs contact and joint Jacobians.
            public NativeList<Scheduler.DispatchPair> PhasedDispatchPairs;

            // Built by the scheduler. Describes the grouping of PhasedBodyPairs
            // which informs how we can schedule the solver jobs and where they read info from.
            public Scheduler.SolverSchedulerInfo SolverSchedulerInfo;

            public BlockStream Contacts;
            public BlockStream Jacobians;
            public BlockStream JointJacobians;
            public BlockStream CollisionEventStream;
            public BlockStream TriggerEventStream;

            public JobHandle DisposeOverlapPairs0;
            public JobHandle DisposeOverlapPairs1;
            public JobHandle DisposeBroadphasePairs0;
            public JobHandle DisposeBroadphasePairs1;
            public JobHandle DisposeContacts;
            public JobHandle DisposeJacobians;
            public JobHandle DisposeJointJacobians;
            public JobHandle DisposeSolverSchedulerData;
            public JobHandle DisposeProcessBodyPairs;
            public JobHandle DisposePhasedDispatchPairs;
        }

        #region Obsolete
        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("ScheduleStepJobs(SimulationStepInput, JobHandle, out JobHandle, out JobHandle) has been deprecated. Use ScheduleStepJobs(SimulationStepInput, JobHandle) instead. (RemovedAfter 2019-10-15)")]
        public void ScheduleStepJobs(SimulationStepInput input, JobHandle inputDeps, out JobHandle finalSimulationJobHandle, out JobHandle finalJobHandle)
        {
            finalSimulationJobHandle = FinalSimulationJobHandle;
            finalJobHandle = FinalJobHandle;
            ScheduleStepJobs(input, inputDeps);
        }
        #endregion
    }
}
