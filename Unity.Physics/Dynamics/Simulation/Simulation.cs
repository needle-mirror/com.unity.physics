using System;
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

        // Schedule a set of jobs to step the simulation. Returns two job handles:
        // - Jobs which use the simulation results should depend on "finalSimulationJobHandle" 
        // - The end of each step should depend on "finalHandle" (makes sure all simulation and cleanup is finished)
        void ScheduleStepJobs(SimulationStepInput input, JobHandle inputDeps, out JobHandle finalSimulationJobHandle, out JobHandle finalJobHandle);
    }

    // Default simulation implementation
    public class Simulation : ISimulation
    {
        public SimulationType Type => SimulationType.UnityPhysics;

        internal LowLevel.CollisionEvents CollisionEvents => new LowLevel.CollisionEvents(m_Context.CollisionEventStream);
        internal LowLevel.TriggerEvents TriggerEvents => new LowLevel.TriggerEvents(m_Context.TriggerEventStream);

        private readonly Scheduler m_Scheduler;
        internal Context m_Context;

        public Simulation()
        {
            m_Scheduler = new Scheduler();
            m_Context = new Context();
        }

        public void Dispose()
        {
            m_Scheduler.Dispose();
            DisposeEventStreams();
        }

        public void Step(SimulationStepInput input)
        {
            // TODO : Using the multithreaded version for now, but should do a proper single threaded version
            ScheduleStepJobs(input, new JobHandle(), out JobHandle handle1, out JobHandle handle2);
            JobHandle.CombineDependencies(handle1, handle2).Complete();
        }

        // Schedule all the jobs for the simulation step.
        // Enqueued callbacks can choose to inject additional jobs at defined sync points.
        public unsafe void ScheduleStepJobs(SimulationStepInput input, JobHandle inputDeps, out JobHandle finalSimulationJobHandle, out JobHandle finalJobHandle)
        {
            // Dispose event streams from previous frame
            DisposeEventStreams();

            m_Context = new Context();

            if (input.World.NumDynamicBodies == 0)
            {
                // No need to do anything, since nothing can move
                finalSimulationJobHandle = new JobHandle();
                finalJobHandle = new JobHandle();
                return;
            }

            SimulationCallbacks callbacks = input.Callbacks ?? new SimulationCallbacks();
            JobHandle handle = inputDeps;

            // Find all body pairs that overlap in the broadphase
            handle = input.World.CollisionWorld.Broadphase.ScheduleFindOverlapsJobs(
                out BlockStream dynamicVsDynamicBroadphasePairsStream, out BlockStream staticVsDynamicBroadphasePairsStream, ref m_Context, handle);

            // Create phased dispatch pairs for all interacting body pairs
            handle = m_Scheduler.ScheduleCreatePhasedDispatchPairsJob(
                ref input.World, ref dynamicVsDynamicBroadphasePairsStream, ref staticVsDynamicBroadphasePairsStream, ref m_Context, handle);

            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateDispatchPairs, this, ref input.World, handle);

            // Create contact points & joint Jacobians
            handle = NarrowPhase.ScheduleProcessBodyPairsJobs(ref input.World, input.TimeStep, input.NumSolverIterations, ref m_Context, handle);
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateContacts, this, ref input.World, handle);

            // Create contact Jacobians
            handle = Solver.ScheduleBuildContactJacobiansJobs(ref input.World.DynamicsWorld, input.TimeStep, math.length(input.Gravity), ref m_Context, handle);
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateContactJacobians, this, ref input.World, handle);

            // Solve all Jacobians
            int numIterations = input.NumSolverIterations > 0 ? input.NumSolverIterations : 4;
            handle = Solver.ScheduleSolveJacobiansJobs(ref input.World.DynamicsWorld, input.TimeStep, numIterations, ref m_Context, handle);

            handle = callbacks.Execute(SimulationCallbacks.Phase.PostSolveJacobians, this, ref input.World, handle);

            // Integrate motions
            handle = Integrator.ScheduleIntegrateJobs(ref input.World.DynamicsWorld, input.TimeStep, input.Gravity, handle);

            // Synchronize the collision world
            if (input.SynchronizeCollisionWorld)
            {
                handle = input.World.CollisionWorld.ScheduleUpdateDynamicLayer(ref input.World, input.TimeStep, input.ThreadCountHint, handle);  // TODO: timeStep = 0?
            }

            // Return the final simulation handle
            finalSimulationJobHandle = handle;

            // Return the final handle, which includes disposing temporary arrays
            JobHandle* deps = stackalloc JobHandle[11]
            {
                finalSimulationJobHandle,
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
            finalJobHandle = JobHandleUnsafeUtility.CombineDependencies(deps, 11);
        }

        // Event streams live until the next step, so they should be disposed at the beginning of it
        private void DisposeEventStreams()
        {
            if (m_Context.CollisionEventStream.IsCreated)
            {
                m_Context.CollisionEventStream.Dispose();
            }
            if (m_Context.TriggerEventStream.IsCreated)
            {
                m_Context.TriggerEventStream.Dispose();
            }
        }

        // Temporary data created and destroyed during the step
        public struct Context
        {
            // Built by the scheduler. Groups body pairs into phases in which each
            // body appears at most once, so that the interactions within each phase can be solved
            // in parallel with each other but not with other phases. This is consumed by the
            // ProcessBodyPairsJob, which outputs Contacts and Joint jacobians.
            public NativeList<Scheduler.DispatchPair> PhasedDispatchPairs;

            // Built by the scheduler. Describes the grouping of PhasedBodyPairs
            // which informs how we can schedule the solver jobs and where they read info from.
            // Freed by the PhysicsEnd system.
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
    }
}
