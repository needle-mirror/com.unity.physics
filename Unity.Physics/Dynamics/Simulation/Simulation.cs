using System;
using Unity.Collections;
using Unity.Jobs;
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
        
        // Read-write access to simulation data flowing through the step.
        // Warning: Only valid at specific sync points during the step!
        SimulationData.BodyPairs BodyPairs { get; }
        SimulationData.Contacts Contacts { get; }
        SimulationData.Jacobians Jacobians { get; }

        // Events produced by the simulation step.
        // Valid until the start of the next step.
        CollisionEvents CollisionEvents { get; }
        TriggerEvents TriggerEvents { get; }
    }

    // Default simulation implementation
    public class Simulation : ISimulation
    {
        public SimulationType Type => SimulationType.UnityPhysics;

        public SimulationData.BodyPairs BodyPairs => new SimulationData.BodyPairs(m_Context.PhasedDispatchPairsArray);
        public SimulationData.Contacts Contacts => new SimulationData.Contacts(m_Context.Contacts, m_Context.SolverSchedulerInfo);
        public SimulationData.Jacobians Jacobians => new SimulationData.Jacobians(m_Context.Jacobians, m_Context.SolverSchedulerInfo.NumWorkItems);
        public CollisionEvents CollisionEvents => new CollisionEvents(m_Context.CollisionEventStream);
        public TriggerEvents TriggerEvents => new TriggerEvents(m_Context.TriggerEventStream);

        private readonly Scheduler m_Scheduler;
        private Context m_Context;

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
        public void ScheduleStepJobs(SimulationStepInput input, JobHandle inputDeps, out JobHandle finalSimulationJobHandle, out JobHandle finalJobHandle)
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

            // We need to make sure that broadphase tree building is done before we schedule FindOverlapsJobs.
            handle.Complete();

            // Find all body pairs that overlap in the broadphase
            handle = input.World.CollisionWorld.Broadphase.ScheduleFindOverlapsJobs(
                out BlockStream dynamicVsDynamicBroadphasePairsStream, out BlockStream staticVsDynamicBroadphasePairsStream, handle);
            handle.Complete();  // Need to know the total number of pairs before continuing

            // Create phased dispatch pairs for all interacting body pairs
            handle = m_Scheduler.ScheduleCreatePhasedDispatchPairsJob(
                ref input.World, ref dynamicVsDynamicBroadphasePairsStream,  ref staticVsDynamicBroadphasePairsStream, ref m_Context, handle);
            handle.Complete();  // Need to know the total number of work items before continuing
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateDispatchPairs, this, handle);
            m_Context.CreateBodyPairsHandle = handle;

            // Create contact points & joint Jacobians
            handle = NarrowPhase.ScheduleProcessBodyPairsJobs(ref input.World, input.TimeStep, input.NumSolverIterations, ref m_Context, handle);
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateContacts, this, handle);
            m_Context.CreateContactsHandle = handle;

            // Create contact Jacobians
            handle = Solver.ScheduleBuildContactJacobiansJobs(ref input.World.DynamicsWorld, input.TimeStep, ref m_Context, handle);
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostCreateContactJacobians, this, handle);
            m_Context.CreateContactJacobiansHandle = handle;

            // Solve all Jacobians
            int numIterations = input.NumSolverIterations > 0 ? input.NumSolverIterations : 4;
            handle = Solver.ScheduleSolveJacobiansJobs(ref input.World.DynamicsWorld, input.TimeStep, input.Gravity, numIterations, ref m_Context, handle);
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostSolveJacobians, this, handle);
            m_Context.SolveContactJacobiansHandle = handle;

            // Integration motions
            handle = Integrator.ScheduleIntegrateJobs(ref input.World.DynamicsWorld, input.TimeStep, input.Gravity, handle);
            handle = callbacks.Execute(SimulationCallbacks.Phase.PostIntegrateMotions, this, handle);
            m_Context.IntegrateMotionsHandle = handle;

            // Synchronize the collision world
            if (input.SynchronizeCollisionWorld)
            {
                handle = input.World.CollisionWorld.ScheduleUpdateDynamicLayer(ref input.World, input.TimeStep, input.ThreadCountHint, handle);  // TODO: timeStep = 0?
            }

            // Return the final simulation handle
            finalSimulationJobHandle = handle;

            // Return the final handle, which includes disposing temporary arrays
            finalJobHandle = JobHandle.CombineDependencies(finalSimulationJobHandle, m_Context.DisposeBroadphasePairs, m_Context.DisposeContacts);
            finalJobHandle = JobHandle.CombineDependencies(finalJobHandle, m_Context.DisposeJacobians, m_Context.DisposeJointJacobians);
            finalJobHandle = JobHandle.CombineDependencies(finalJobHandle, m_Context.DisposeSolverSchedulerData);
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
            public NativeArray<Scheduler.DispatchPair> PhasedDispatchPairsArray;

            // Built by the scheduler. Describes the grouping of PhasedBodyPairs
            // which informs how we can schedule the solver jobs and where they read info from.
            // Freed by the PhysicsEnd system.
            public Scheduler.SolverSchedulerInfo SolverSchedulerInfo;

            public BlockStream Contacts;
            public BlockStream Jacobians;
            public BlockStream JointJacobians;
            public BlockStream CollisionEventStream;
            public BlockStream TriggerEventStream;

            public JobHandle CreateBodyPairsHandle;
            public JobHandle CreateContactsHandle;
            public JobHandle CreateContactJacobiansHandle;
            public JobHandle SolveContactJacobiansHandle;
            public JobHandle IntegrateMotionsHandle;

            public JobHandle DisposeBroadphasePairs;
            public JobHandle DisposeContacts;
            public JobHandle DisposeJacobians;
            public JobHandle DisposeJointJacobians;
            public JobHandle DisposeSolverSchedulerData;
        }
    }
}
