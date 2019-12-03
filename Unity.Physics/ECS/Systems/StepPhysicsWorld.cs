using System;
using Unity.Entities;
using Unity.Jobs;
using UnityEngine.Assertions;

namespace Unity.Physics.Systems
{
    // Simulates the physics world forwards in time
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(ExportPhysicsWorld)), AlwaysUpdateSystem]
    public class StepPhysicsWorld : JobComponentSystem
    {
        // The simulation implementation
        public ISimulation Simulation { get; private set; } = new DummySimulation();

        // The final job handle produced by this system.
        // This includes all simulation jobs as well as array disposal jobs.
        public JobHandle FinalJobHandle => Simulation.FinalJobHandle;

        // The final simulation job handle produced by this system.
        // Systems which read the simulation results should depend on this.
        public JobHandle FinalSimulationJobHandle => Simulation.FinalSimulationJobHandle;

        // Optional callbacks to execute while scheduling the next simulation step
        private SimulationCallbacks m_Callbacks = new SimulationCallbacks();

        // Simulation factory
        public delegate ISimulation SimulationCreator();
        private readonly SimulationCreator[] m_SimulationCreators = new SimulationCreator[k_NumSimulationTypes];

        BuildPhysicsWorld m_BuildPhysicsWorldSystem;

        // needs to match the number of SimulationType enum members
        internal static int k_NumSimulationTypes = 3;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();

#if !NET_DOTS
            Assert.AreEqual(Enum.GetValues(typeof(SimulationType)).Length, k_NumSimulationTypes);
#endif

            RegisterSimulation(SimulationType.NoPhysics, () => new DummySimulation());
            RegisterSimulation(SimulationType.UnityPhysics, () => new Simulation());
            RegisterSimulation(SimulationType.HavokPhysics, () =>
                throw new NotSupportedException("Havok Physics package not present. Use the package manager to add it."));

            base.OnCreate();
        }

        protected override void OnDestroy()
        {
            Simulation.Dispose();
            base.OnDestroy();
        }

        // Register a simulation creator
        public void RegisterSimulation(SimulationType type, SimulationCreator creator)
        {
            m_SimulationCreators[(int)type] = creator;
        }

        // Enqueue a callback to run during scheduling of the next simulation step
        public void EnqueueCallback(SimulationCallbacks.Phase phase, SimulationCallbacks.Callback callback, JobHandle dependency = default(JobHandle))
        {
            m_Callbacks.Enqueue(phase, callback, dependency);
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var handle = JobHandle.CombineDependencies(m_BuildPhysicsWorldSystem.FinalJobHandle, inputDeps);

            PhysicsStep stepComponent = PhysicsStep.Default;
            if (HasSingleton<PhysicsStep>())
            {
                stepComponent = GetSingleton<PhysicsStep>();
            }

            // Swap the simulation implementation if the requested type changed
            if (Simulation.Type != stepComponent.SimulationType)
            {
                Simulation.Dispose();
                Simulation = m_SimulationCreators[(int)stepComponent.SimulationType]();
            }

#if !UNITY_DOTSPLAYER
            float timeStep = UnityEngine.Time.fixedDeltaTime;
#else
            float timeStep = Time.DeltaTime;
#endif

            // Schedule the simulation jobs
            Simulation.ScheduleStepJobs(new SimulationStepInput()
            {
                World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                TimeStep = timeStep,
                ThreadCountHint = stepComponent.ThreadCountHint,
                Gravity = stepComponent.Gravity,
                SynchronizeCollisionWorld = false,
                NumSolverIterations = stepComponent.SolverIterationCount,
                Callbacks = m_Callbacks
            }, handle);

            // Clear the callbacks. User must enqueue them again before the next step.
            m_Callbacks.Clear();

            // Return the final simulation handle
            // (Not FinalJobHandle since other systems shouldn't need to depend on the dispose jobs) 
            return JobHandle.CombineDependencies(Simulation.FinalSimulationJobHandle, handle);
        }

        // A simulation which does nothing
        class DummySimulation : ISimulation
        {
            public SimulationType Type => SimulationType.NoPhysics;

            public void Dispose() { }
            public void Step(SimulationStepInput input) { }
            public void ScheduleStepJobs(SimulationStepInput input, JobHandle inputDeps) { }
            public JobHandle FinalSimulationJobHandle => new JobHandle();
            public JobHandle FinalJobHandle => new JobHandle();
        }
    }
}
