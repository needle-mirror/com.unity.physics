using System;
using System.ComponentModel;
using Unity.Entities;
using Unity.Jobs;
using UnityEngine.Assertions;

namespace Unity.Physics.Systems
{
    // Simulates the physics world forwards in time
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(ExportPhysicsWorld))]
    public class StepPhysicsWorld : JobComponentSystem
    {
        // The simulation implementation
        public ISimulation Simulation { get; private set; }

        // The final job handle produced by this system.
        // This includes all simulation jobs as well as array disposal jobs.
        public JobHandle FinalJobHandle { get; private set; }

        // The final simulation job handle produced by this system.
        // Systems which read the simulation results should depend on this.
        public JobHandle FinalSimulationJobHandle { get; private set; }

        // Optional callbacks to execute while scheduling the next simulation step
        private SimulationCallbacks m_Callbacks;

        // Simulation factory
        public delegate ISimulation SimulationCreator();
        private readonly SimulationCreator[] m_SimulationCreators = new SimulationCreator[k_NumSimulationTypes];

        BuildPhysicsWorld m_BuildPhysicsWorldSystem;

        // Entity group queries
        private EntityQuery m_PhysicsEntityGroup;

        // needs to match the number of SimulationType enum members
        internal static int k_NumSimulationTypes = 3;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();

#if !NET_DOTS
            Assert.AreEqual(Enum.GetValues(typeof(SimulationType)).Length, k_NumSimulationTypes);
#endif

            Simulation = new DummySimulation();
            RegisterSimulation(SimulationType.NoPhysics, () => new DummySimulation());
            RegisterSimulation(SimulationType.UnityPhysics, () => new Simulation());
            RegisterSimulation(SimulationType.HavokPhysics, () =>
                throw new NotSupportedException("Havok Physics package not present. Use the package manager to add it."));

            FinalSimulationJobHandle = new JobHandle();
            FinalJobHandle = new JobHandle();

            m_Callbacks = new SimulationCallbacks();

            base.OnCreate();

            // Needed to keep ComponentSystem active when no Entity has PhysicsStep component
            m_PhysicsEntityGroup = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[]
                {
                    typeof(PhysicsVelocity)
                }
            });
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
            var stepInput = new SimulationStepInput
            {
                World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                TimeStep = timeStep,
                ThreadCountHint = stepComponent.ThreadCountHint,
                Gravity = stepComponent.Gravity,
                SynchronizeCollisionWorld = false,
                NumSolverIterations = stepComponent.SolverIterationCount,
                Callbacks = m_Callbacks
            };
            Simulation.ScheduleStepJobs(stepInput, handle);
            FinalSimulationJobHandle = Simulation.FinalSimulationJobHandle;
            FinalJobHandle = Simulation.FinalJobHandle;

            // Clear the callbacks. User must enqueue them again before the next step.
            m_Callbacks.Clear();

            return handle;
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

            #region Obsolete
            [EditorBrowsable(EditorBrowsableState.Never)]
            [Obsolete("ScheduleStepJobs(SimulationStepInput, JobHandle, out JobHandle, out JobHandle) has been deprecated. Use ScheduleStepJobs(SimulationStepInput, JobHandle) instead. (RemovedAfter 2019-10-15)", true)]
            void ISimulation.ScheduleStepJobs(SimulationStepInput input, JobHandle inputDeps, out JobHandle finalSimulationJobHandle, out JobHandle finalJobHandle) =>
                throw new NotImplementedException();
            #endregion
        }
    }
}
