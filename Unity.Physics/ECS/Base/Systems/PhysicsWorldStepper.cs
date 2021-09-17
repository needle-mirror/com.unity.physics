using System;
using Unity.Jobs;

namespace Unity.Physics.Systems
{
    /// <summary>
    /// Utility class for scheduling physics simulation jobs
    /// </summary>
    public class PhysicsWorldStepper : IDisposable
    {
        /// <summary>
        /// Simulation factory delegate
        /// </summary>
        public delegate ISimulation SimulationCreator(uint physicsWorldIndex);

        /// <summary>
        /// Current simulation implementation
        /// </summary>
        public ISimulation Simulation { get; set; } = new DummySimulation();

        /// <summary>
        /// The final simulation job handle produced by this system. Systems which read the simulation results should depend on this.
        /// </summary>
        public JobHandle FinalSimulationJobHandle => Simulation.FinalSimulationJobHandle;

        /// <summary>
        /// The final scheduled job, including all simulation and cleanup. The end of each step should depend on this.
        /// </summary>
        public JobHandle FinalJobHandle => Simulation.FinalJobHandle;

        /// <summary>
        /// Needs to match the number of SimulationType enum members
        /// </summary>
        internal static int k_NumSimulationTypes = 3;

        /// <summary>
        /// Simulation factories that are currently in use
        /// </summary>
        private static SimulationCreator[] s_SimulationCreators;

        /// <summary>
        /// Optional callbacks to execute while scheduling the next simulation step
        /// </summary>
        private SimulationCallbacks m_Callbacks = new SimulationCallbacks();

        static PhysicsWorldStepper()
        {
#if !NET_DOTS
            UnityEngine.Assertions.Assert.AreEqual(Enum.GetValues(typeof(SimulationType)).Length, k_NumSimulationTypes);
#endif

            s_SimulationCreators = new SimulationCreator[k_NumSimulationTypes];

            RegisterSimulation(SimulationType.NoPhysics, (uint physicsWorldIndex) => new DummySimulation());
            RegisterSimulation(SimulationType.UnityPhysics, (uint physicsWorldIndex) => new Simulation());
            RegisterSimulation(SimulationType.HavokPhysics, (uint physicsWorldIndex) =>
                throw new NotSupportedException("Havok Physics package not present. Use the package manager to add it."));
        }

        /// <summary>
        ///  Register a simulation creator
        /// </summary>
        public static void RegisterSimulation(SimulationType type, SimulationCreator creator)
        {
            s_SimulationCreators[(int)type] = creator;
        }

        /// <summary>
        /// Returns a new ISimulation of specified type (per last registered creator)
        /// </summary>
        public static ISimulation CreateSimulation(SimulationType type, uint physicsWorldIndex)
        {
            return s_SimulationCreators[(int)type](physicsWorldIndex);
        }

        public void Dispose()
        {
            Simulation.Dispose();
        }

        /// <summary>
        /// Enqueue a callback to run during scheduling of the next simulation step
        /// </summary>
        public void EnqueueCallback(SimulationCallbacks.Phase phase, SimulationCallbacks.Callback callback, JobHandle dependency = default) => m_Callbacks.Enqueue(phase, callback, dependency);

        /// <summary>
        /// Schedule a set of jobs to step the simulation.
        /// </summary>
        public SimulationJobHandles ScheduleSimulationStepJobs(SimulationType simType, uint physicsWorldIndex, in SimulationStepInput stepInput, in JobHandle inputDep, bool multiThreaded)
        {
            // Swap the simulation implementation if the requested type changed
            if (Simulation.Type != simType)
            {
                Simulation.Dispose();
                Simulation = s_SimulationCreators[(int)simType](physicsWorldIndex);
            }

            // Schedule the simulation jobs, not necessarily per stepComponent
            SimulationJobHandles handles = Simulation.ScheduleStepJobs(stepInput, m_Callbacks, inputDep, multiThreaded);

            // Clear the callbacks. User must enqueue them again before the next step.
            m_Callbacks.Clear();

            return handles;
        }
    }

    // A simulation which does nothing
    internal class DummySimulation : ISimulation
    {
        public SimulationType Type => SimulationType.NoPhysics;

        public void Dispose() {}
        public void Step(SimulationStepInput input) {}
        public SimulationJobHandles ScheduleStepJobs(SimulationStepInput input, SimulationCallbacks callbacks, JobHandle inputDeps, bool multiThreaded = true) =>
            new SimulationJobHandles(inputDeps);
        public JobHandle FinalSimulationJobHandle => new JobHandle();
        public JobHandle FinalJobHandle => new JobHandle();
    }
}
