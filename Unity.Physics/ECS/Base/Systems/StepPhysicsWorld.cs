using System;
using Unity.Entities;
using Unity.Jobs;

namespace Unity.Physics.Systems
{
    // Simulates the physics world forwards in time
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateBefore(typeof(ExportPhysicsWorld)), AlwaysUpdateSystem]
    public partial class StepPhysicsWorld : SystemBase
    {
        // The simulation implementation
        public ISimulation Simulation => m_Stepper.Simulation;

        // The final simulation job handle produced by this system.
        // Systems which read the simulation results should depend on this.
        public JobHandle FinalSimulationJobHandle => Simulation.FinalSimulationJobHandle;

        private PhysicsWorldStepper m_Stepper;

        BuildPhysicsWorld m_BuildPhysicsWorldSystem;

        // Entity query that is used for preventing other systems from changing colliders through PhysicsCollider component
        // (both RigidBody from PhysicsWorld and PhysicsCollider component point to the same collider blob asset).
        internal EntityQuery PhysicsColliderQuery;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();

            m_Stepper = new PhysicsWorldStepper();

            // This line is introduced for the following reason:
            // Imagine a situation: system updates after ExportPhysicsWorld that
            // wants to modify PhysicsCollider component data using an EntityManager,
            // the code for that would look something like:
            // 1.    var colliderData = EntityManager.GetComponentData<PhysicsCollider>(desiredEntity);
            // 2.    modifyCollider(ref colliderData);
            // 3.    EntityManager.SetComponentData<PhysicsCollider>(desiredEntity, colliderData);
            // The problem in the above code is that line 1 executes after all the jobs that write to PhysicsCollider
            // component data, which DOES NOT include physics simulation jobs, so it means that the line 2 can proceed
            // in a middle of a simulation and modify collider data, which would lead to non desired/correct/deterministic situation.
            // To make the matters worse, this will not get caught by integrity checks, since before executing line 3
            // ECS will complete all of the jobs scheduled previously that read or write PhysicsCollider data, one of which is the
            // CheckColliderIntegrityJob, meaning that integrity will be checked before the new collider gets written into a chunk,
            // and it will raise no errors, since integrity jobs depend on ECS versions, which get updated only when something writes to a chunk.
            // In samples project, there is a system that does all of that (VerifyActivation.cs, TestSystem).
            // The line below solves that problem, since it makes an artificial read/write dependency on PhysicsCollider data, with this modification,
            // line 1 of the above code would complete all jobs that write to PhysicsCollider, including the simulation jobs.
            PhysicsColliderQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new ComponentType[] { typeof(PhysicsCollider) }
            });

            base.OnCreate();
        }

        protected override void OnDestroy()
        {
            m_Stepper.Dispose();
            base.OnDestroy();
        }

        // Enqueue a callback to run during scheduling of the next simulation step
        public void EnqueueCallback(SimulationCallbacks.Phase phase, SimulationCallbacks.Callback callback, JobHandle dependency = default(JobHandle))
        {
            m_Stepper.EnqueueCallback(phase, callback, dependency);
        }

        protected override void OnStartRunning()
        {
            base.OnStartRunning();
            this.RegisterPhysicsRuntimeSystemReadWrite();
        }

        protected override void OnUpdate()
        {
            PhysicsStep stepComponent = PhysicsStep.Default;
            if (HasSingleton<PhysicsStep>())
            {
                stepComponent = GetSingleton<PhysicsStep>();
            }

            float timeStep = Time.DeltaTime;

            // Schedule the simulation jobs
            m_Stepper.ScheduleSimulationStepJobs(
                stepComponent.SimulationType,
                m_BuildPhysicsWorldSystem.WorldFilter.Value,
                new SimulationStepInput()
                {
                    World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                    TimeStep = timeStep,
                    Gravity = stepComponent.Gravity,
                    SynchronizeCollisionWorld = stepComponent.SynchronizeCollisionWorld > 0,
                    NumSolverIterations = stepComponent.SolverIterationCount,
                    SolverStabilizationHeuristicSettings = stepComponent.SolverStabilizationHeuristicSettings,
                    HaveStaticBodiesChanged = m_BuildPhysicsWorldSystem.PhysicsData.HaveStaticBodiesChanged
                },
                Dependency, stepComponent.MultiThreaded > 0);

            // Include the final simulation handle
            // (Not FinalJobHandle since other systems shouldn't need to depend on the dispose jobs)
            Dependency = JobHandle.CombineDependencies(Dependency, FinalSimulationJobHandle);
        }

        [Obsolete("AddInputDependency() has been deprecated. Please call RegisterPhysicsRuntimeSystemReadWrite() or RegisterPhysicsRuntimeSystemReadOnly() in your system's OnStartRunning() to achieve the same effect. (RemovedAfter 2021-05-01)", true)]
        public void AddInputDependency(JobHandle inputDep) {}

        [Obsolete("GetOutputDependency() has been deprecated. Please call RegisterPhysicsRuntimeSystemReadWrite() or RegisterPhysicsRuntimeSystemReadOnly() in your system's OnStartRunning() to achieve the same effect. (RemovedAfter 2021-05-01)", true)]
        public JobHandle GetOutputDependency() => default;
    }
}
