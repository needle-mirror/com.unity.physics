using System;
using Unity.Entities;
using Unity.Jobs;

namespace Unity.Physics.Systems
{
    // A system which combines the dependencies of all other physics jobs created during this frame into a single handle,
    // so that any system which depends on all physics work to be finished can just depend on this single handle.
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateAfter(typeof(StepPhysicsWorld)), UpdateAfter(typeof(ExportPhysicsWorld))]
    public partial class EndFramePhysicsSystem : SystemBase
    {
        private BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        private StepPhysicsWorld m_StepPhysicsWorldSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_StepPhysicsWorldSystem = World.GetOrCreateSystem<StepPhysicsWorld>();
        }

        protected override void OnDestroy()
        {
            // As the last physics system, make sure to complete all dependencies
            JobHandle.CombineDependencies(Dependency, m_StepPhysicsWorldSystem.Simulation.FinalJobHandle).Complete();
        }

        protected override void OnStartRunning()
        {
            base.OnStartRunning();

            // Declare as read/write to physics although it doesn't really write, but to make sure systems before and after it have a proper dependency.
            this.RegisterPhysicsRuntimeSystemReadWrite();
        }

        protected override void OnUpdate()
        {
            // Inform the BuildPhysicsWorld of the dependency to complete before scheduling new jobs.
            // Using Simulation.FinalJobHandle to make BuildPhysicsWorld wait for dispose handles as well,
            // so we're sure that temp memory from current step is disposed before the next step starts.
            m_BuildPhysicsWorldSystem.AddInputDependencyToComplete(JobHandle.CombineDependencies(Dependency, m_StepPhysicsWorldSystem.Simulation.FinalJobHandle));
        }

        [Obsolete("AddInputDependency() has been deprecated. Please call RegisterPhysicsRuntimeSystemReadWrite() or RegisterPhysicsRuntimeSystemReadOnly() in your system's OnStartRunning() to achieve the same effect. (RemovedAfter 2021-05-01)", true)]
        public void AddInputDependency(JobHandle inputDep) {}

        [Obsolete("GetOutputDependency() has been deprecated. Please call RegisterPhysicsRuntimeSystemReadWrite() or RegisterPhysicsRuntimeSystemReadOnly() in your system's OnStartRunning() to achieve the same effect. (RemovedAfter 2021-05-01)", true)]
        public JobHandle GetOutputDependency() => default;
    }
}
