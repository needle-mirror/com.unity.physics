using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;

namespace Unity.Physics.Systems
{
    // A system which combines the dependencies of all other physics jobs created during this frame into a single handle,
    // so that any system which depends on all physics work to be finished can just depend on this single handle.
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateAfter(typeof(StepPhysicsWorld)), UpdateAfter(typeof(ExportPhysicsWorld))]
    public class EndFramePhysicsSystem : JobComponentSystem
    {
        // Extra physics jobs added by user systems
        public NativeList<JobHandle> HandlesToWaitFor;

        // A combined handle of all built-in and user physics jobs
        public JobHandle FinalJobHandle { get; private set; }

        BuildPhysicsWorld m_BuildPhysicsWorld;
        StepPhysicsWorld m_StepPhysicsWorld;
        ExportPhysicsWorld m_ExportPhysicsWorld;

        JobHandle CombineDependencies()
        {
            // Add built-in jobs
            HandlesToWaitFor.Add(m_BuildPhysicsWorld.FinalJobHandle);
            HandlesToWaitFor.Add(m_StepPhysicsWorld.FinalJobHandle);
            HandlesToWaitFor.Add(m_ExportPhysicsWorld.FinalJobHandle);
            var handle = JobHandle.CombineDependencies(HandlesToWaitFor);
            HandlesToWaitFor.Clear();
            return handle;
        }

        protected override void OnCreate()
        {
            HandlesToWaitFor = new NativeList<JobHandle>(16, Allocator.Persistent);
            FinalJobHandle = new JobHandle();

            m_BuildPhysicsWorld = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_StepPhysicsWorld = World.GetOrCreateSystem<StepPhysicsWorld>();
            m_ExportPhysicsWorld = World.GetOrCreateSystem<ExportPhysicsWorld>();
        }

        protected override void OnDestroy()
        {
            CombineDependencies().Complete();
            HandlesToWaitFor.Dispose();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            FinalJobHandle = JobHandle.CombineDependencies(CombineDependencies(), inputDeps);
            return FinalJobHandle;
        }
    }
}
