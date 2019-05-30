using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;

namespace Unity.Physics.Systems
{
    // A system which waits for any remaining physics jobs to finish
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateAfter(typeof(StepPhysicsWorld)), UpdateAfter(typeof(ExportPhysicsWorld))]
    public class EndFramePhysicsSystem : JobComponentSystem
    {
        public NativeList<JobHandle> HandlesToWaitFor;

        BuildPhysicsWorld m_BuildPhysicsWorld;
        StepPhysicsWorld m_StepPhysicsWorld;
        ExportPhysicsWorld m_ExportPhysicsWorld;

        void AddHandles()
        {
            HandlesToWaitFor.Add(m_BuildPhysicsWorld.FinalJobHandle);
            HandlesToWaitFor.Add(m_StepPhysicsWorld.FinalJobHandle);
            HandlesToWaitFor.Add(m_ExportPhysicsWorld.FinalJobHandle);
        }

        protected override void OnCreate()
        {
            HandlesToWaitFor = new NativeList<JobHandle>(16, Allocator.Persistent);

            m_BuildPhysicsWorld = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_StepPhysicsWorld = World.GetOrCreateSystem<StepPhysicsWorld>();
            m_ExportPhysicsWorld = World.GetOrCreateSystem<ExportPhysicsWorld>();
        }

        protected override void OnDestroyManager()
        {
            AddHandles();
            JobHandle.CompleteAll(HandlesToWaitFor);
            HandlesToWaitFor.Dispose();
        }

        protected override JobHandle OnUpdate(JobHandle jobs)
        {
            // Wait for previous frame jobs
            JobHandle.CompleteAll(HandlesToWaitFor);
            
            HandlesToWaitFor.Clear();
            AddHandles();

            return jobs;
        }
    }
}
