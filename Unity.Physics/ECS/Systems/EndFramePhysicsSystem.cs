using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;

namespace Unity.Physics.Systems
{
    // A system which waits for any remaining physics jobs to finish
    [UpdateAfter(typeof(BuildPhysicsWorld)), UpdateAfter(typeof(StepPhysicsWorld)),
     UpdateAfter(typeof(ExportPhysicsWorld))]
    public class EndFramePhysicsSystem : ComponentSystem
    {
        public NativeList<JobHandle> HandlesToWaitFor;

        BuildPhysicsWorld m_BuildPhysicsWorld;
        StepPhysicsWorld m_StepPhysicsWorld;
        ExportPhysicsWorld m_ExportPhysicsWorld;

        protected override void OnCreateManager()
        {
            HandlesToWaitFor = new NativeList<JobHandle>(16, Allocator.Persistent);

            m_BuildPhysicsWorld = World.GetOrCreateManager<BuildPhysicsWorld>();
            m_StepPhysicsWorld = World.GetOrCreateManager<StepPhysicsWorld>();
            m_ExportPhysicsWorld = World.GetOrCreateManager<ExportPhysicsWorld>();
        }

        protected override void OnDestroyManager()
        {
            HandlesToWaitFor.Dispose();
        }

        protected override void OnUpdate()
        {
            HandlesToWaitFor.Add(m_BuildPhysicsWorld.FinalJobHandle);
            HandlesToWaitFor.Add(m_StepPhysicsWorld.FinalJobHandle);
            HandlesToWaitFor.Add(m_ExportPhysicsWorld.FinalJobHandle);

            JobHandle handle = JobHandle.CombineDependencies(HandlesToWaitFor);
            HandlesToWaitFor.Clear();
            handle.Complete();
        }
    }
}
