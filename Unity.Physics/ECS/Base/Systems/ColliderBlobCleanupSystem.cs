using Unity.Burst;
using Unity.Collections;
using Unity.Entities;

namespace Unity.Physics.Systems
{
    [RequireMatchingQueriesForUpdate]
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    partial struct ColliderBlobCleanupSystem : ISystem
    {
        EntityQuery m_ColliderBlobCleanupGroup;

        partial struct ColliderBlobCleanupJob : IJobEntity
        {
            public EntityCommandBuffer.ParallelWriter ECB;

            // Process all ColliderBlobCleanupData components on entities that don't have a PhysicsCollider anymore.
            // That is, the containing entity was destroyed but there is still cleanup work to do.
            // For all those, dispose of the collider blob.
            void Execute(in Entity entity, in ColliderBlobCleanupData collider, [ChunkIndexInQuery] int chunkIndex)
            {
                collider.Value.Dispose();
                ECB.RemoveComponent<ColliderBlobCleanupData>(chunkIndex, entity);
            }
        }

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<EndFixedStepSimulationEntityCommandBufferSystem.Singleton>();

            m_ColliderBlobCleanupGroup = new EntityQueryBuilder(Allocator.Temp)
                .WithAll<ColliderBlobCleanupData>()
                .WithAbsent<PhysicsCollider>()
                .Build(ref state);
            state.RequireForUpdate(m_ColliderBlobCleanupGroup);
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var ecb = SystemAPI.GetSingleton<EndFixedStepSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);

            state.Dependency = new ColliderBlobCleanupJob
            {
                ECB = ecb.AsParallelWriter()
            }.ScheduleParallel(m_ColliderBlobCleanupGroup, state.Dependency);
        }
    }
}
