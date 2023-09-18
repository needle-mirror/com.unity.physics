using Unity.Burst;
using Unity.Entities;
using Unity.Physics.Extensions;

namespace Unity.Physics.Systems
{
    [RequireMatchingQueriesForUpdate]
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateAfter(typeof(AfterPhysicsSystemGroup))]
    public partial struct EnsureUniqueColliderSystem : ISystem
    {
        [BurstCompile]
        partial struct MakeUniqueJob : IJobEntity
        {
            public EntityCommandBuffer.ParallelWriter ECB;

            void Execute(in Entity entity, in EnsureUniqueColliderBlobTag tag, ref PhysicsCollider collider, [ChunkIndexInQuery] int chunkIndex)
            {
                // If the collider is not unique but should be, we need to ensure it is
                if (!collider.IsUnique)
                {
                    collider.MakeUnique(entity, ECB, chunkIndex);
                    ECB.RemoveComponent<EnsureUniqueColliderBlobTag>(chunkIndex, entity);
                }
            }
        }

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.RequireForUpdate<BeginFixedStepSimulationEntityCommandBufferSystem.Singleton>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var ecb = SystemAPI.GetSingleton<BeginFixedStepSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(state.WorldUnmanaged);

            // Run on all entities with colliders which are required to be unique and ensure that they are.
            state.Dependency = new MakeUniqueJob
            {
                ECB = ecb.AsParallelWriter()
            }.ScheduleParallel(state.Dependency);
        }
    }
}
