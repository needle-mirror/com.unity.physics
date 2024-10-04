using Unity.Burst;
using Unity.Entities;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// Baking system which ensures that baked rigid body entities that have to be processed incrementally during
    /// the broadphase contain the critical <see cref="PhysicsTemporalCoherenceTag"/> component whenever
    /// incremental broadphase updates are enabled in the <see cref="PhysicsStep"/> component.
    /// </summary>
    [RequireMatchingQueriesForUpdate]
    [UpdateAfter(typeof(EndColliderBakingSystem))]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    public partial struct TemporalCoherenceBakingSystem : ISystem
    {
        private EntityQueryMask m_DynamicBodyQueryMask;
        private EntityQueryMask m_StaticBodyQueryMask;
        private EntityQuery m_DynamicBodyWithTemporalCoherenceQuery;
        private EntityQuery m_DynamicBodyWithoutTemporalCoherenceQuery;
        private EntityQuery m_StaticBodyWithTemporalCoherenceQuery;
        private EntityQuery m_StaticBodyWithoutTemporalCoherenceQuery;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_DynamicBodyQueryMask = TemporalCoherenceUtilities.CreateDynamicBodyQueryBuilder().Build(ref state).GetEntityQueryMask();
            m_StaticBodyQueryMask = TemporalCoherenceUtilities.CreateStaticBodyQueryBuilder().Build(ref state).GetEntityQueryMask();

            m_DynamicBodyWithTemporalCoherenceQuery = TemporalCoherenceUtilities.CreateDynamicBodyQueryBuilder().WithAll<PhysicsTemporalCoherenceTag>().Build(ref state);
            m_DynamicBodyWithoutTemporalCoherenceQuery = TemporalCoherenceUtilities.CreateDynamicBodyQueryBuilder().WithNone<PhysicsTemporalCoherenceTag>().Build(ref state);

            m_StaticBodyWithTemporalCoherenceQuery = TemporalCoherenceUtilities.CreateStaticBodyQueryBuilder().WithAll<PhysicsTemporalCoherenceTag>().Build(ref state);
            m_StaticBodyWithoutTemporalCoherenceQuery = TemporalCoherenceUtilities.CreateStaticBodyQueryBuilder().WithNone<PhysicsTemporalCoherenceTag>().Build(ref state);
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            if (!SystemAPI.TryGetSingleton<PhysicsStep>(out var physicsStep))
            {
                physicsStep = PhysicsStep.Default;
            }

            using var ecb = new EntityCommandBuffer(state.WorldUpdateAllocator);

            // add temporal coherence data if incremental broadphase is used
            if (physicsStep.IncrementalDynamicBroadphase || physicsStep.IncrementalStaticBroadphase)
            {
                // dynamic bodies
                ecb.AddComponent<PhysicsTemporalCoherenceTag>(m_DynamicBodyWithoutTemporalCoherenceQuery,
                    EntityQueryCaptureMode.AtPlayback);

                // static bodies
                ecb.AddComponent<PhysicsTemporalCoherenceTag>(m_StaticBodyWithoutTemporalCoherenceQuery,
                    EntityQueryCaptureMode.AtPlayback);
            }
            else
            {
                // dynamic bodies
                ecb.RemoveComponent<PhysicsTemporalCoherenceTag>(m_DynamicBodyWithTemporalCoherenceQuery,
                    EntityQueryCaptureMode.AtPlayback);

                // static bodies
                ecb.RemoveComponent<PhysicsTemporalCoherenceTag>(m_StaticBodyWithTemporalCoherenceQuery,
                    EntityQueryCaptureMode.AtPlayback);
            }

            // cleanup temporal coherence data for entities that stopped being rigid bodies (either dynamic or static)
            {
                foreach (var(_, entity) in SystemAPI.Query<PhysicsTemporalCoherenceTag>()
                         .WithEntityAccess()
                         .WithOptions(EntityQueryOptions.IncludePrefab |
                             EntityQueryOptions.IncludeDisabledEntities))
                {
                    if (!(m_DynamicBodyQueryMask.MatchesIgnoreFilter(entity) ||
                          m_StaticBodyQueryMask.MatchesIgnoreFilter(entity)))
                    {
                        ecb.RemoveComponent<PhysicsTemporalCoherenceTag>(entity);
                    }
                }
            }

            ecb.Playback(state.EntityManager);
        }
    }
}
