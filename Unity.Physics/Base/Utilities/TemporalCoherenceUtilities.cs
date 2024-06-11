using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;

namespace Unity.Physics
{
    internal static class TemporalCoherenceUtilities
    {
        public struct Queries
        {
            public EntityQuery DynamicBodyWithoutTemporalCoherenceTagQuery;
            public EntityQuery DynamicBodyWithoutTemporalCoherenceInfoQuery;
            public EntityQuery StaticBodyWithoutTemporalCoherenceTagQuery;
            public EntityQuery StaticBodyWithoutTemporalCoherenceInfoQuery;

            public bool IsEmpty =>
                DynamicBodyWithoutTemporalCoherenceInfoQuery.IsEmpty &&
                DynamicBodyWithoutTemporalCoherenceTagQuery.IsEmpty &&
                StaticBodyWithoutTemporalCoherenceInfoQuery.IsEmpty &&
                StaticBodyWithoutTemporalCoherenceTagQuery.IsEmpty;
        }

        public static Queries CreateQueries(ref SystemState state)
        {
            return new Queries
            {
                DynamicBodyWithoutTemporalCoherenceTagQuery = CreateDynamicBodyQueryBuilder().WithNone<PhysicsTemporalCoherenceTag>().Build(ref state),
                DynamicBodyWithoutTemporalCoherenceInfoQuery = CreateDynamicBodyQueryBuilder().WithNone<PhysicsTemporalCoherenceInfo>().Build(ref state),

                StaticBodyWithoutTemporalCoherenceTagQuery = CreateStaticBodyQueryBuilder().WithNone<PhysicsTemporalCoherenceTag>().Build(ref state),
                StaticBodyWithoutTemporalCoherenceInfoQuery = CreateStaticBodyQueryBuilder().WithNone<PhysicsTemporalCoherenceInfo>().Build(ref state),
            };
        }

        public static void AddTemporalCoherenceComponents(ref Queries queries, ref EntityCommandBuffer ecb)
        {
            // dynamic bodies
            ecb.AddComponent<PhysicsTemporalCoherenceTag>(queries.DynamicBodyWithoutTemporalCoherenceTagQuery,
                EntityQueryCaptureMode.AtPlayback);
            ecb.AddComponent<PhysicsTemporalCoherenceInfo>(queries.DynamicBodyWithoutTemporalCoherenceInfoQuery,
                EntityQueryCaptureMode.AtPlayback);

            // static bodies
            ecb.AddComponent<PhysicsTemporalCoherenceTag>(queries.StaticBodyWithoutTemporalCoherenceTagQuery,
                EntityQueryCaptureMode.AtPlayback);
            ecb.AddComponent<PhysicsTemporalCoherenceInfo>(queries.StaticBodyWithoutTemporalCoherenceInfoQuery,
                EntityQueryCaptureMode.AtPlayback);
        }

        public static EntityQueryBuilder CreateDynamicBodyQueryBuilder()
        {
            return new EntityQueryBuilder(Allocator.Temp)
                .WithAll<PhysicsVelocity, LocalTransform, PhysicsWorldIndex>()
                .WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities);
        }

        public static EntityQueryBuilder CreateStaticBodyQueryBuilder()
        {
            return new EntityQueryBuilder(Allocator.Temp)
                .WithAll<PhysicsCollider, PhysicsWorldIndex>()
                .WithAny<LocalToWorld, LocalTransform>()
                .WithNone<PhysicsVelocity>()
                .WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities);
        }
    }
}
