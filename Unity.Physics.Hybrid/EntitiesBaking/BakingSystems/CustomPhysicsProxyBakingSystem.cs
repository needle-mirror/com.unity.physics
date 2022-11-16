using Unity.Entities;
using Unity.Transforms;

namespace Unity.Physics.Authoring
{
    [UpdateInGroup(typeof(PostBakingSystemGroup))]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    public partial class CustomPhysicsProxyBakingSystem : SystemBase
    {
        protected override void OnUpdate()
        {
#if !ENABLE_TRANSFORM_V1
            var transformFromEntity = GetComponentLookup<LocalTransform>();
#else
            var translationFromEntity = GetComponentLookup<Translation>();
            var rotationFromEntity = GetComponentLookup<Rotation>();
#endif
            var physicsMassFromEntity = GetComponentLookup<PhysicsMass>();
            var physicsColliderFromEntity = GetComponentLookup<PhysicsCollider>();
            foreach (var (driver, entity) in SystemAPI.Query<RefRW<CustomPhysicsProxyDriver>>().WithEntityAccess()
                         .WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities))
            {
#if !ENABLE_TRANSFORM_V1
                transformFromEntity[entity] = transformFromEntity[driver.ValueRW.rootEntity];
#else
                translationFromEntity[entity] = translationFromEntity[driver.ValueRW.rootEntity];
                rotationFromEntity[entity] = rotationFromEntity[driver.ValueRW.rootEntity];
#endif
                physicsMassFromEntity[entity] = PhysicsMass.CreateKinematic(physicsColliderFromEntity[driver.ValueRW.rootEntity].MassProperties);
                physicsColliderFromEntity[entity] = physicsColliderFromEntity[driver.ValueRW.rootEntity];
            }
        }
    }
}
