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
            var translationFromEntity = GetComponentLookup<Translation>();
            var rotationFromEntity = GetComponentLookup<Rotation>();
            var physicsMassFromEntity = GetComponentLookup<PhysicsMass>();
            var physicsColliderFromEntity = GetComponentLookup<PhysicsCollider>();
            Entities.ForEach((Entity ent, in CustomPhysicsProxyDriver driver) => {
                translationFromEntity[ent] = translationFromEntity[driver.rootEntity];
                rotationFromEntity[ent] = rotationFromEntity[driver.rootEntity];

                physicsMassFromEntity[ent] = PhysicsMass.CreateKinematic(physicsColliderFromEntity[driver.rootEntity].MassProperties);
                physicsColliderFromEntity[ent] = physicsColliderFromEntity[driver.rootEntity];
            }).WithEntityQueryOptions(EntityQueryOptions.IncludePrefab).Run();
        }
    }
}
