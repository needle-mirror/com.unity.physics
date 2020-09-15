using Unity.Collections;
using Unity.Entities;

namespace Unity.Physics.Authoring
{
    [AlwaysUpdateSystem]
    public sealed class BeginColliderConversionSystem : GameObjectConversionSystem
    {
        // TODO: move this to a ColliderConversionSystemGroup when groups are supported for GameObjectConversionSystems
        internal BlobAssetComputationContext<int, Collider> BlobComputationContext;

        protected override void OnCreate()
        {
            base.OnCreate();
            HashUtility.Initialize();
        }

        protected override void OnUpdate() =>
            BlobComputationContext = new BlobAssetComputationContext<int, Collider>(BlobAssetStore, 128, Allocator.TempJob);
    }
}
