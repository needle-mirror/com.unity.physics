using Unity.Collections;
using Unity.Entities;

namespace Unity.Physics.Authoring
{
    /// <summary>
    ///     Baking system for colliders, called before all other collider baking systems.
    /// </summary>
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    public partial class BeginColliderBakingSystem : SystemBase
    {
        internal BlobAssetComputationContext<int, Collider> BlobComputationContext;

        protected override void OnCreate()
        {
            base.OnCreate();
            HashUtility.Initialize();
        }

        protected override void OnUpdate()
        {
            var bakingSystem = World.GetExistingSystemManaged<BakingSystem>();
            BlobComputationContext = new BlobAssetComputationContext<int, Collider>(bakingSystem.BlobAssetStore, 128, Allocator.TempJob);
        }
    }
}
