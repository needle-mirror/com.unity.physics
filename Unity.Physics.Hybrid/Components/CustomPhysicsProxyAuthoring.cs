using System;
using UnityEngine;
using Unity.Entities;
using Unity.Transforms;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// This authoring component will create multiple proxy entities in different physics worlds.
    /// </summary>
    [AddComponentMenu("Entities/Physics/Custom Physics Proxy")]
    [HelpURL(HelpURLs.CustomPhysicsProxyAuthoring)]
    public sealed class CustomPhysicsProxyAuthoring : MonoBehaviour
    {
        [Range(0f, 1f)]
        [Tooltip("Coefficient in range [0,1] denoting how much the client body will be driven by position (teleported), while the rest of position diff will be velocity-driven")]
        public float FirstOrderGain = 0.0f;

        /// <summary>
        /// A bitmask enum for physics world indices.
        /// </summary>
        [Flags]
        public enum TargetWorld : byte
        {
            DefaultWorld = 1,
            World1 = 2,
            World2 = 4,
            World3 = 8
        }
        /// <summary>
        /// A mask of physics world indices in which proxy entities should be created.
        /// </summary>
        public TargetWorld TargetPhysicsWorld = TargetWorld.World1;
    }


    class CustomPhysicsProxyBaker : Baker<CustomPhysicsProxyAuthoring>
    {
        public override void Bake(CustomPhysicsProxyAuthoring authoring)
        {
            for (int i = 0; i < 4; ++i)
            {
                if (((int)authoring.TargetPhysicsWorld & (1 << i)) == 0)
                    continue;
                var proxyEnt = CreateAdditionalEntity(TransformUsageFlags.WriteGlobalTransform | TransformUsageFlags.ReadLocalToWorld);
                AddComponent(proxyEnt, new CustomPhysicsProxyDriver {rootEntity = GetEntity(), FirstOrderGain = authoring.FirstOrderGain});
#if !ENABLE_TRANSFORM_V1
                AddComponent(proxyEnt, default(LocalTransform));
                AddComponent(proxyEnt, default(WorldTransform));
#else
                AddComponent(proxyEnt, default(Translation));
                AddComponent(proxyEnt, default(Rotation));
#endif
                AddComponent(proxyEnt, default(PhysicsMass));
                AddComponent(proxyEnt, default(PhysicsVelocity));
                AddComponent(proxyEnt, default(PhysicsCollider));
                AddSharedComponent(proxyEnt, new PhysicsWorldIndex((uint)i));
            }
        }
    }
}
