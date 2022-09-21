#if LEGACY_PHYSICS
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.GraphicsIntegration;
using Unity.Transforms;
using UnityEngine;
using LegacyRigidBody = UnityEngine.Rigidbody;
using LegacyCollider = UnityEngine.Collider;
using LegacyBox = UnityEngine.BoxCollider;
using LegacyCapsule = UnityEngine.CapsuleCollider;
using LegacyMesh = UnityEngine.MeshCollider;
using LegacySphere = UnityEngine.SphereCollider;

namespace Unity.Physics.Authoring
{
    [TemporaryBakingType]
    public struct PhysicsPostProcessData : IComponentData
    {
        public float4x4 LocalToWorldMatrix;
        public float3 LossyScale;
    }

    [TemporaryBakingType]
    public struct PhysicsRootBaked : IComponentData {}

    [BakingType]
    public struct PhysicsCompoundData : IComponentData
    {
        public Unity.Entities.Hash128 Hash;
        public int ConvertedBodyInstanceID;
        public bool AssociateBlobToBody;
        public bool DeferredCompoundBlob;
        public bool RegisterBlob;
    }

    public abstract class BasePhysicsBaker<T> : Baker<T> where T : Component
    {
        protected bool NeedsPostProcessTransform(Transform worldTransform, bool gameObjectStatic, BodyMotionType motionType, out PhysicsPostProcessData data)
        {
            var transformParent         = worldTransform.parent;
            var haveParentEntity    = transformParent != null;
            var haveBakedTransform  = gameObjectStatic;
            var unparent            = motionType != BodyMotionType.Static || !haveParentEntity || haveBakedTransform;

            data = default;
            if (unparent)
            {
                data = new PhysicsPostProcessData()
                {
                    LocalToWorldMatrix = worldTransform.localToWorldMatrix,
                    LossyScale = worldTransform.lossyScale
                };
            }
            return unparent;
        }

        protected void PostProcessTransform(Transform bodyTransform, BodyMotionType motionType)
        {
            if (NeedsPostProcessTransform(bodyTransform, IsStatic(), motionType, out PhysicsPostProcessData data))
            {
                // We need to set manual override so we can add CompositeScale and prevent NonUniformScale from been added
                GetEntity(TransformUsageFlags.ManualOverride);

                // Need to add all the necessary transform elements
                AddComponent(new LocalToWorld { Value = bodyTransform.localToWorldMatrix });

                AddComponent(new Translation { Value = bodyTransform.localPosition });
                AddComponent(new Rotation { Value = bodyTransform.localRotation });

                if (math.lengthsq((float3)bodyTransform.lossyScale - new float3(1f)) > 0f)
                {
                    AddComponent(new CompositeScale() { Value = float4x4.Scale(bodyTransform.localScale) });
                }

                AddComponent(data);
            }
        }
    }
}
#endif
