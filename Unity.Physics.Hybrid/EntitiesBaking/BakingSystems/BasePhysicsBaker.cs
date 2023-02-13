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
            Transform transformParent = worldTransform.parent;
            bool haveParentEntity    = transformParent != null;
            bool haveBakedTransform  = gameObjectStatic;
            bool hasNonIdentityScale = HasNonIdentityScale(worldTransform);
            bool unparent            = motionType != BodyMotionType.Static || hasNonIdentityScale || !haveParentEntity || haveBakedTransform;

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

        bool HasNonIdentityScale(Transform bodyTransform)
        {
            return math.lengthsq((float3)bodyTransform.lossyScale - new float3(1f)) > 0f;
        }

        /// <summary>
        /// Post processing set up of this entity's transformation.
        /// </summary>
        /// <param name="bodyTransform">Transformation of this entity.</param>
        /// <param name="motionType">Motion type of this entity. Default is BodyMotionType.Static.</param>
        /// <param name="hasPropagateLocalToWorld">Specifies whether this entity already has the PropagateLocalToWorld component. Default is false.</param>
        protected void PostProcessTransform(Transform bodyTransform, BodyMotionType motionType = BodyMotionType.Static, bool hasPropagateLocalToWorld = false)
        {
            if (NeedsPostProcessTransform(bodyTransform, IsStatic(), motionType, out PhysicsPostProcessData data))
            {
                // We need to set manual override so we can add CompositeScale and prevent NonUniformScale from been added
                GetEntity(TransformUsageFlags.ManualOverride);

                // Need to add all the necessary transform elements
                AddComponent(new LocalToWorld { Value = bodyTransform.localToWorldMatrix });

#if !ENABLE_TRANSFORM_V1
                if (HasNonIdentityScale(bodyTransform))
                {
                    // Any non-identity scale at authoring time is baked into the physics collision shape/mass data.
                    // In this case, the LocalTransform scale field should be set to 1.0 to avoid double-scaling
                    // within the physics simulation. We bake the scale into the PostTransformScale to make sure the object
                    // is rendered correctly.
                    // TODO(DOTS-7098): should potentially add a tag component here to indicate that scale is baked in?
                    var compositeScale = float3x3.Scale(bodyTransform.localScale);
                    AddComponent(new PostTransformScale { Value = compositeScale });
                    if (!hasPropagateLocalToWorld)
                    {
                        // Add PropagateLocalToWorld component if not yet present.
                        AddComponent<PropagateLocalToWorld>();
                    }
                }
                var uniformScale = 1.0f;
                LocalTransform transform = LocalTransform.FromPositionRotationScale(bodyTransform.localPosition,
                    bodyTransform.localRotation, uniformScale);
                AddComponent(transform);
                AddComponent((WorldTransform)transform);
#else
                AddComponent(new Translation { Value = bodyTransform.localPosition });
                AddComponent(new Rotation { Value = bodyTransform.localRotation });

                if (HasNonIdentityScale(bodyTransform))
                {
                    AddComponent(new CompositeScale() { Value = float4x4.Scale(bodyTransform.localScale) });
                }
#endif
                AddComponent(data);
            }
        }
    }
}
#endif
