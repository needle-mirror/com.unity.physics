using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    /// <summary>
    /// Component that specifies the local to world matrix and scale of an entity
    /// </summary>
    [TemporaryBakingType]
    public struct PhysicsPostProcessData : IComponentData
    {
        /// <summary> A matrix that transforms a point from local space into world space </summary>
        public float4x4 LocalToWorldMatrix;

        /// <summary> Represents the global scale of a body </summary>
        public float3 LossyScale;
    }

    /// <summary>
    /// An empty component that is used to indicate if the root of a compound collider has been baked
    /// </summary>
    [TemporaryBakingType]
    public struct PhysicsRootBaked : IComponentData {}

    /// <summary>
    /// Component that specifies data relating to compound colliders and blobs
    /// </summary>
    [BakingType]
    public struct PhysicsCompoundData : IComponentData
    {
        /// <summary> A hash associated with a compound collider </summary>
        public Unity.Entities.Hash128 Hash;

        /// <summary> Instance ID of the GameObject associated with the body </summary>
        public int ConvertedBodyInstanceID;

        /// <summary> Indicates if a blob is associated to a collider </summary>
        public bool AssociateBlobToBody;

        /// <summary> A flag to indicate that calculation of a compound blob should be deferred </summary>
        public bool DeferredCompoundBlob;

        /// <summary> A flag to indicate that a compound collider has been calculated </summary>
        public bool RegisterBlob;
    }

    internal abstract class BasePhysicsBaker<T> : Baker<T> where T : Component
    {
        bool NeedsPostProcessTransform(Transform worldTransform, bool gameObjectStatic, BodyMotionType motionType, out PhysicsPostProcessData data)
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
        protected void PostProcessTransform(Transform bodyTransform, BodyMotionType motionType = BodyMotionType.Static)
        {
            if (NeedsPostProcessTransform(bodyTransform, IsStatic(), motionType, out PhysicsPostProcessData data))
            {
                // We need to set manual override so we can add CompositeScale and prevent NonUniformScale from been added
                var entity = GetEntity(TransformUsageFlags.ManualOverride);

                // Need to add all the necessary transform elements
                AddComponent(entity, new LocalToWorld { Value = bodyTransform.localToWorldMatrix });


                if (HasNonIdentityScale(bodyTransform))
                {
                    // Any non-identity scale at authoring time is baked into the physics collision shape/mass data.
                    // In this case, the LocalTransform scale field should be set to 1.0 to avoid double-scaling
                    // within the physics simulation. We bake the scale into the PostTransformMatrix to make sure the object
                    // is rendered correctly.
                    // TODO(DOTS-7098): should potentially add a tag component here to indicate that scale is baked in?
                    var compositeScale = float4x4.Scale(bodyTransform.localScale);
                    AddComponent(entity, new PostTransformMatrix { Value = compositeScale });
                }
                var uniformScale = 1.0f;
                LocalTransform transform = LocalTransform.FromPositionRotationScale(bodyTransform.localPosition,
                    bodyTransform.localRotation, uniformScale);
                AddComponent(entity, transform);

                AddComponent(entity, data);
            }
        }
    }
}
