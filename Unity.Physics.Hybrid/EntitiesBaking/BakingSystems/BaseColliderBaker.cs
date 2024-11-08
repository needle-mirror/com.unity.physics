using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [TemporaryBakingType]
    internal struct PhysicsColliderAuthoringData : IComponentData
    {
        public ShapeComputationDataBaking ShapeComputationalData;
        public int BlobIndex;
        public bool RecalculateBlob;
    }

    [TemporaryBakingType]
    internal struct PhysicsMeshAuthoringData : IComponentData
    {
        public bool Convex;
        public UnityObjectRef<UnityEngine.Mesh> Mesh;
        public Bounds MeshBounds;
        public float4x4 BakeFromShape;
        public float4x4 ChildToShape;
        public int MeshArrayIndex;
    }

    [BakingType]
    internal struct PhysicsColliderBakedData : IComponentData
    {
        public Entities.Hash128 Hash;
        public Entity BodyEntity;
        public Entity ChildEntity;
        public RigidTransform BodyFromShape;
        public bool IsLeafEntityBody;
        // Collider reference which ensures that all colliders remain available during incremental baking of
        // compound colliders via the BuildCompoundColliderBakingSystem. They might otherwise be removed from
        // the BlobAssetStore since no reference is made to them anymore at the Entities
        // data level after the compound collider is built.
        // The PhysicsColliderBakedData component must therefore also be a BakingType and not a TemporaryBakingType,
        // ensuring that it is not removed after every baking iteration.
        public BlobAssetReference<Collider> BakedCollider;
    }

    internal abstract class BaseColliderBaker<T> : BasePhysicsBaker<T> where T : Component
    {
        /// <summary>
        /// Gets the collider bake matrix for the given collider authoring component world transform.
        /// </summary>
        /// <param name="localToWorldMatrix">World transform of the collider authoring component.</param>
        /// <param name="bodyLocalToWorldMatrix">World transform of the body the resultant baked collider will be added to.</param>
        /// <param name="bodyUniformScale">Uniform scale which will be assigned to the baked rigid body.</param>
        /// <returns>Bake matrix, applied to the collider geometry during baking.</returns>
        protected Matrix4x4 GetColliderBakeMatrix(Matrix4x4 localToWorldMatrix, Matrix4x4 bodyLocalToWorldMatrix, out float bodyUniformScale)
        {
            bodyUniformScale = 1f;
            var localToWorld = (float4x4)localToWorldMatrix;
            var bodyLocalToWorld = (float4x4)bodyLocalToWorldMatrix;

            // We don't bake pure uniform scales into colliders since edit-time uniform scales
            // are baked into the entity's LocalTransform.Scale property, unless the shape has non-identity scale
            // relative to its contained body. In this case we need to bake all scales into the collider geometry.
            var relativeTransform = math.mul(math.inverse(bodyLocalToWorld), localToWorld);

            var hasNonIdentityScaleRelativeToBody = relativeTransform.HasNonIdentityScale();
            var hasShearRelativeToBody = relativeTransform.HasShear();
            var hasScaleOrShearRelativeToBody = hasNonIdentityScaleRelativeToBody || hasShearRelativeToBody;

            // If the body transform has purely uniform scale, and there is any scale or shear between the body and the shape,
            // then we need to extract the uniform body scale from the shape transform before baking
            // to prevent the shape from being scaled by the body's uniform scale twice. This is because pure top level body uniform scales
            // are not baked into collider geometry but represented by the body entity's LocalTransform.Scale property.
            if (hasScaleOrShearRelativeToBody)
            {
                var bodyHasShear = bodyLocalToWorld.HasShear();
                var bodyHasNonUniformScale = bodyLocalToWorld.HasNonUniformScale();
                if (!bodyHasShear && !bodyHasNonUniformScale)
                {
                    // extract uniform scale of body and remove it from the shape transform
                    var bodyScale = bodyLocalToWorld.DecomposeScale();
                    var bodyScaleInverse = 1 / bodyScale;
                    localToWorld = math.mul(localToWorld, float4x4.Scale(bodyScaleInverse));
                }
            }

            // only bake shear or non-uniform scales into the collider geometry
            if (hasScaleOrShearRelativeToBody || localToWorld.HasShear() || localToWorld.HasNonUniformScale())
            {
                var rigidBodyTransform = Math.DecomposeRigidBodyTransform(localToWorld);
                var bakeMatrix = math.mul(math.inverse(new float4x4(rigidBodyTransform)), localToWorld);
                // make sure we have a valid transformation matrix
                bakeMatrix.c0[3] = 0;
                bakeMatrix.c1[3] = 0;
                bakeMatrix.c2[3] = 0;
                bakeMatrix.c3[3] = 1;
                return bakeMatrix;
            }
            // else:

            {
                var bodyScale = bodyLocalToWorldMatrix.lossyScale;
                bodyUniformScale = bodyScale[0];
            }

            return float4x4.identity;
        }

        protected GameObject FindFirstEnabledAncestor<TU>(GameObject shape, List<TU> buffer) where TU : Component
        {
            // include inactive in case the supplied shape GameObject is a prefab that has not been instantiated
            GetComponentsInParent(buffer);
            GameObject result = null;
            for (int i = 0, count = buffer.Count; i < count; ++i)
            {
                if (
                    (buffer[i] as UnityEngine.Collider)?.enabled ??
                    (buffer[i] as MonoBehaviour)?.enabled ?? true)
                {
                    result = buffer[i].gameObject;
                    break;
                }
            }
            buffer.Clear();
            return result;
        }

        protected GameObject FindTopmostEnabledAncestor<TU>(GameObject shape, List<TU> buffer) where TU : Component
        {
            // include inactive in case the supplied shape GameObject is a prefab that has not been instantiated
            GetComponentsInParent(buffer);
            GameObject result = null;
            for (var i = buffer.Count - 1; i >= 0; --i)
            {
                if (
                    (buffer[i] as UnityEngine.Collider)?.enabled ??
                    (buffer[i] as MonoBehaviour)?.enabled ?? true
                )
                {
                    result = buffer[i].gameObject;
                    break;
                }
            }
            buffer.Clear();
            return result;
        }

        protected bool FindTopmostStaticEnabledAncestor(GameObject gameObject, out GameObject topStatic)
        {
            return ColliderExtensions.FindTopmostStaticEnabledAncestor(gameObject, out topStatic);
        }
    }
}
