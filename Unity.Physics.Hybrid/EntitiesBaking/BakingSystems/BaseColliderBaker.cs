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
    }

    internal abstract class BaseColliderBaker<T> : BasePhysicsBaker<T> where T : Component
    {
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
