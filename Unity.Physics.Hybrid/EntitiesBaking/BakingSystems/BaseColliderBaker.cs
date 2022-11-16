using System;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using LegacyCollider = UnityEngine.Collider;
using UnityMesh = UnityEngine.Mesh;

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
        public UnityObjectRef<UnityMesh> Mesh;
        public int MeshID;
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

    public abstract class BaseColliderBaker<T> : BasePhysicsBaker<T> where T : Component
    {
        static List<GameObject> s_Parents = new List<GameObject>();

        protected GameObject FindFirstEnabledAncestor<TU>(GameObject shape, List<TU> buffer) where TU : Component
        {
            // include inactive in case the supplied shape GameObject is a prefab that has not been instantiated
            GetComponentsInParent(buffer);
            GameObject result = null;
            for (int i = 0, count = buffer.Count; i < count; ++i)
            {
                if (
#if LEGACY_PHYSICS
                    (buffer[i] as LegacyCollider)?.enabled ??
#endif
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
#if LEGACY_PHYSICS
                    (buffer[i] as LegacyCollider)?.enabled ??
#endif
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

        protected GameObject GetPrimaryBody(GameObject shape, out bool hasBodyComponent, out bool isStaticBody)
        {
            var pb = FindFirstEnabledAncestor(shape, PhysicsShapeExtensions_NonBursted.s_PhysicsBodiesBuffer);
#if LEGACY_PHYSICS
            var rb = FindFirstEnabledAncestor(shape, PhysicsShapeExtensions_NonBursted.s_RigidbodiesBuffer);
#else
            GameObject rb = null;
#endif
            hasBodyComponent = (pb != null || rb != null);
            isStaticBody = false;

            if (pb != null)
            {
                return rb == null ? pb.gameObject :
                    pb.transform.IsChildOf(rb.transform) ? pb.gameObject : rb.gameObject;
            }

            if (rb != null)
                return rb.gameObject;

            // for implicit static shape, first see if it is part of static optimized hierarchy
            isStaticBody = FindTopmostStaticEnabledAncestor(shape, out var topStatic);
            if (topStatic != null)
                return topStatic;

            // otherwise, find topmost enabled Collider or PhysicsShapeAuthoring
#if LEGACY_PHYSICS
            var topCollider = FindTopmostEnabledAncestor(shape, PhysicsShapeExtensions_NonBursted.s_CollidersBuffer);
#else
            GameObject topCollider = null;
#endif
            var topShape = FindTopmostEnabledAncestor(shape, PhysicsShapeExtensions_NonBursted.s_ShapesBuffer);

            return topCollider == null
                ? topShape == null ? shape.gameObject : topShape
                : topShape == null
                ? topCollider
                : topShape.transform.IsChildOf(topCollider.transform)
                ? topCollider
                : topShape;
        }

        private bool FindTopmostStaticEnabledAncestor(GameObject gameObject, out GameObject topStatic)
        {
            topStatic = null;
            if (gameObject == null)
                return false;

            if (IsStatic(gameObject))
            {
                // get the list of ancestors and set a dependency on the entire ancestor hierarchy
                var parents = s_Parents;
                GetParents(parents);

                for (int i = parents.Count - 1; i >= 0; --i)
                {
                    // find the top most static parent and take a dependency on it and all its non-static ancestors
                    if (IsStatic(parents[i]))
                    {
                        topStatic = parents[i];
                        break;
                    }
                }
            }

            return topStatic != null;
        }
    }
}
