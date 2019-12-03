using System;
using System.ComponentModel;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;
using LegacyCollider = UnityEngine.Collider;
using UnityMesh = UnityEngine.Mesh;

// all deprecated API points in this assembly should go in this file, if possible

namespace Unity.Physics.Authoring
{
    public abstract partial class BaseShapeConversionSystem<T> where T : UnityEngine.Component
    {
        [Obsolete("ProduceColliderBlob() has been deprecated. BaseShapeConversionSystem is not intended to be sub-classed outside its assembly. (RemovedAfter 2019-12-20)")]
        protected virtual BlobAssetReference<Collider> ProduceColliderBlob(T shape) => default;
    }

    public sealed partial class PhysicsShapeAuthoring
    {
        [FormerlySerializedAs("m_ConvexRadius")]
        [SerializeField, HideInInspector]
        float m_ConvexRadius_Deprecated = -1f;

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("GetMeshProperties() using NativeList<int> has been deprecated. Use the signature taking NativeList<int3> instead. (RemovedAfter 2020-02-04)")]
        public void GetMeshProperties(NativeList<float3> vertices, NativeList<int> triangles)
        {
            var tmp = new NativeList<int3>(triangles.Capacity / 3, Allocator.Temp);
            GetMeshProperties(vertices, tmp);
            triangles.Clear();
            for (var i = 0; i < tmp.Length; ++i)
            {
                triangles.Add(tmp[i][0]);
                triangles.Add(tmp[i][1]);
                triangles.Add(tmp[i][2]);
            }
        }
    }
}
