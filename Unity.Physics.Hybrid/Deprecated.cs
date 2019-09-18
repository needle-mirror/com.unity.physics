using System;
using System.Collections.Generic;
using System.ComponentModel;
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
        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("GetCustomFlags() has been deprecated. Set up Material.CustomTags inside of ProduceColliderBlob(). (RemovedAfter 2019-10-15)", true)]
        protected virtual byte GetCustomFlags(T shape) => throw new NotImplementedException();
    }

    [Obsolete("PhysicsBody has been renamed PhysicsBodyAuthoring. (RemovedAfter 2019-11-27) (UnityUpgradable) -> PhysicsBodyAuthoring", true)]
    public sealed class PhysicsBody : MonoBehaviour { }

    [Obsolete("PhysicsShape has been renamed PhysicsShapeAuthoring. (RemovedAfter 2019-11-27) (UnityUpgradable) -> PhysicsShapeAuthoring", true)]
    public sealed class PhysicsShape : MonoBehaviour { }

    [Obsolete("PhysicsStep has been renamed PhysicsStepAuthoring. (RemovedAfter 2019-11-27) (UnityUpgradable) -> PhysicsStepAuthoring", true)]
    public sealed class PhysicsStep : MonoBehaviour { }

    [Obsolete("PhysicsDebugDisplay has been renamed PhysicsDebugDisplayAuthoring. (RemovedAfter 2019-11-27) (UnityUpgradable) -> PhysicsDebugDisplayAuthoring", true)]
    public sealed class PhysicsDebugDisplay : MonoBehaviour { }

    public sealed partial class PhysicsShapeAuthoring
    {
        [FormerlySerializedAs("m_ConvexRadius")]
        [SerializeField, HideInInspector]
        float m_ConvexRadius_Deprecated = -1f;

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use a signature passing ConvexHullGenerationParameters. (RemovedAfter 2019-11-15)")]
        public void SetConvexHull(UnityMesh convexHull = null) =>
            SetConvexHull(ConvexHullGenerationParameters.Default, convexHull);

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("GetMesh() has been deprecated. Please use GetMeshProperties() instead. (RemovedAfter 2019-11-20)")]
        public UnityMesh GetMesh()
        {
            if (m_CustomMesh != null)
                return m_CustomMesh;
            var meshFilter = gameObject.GetComponent<MeshFilter>();
            return meshFilter == null ? null : meshFilter.sharedMesh;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("ConvexRadius has been deprecated. Please use BevelRadius instead. (RemovedAfter 2019-11-15) (UnityUpgradable) -> BevelRadius", true)]
        public float ConvexRadius { get; set; }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature returning BoxGeometry instead. (RemovedAfter 2019-11-22")]
        public void GetBoxProperties(out float3 center, out float3 size, out quaternion orientation)
        {
            var box = GetBoxProperties();
            center = box.Center;
            size = box.Size;
            orientation = box.Orientation;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature passing BoxGeometry instead. (RemovedAfter 2019-11-22)")]
        public void SetBox(float3 center, float3 size, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            var box = new BoxGeometry
            {
                Center = center,
                Size = size,
                Orientation = orientation,
                BevelRadius = BevelRadius
            };
            SetBox(box, euler);
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature returning CapsuleGeometry instead. (RemovedAfter 2019-11-22")]
        public void GetCapsuleProperties(
            out float3 center, out float height, out float radius, out quaternion orientation
        )
        {
            var capsule = GetCapsuleProperties(out orientation);
            center = capsule.GetCenter();
            height = capsule.GetHeight();
            radius = capsule.Radius;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature passing CapsuleGeometryAuthoring instead. (RemovedAfter 2019-11-22)")]
        public void SetCapsule(float3 center, float height, float radius, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            var capsule = new CapsuleGeometry
            {
                Vertex0 = new float3(0f, 0f, -0.5f * height),
                Vertex1 = new float3(0f, 0f, 0.5f * height),
                Radius = radius
            };
            SetCapsule(capsule, euler);
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature returning CylinderGeometry instead. (RemovedAfter 2019-11-22")]
        public void GetCylinderProperties(
            out float3 center, out float height, out float radius, out quaternion orientation
        )
        {
            var cylinder = GetCylinderProperties();
            center = cylinder.Center;
            height = cylinder.Height;
            radius = cylinder.Radius;
            orientation = cylinder.Orientation;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature passing CylinderGeometry instead. (RemovedAfter 2019-11-22)")]
        public void SetCylinder(float3 center, float height, float radius, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            var cylinder = new CylinderGeometry
            {
                Center = center,
                Height = height,
                Radius = radius,
                Orientation = orientation,
                BevelRadius = BevelRadius,
                SideCount = 20
            };
            SetCylinder(cylinder, euler);
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature returning SphereGeometry instead. (RemovedAfter 2019-11-22")]
        public void GetSphereProperties(out float3 center, out float radius, out quaternion orientation)
        {
            var sphere = GetSphereProperties(out orientation);
            center = sphere.Center;
            radius = sphere.Radius;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature passing SphereGeometry instead. (RemovedAfter 2019-11-22)")]
        public void SetSphere(float3 center, float radius, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            SetSphere(new SphereGeometry { Center = center, Radius = radius }, euler);
        }
    }
}