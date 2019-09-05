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

    [Obsolete("CustomFlagNames has been deprecated. Use CustomPhysicsMaterialTagNames instead. (RemovedAfter 2019-09-06) (UnityUpgradable) -> CustomPhysicsMaterialTagNames", true)]
    public class CustomFlagNames : ScriptableObject
    {
        public virtual IReadOnlyList<string> FlagNames => throw new NotImplementedException();
    }

    public sealed partial class CustomPhysicsMaterialTagNames
    {
        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("FlagNames has been deprecated. Use TagNames instead. (RemovedAfter 2019-09-06) (UnityUpgradable) -> TagNames", true)]
        public IReadOnlyList<string> FlagNames => throw new NotImplementedException();
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
        [Obsolete("GetBelongsTo() has been deprecated. Use BelongsTo instead. (RemovedAfter 2019-09-06)")]
        public bool GetBelongsTo(int categoryIndex) => m_Material.BelongsTo[categoryIndex];

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("SetBelongsTo() has been deprecated. Use BelongsTo instead. (RemovedAfter 2019-09-06)")]
        public void SetBelongsTo(int categoryIndex, bool value)
        {
            var tags = m_Material.BelongsTo;
            tags[categoryIndex] = value;
            m_Material.BelongsTo = tags;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("GetCollidesWith() has been deprecated. Use CollidesWith instead. (RemovedAfter 2019-09-06)")]
        public bool GetCollidesWith(int categoryIndex) => m_Material.CollidesWith[categoryIndex];

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("SetCollidesWith() has been deprecated. Use CollidesWith instead. (RemovedAfter 2019-09-06)")]
        public void SetCollidesWith(int categoryIndex, bool value)
        {
            var tags = m_Material.CollidesWith;
            tags[categoryIndex] = value;
            m_Material.CollidesWith = tags;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("OverrideCustomFlags has been deprecated. Use OverrideCustomTags instead. (RemovedAfter 2019-09-06) (UnityUpgradable) -> OverrideCustomTags")]
        public bool OverrideCustomFlags { get => OverrideCustomTags; set => OverrideCustomTags = value; }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("CustomFlags has been deprecated. Use CustomTags instead. (RemovedAfter 2019-09-06)")]
        public byte CustomFlags { get => CustomTags.Value; set => CustomTags = new CustomPhysicsMaterialTags { Value = value }; }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("GetCustomFlag() has been deprecated. Use GetCustomTag() instead. (RemovedAfter 2019-09-06)")]
        public bool GetCustomFlag(int customFlagIndex) => m_Material.CustomTags[customFlagIndex];

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("SetCustomFlag() has been deprecated. Use SetCustomTag() instead. (RemovedAfter 2019-09-06)")]
        public void SetCustomFlag(int customFlagIndex, bool value)
        {
            var tags = m_Material.CustomTags;
            tags[customFlagIndex] = value;
            m_Material.CustomTags = tags;
        }

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
            GetBoxProperties(out center, out size, out var euler, out _);
            orientation = euler;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature passing BoxGeometry instead. (RemovedAfter 2019-11-22)")]
        public void SetBox(float3 center, float3 size, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            SetBox(center, size, euler);
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature returning CapsuleGeometry instead. (RemovedAfter 2019-11-22")]
        public void GetCapsuleProperties(
            out float3 center, out float height, out float radius, out quaternion orientation
        )
        {
            GetCapsuleProperties(out center, out height, out radius, out EulerAngles euler);
            orientation = euler;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature passing CapsuleGeometryAuthoring instead. (RemovedAfter 2019-11-22)")]
        public void SetCapsule(float3 center, float height, float radius, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            SetCapsule(center, height, radius, euler);
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature returning CylinderGeometry instead. (RemovedAfter 2019-11-22")]
        public void GetCylinderProperties(
            out float3 center, out float height, out float radius, out quaternion orientation
        )
        {
            GetCylinderProperties(out center, out height, out radius, out var euler, out _, out _);
            orientation = euler;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature passing CylinderGeometry instead. (RemovedAfter 2019-11-22)")]
        public void SetCylinder(float3 center, float height, float radius, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            SetCylinder(center, height, radius, euler);
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature returning SphereGeometry instead. (RemovedAfter 2019-11-22")]
        public void GetSphereProperties(out float3 center, out float radius, out quaternion orientation)
        {
            GetSphereProperties(out center, out radius, out EulerAngles euler);
            orientation = euler;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("This signature has been deprecated. Please use the signature passing SphereGeometry instead. (RemovedAfter 2019-11-22)")]
        public void SetSphere(float3 center, float radius, quaternion orientation)
        {
            var euler = m_PrimitiveOrientation;
            euler.SetValue(orientation);
            SetSphere(center, radius, euler);
        }
    }

    public sealed partial class PhysicsMaterialTemplate
    {
        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("GetBelongsTo() has been deprecated. Use BelongsTo instead. (RemovedAfter 2019-09-06)")]
        public bool GetBelongsTo(int categoryIndex) => m_Value.BelongsTo[categoryIndex];

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("SetBelongsTo() has been deprecated. Use BelongsTo instead. (RemovedAfter 2019-09-06)")]
        public void SetBelongsTo(int categoryIndex, bool value)
        {
            var tags = m_Value.BelongsTo;
            tags[categoryIndex] = value;
            m_Value.BelongsTo = tags;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("GetCollidesWith() has been deprecated. Use CollidesWith instead. (RemovedAfter 2019-09-06)")]
        public bool GetCollidesWith(int categoryIndex) => m_Value.CollidesWith[categoryIndex];

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("SetCollidesWith() has been deprecated. Use CollidesWith instead. (RemovedAfter 2019-09-06)")]
        public void SetCollidesWith(int categoryIndex, bool value)
        {
            var tags = m_Value.CollidesWith;
            tags[categoryIndex] = value;
            m_Value.CollidesWith = tags;
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("CustomFlags has been deprecated. Use CustomTags instead. (RemovedAfter 2019-09-06)")]
        public byte CustomFlags { get => CustomTags.Value; set => CustomTags = new CustomPhysicsMaterialTags { Value = value }; }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("GetCustomFlag() has been deprecated. Use CustomTags instead. (RemovedAfter 2019-09-06)")]
        public bool GetCustomFlag(int customFlagIndex) => CustomTags[customFlagIndex];

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("SetCustomFlag() has been deprecated. Use CustomTags instead. (RemovedAfter 2019-09-06)")]
        public void SetCustomFlag(int customFlagIndex, bool value)
        {
            var tags = CustomTags;
            tags[customFlagIndex] = value;
            CustomTags = tags;
        }
    }
}