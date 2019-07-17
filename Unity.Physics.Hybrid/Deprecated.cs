using System;
using System.Collections.Generic;
using System.ComponentModel;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using LegacyCollider = UnityEngine.Collider;

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

    [DisableAutoCreation]
    [EditorBrowsable(EditorBrowsableState.Never)]
    [Obsolete("FirstPassLegacyRigidbodyConversionSystem is no longer used. (RemovedAfter 2019-08-24)", true)]
    public class FirstPassLegacyRigidbodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate() => throw new NotImplementedException();
    }

    [DisableAutoCreation]
    [EditorBrowsable(EditorBrowsableState.Never)]
    [Obsolete("FirstPassPhysicsBodyConversionSystem is no longer used. (RemovedAfter 2019-08-24)", true)]
    public class FirstPassPhysicsBodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate() => throw new NotImplementedException();
    }

    public sealed partial class PhysicsShape
    {
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
        [Obsolete("RemovedAfter 2019-08-10", true)]
        public void GetCapsuleProperties(out float3 vertex0, out float3 vertex1, out float radius) =>
            throw new NotImplementedException();

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("RemovedAfter 2019-08-10", true)]
        public void GetPlaneProperties(out float3 vertex0, out float3 vertex1, out float3 vertex2, out float3 vertex3) =>
            throw new NotImplementedException();

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("FitToGeometry() has been deprecated. Use FitToEnabledRenderMeshes() instead. (RemovedAfter 2019-08-27) (UnityUpgradable) -> FitToEnabledRenderMeshes(*)", true)]
        public void FitToGeometry() => throw new NotImplementedException();
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

    [DisableAutoCreation]
    [EditorBrowsable(EditorBrowsableState.Never)]
    [Obsolete("SecondPassLegacyRigidbodyConversionSystem has been deprecated. Use LegacyRigidbodyConversionSystem instead. (RemovedAfter 2019-08-28) (UnityUpgradable) -> LegacyRigidbodyConversionSystem", true)]
    public class SecondPassLegacyRigidbodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate() => throw new NotImplementedException();
    }

    [DisableAutoCreation]
    [EditorBrowsable(EditorBrowsableState.Never)]
    [Obsolete("SecondPassPhysicsBodyConversionSystem has been deprecated. Use PhysicsBodyConversionSystem instead. (RemovedAfter 2019-08-28) (UnityUpgradable) -> PhysicsBodyConversionSystem", true)]
    public class SecondPassPhysicsBodyConversionSystem : GameObjectConversionSystem
    {
        protected override void OnUpdate() => throw new NotImplementedException();
    }
}