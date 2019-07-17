using System;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;

namespace Unity.Physics.Authoring
{
    interface IPhysicsMaterialProperties
    {
        bool IsTrigger { get; set; }

        PhysicsMaterialCoefficient Friction { get; set; }

        PhysicsMaterialCoefficient Restitution { get; set; }

        PhysicsCategoryTags BelongsTo { get; set; }

        PhysicsCategoryTags CollidesWith { get; set; }

        bool RaisesCollisionEvents { get; set; }

        // TODO: Enable Mass Factors?
        // TODO: Surface Velocity?
        // TODO: Max Impulse?

        CustomPhysicsMaterialTags CustomTags { get; set; }
    }

    interface IInheritPhysicsMaterialProperties : IPhysicsMaterialProperties
    {
        PhysicsMaterialTemplate Template { get; set; }
        bool OverrideIsTrigger { get; set; }
        bool OverrideFriction { get; set; }
        bool OverrideRestitution { get; set; }
        bool OverrideBelongsTo { get; set; }
        bool OverrideCollidesWith { get; set; }
        bool OverrideRaisesCollisionEvents { get; set; }
        bool OverrideCustomTags { get; set; }
    }

    [Serializable]
    public struct PhysicsMaterialCoefficient
    {
        [SoftRange(0f, 1f, TextFieldMax = float.MaxValue)]
        public float Value;
        public Material.CombinePolicy CombineMode;
    }

    abstract class OverridableValue
    {
        public bool Override { get => m_Override; set => m_Override = value; }
        [SerializeField]
        bool m_Override;
    }

    abstract class OverridableValue<T> : OverridableValue where T : struct
    {
        public T Value
        {
            get => m_Value;
            set
            {
                m_Value = value;
                Override = true;
            }
        }
        [SerializeField]
        T m_Value;

        public void OnValidate() => OnValidate(ref m_Value);
        protected virtual void OnValidate(ref T value) {}
    }

    [Serializable]
    class OverridableBool : OverridableValue<bool> {}

    [Serializable]
    class OverridableMaterialCoefficient : OverridableValue<PhysicsMaterialCoefficient>
    {
        protected override void OnValidate(ref PhysicsMaterialCoefficient value) =>
            value.Value = math.max(0f, value.Value);
    }

    [Serializable]
    class OverridableTags : OverridableValue
    {
        public OverridableTags(int capacity) => m_Value = new bool[capacity];

        public uint Value
        {
            get
            {
                var result = 0;
                for (var i = 0; i < m_Value.Length; ++i)
                    result |= (m_Value[i] ? 1 : 0) << i;
                return unchecked((uint)result);
            }
            set
            {
                for (var i = 0; i < m_Value.Length; ++i)
                    m_Value[i] = (value & (1 << i)) != 0;
                Override = true;
            }
        }

        public bool this[int index]
        {
            get => m_Value[index];
            set
            {
                m_Value[index] = value;
                Override = true;
            }
        }

        [SerializeField]
        bool[] m_Value;

        public void OnValidate(int capacity) => Array.Resize(ref m_Value, capacity);
    }

    [Serializable]
    class PhysicsMaterialProperties : IInheritPhysicsMaterialProperties
    {
        public PhysicsMaterialProperties(bool supportsTemplate) => m_SupportsTemplate = supportsTemplate;

        [SerializeField, HideInInspector]
        bool m_SupportsTemplate;

        public PhysicsMaterialTemplate Template
        {
            get => m_Template;
            set => m_Template = m_SupportsTemplate ? value : null;
        }
        [SerializeField]
        [Tooltip("Assign a template to use its values.")]
        PhysicsMaterialTemplate m_Template;

        static T Get<T>(OverridableValue<T> value, T? templateValue) where T : struct =>
            value.Override || templateValue == null ? value.Value : templateValue.Value;

        public bool OverrideIsTrigger { get => m_IsTrigger.Override; set => m_IsTrigger.Override = value; }
        public bool IsTrigger
        {
            get => Get(m_IsTrigger, m_Template == null ? null : m_Template?.IsTrigger);
            set => m_IsTrigger.Value = value;
        }
        [SerializeField]
        OverridableBool m_IsTrigger = new OverridableBool();

        public bool OverrideFriction { get => m_Friction.Override; set => m_Friction.Override = value; }
        public PhysicsMaterialCoefficient Friction
        {
            get => Get(m_Friction, m_Template == null ? null : m_Template?.Friction);
            set => m_Friction.Value= value;
        }
        [SerializeField]
        OverridableMaterialCoefficient m_Friction = new OverridableMaterialCoefficient
        {
            Value = new PhysicsMaterialCoefficient { Value = 0.5f, CombineMode = Material.CombinePolicy.GeometricMean },
            Override = false
        };

        public bool OverrideRestitution { get => m_Restitution.Override; set => m_Restitution.Override = value; }
        public PhysicsMaterialCoefficient Restitution
        {
            get => Get(m_Restitution, m_Template == null ? null : m_Template?.Restitution);
            set => m_Restitution.Value = value;
        }
        [SerializeField]
        OverridableMaterialCoefficient m_Restitution = new OverridableMaterialCoefficient
        {
            Value = new PhysicsMaterialCoefficient { Value = 0f, CombineMode = Material.CombinePolicy.Maximum },
            Override = false
        };

        static uint Get(OverridableTags tags, uint? templateValue) =>
            tags.Override || templateValue == null ? tags.Value : templateValue.Value;

        public bool OverrideBelongsTo { get => m_BelongsTo.Override; set => m_BelongsTo.Override = value; }
        public PhysicsCategoryTags BelongsTo
        {
            get => new PhysicsCategoryTags { Value = Get(m_BelongsTo, m_Template?.BelongsTo.Value) };
            set => m_BelongsTo.Value = value.Value;
        }
        [SerializeField]
        OverridableTags m_BelongsTo = new OverridableTags(32) { Value = unchecked((uint)~0), Override = false};

        public bool OverrideCollidesWith { get => m_CollidesWith.Override; set => m_CollidesWith.Override = value; }
        public PhysicsCategoryTags CollidesWith
        {
            get => new PhysicsCategoryTags { Value = Get(m_CollidesWith, m_Template?.CollidesWith.Value) };
            set => m_CollidesWith.Value = value.Value;
        }
        [SerializeField]
        OverridableTags m_CollidesWith = new OverridableTags(32) { Value = unchecked((uint)~0), Override = false };

        public bool OverrideRaisesCollisionEvents { get => m_RaisesCollisionEvents.Override; set => m_RaisesCollisionEvents.Override = value; }
        public bool RaisesCollisionEvents
        {
            get => Get(m_RaisesCollisionEvents, m_Template == null ? null : m_Template?.RaisesCollisionEvents);
            set => m_RaisesCollisionEvents.Value = value;
        }
        [SerializeField]
        OverridableBool m_RaisesCollisionEvents = new OverridableBool();

        public bool OverrideCustomTags { get => m_CustomTags.Override; set => m_CustomTags.Override = value; }
        public CustomPhysicsMaterialTags CustomTags
        {
            get => new CustomPhysicsMaterialTags { Value = unchecked((byte)Get(m_CustomTags, m_Template?.CustomTags.Value)) };
            set => m_CustomTags.Value= value.Value;
        }
        [SerializeField]
        [FormerlySerializedAs("m_CustomFlags")]
        OverridableTags m_CustomTags = new OverridableTags(8);

        internal static void OnValidate(ref PhysicsMaterialProperties material, bool supportsTemplate)
        {
            material.m_SupportsTemplate = supportsTemplate;
            if (!supportsTemplate)
            {
                material.m_Template = null;
                material.m_IsTrigger.Override = true;
                material.m_Friction.Override = true;
                material.m_Restitution.Override = true;
                material.m_BelongsTo.Override = true;
                material.m_CollidesWith.Override = true;
                material.m_RaisesCollisionEvents.Override = true;
                material.m_CustomTags.Override = true;
            }
            material.m_Friction.OnValidate();
            material.m_Restitution.OnValidate();
            material.m_BelongsTo.OnValidate(32);
            material.m_CollidesWith.OnValidate(32);
            material.m_CustomTags.OnValidate(8);
        }
    }
}
