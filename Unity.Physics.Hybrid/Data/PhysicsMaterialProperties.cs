using System;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    interface IPhysicsMaterialProperties
    {
        bool IsTrigger { get; set; }

        PhysicsMaterialCoefficient Friction { get; set; }

        PhysicsMaterialCoefficient Restitution { get; set; }

        int BelongsTo { get; set; }
        bool GetBelongsTo(int categoryIndex);
        void SetBelongsTo(int categoryIndex, bool value);

        int CollidesWith { get; set; }
        bool GetCollidesWith(int categoryIndex);
        void SetCollidesWith(int categoryIndex, bool value);

        bool RaisesCollisionEvents { get; set; }

        // TODO: Enable Mass Factors?
        // TODO: Surface Velocity?
        // TODO: Max Impulse?

        byte CustomFlags { get; set; }
        bool GetCustomFlag(int customFlagIndex);
        void SetCustomFlag(int customFlagIndex, bool value);
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
        bool OverrideCustomFlags { get; set; }
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
    class OverridableFlags : OverridableValue
    {
        public OverridableFlags(int capacity) => m_Value = new bool[capacity];

        public int Value
        {
            get
            {
                var result = 0;
                for (var i = 0; i < m_Value.Length; ++i)
                    result |= (m_Value[i] ? 1 : 0) << i;
                return result;
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

        static int Get(OverridableFlags flags, int? templateValue) =>
            flags.Override || templateValue == null ? flags.Value : templateValue.Value;

        public bool OverrideBelongsTo { get => m_BelongsTo.Override; set => m_BelongsTo.Override = value; }
        public int BelongsTo { get => Get(m_BelongsTo, m_Template?.BelongsTo); set => m_BelongsTo.Value = value; }
        public bool GetBelongsTo(int categoryIndex) => m_BelongsTo.Override || m_Template == null
            ? m_BelongsTo[categoryIndex]
            : m_Template.GetBelongsTo(categoryIndex);
        public void SetBelongsTo(int categoryIndex, bool value) => m_BelongsTo[categoryIndex] = value;
        [SerializeField]
        OverridableFlags m_BelongsTo = new OverridableFlags(32) { Value = ~0, Override = false};

        public bool OverrideCollidesWith { get => m_CollidesWith.Override; set => m_CollidesWith.Override = value; }
        public int CollidesWith
        {
            get => Get(m_CollidesWith, m_Template?.CollidesWith);
            set => m_CollidesWith.Value = value;
        }
        public bool GetCollidesWith(int categoryIndex) => m_CollidesWith.Override || m_Template == null
            ? m_CollidesWith[categoryIndex]
            : m_Template.GetCollidesWith(categoryIndex);
        public void SetCollidesWith(int categoryIndex, bool value) => m_CollidesWith[categoryIndex] = value;
        [SerializeField]
        OverridableFlags m_CollidesWith = new OverridableFlags(32) { Value = ~0, Override = false };

        public bool OverrideRaisesCollisionEvents { get => m_RaisesCollisionEvents.Override; set => m_RaisesCollisionEvents.Override = value; }
        public bool RaisesCollisionEvents
        {
            get => Get(m_RaisesCollisionEvents, m_Template == null ? null : m_Template?.RaisesCollisionEvents);
            set => m_RaisesCollisionEvents.Value = value;
        }
        [SerializeField]
        OverridableBool m_RaisesCollisionEvents = new OverridableBool();

        public bool OverrideCustomFlags { get => m_CustomFlags.Override; set => m_CustomFlags.Override = value; }
        public byte CustomFlags
        {
            get => (byte)Get(m_CustomFlags, m_Template?.CustomFlags);
            set => m_CustomFlags.Value = value;
        }
        public bool GetCustomFlag(int customFlagIndex) => m_CustomFlags.Override || m_Template == null
            ? m_CustomFlags[customFlagIndex]
            : m_Template.GetCustomFlag(customFlagIndex);
        public void SetCustomFlag(int customFlagIndex, bool value) => m_CustomFlags[customFlagIndex] = value;
        [SerializeField]
        OverridableFlags m_CustomFlags = new OverridableFlags(8);

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
                material.m_CustomFlags.Override = true;
            }
            material.m_Friction.OnValidate();
            material.m_Restitution.OnValidate();
            material.m_BelongsTo.OnValidate(32);
            material.m_CollidesWith.OnValidate(32);
            material.m_CustomFlags.OnValidate(8);
        }
    }
}
