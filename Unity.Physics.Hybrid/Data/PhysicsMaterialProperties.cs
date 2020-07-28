using System;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    interface IPhysicsMaterialProperties
    {
        CollisionResponsePolicy CollisionResponse { get; set; }

        PhysicsMaterialCoefficient Friction { get; set; }

        PhysicsMaterialCoefficient Restitution { get; set; }

        PhysicsCategoryTags BelongsTo { get; set; }

        PhysicsCategoryTags CollidesWith { get; set; }

        // TODO: Enable Mass Factors?
        // TODO: Surface Velocity?

        CustomPhysicsMaterialTags CustomTags { get; set; }
    }

    interface IInheritPhysicsMaterialProperties : IPhysicsMaterialProperties
    {
        PhysicsMaterialTemplate Template { get; set; }
        bool OverrideCollisionResponse { get; set; }
        bool OverrideFriction { get; set; }
        bool OverrideRestitution { get; set; }
        bool OverrideBelongsTo { get; set; }
        bool OverrideCollidesWith { get; set; }
        bool OverrideCustomTags { get; set; }
    }

    [Serializable]
    public struct PhysicsMaterialCoefficient
    {
        [SoftRange(0f, 1f, TextFieldMax = float.MaxValue)]
        public float Value;
        public Material.CombinePolicy CombineMode;
    }

    abstract class OverridableValue<T> where T : struct
    {
        public bool Override { get => m_Override; set => m_Override = value; }
        [SerializeField]
        bool m_Override;

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
    class OverridableCollisionResponse : OverridableValue<CollisionResponsePolicy> { }

    [Serializable]
    class OverridableMaterialCoefficient : OverridableValue<PhysicsMaterialCoefficient>
    {
        protected override void OnValidate(ref PhysicsMaterialCoefficient value) =>
            value.Value = math.max(0f, value.Value);
    }

    [Serializable]
    class OverridableCategoryTags : OverridableValue<PhysicsCategoryTags> { }

    [Serializable]
    class OverridableCustomMaterialTags : OverridableValue<CustomPhysicsMaterialTags> { }

    [Serializable]
    partial class PhysicsMaterialProperties : IInheritPhysicsMaterialProperties, ISerializationCallbackReceiver
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

        public bool OverrideCollisionResponse { get => m_CollisionResponse.Override; set => m_CollisionResponse.Override = value; }

        public CollisionResponsePolicy CollisionResponse
        {
            get => Get(m_CollisionResponse, m_Template == null ? null : m_Template?.CollisionResponse);
            set => m_CollisionResponse.Value = value;
        }
        [SerializeField]
        OverridableCollisionResponse m_CollisionResponse = new OverridableCollisionResponse
        {
            Value = CollisionResponsePolicy.Collide,
            Override = false
        };

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

        public bool OverrideBelongsTo { get => m_BelongsToCategories.Override; set => m_BelongsToCategories.Override = value; }
        public PhysicsCategoryTags BelongsTo
        {
            get => Get(m_BelongsToCategories, m_Template == null ? null : m_Template?.BelongsTo);
            set => m_BelongsToCategories.Value = value;
        }
        [SerializeField]
        OverridableCategoryTags m_BelongsToCategories =
            new OverridableCategoryTags { Value = PhysicsCategoryTags.Everything, Override = false };

        public bool OverrideCollidesWith { get => m_CollidesWithCategories.Override; set => m_CollidesWithCategories.Override = value; }
        public PhysicsCategoryTags CollidesWith
        {
            get => Get(m_CollidesWithCategories, m_Template == null ? null : m_Template?.CollidesWith);
            set => m_CollidesWithCategories.Value = value;
        }
        [SerializeField]
        OverridableCategoryTags m_CollidesWithCategories =
            new OverridableCategoryTags { Value = PhysicsCategoryTags.Everything, Override = false };

        public bool OverrideCustomTags { get => m_CustomMaterialTags.Override; set => m_CustomMaterialTags.Override = value; }
        public CustomPhysicsMaterialTags CustomTags
        {
            get => Get(m_CustomMaterialTags, m_Template == null ? null : m_Template?.CustomTags);
            set => m_CustomMaterialTags.Value = value;
        }
        [SerializeField]
        OverridableCustomMaterialTags m_CustomMaterialTags =
            new OverridableCustomMaterialTags { Value = default, Override = false };

        internal static void OnValidate(ref PhysicsMaterialProperties material, bool supportsTemplate)
        {
            material.UpgradeVersionIfNecessary();

            material.m_SupportsTemplate = supportsTemplate;
            if (!supportsTemplate)
            {
                material.m_Template = null;
                material.m_CollisionResponse.Override = true;
                material.m_Friction.Override = true;
                material.m_Restitution.Override = true;
            }
            material.m_Friction.OnValidate();
            material.m_Restitution.OnValidate();
        }

        const int k_LatestVersion = 1;

        [SerializeField]
        int m_SerializedVersion = 0;

        void ISerializationCallbackReceiver.OnBeforeSerialize() { }

        void ISerializationCallbackReceiver.OnAfterDeserialize() => UpgradeVersionIfNecessary();

        internal static bool s_SuppressUpgradeWarnings;

#pragma warning disable 618
        void UpgradeVersionIfNecessary()
        {
            if (m_SerializedVersion < k_LatestVersion)
            {
                if (m_SerializedVersion < 1)
                {
                    // keep track of whether anything was actually updated
                    // otherwise new objects created from editor scripts emit warnings by default
                    var upgradedSomething = false;

                    if (m_BelongsTo_Deprecated.HasData)
                    {
                        m_BelongsToCategories.Value = new PhysicsCategoryTags { Value = m_BelongsTo_Deprecated.Value };
                        m_BelongsToCategories.Override = m_BelongsTo_Deprecated.Override;
                        m_BelongsTo_Deprecated = new OverridableTags_Deprecated(0);
                        upgradedSomething = true;
                    }

                    if (m_CollidesWith_Deprecated.HasData)
                    {
                        m_CollidesWithCategories.Value = new PhysicsCategoryTags { Value = m_CollidesWith_Deprecated.Value };
                        m_CollidesWithCategories.Override = m_CollidesWith_Deprecated.Override;
                        m_CollidesWith_Deprecated = new OverridableTags_Deprecated(0);
                        upgradedSomething = true;
                    }

                    if (m_CustomTags_Deprecated.HasData)
                    {
                        m_CustomMaterialTags.Value = new CustomPhysicsMaterialTags { Value = (byte)m_CustomTags_Deprecated.Value };
                        m_CustomMaterialTags.Override = m_CustomTags_Deprecated.Override;
                        m_CustomTags_Deprecated = new OverridableTags_Deprecated(0);
                        upgradedSomething = true;
                    }

                    if (m_IsTrigger_Deprecated.Value)
                    {
                        CollisionResponse = CollisionResponsePolicy.RaiseTriggerEvents;
                        OverrideCollisionResponse = m_IsTrigger_Deprecated.Override;
                        upgradedSomething = true;
                    }
                    else if (m_SupportsTemplate && m_IsTrigger_Deprecated.Override)
                    {
                        CollisionResponse = CollisionResponsePolicy.Collide;
                        OverrideCollisionResponse = m_IsTrigger_Deprecated.Override;
                        upgradedSomething = true;
                    }
                    else if (m_RaisesCollisionEvents_Deprecated.Value)
                    {
                        CollisionResponse = CollisionResponsePolicy.CollideRaiseCollisionEvents;
                        OverrideCollisionResponse = m_RaisesCollisionEvents_Deprecated.Override;
                        upgradedSomething = true;
                    }
                    else if (m_SupportsTemplate && m_RaisesCollisionEvents_Deprecated.Override)
                    {
                        CollisionResponse = CollisionResponsePolicy.Collide;
                        OverrideCollisionResponse = m_RaisesCollisionEvents_Deprecated.Override;
                        upgradedSomething = true;
                    }

                    m_SerializedVersion = 1;

                    if (upgradedSomething && !s_SuppressUpgradeWarnings)
                    {
                        Debug.LogWarning(
                            "Physics material properties were implicitly upgraded on an object. " +
                            "To ensure proper upgrading when moving to Unity Physics 0.4.0, it is recommended you use the tool in the main menu at Window -> DOTS -> Physics -> Upgrade Data."
                        );
                    }
                }
            }

            // Prevent user from setting values for deprecated fields
            m_BelongsTo_Deprecated.Value = 0;
            m_BelongsTo_Deprecated.Override = false;
            m_CollidesWith_Deprecated.Value = 0;
            m_CollidesWith_Deprecated.Override = false;
            m_CustomTags_Deprecated.Value = 0;
            m_CustomTags_Deprecated.Override = false;
            m_IsTrigger_Deprecated.Value = false;
            m_IsTrigger_Deprecated.Override = false;
            m_RaisesCollisionEvents_Deprecated.Value = false;
            m_RaisesCollisionEvents_Deprecated.Override = false;
        }
#pragma warning restore 618
    }
}
