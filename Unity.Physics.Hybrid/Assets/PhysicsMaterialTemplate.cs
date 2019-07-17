using System;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [CreateAssetMenu(menuName = "DOTS/Physics/Physics Material Template")]
    public sealed partial class PhysicsMaterialTemplate : ScriptableObject, IPhysicsMaterialProperties
    {
        PhysicsMaterialTemplate() { }

        public bool IsTrigger { get => m_Value.IsTrigger; set => m_Value.IsTrigger = value; }

        public PhysicsMaterialCoefficient Friction { get => m_Value.Friction; set => m_Value.Friction = value; }
        public PhysicsMaterialCoefficient Restitution { get => m_Value.Restitution; set => m_Value.Restitution = value; }

        public PhysicsCategoryTags BelongsTo { get => m_Value.BelongsTo; set => m_Value.BelongsTo = value; }

        public PhysicsCategoryTags CollidesWith { get => m_Value.CollidesWith; set => m_Value.CollidesWith = value; }

        public bool RaisesCollisionEvents
        {
            get => m_Value.RaisesCollisionEvents;
            set => m_Value.RaisesCollisionEvents = value;
        }

        public CustomPhysicsMaterialTags CustomTags { get => m_Value.CustomTags; set => m_Value.CustomTags = value; }

        [SerializeField]
        PhysicsMaterialProperties m_Value = new PhysicsMaterialProperties(false);

        void OnValidate() => PhysicsMaterialProperties.OnValidate(ref m_Value, false);
    }
}
