using UnityEngine;

namespace Unity.Physics.Authoring
{
    [CreateAssetMenu(menuName = "DOTS/Physics/Physics Material Template")]
    public class PhysicsMaterialTemplate : ScriptableObject, IPhysicsMaterialProperties
    {
        public bool IsTrigger { get => m_Value.IsTrigger; set => m_Value.IsTrigger = value; }

        public PhysicsMaterialCoefficient Friction { get => m_Value.Friction; set => m_Value.Friction = value; }
        public PhysicsMaterialCoefficient Restitution { get => m_Value.Restitution; set => m_Value.Restitution = value; }

        public int BelongsTo { get => m_Value.BelongsTo; set => m_Value.BelongsTo = value; }
        public bool GetBelongsTo(int categoryIndex) => m_Value.GetBelongsTo(categoryIndex);
        public void SetBelongsTo(int categoryIndex, bool value) => m_Value.SetBelongsTo(categoryIndex, value);

        public int CollidesWith { get => m_Value.CollidesWith; set => m_Value.CollidesWith = value; }
        public bool GetCollidesWith(int categoryIndex) => m_Value.GetCollidesWith(categoryIndex);
        public void SetCollidesWith(int categoryIndex, bool value) => m_Value.SetCollidesWith(categoryIndex, value);

        public bool RaisesCollisionEvents
        {
            get => m_Value.RaisesCollisionEvents;
            set => m_Value.RaisesCollisionEvents = value;
        }

        public byte CustomFlags { get => m_Value.CustomFlags; set => m_Value.CustomFlags = value; }
        public bool GetCustomFlag(int customFlagIndex) => m_Value.GetCustomFlag(customFlagIndex);
        public void SetCustomFlag(int customFlagIndex, bool value) => m_Value.SetCustomFlag(customFlagIndex, value);

        [SerializeField]
        PhysicsMaterialProperties m_Value = new PhysicsMaterialProperties(false);

        void OnValidate()
        {
            PhysicsMaterialProperties.OnValidate(ref m_Value, false);
        }
    }
}
