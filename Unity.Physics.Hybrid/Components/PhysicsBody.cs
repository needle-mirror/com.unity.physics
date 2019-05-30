using System;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    public enum BodyMotionType
    {
        Dynamic,
        Kinematic,
        Static
    }

    [AddComponentMenu("DOTS/Physics/Physics Body")]
    [DisallowMultipleComponent]
    [RequiresEntityConversion]
    public sealed class PhysicsBody : MonoBehaviour
    {
        public BodyMotionType MotionType { get => m_MotionType; set => m_MotionType = value; }
        [SerializeField]
        [Tooltip("Specifies whether the body should be fully physically simulated, moved directly, or fixed in place.")]
        BodyMotionType m_MotionType;

        const float k_MinimumMass = 0.001f;

        public float Mass
        {
            get => m_MotionType == BodyMotionType.Dynamic ? m_Mass : float.PositiveInfinity;
            set => m_Mass = math.max(k_MinimumMass, value);
        }
        [SerializeField]
        float m_Mass = 1.0f;

        public float LinearDamping { get => m_LinearDamping; set => m_LinearDamping = value; }
        [SerializeField]
        [Tooltip("This is applied to a body's linear velocity reducing it over time.")]
        float m_LinearDamping = 0.01f;

        public float AngularDamping { get => m_AngularDamping; set => m_AngularDamping = value; }
        [SerializeField]
        [Tooltip("This is applied to a body's angular velocity reducing it over time.")]
        float m_AngularDamping = 0.05f;

        public float3 InitialLinearVelocity { get => m_InitialLinearVelocity; set => m_InitialLinearVelocity = value; }
        [SerializeField]
        [Tooltip("The initial linear velocity of the body in world space")]
        float3 m_InitialLinearVelocity = float3.zero;

        public float3 InitialAngularVelocity { get => m_InitialAngularVelocity; set => m_InitialAngularVelocity = value; }
        [SerializeField]
        [Tooltip("This represents the initial rotation speed around each axis in the local motion space of the body i.e. around the center of mass")]
        float3 m_InitialAngularVelocity = float3.zero;

        public float GravityFactor
        {
            get => m_MotionType == BodyMotionType.Dynamic ? m_GravityFactor : 0f;
            set => m_GravityFactor = value;
        }
        [SerializeField]
        [Tooltip("Scales the amount of gravity to apply to this body.")]
        float m_GravityFactor = 1f;

        public bool OverrideDefaultMassDistribution
        {
            get => m_OverrideDefaultMassDistribution;
            set => m_OverrideDefaultMassDistribution = value;
        }
        [SerializeField]
        [Tooltip("Default mass distribution is based on the shapes associated with this body.")]
        public bool m_OverrideDefaultMassDistribution;

        public MassDistribution CustomMassDistribution
        {
            get => new MassDistribution
            {
                Transform = new RigidTransform(m_Orientation, m_CenterOfMass),
                InertiaTensor =
                    m_MotionType == BodyMotionType.Dynamic ? m_InertiaTensor : new float3(float.PositiveInfinity)
            };
            set
            {
                m_CenterOfMass = value.Transform.pos;
                m_Orientation.SetValue(value.Transform.rot);
                m_InertiaTensor = value.InertiaTensor;
                m_OverrideDefaultMassDistribution = true;
            }
        }

        [SerializeField]
        float3 m_CenterOfMass;

        [SerializeField]
        EulerAngles m_Orientation = EulerAngles.Default;

        [SerializeField]
        // Default value to solid unit sphere : https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        float3 m_InertiaTensor = new float3(2f/5f); 

        void OnEnable()
        {
            // included so tick box appears in Editor
        }

        void OnValidate()
        {
            m_Mass = math.max(k_MinimumMass, m_Mass);
            m_LinearDamping = math.max(m_LinearDamping, 0f);
            m_AngularDamping = math.max(m_AngularDamping, 0f);
        }
    }
}
