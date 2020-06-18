using System;
using UnityEngine;
using UnityEngine.Serialization;
using UnityMesh = UnityEngine.Mesh;

// all deprecated API points in this assembly should go in this file, if possible

namespace Unity.Physics.Authoring
{
    partial class PhysicsMaterialProperties
    {
        [Obsolete("No new call sites should be added for this type. It should be removed sometime before 1.0.0. (RemovedAfter 2020-09-12)")]
        [Serializable]
        class OverridableBool_Deprecated : OverridableValue<bool> { }

        [Obsolete("No new call sites should be added for this type. It should be removed sometime before 1.0.0. (RemovedAfter 2020-09-12)")]
        [Serializable]
        internal class OverridableTags_Deprecated
        {
            public OverridableTags_Deprecated(int capacity) => m_Value = new bool[capacity];

            public bool Override { get => m_Override; set => m_Override = value; }
            [SerializeField]
            bool m_Override;

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

            [SerializeField]
            bool[] m_Value;

            public bool HasData => m_Value?.Length > 0;
        }

#pragma warning disable 618
        [Obsolete("No new call sites should be added for this field. It should be removed sometime before 1.0.0. (RemovedAfter 2020-09-12)")]
        [SerializeField, HideInInspector]
        [FormerlySerializedAs("m_BelongsTo")]
        OverridableTags_Deprecated m_BelongsTo_Deprecated = new OverridableTags_Deprecated(0);

        [Obsolete("No new call sites should be added for this field. It should be removed sometime before 1.0.0. (RemovedAfter 2020-09-12)")]
        [SerializeField, HideInInspector]
        [FormerlySerializedAs("m_CollidesWith")]
        OverridableTags_Deprecated m_CollidesWith_Deprecated = new OverridableTags_Deprecated(0);

        [Obsolete("No new call sites should be added for this field. It should be removed sometime before 1.0.0. (RemovedAfter 2020-09-12)")]
        [SerializeField, HideInInspector]
        [FormerlySerializedAs("m_CustomFlags")]
        [FormerlySerializedAs("m_CustomTags")]
        OverridableTags_Deprecated m_CustomTags_Deprecated = new OverridableTags_Deprecated(0);

        [Obsolete("No new call sites should be added for this field. It should be removed sometime before 1.0.0. (RemovedAfter 2020-09-12)")]
        [FormerlySerializedAs("m_IsTrigger")]
        [SerializeField, HideInInspector]
        OverridableBool_Deprecated m_IsTrigger_Deprecated = new OverridableBool_Deprecated();

        [Obsolete("No new call sites should be added for this field. It should be removed sometime before 1.0.0. (RemovedAfter 2020-09-12)")]
        [FormerlySerializedAs("m_RaisesCollisionEvents")]
        [SerializeField, HideInInspector]
        OverridableBool_Deprecated m_RaisesCollisionEvents_Deprecated = new OverridableBool_Deprecated();
#pragma warning restore 618
    }

    public sealed partial class PhysicsShapeAuthoring
    {
        [Obsolete("No new call sites should be added for this field. It should be removed sometime before 1.0.0. (RemovedAfter 2020-09-12)")]
        [SerializeField, HideInInspector]
        [FormerlySerializedAs("m_ConvexRadius")]
        float m_ConvexRadius_Deprecated = -1f;

        [Obsolete("OverrideIsTrigger has been deprecated. Use OverrideCollisionResponse instead. (RemovedAfter 2020-08-20)", true)]
        public bool OverrideIsTrigger { get; set; }

        [Obsolete("IsTrigger has been deprecated. Use CollisionResponse instead. (RemovedAfter 2020-08-20)")]
        public bool IsTrigger
        {
            get => CollisionResponse == CollisionResponsePolicy.RaiseTriggerEvents;
            set => CollisionResponse = value ? CollisionResponsePolicy.RaiseTriggerEvents : CollisionResponsePolicy.Collide;
        }

        [Obsolete("OverrideRaisesCollisionEvents has been deprecated. Use OverrideCollisionResponse instead. (RemovedAfter 2020-08-20)", true)]
        public bool OverrideRaisesCollisionEvents { get; set; }

        [Obsolete("RaisesCollisionEvents has been deprecated. Use CollisionResponse instead. (RemovedAfter 2020-08-20)")]
        public bool RaisesCollisionEvents
        {
            get => CollisionResponse == CollisionResponsePolicy.CollideRaiseCollisionEvents;
            set => CollisionResponse = value ? CollisionResponsePolicy.CollideRaiseCollisionEvents : CollisionResponsePolicy.Collide;
        }
    }

    public sealed partial class PhysicsMaterialTemplate
    {
        [Obsolete("IsTrigger has been deprecated. Use CollisionResponse instead. (RemovedAfter 2020-08-20)")]
        public bool IsTrigger
        {
            get => CollisionResponse == CollisionResponsePolicy.RaiseTriggerEvents;
            set => CollisionResponse = value ? CollisionResponsePolicy.RaiseTriggerEvents : CollisionResponsePolicy.Collide;
        }

        [Obsolete("RaisesCollisionEvents has been deprecated. Use CollisionResponse instead. (RemovedAfter 2020-08-20)")]
        public bool RaisesCollisionEvents
        {
            get => CollisionResponse == CollisionResponsePolicy.CollideRaiseCollisionEvents;
            set => CollisionResponse = value ? CollisionResponsePolicy.CollideRaiseCollisionEvents : CollisionResponsePolicy.Collide;
        }
    }
}
