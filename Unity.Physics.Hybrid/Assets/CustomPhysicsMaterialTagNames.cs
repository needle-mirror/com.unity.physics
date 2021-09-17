using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Serialization;

namespace Unity.Physics.Authoring
{
    interface ITagNames
    {
        IReadOnlyList<string> TagNames { get; }
    }

    [CreateAssetMenu(menuName = "DOTS/Physics/Custom Physics Material Tag Names", fileName = "Custom Material Tag Names")]
    [HelpURL(HelpURLs.CustomPhysicsMaterialTagNames)]
    public sealed partial class CustomPhysicsMaterialTagNames : ScriptableObject, ITagNames
    {
        CustomPhysicsMaterialTagNames() {}

        public IReadOnlyList<string> TagNames => m_TagNames;
        [SerializeField]
        [FormerlySerializedAs("m_FlagNames")]
        string[] m_TagNames = Enumerable.Range(0, 8).Select(i => string.Empty).ToArray();

        void OnValidate()
        {
            if (m_TagNames.Length != 8)
                Array.Resize(ref m_TagNames, 8);
        }
    }
}
