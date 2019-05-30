using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    [CreateAssetMenu(menuName = "DOTS/Physics/Physics Category Names", fileName = "Physics Category Names")]
    public sealed class PhysicsCategoryNames : ScriptableObject, IFlagNames
    {
        IReadOnlyList<string> IFlagNames.FlagNames => CategoryNames;
        public IReadOnlyList<string> CategoryNames => m_CategoryNames;
        [SerializeField]
        string[] m_CategoryNames = Enumerable.Range(0, 32).Select(i => string.Empty).ToArray();

        void OnValidate()
        {
            if (m_CategoryNames.Length != 32)
            {
                Array.Resize(ref m_CategoryNames, 32);
            }
        }
    }
}