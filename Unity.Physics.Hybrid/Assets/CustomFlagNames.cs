using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Unity.Physics.Authoring
{
    interface IFlagNames
    {
        IReadOnlyList<string> FlagNames { get; }
    }
    
    [CreateAssetMenu(menuName = "DOTS/Physics/Custom Flag Names", fileName = "Custom Flag Names")]
    public class CustomFlagNames : ScriptableObject, IFlagNames
    {
        public IReadOnlyList<string> FlagNames => m_FlagNames;
        [SerializeField]
        string[] m_FlagNames = Enumerable.Range(0, 8).Select(i => string.Empty).ToArray();

        void OnValidate()
        {
            if (m_FlagNames.Length != 8)
            {
                Array.Resize(ref m_FlagNames, 8);
            }
        }
    }
}
