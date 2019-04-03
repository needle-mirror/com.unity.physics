using Unity.Physics.Authoring;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;

namespace Unity.Physics.Editor
{
    [CustomEditor(typeof(CustomFlagNames))]
    [CanEditMultipleObjects]
    class CustomFlagNamesEditor : BaseEditor
    {
        #pragma warning disable 649
        [AutoPopulate(ElementFormatString = "Custom Flag {0}", Resizable = false, Reorderable = false)]
        ReorderableList m_FlagNames;
        #pragma warning restore 649
    }
}
