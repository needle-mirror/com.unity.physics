using System.Linq;
using Unity.Physics.Authoring;
using UnityEditor;
using UnityEngine;

namespace Unity.Physics.Editor
{
    abstract class TagsDrawer<T> : PropertyDrawer where T : ScriptableObject, ITagNames
    {
        static class Styles
        {
            public const float CreateAssetButtonWidth = 24;

            public static readonly string CreateAssetButtonTooltip =
                L10n.Tr("Click to create a {0} asset and define category names.");
            public static readonly string MultipleAssetsTooltip =
                L10n.Tr("Multiple {0} assets found. UI will display labels defined in {1}.");

            public static readonly GUIContent CreateAssetLabel = new GUIContent { text = "+" };
            public static readonly GUIContent MultipleAssetsWarning =
                new GUIContent { image = EditorGUIUtility.Load("console.warnicon") as Texture };
        }

        protected abstract int MaxNumCategories { get; }
        protected abstract string DefaultCategoryName { get; }
        internal string FirstChildPropertyPath { get; set; } // TODO: remove when all usages of bool[] are migrated

        string DefaultFormatString => L10n.Tr($"(Undefined {DefaultCategoryName} {{0}})");

        string[] DefaultOptions =>
            m_DefaultOptions ?? (
                m_DefaultOptions =
                    Enumerable.Range(0, MaxNumCategories)
                        .Select(i => string.Format(DefaultFormatString, i))
                        .ToArray()
            );
        string[] m_DefaultOptions;

        string[] GetOptions()
        {
            if (m_Options != null)
                return m_Options;

            var guids = AssetDatabase.FindAssets($"t:{typeof(T).Name}");
            m_NamesAssets = guids
                .Select(AssetDatabase.GUIDToAssetPath)
                .Select(AssetDatabase.LoadAssetAtPath<T>)
                .Where(c => c != null)
                .ToArray();

            m_Options = m_NamesAssets.FirstOrDefault()?.TagNames.ToArray() ?? DefaultOptions;
            for (var i = 0; i < m_Options.Length; ++i)
            {
                if (string.IsNullOrEmpty(m_Options[i]))
                    m_Options[i] = DefaultOptions[i];
            }

            return m_Options;
        }
        string[] m_Options;

        T[] m_NamesAssets;

        // TODO: remove when all usages of bool[] are migrated
        SerializedProperty GetFirstChildProperty(SerializedProperty property)
        {
            if (!string.IsNullOrEmpty(FirstChildPropertyPath))
                return property.FindPropertyRelative(FirstChildPropertyPath);
            var sp = property.Copy();
            sp.NextVisible(true);
            return sp;
        }

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            if (m_NamesAssets?.Length == 0)
                position.xMax -= Styles.CreateAssetButtonWidth + EditorGUIUtility.standardVerticalSpacing;
            else if (m_NamesAssets?.Length > 1)
                position.xMax -= EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;

            EditorGUI.BeginProperty(position, label, property);

            var controlPosition = EditorGUI.PrefixLabel(position, label);

            var indent = EditorGUI.indentLevel;
            EditorGUI.indentLevel = 0;
            var showMixed = EditorGUI.showMixedValue;

            var value = 0;
            var everything = 0;
            var sp = GetFirstChildProperty(property);
            for (int i = 0, count = MaxNumCategories; i < count; ++i)
            {
                EditorGUI.showMixedValue |= sp.hasMultipleDifferentValues;
                value |= sp.boolValue ? 1 << i : 0;
                everything |= 1 << i;
                sp.NextVisible(false);
            }
            // in case size is smaller than 32
            if (value == everything)
                value = ~0;

            EditorGUI.BeginChangeCheck();
            value = EditorGUI.MaskField(controlPosition, GUIContent.none, value, GetOptions());
            if (EditorGUI.EndChangeCheck())
            {
                sp = GetFirstChildProperty(property);
                for (int i = 0, count = MaxNumCategories; i < count; ++i)
                {
                    sp.boolValue = (value & (1 << i)) != 0;
                    sp.NextVisible(false);
                }
            }

            EditorGUI.showMixedValue = showMixed;
            EditorGUI.indentLevel = indent;

            EditorGUI.EndProperty();

            if (m_NamesAssets?.Length == 0)
            {
                position.width = Styles.CreateAssetButtonWidth;
                position.x = controlPosition.xMax + EditorGUIUtility.standardVerticalSpacing;
                Styles.CreateAssetLabel.tooltip =
                    string.Format(Styles.CreateAssetButtonTooltip, ObjectNames.NicifyVariableName(typeof(T).Name));
                if (GUI.Button(position, Styles.CreateAssetLabel, EditorStyles.miniButton))
                {
                    var assetPath = AssetDatabase.GenerateUniqueAssetPath($"Assets/{typeof(T).Name}.asset");
                    AssetDatabase.CreateAsset(ScriptableObject.CreateInstance<T>(), assetPath);
                    Selection.activeObject = AssetDatabase.LoadAssetAtPath<T>(assetPath);
                    m_Options = null;
                }
            }
            else if (m_NamesAssets.Length > 1)
            {
                var id = GUIUtility.GetControlID(FocusType.Passive);
                if (Event.current.type == EventType.Repaint)
                {
                    position.width = EditorGUIUtility.singleLineHeight;
                    position.x = controlPosition.xMax + EditorGUIUtility.standardVerticalSpacing;
                    Styles.MultipleAssetsWarning.tooltip = string.Format(
                        Styles.MultipleAssetsTooltip,
                        ObjectNames.NicifyVariableName(typeof(T).Name),
                        m_NamesAssets.FirstOrDefault(n => n != null)?.name
                    );
                    GUIStyle.none.Draw(position, Styles.MultipleAssetsWarning, id);
                }
            }
        }
    }

    [CustomPropertyDrawer(typeof(CustomPhysicsBodyTags))]
    class CustomBodyTagsDrawer : TagsDrawer<CustomPhysicsBodyTagNames>
    {
        protected override string DefaultCategoryName => "Custom Physics Body Tag";
        protected override int MaxNumCategories => 8;
    }

    [CustomPropertyDrawer(typeof(CustomPhysicsMaterialTags))]
    class CustomMaterialTagsDrawer : TagsDrawer<CustomPhysicsMaterialTagNames>
    {
        protected override string DefaultCategoryName => "Custom Physics Material Tag";
        protected override int MaxNumCategories => 8;
    }

    [CustomPropertyDrawer(typeof(PhysicsCategoryTags))]
    class PhysicsCategoryTagsDrawer : TagsDrawer<PhysicsCategoryNames>
    {
        protected override string DefaultCategoryName => "Physics Category";
        protected override int MaxNumCategories => 32;
    }
}
