using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEditor;
using UnityEngine;

namespace Unity.Physics.Editor
{
    interface ICustomOptionNamesProvider<T> where T : ScriptableObject
    {
        string[] GetOptions();
        IReadOnlyList<T> NameAssets { get; }
        void Update();
    }

    static class CustomNamesProviderExtensions
    {
        public static void GetOptionsAndNameAssets<T>(
            this ICustomOptionNamesProvider<T> provider,
            ref string[] optionNames, ref T[] nameAssets, string[] defaultOptions
        ) where T : ScriptableObject, IFlagNames
        {
            if (optionNames != null)
                return;
            var guids = AssetDatabase.FindAssets($"t:{typeof(T).Name}");
            nameAssets = guids
                .Select(AssetDatabase.GUIDToAssetPath)
                .Select(AssetDatabase.LoadAssetAtPath<T>)
                .Where(c => c != null)
                .ToArray();
            optionNames = nameAssets.FirstOrDefault()?.FlagNames.ToArray() ?? defaultOptions;
            for (var i = 0; i < optionNames.Length; ++i)
                optionNames[i] = string.IsNullOrEmpty(optionNames[i]) ? defaultOptions[i] : optionNames[i];
        }
    }

    [InitializeOnLoad]
    static class EditorGUIControls
    {
        static EditorGUIControls()
        {
            if (k_SoftSlider == null)
                Debug.LogException(new MissingMemberException("Could not find expected signature of EditorGUI.Slider() for soft slider."));
        }

        static class Styles
        {
            public const float CreateAssetButtonWidth = 24;

            public static readonly string CompatibilityWarning = L10n.Tr("Not compatible with {0}.");
            public static readonly string CreateAssetButtonTooltip =
                L10n.Tr("Click to create a {0} asset and define category names.");
            public static readonly string MultipleAssetsTooltip =
                L10n.Tr("Multiple {0} assets found. UI will display labels defined in {1}.");

            public static readonly GUIContent CreateAssetLabel = new GUIContent { text = "+" };
            public static readonly GUIContent MultipleAssetsWarning =
                new GUIContent { image = EditorGUIUtility.Load("console.warnicon") as Texture };
        }

        public static void DisplayCompatibilityWarning(Rect position, GUIContent label, string incompatibleType)
        {
            EditorGUI.HelpBox(
                EditorGUI.PrefixLabel(position, label),
                string.Format(Styles.CompatibilityWarning, incompatibleType),
                MessageType.Error
            );
        }

        public static void DoCustomNamesPopup<T>(
            Rect position, SerializedProperty property, GUIContent label, ICustomOptionNamesProvider<T> optionsProvider
        ) where T : ScriptableObject
        {
            if (optionsProvider.NameAssets?.Count == 0)
                position.xMax -= Styles.CreateAssetButtonWidth + EditorGUIUtility.standardVerticalSpacing;
            else if (optionsProvider.NameAssets?.Count > 1)
                position.xMax -= EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;

            EditorGUI.BeginProperty(position, label, property);

            var controlPosition = EditorGUI.PrefixLabel(position, label);

            var indent = EditorGUI.indentLevel;
            EditorGUI.indentLevel = 0;
            var showMixed = EditorGUI.showMixedValue;

            var value = 0;
            var everything = 0;
            for (int i = 0, count = property.arraySize; i < count; ++i)
            {
                var sp = property.GetArrayElementAtIndex(i);
                EditorGUI.showMixedValue |= sp.hasMultipleDifferentValues;
                value |= sp.boolValue ? 1 << i : 0;
                everything |= 1 << i;
            }
            // in case size is smaller than 32
            if (value == everything)
                value = ~0;

            EditorGUI.BeginChangeCheck();
            value = EditorGUI.MaskField(controlPosition, GUIContent.none, value, optionsProvider.GetOptions());
            if (EditorGUI.EndChangeCheck())
            {
                for (int i = 0, count = property.arraySize; i < count; ++i)
                    property.GetArrayElementAtIndex(i).boolValue = (value & (1 << i)) != 0;
            }

            EditorGUI.showMixedValue = showMixed;
            EditorGUI.indentLevel = indent;

            EditorGUI.EndProperty();

            if (optionsProvider.NameAssets?.Count == 0)
            {
                position.width = Styles.CreateAssetButtonWidth;
                position.x = controlPosition.xMax + EditorGUIUtility.standardVerticalSpacing;
                Styles.CreateAssetLabel.tooltip =
                    string.Format(Styles.CreateAssetButtonTooltip, ObjectNames.NicifyVariableName(typeof(T).Name));
                if (GUI.Button(position, Styles.CreateAssetLabel, EditorStyles.miniButton))
                {
                    var assetPath =
                        AssetDatabase.GenerateUniqueAssetPath($"Assets/{typeof(T).Name}.asset");
                    AssetDatabase.CreateAsset(ScriptableObject.CreateInstance<T>(), assetPath);
                    Selection.activeObject = AssetDatabase.LoadAssetAtPath<T>(assetPath);
                    optionsProvider.Update();
                }
            }
            else if (optionsProvider.NameAssets.Count > 1)
            {
                var id = GUIUtility.GetControlID(FocusType.Passive);
                if (Event.current.type == EventType.Repaint)
                {
                    position.width = EditorGUIUtility.singleLineHeight;
                    position.x = controlPosition.xMax + EditorGUIUtility.standardVerticalSpacing;
                    Styles.MultipleAssetsWarning.tooltip = string.Format(
                        Styles.MultipleAssetsTooltip,
                        ObjectNames.NicifyVariableName(typeof(T).Name),
                        optionsProvider.NameAssets.FirstOrDefault(n => n != null)?.name
                    );
                    GUIStyle.none.Draw(position, Styles.MultipleAssetsWarning, id);
                }
            }
        }

        static readonly MethodInfo k_SoftSlider = typeof(EditorGUI).GetMethod(
            "Slider",
            BindingFlags.Static | BindingFlags.NonPublic,
            null,
            new[]
            {
                typeof(Rect),          // position
                typeof(GUIContent),    // label
                typeof(float),         // value
                typeof(float),         // sliderMin
                typeof(float),         // sliderMax
                typeof(float),         // textFieldMin
                typeof(float)          // textFieldMax
            },
            Array.Empty<ParameterModifier>()
        );

        static readonly object[] k_SoftSliderArgs = new object[7];

        public static void SoftSlider(
            Rect position, GUIContent label, SerializedProperty property,
            float sliderMin, float sliderMax,
            float textFieldMin, float textFieldMax
        )
        {
            if (property.propertyType != SerializedPropertyType.Float)
            {
                DisplayCompatibilityWarning(position, label, property.propertyType.ToString());
            }
            else if (k_SoftSlider == null)
            {
                EditorGUI.BeginChangeCheck();
                EditorGUI.PropertyField(position, property, label);
                if (EditorGUI.EndChangeCheck())
                    property.floatValue = math.clamp(property.floatValue, textFieldMin, textFieldMax);
            }
            else
            {
                k_SoftSliderArgs[0] = position;
                k_SoftSliderArgs[1] = label;
                k_SoftSliderArgs[2] = property.floatValue;
                k_SoftSliderArgs[3] = sliderMin;
                k_SoftSliderArgs[4] = sliderMax;
                k_SoftSliderArgs[5] = textFieldMin;
                k_SoftSliderArgs[6] = textFieldMax;
                EditorGUI.BeginProperty(position, label, property);
                EditorGUI.BeginChangeCheck();
                var result = k_SoftSlider.Invoke(null, k_SoftSliderArgs);
                if (EditorGUI.EndChangeCheck())
                    property.floatValue = (float)result;
                EditorGUI.EndProperty();
            }
        }
    }
}