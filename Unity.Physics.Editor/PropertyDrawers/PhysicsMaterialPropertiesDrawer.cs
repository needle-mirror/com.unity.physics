using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Physics.Authoring;
using UnityEditor;
using UnityEngine;

namespace Unity.Physics.Editor
{
    [CustomPropertyDrawer(typeof(PhysicsMaterialProperties))]
    class PhysicsMaterialPropertiesDrawer : BaseDrawer,
        ICustomOptionNamesProvider<PhysicsCategoryNames>,
        ICustomOptionNamesProvider<CustomFlagNames>
    {
        static class Content
        {
            static readonly string DefaultCategoryFormatString = L10n.Tr("(Undefined Category {0})");
            static readonly string DefaultCustomFlagFormatString = L10n.Tr("Custom Flag {0}");

            public static readonly string[] DefaultCategoriesOptions =
                Enumerable.Range(0, 32).Select(i => string.Format(DefaultCategoryFormatString, i)).ToArray();
            public static readonly string[] DefaultCustomFlags =
                Enumerable.Range(0, 8).Select(i => string.Format(DefaultCustomFlagFormatString, i)).ToArray();

            public static readonly GUIContent AdvancedGroupFoldout = EditorGUIUtility.TrTextContent("Advanced");
            public static readonly GUIContent BelongsToLabel = EditorGUIUtility.TrTextContent(
                "Belongs To",
                "Specifies the categories to which this object belongs."
            );
            public static readonly GUIContent CollidesWithLabel = EditorGUIUtility.TrTextContent(
                "Collides With",
                "Specifies the categories of objects with which this object will collide, " +
                "or with which it will raise events if intersecting a trigger."
            );
            public static readonly GUIContent CollisionFilterGroupFoldout =
                EditorGUIUtility.TrTextContent("Collision Filter");
            public static readonly GUIContent CustomFlagsLabel =
                EditorGUIUtility.TrTextContent("Custom Flags", "Specify custom flags to read at run-time.");
            public static readonly GUIContent FrictionLabel = EditorGUIUtility.TrTextContent(
                "Friction",
                "Specifies how resistant the body is to motion when sliding along other surfaces, " +
                "as well as what value should be used when colliding with an object that has a different value."
            );
            public static readonly GUIContent RaisesCollisionEventsLabel = EditorGUIUtility.TrTextContent(
                "Raises Collision Events",
                "Specifies whether the shape should raise notifications of collisions with other shapes."
            );
            public static readonly GUIContent RestitutionLabel = EditorGUIUtility.TrTextContent(
                "Restitution",
                "Specifies how bouncy the object will be when colliding with other surfaces, " +
                "as well as what value should be used when colliding with an object that has a different value."
                );
            public static readonly GUIContent TriggerLabel = EditorGUIUtility.TrTextContent(
                "Is Trigger",
                "Specifies that the shape is a volume that will raise events when intersecting other shapes, but will not cause a collision response."
            );
        }

        string[] ICustomOptionNamesProvider<PhysicsCategoryNames>.GetOptions()
        {
            this.GetOptionsAndNameAssets(ref m_CategoriesOptionNames, ref m_CategoriesAssets, Content.DefaultCategoriesOptions);
            return m_CategoriesOptionNames;
        }
        string[] m_CategoriesOptionNames;

        IReadOnlyList<PhysicsCategoryNames> ICustomOptionNamesProvider<PhysicsCategoryNames>.NameAssets => m_CategoriesAssets;
        PhysicsCategoryNames[] m_CategoriesAssets;

        void ICustomOptionNamesProvider<PhysicsCategoryNames>.Update() => m_CategoriesOptionNames = null;

        string[] ICustomOptionNamesProvider<CustomFlagNames>.GetOptions()
        {
            this.GetOptionsAndNameAssets(ref m_CustomFlagsOptionNames, ref m_CustomFlagsAssets, Content.DefaultCustomFlags);
            return m_CustomFlagsOptionNames;
        }
        string[] m_CustomFlagsOptionNames;

        IReadOnlyList<CustomFlagNames> ICustomOptionNamesProvider<CustomFlagNames>.NameAssets => m_CustomFlagsAssets;
        CustomFlagNames[] m_CustomFlagsAssets;

        void ICustomOptionNamesProvider<CustomFlagNames>.Update() => m_CustomFlagsOptionNames = null;

        const string k_CollisionFilterGroupKey = "m_BelongsTo";
        const string k_AdvancedGroupKey = "m_RaisesCollisionEvents";

        Dictionary<string, SerializedObject> m_SerializedTemplates = new Dictionary<string, SerializedObject>();

        SerializedProperty GetTemplateValueProperty(SerializedProperty property)
        {
            var key = property.propertyPath;
            var template = property.FindPropertyRelative("m_Template").objectReferenceValue;
            SerializedObject serializedTemplate;
            if (
                !m_SerializedTemplates.TryGetValue(key, out serializedTemplate)
                || serializedTemplate?.targetObject != template
            )
                m_SerializedTemplates[key] = serializedTemplate = template == null ? null : new SerializedObject(template);
            serializedTemplate?.Update();
            return serializedTemplate?.FindProperty("m_Value");
        }

        void FindToggleAndValueProperties(
            SerializedProperty property, SerializedProperty templateValueProperty, string relativePath,
            out SerializedProperty toggle, out SerializedProperty value
        )
        {
            var relative = property.FindPropertyRelative(relativePath);
            toggle = relative.FindPropertyRelative("m_Override");
            value = toggle.boolValue || templateValueProperty == null
                ? relative.FindPropertyRelative("m_Value")
                : templateValueProperty.FindPropertyRelative(relativePath).FindPropertyRelative("m_Value");
        }

        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            var templateValueProperty = GetTemplateValueProperty(property);

            // m_IsTrigger, collision filter foldout, advanced foldout
            var height = 3f * EditorGUIUtility.singleLineHeight + 2f * EditorGUIUtility.standardVerticalSpacing;

            // m_BelongsTo, m_CollidesWith
            var group = property.FindPropertyRelative(k_CollisionFilterGroupKey);
            if (group.isExpanded)
                height += 2f * (EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing);

            // m_RaisesCollisionEvents, m_CustomFlags
            group = property.FindPropertyRelative(k_AdvancedGroupKey);
            if (group.isExpanded)
                height += 2f * (EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing);

            // m_Template
            if (property.FindPropertyRelative("m_SupportsTemplate").boolValue)
                height += EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing;

            // m_Friction, m_Restitution
            FindToggleAndValueProperties(property, templateValueProperty, "m_IsTrigger", out _, out var trigger);
            if (!trigger.boolValue)
                height += 2f * (EditorGUIUtility.singleLineHeight + EditorGUIUtility.standardVerticalSpacing);

            return height;
        }

        protected override bool IsCompatible(SerializedProperty property) => true;

        delegate bool DisplayPropertyCallback(
            Rect position, SerializedProperty property, GUIContent label, bool includeChildren
        );

        static void DisplayOverridableProperty(
            Rect position, GUIContent label, SerializedProperty toggle, SerializedProperty value, bool
            templateAssigned, DisplayPropertyCallback drawPropertyField
        )
        {
            if (templateAssigned)
            {
                var labelWidth = EditorGUIUtility.labelWidth;
                EditorGUIUtility.labelWidth -= 16f;
                var togglePosition = new Rect(position) { width = EditorGUIUtility.labelWidth + 16f };
                EditorGUI.PropertyField(togglePosition, toggle, label);
                EditorGUIUtility.labelWidth = labelWidth;

                EditorGUI.BeginDisabledGroup(!toggle.boolValue);
                var indent = EditorGUI.indentLevel;
                EditorGUI.indentLevel = 0;
                drawPropertyField(
                    new Rect(position) { xMin = togglePosition.xMax }, value, GUIContent.none, true
                );
                EditorGUI.indentLevel = indent;
                EditorGUI.EndDisabledGroup();
            }
            else
            {
                drawPropertyField(position, value,  label, true);
            }
        }

        static void DisplayOverridableProperty(
            Rect position, GUIContent label, SerializedProperty toggle, SerializedProperty value, bool templateAssigned
        )
        {
            DisplayOverridableProperty(position, label, toggle, value, templateAssigned, EditorGUI.PropertyField);
        }

        bool CategoriesPopup(Rect position, SerializedProperty property, GUIContent label, bool includeChildren)
        {
            EditorGUIControls.DoCustomNamesPopup(
                position, property, label, this as ICustomOptionNamesProvider<PhysicsCategoryNames>
            );
            return includeChildren && property.hasChildren && property.isExpanded;
        }

        bool FlagsPopup(Rect position, SerializedProperty property, GUIContent label, bool includeChildren)
        {
            EditorGUIControls.DoCustomNamesPopup(
                position, property, label, this as ICustomOptionNamesProvider<CustomFlagNames>
            );
            return includeChildren && property.hasChildren && property.isExpanded;
        }

        protected override void DoGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            var template = property.FindPropertyRelative("m_Template");
            var templateAssigned = template.objectReferenceValue != null;
            var supportsTemplate = property.FindPropertyRelative("m_SupportsTemplate");
            if (supportsTemplate.boolValue)
            {
                position.height = EditorGUI.GetPropertyHeight(template);
                EditorGUI.PropertyField(position, template);

                position.y = position.yMax + EditorGUIUtility.standardVerticalSpacing;
            }

            var templateValue = GetTemplateValueProperty(property);

            FindToggleAndValueProperties(property, templateValue, "m_IsTrigger", out var toggle, out var trigger);
            position.height = EditorGUIUtility.singleLineHeight;
            DisplayOverridableProperty(position, Content.TriggerLabel, toggle, trigger, templateAssigned);

            if (!trigger.boolValue)
            {
                FindToggleAndValueProperties(property, templateValue, "m_Friction", out toggle, out var friction);
                position.y = position.yMax + EditorGUIUtility.standardVerticalSpacing;
                position.height = EditorGUIUtility.singleLineHeight;
                DisplayOverridableProperty(position, Content.FrictionLabel, toggle, friction, templateAssigned);

                FindToggleAndValueProperties(property, templateValue, "m_Restitution", out toggle, out var restitution);
                position.y = position.yMax + EditorGUIUtility.standardVerticalSpacing;
                position.height = EditorGUIUtility.singleLineHeight;
                DisplayOverridableProperty(position, Content.RestitutionLabel, toggle, restitution, templateAssigned);
            }

            // collision filter group
            var collisionFilterGroup = property.FindPropertyRelative(k_CollisionFilterGroupKey);
            position.y = position.yMax + EditorGUIUtility.standardVerticalSpacing;
            position.height = EditorGUIUtility.singleLineHeight;
            collisionFilterGroup.isExpanded =
                EditorGUI.Foldout(position, collisionFilterGroup.isExpanded, Content.CollisionFilterGroupFoldout);
            if (collisionFilterGroup.isExpanded)
            {
                ++EditorGUI.indentLevel;

                FindToggleAndValueProperties(property, templateValue, "m_BelongsTo", out toggle, out var belongsTo);
                position.y = position.yMax + EditorGUIUtility.standardVerticalSpacing;
                position.height = EditorGUIUtility.singleLineHeight;
                DisplayOverridableProperty(
                    position, Content.BelongsToLabel, toggle, belongsTo, templateAssigned, CategoriesPopup
                );

                FindToggleAndValueProperties(property, templateValue, "m_CollidesWith", out toggle, out var collidesWith);
                position.y = position.yMax + EditorGUIUtility.standardVerticalSpacing;
                position.height = EditorGUIUtility.singleLineHeight;
                DisplayOverridableProperty(
                    position, Content.CollidesWithLabel, toggle, collidesWith, templateAssigned, CategoriesPopup
                );

                --EditorGUI.indentLevel;
            }

            // advanced group
            var advancedGroup = property.FindPropertyRelative(k_AdvancedGroupKey);
            position.y = position.yMax + EditorGUIUtility.standardVerticalSpacing;
            position.height = EditorGUIUtility.singleLineHeight;
            advancedGroup.isExpanded =
                EditorGUI.Foldout(position, advancedGroup.isExpanded, Content.AdvancedGroupFoldout);
            if (advancedGroup.isExpanded)
            {
                ++EditorGUI.indentLevel;

                if (!trigger.boolValue)
                {
                    FindToggleAndValueProperties(property, templateValue, "m_RaisesCollisionEvents", out toggle, out var raisesEvents);
                    position.y = position.yMax + EditorGUIUtility.standardVerticalSpacing;
                    position.height = EditorGUIUtility.singleLineHeight;
                    DisplayOverridableProperty(
                        position,
                        Content.RaisesCollisionEventsLabel,
                        toggle,
                        raisesEvents,
                        templateAssigned,
                        (DisplayPropertyCallback)EditorGUI.PropertyField
                    );
                }

                FindToggleAndValueProperties(property, templateValue, "m_CustomFlags", out toggle, out var customFlags);
                position.y = position.yMax + EditorGUIUtility.standardVerticalSpacing;
                position.height = EditorGUIUtility.singleLineHeight;
                DisplayOverridableProperty(
                    position, Content.CustomFlagsLabel, toggle, customFlags, templateAssigned, FlagsPopup
                );

                --EditorGUI.indentLevel;
            }
        }
    }
}
