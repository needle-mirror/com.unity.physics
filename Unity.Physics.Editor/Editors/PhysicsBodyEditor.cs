using System.Collections.Generic;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEditor;
using UnityEngine;

namespace Unity.Physics.Editor
{
    [CustomEditor(typeof(PhysicsBody))]
    [CanEditMultipleObjects]
    class PhysicsBodyEditor : BaseEditor
    {
        static class Content
        {
            public static readonly GUIContent MassLabel = EditorGUIUtility.TrTextContent("Mass");
            public static readonly GUIContent CenterOfMassLabel = EditorGUIUtility.TrTextContent(
                "Center of Mass", "Center of mass in the space of this body's transform."
                );
            public static readonly GUIContent InertiaTensorLabel = EditorGUIUtility.TrTextContent(
                "Inertia Tensor", "Resistance to angular motion about each axis of rotation."
            );
            public static readonly GUIContent OrientationLabel = EditorGUIUtility.TrTextContent(
                "Orientation", "Orientation of the body's inertia tensor in the space of its transform."
                );
        }

        #pragma warning disable 649
        [AutoPopulate] SerializedProperty m_MotionType;
        [AutoPopulate] SerializedProperty m_Mass;
        [AutoPopulate] SerializedProperty m_GravityFactor;
        [AutoPopulate] SerializedProperty m_LinearDamping;
        [AutoPopulate] SerializedProperty m_AngularDamping;
        [AutoPopulate] SerializedProperty m_InitialLinearVelocity;
        [AutoPopulate] SerializedProperty m_InitialAngularVelocity;
        [AutoPopulate] SerializedProperty m_OverrideDefaultMassDistribution;
        [AutoPopulate] SerializedProperty m_CenterOfMass;
        [AutoPopulate] SerializedProperty m_Orientation;
        [AutoPopulate] SerializedProperty m_InertiaTensor;
        #pragma warning restore 649

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUI.BeginChangeCheck();

            EditorGUILayout.PropertyField(m_MotionType);

            var dynamic = m_MotionType.intValue == (int)BodyMotionType.Dynamic;

            if (dynamic)
                EditorGUILayout.PropertyField(m_Mass, Content.MassLabel);
            else
            {
                EditorGUI.BeginDisabledGroup(true);
                var position = EditorGUILayout.GetControlRect(true, EditorGUIUtility.singleLineHeight);
                EditorGUI.BeginProperty(position, Content.MassLabel, m_Mass);
                EditorGUI.FloatField(position, Content.MassLabel, float.PositiveInfinity);
                EditorGUI.EndProperty();
                EditorGUI.EndDisabledGroup();
            }

            if (m_MotionType.intValue == (int)BodyMotionType.Dynamic)
            {
                EditorGUILayout.PropertyField(m_LinearDamping, true);
                EditorGUILayout.PropertyField(m_AngularDamping, true);
            }

            if (m_MotionType.intValue != (int)BodyMotionType.Static)
            {
                EditorGUILayout.PropertyField(m_InitialLinearVelocity, true);
                EditorGUILayout.PropertyField(m_InitialAngularVelocity, true);
            }

            if (m_MotionType.intValue == (int)BodyMotionType.Dynamic)
            {
                EditorGUILayout.PropertyField(m_GravityFactor, true);
            }

            if (m_MotionType.intValue != (int)BodyMotionType.Static)
            {
                EditorGUILayout.PropertyField(m_OverrideDefaultMassDistribution);
                if (m_OverrideDefaultMassDistribution.boolValue)
                {
                    ++EditorGUI.indentLevel;
                    EditorGUILayout.PropertyField(m_CenterOfMass, Content.CenterOfMassLabel);

                    EditorGUI.BeginDisabledGroup(!dynamic);
                    if (dynamic)
                    {
                        EditorGUILayout.PropertyField(m_Orientation, Content.OrientationLabel);
                        EditorGUILayout.PropertyField(m_InertiaTensor, Content.InertiaTensorLabel);
                    }
                    else
                    {
                        EditorGUI.BeginDisabledGroup(true);
                        var position =
                            EditorGUILayout.GetControlRect(true, EditorGUI.GetPropertyHeight(m_InertiaTensor));
                        EditorGUI.BeginProperty(position, Content.InertiaTensorLabel, m_InertiaTensor);
                        EditorGUI.Vector3Field(position, Content.InertiaTensorLabel,
                            Vector3.one * float.PositiveInfinity);
                        EditorGUI.EndProperty();
                        EditorGUI.EndDisabledGroup();
                    }

                    EditorGUI.EndDisabledGroup();

                    --EditorGUI.indentLevel;
                }
            }

            if (EditorGUI.EndChangeCheck())
                serializedObject.ApplyModifiedProperties();

            DisplayStatusMessages();
        }

        List<MatrixState> m_MatrixStates = new List<MatrixState>();

        void DisplayStatusMessages()
        {
            m_MatrixStates.Clear();
            foreach (var t in targets)
            {
                var localToWorld = (float4x4)(t as PhysicsBody).transform.localToWorldMatrix;
                m_MatrixStates.Add(ManipulatorUtility.GetMatrixState(ref localToWorld));
            }

            string matrixStatusMessage;
            var matrixStatus = MatrixGUIUtility.GetMatrixStatusMessage(m_MatrixStates, out matrixStatusMessage);
            if (matrixStatus != MessageType.None)
                EditorGUILayout.HelpBox(matrixStatusMessage, MessageType.Warning);
        }
    }
}
