using System;
using System.Collections.Generic;
using System.Linq;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEditor;
using UnityEditor.IMGUI.Controls;
using UnityEngine;

namespace Unity.Physics.Editor
{
    [CustomEditor(typeof(PhysicsShape))]
    [CanEditMultipleObjects]
    class PhysicsShapeEditor : BaseEditor
    {
        static class Styles
        {
            public static readonly string GenericUndoMessage = L10n.Tr("Change Shape");
            public static readonly string MultipleShapeTypesLabel =
                L10n.Tr("Multiple shape types in current selection.");

            public static readonly GUIContent FitToRenderMeshesLabel =
                EditorGUIUtility.TrTextContent("Fit to Enabled Render Meshes");
            public static readonly GUIContent CenterLabel = EditorGUIUtility.TrTextContent("Center");
            public static readonly GUIContent SizeLabel = EditorGUIUtility.TrTextContent("Size");
            public static readonly GUIContent OrientationLabel = EditorGUIUtility.TrTextContent(
                "Orientation", "Euler orientation in the shape's local space (ZXY order)."
            );
            public static readonly GUIContent RadiusLabel = EditorGUIUtility.TrTextContent("Radius");
            public static readonly GUIContent MaterialLabel = EditorGUIUtility.TrTextContent("Material");

            static readonly string k_Plural =
                $"One or more selected {ObjectNames.NicifyVariableName(typeof(PhysicsShape).Name)}s";
            static readonly string k_Singular =
                $"This {ObjectNames.NicifyVariableName(typeof(PhysicsShape).Name)}";

            static readonly string[] k_FitToRenderMeshes =
            {
                L10n.Tr($"{k_Singular} has non-uniform scale. Trying to fit the shape to render meshes might produce unexpected results."),
                L10n.Tr($"{k_Plural} has non-uniform scale. Trying to fit the shape to render meshes might produce unexpected results.")
            };

            public static string GetFitToRenderMeshesWarning(int numTargets) =>
                numTargets == 1 ? k_FitToRenderMeshes[0] : k_FitToRenderMeshes[1];

            static readonly string[] k_NoGeometryWarning =
            {
                L10n.Tr($"{k_Singular} has no enabled render meshes in its hierarchy and no custom mesh assigned."),
                L10n.Tr($"{k_Plural} has no enabled render meshes in their hierarchies and no custom mesh assigned.")
            };

            public static string GetNoGeometryWarning(int numTargets) =>
                numTargets == 1 ? k_NoGeometryWarning[0] : k_NoGeometryWarning[1];

            static readonly string[] k_StaticColliderStatusMessage =
            {
                L10n.Tr($"{k_Singular} will be considered static. Add a {ObjectNames.NicifyVariableName(typeof(PhysicsBody).Name)} component if you will move it at run-time."),
                L10n.Tr($"{k_Plural} will be considered static. Add a {ObjectNames.NicifyVariableName(typeof(PhysicsBody).Name)} component if you will move them at run-time.")
            };

            public static string GetStaticColliderStatusMessage(int numTargets) =>
                numTargets == 1 ? k_StaticColliderStatusMessage[0] : k_StaticColliderStatusMessage[1];
        }

        #pragma warning disable 649
        [AutoPopulate] SerializedProperty m_ShapeType;
        [AutoPopulate] SerializedProperty m_PrimitiveCenter;
        [AutoPopulate] SerializedProperty m_PrimitiveSize;
        [AutoPopulate] SerializedProperty m_PrimitiveOrientation;
        [AutoPopulate] SerializedProperty m_Capsule;
        [AutoPopulate] SerializedProperty m_Cylinder;
        [AutoPopulate] SerializedProperty m_SphereRadius;
        [AutoPopulate] SerializedProperty m_ConvexRadius;
        [AutoPopulate] SerializedProperty m_CustomMesh;
        [AutoPopulate] SerializedProperty m_Material;
        #pragma warning restore 649

        bool m_HasGeometry;
        int m_NumImplicitStatic;

        protected override void OnEnable()
        {
            base.OnEnable();

            m_HasGeometry = true;
            var pointCloud = new NativeList<float3>(65535, Allocator.Temp);
            foreach (PhysicsShape shape in targets)
            {
                shape.GetConvexHullProperties(pointCloud);
                m_HasGeometry &= pointCloud.Length > 0;
                if (!m_HasGeometry)
                    break;
            }
            pointCloud.Dispose();

            m_NumImplicitStatic = targets.Cast<PhysicsShape>().Count(
                shape => shape.GetPrimaryBody() == shape.gameObject
                    && shape.GetComponent<PhysicsBody>() == null
                    && shape.GetComponent<Rigidbody>() == null
            );

            var wireframeShader = Shader.Find("Hidden/Physics/ShapeHandle");
            m_PreviewMeshMaterial = new UnityEngine.Material(wireframeShader) { hideFlags = HideFlags.HideAndDontSave };
        }

        void OnDisable()
        {
            foreach (var preview in m_PreviewMeshes.Values)
            {
                if (preview.Mesh != null)
                    DestroyImmediate(preview.Mesh);
            }
            if (m_PreviewMeshMaterial != null)
                DestroyImmediate(m_PreviewMeshMaterial);
        }

        class PreviewMeshData
        {
            public ShapeType Type;
            public UnityEngine.Mesh SourceMesh;
            public UnityEngine.Mesh Mesh;
        }

        Dictionary<PhysicsShape, PreviewMeshData> m_PreviewMeshes = new Dictionary<PhysicsShape, PreviewMeshData>();
        UnityEngine.Material m_PreviewMeshMaterial;

        UnityEngine.Mesh GetPreviewMesh(PhysicsShape shape)
        {
            if (shape.ShapeType != ShapeType.ConvexHull && shape.ShapeType != ShapeType.Mesh)
                return null;

            if (!m_PreviewMeshes.TryGetValue(shape, out var preview))
                preview = m_PreviewMeshes[shape] = new PreviewMeshData();

            if (preview.Type != shape.ShapeType || preview.SourceMesh != shape.GetMesh())
            {
                if (preview.Mesh == null)
                    preview.Mesh = new UnityEngine.Mesh { hideFlags = HideFlags.HideAndDontSave };
                preview.SourceMesh = shape.GetMesh();
                preview.Type = shape.ShapeType;
                preview.Mesh.Clear();
                if (shape.ShapeType == ShapeType.ConvexHull)
                {
                    // TODO: populate with the actual collision data
                }
                else
                {
                    // TODO: populate with the actual collision data
                    if (preview.SourceMesh != null)
                    {
                        preview.Mesh.vertices = preview.SourceMesh.vertices;
                        preview.Mesh.normals = preview.SourceMesh.normals;
                        preview.Mesh.triangles = preview.SourceMesh.triangles;
                    }
                }
                preview.Mesh.RecalculateBounds();
            }
            return preview.Mesh;
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            UpdateStatusMessages();

            EditorGUI.BeginChangeCheck();

            DisplayShapeSelector();

            ++EditorGUI.indentLevel;

            if (m_ShapeType.hasMultipleDifferentValues)
                EditorGUILayout.HelpBox(Styles.MultipleShapeTypesLabel, MessageType.None);
            else
            {
                switch ((ShapeType)m_ShapeType.intValue)
                {
                    case ShapeType.Box:
                        DisplayBoxControls();
                        break;
                    case ShapeType.Capsule:
                        DisplayCapsuleControls();
                        break;
                    case ShapeType.Sphere:
                        DisplaySphereControls();
                        break;
                    case ShapeType.Cylinder:
                        DisplayCylinderControls();
                        break;
                    case ShapeType.Plane:
                        DisplayPlaneControls();
                        break;
                    case ShapeType.ConvexHull:
                        EditorGUILayout.PropertyField(m_ConvexRadius);
                        DisplayMeshControls();
                        break;
                    case ShapeType.Mesh:
                        DisplayMeshControls();
                        break;
                    default:
                        throw new UnimplementedShapeException((ShapeType)m_ShapeType.intValue);
                }
            }

            --EditorGUI.indentLevel;

            EditorGUILayout.LabelField(Styles.MaterialLabel);

            ++EditorGUI.indentLevel;

            EditorGUILayout.PropertyField(m_Material);

            --EditorGUI.indentLevel;

            if (m_StatusMessages.Count > 0)
                EditorGUILayout.HelpBox(string.Join("\n\n", m_StatusMessages), MessageType.None);

            if (EditorGUI.EndChangeCheck())
                serializedObject.ApplyModifiedProperties();
        }

        MessageType m_MatrixStatus;
        List<MatrixState> m_MatrixStates = new List<MatrixState>();
        List<string> m_StatusMessages = new List<string>(8);

        void UpdateStatusMessages()
        {
            m_StatusMessages.Clear();

            if (m_NumImplicitStatic != 0)
                m_StatusMessages.Add(Styles.GetStaticColliderStatusMessage(targets.Length));

            var hierarchyStatusMessage = StatusMessageUtility.GetHierarchyStatusMessage(targets);
            if (!string.IsNullOrEmpty(hierarchyStatusMessage))
                m_StatusMessages.Add(hierarchyStatusMessage);

            m_MatrixStates.Clear();
            foreach (var t in targets)
            {
                var localToWorld = (float4x4)(t as Component).transform.localToWorldMatrix;
                m_MatrixStates.Add(ManipulatorUtility.GetMatrixState(ref localToWorld));
            }

            m_MatrixStatus = StatusMessageUtility.GetMatrixStatusMessage(m_MatrixStates, out var matrixStatusMessage);
            if (m_MatrixStatus != MessageType.None)
                m_StatusMessages.Add(matrixStatusMessage);
        }

        void DisplayShapeSelector()
        {
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(m_ShapeType);
            if (!EditorGUI.EndChangeCheck())
                return;

            Undo.RecordObjects(targets, Styles.GenericUndoMessage);
            foreach (PhysicsShape shape in targets)
            {
                switch ((ShapeType)m_ShapeType.intValue)
                {
                    case ShapeType.Box:
                        shape.GetBoxProperties(out var center, out var size, out EulerAngles orientation);
                        shape.SetBox(center, size, orientation);
                        break;
                    case ShapeType.Capsule:
                        shape.GetCapsuleProperties(out center, out var height, out var radius, out orientation);
                        shape.SetCapsule(center, height, radius, orientation);
                        break;
                    case ShapeType.Sphere:
                        shape.GetSphereProperties(out center, out radius, out orientation);
                        shape.SetSphere(center, radius, orientation);
                        break;
                    case ShapeType.Cylinder:
                        shape.GetCylinderProperties(out center, out height, out radius, out orientation);
                        shape.SetCylinder(center, height, radius, orientation);
                        break;
                    case ShapeType.Plane:
                        shape.GetPlaneProperties(out center, out var size2D, out orientation);
                        shape.SetPlane(center, size2D, orientation);
                        break;
                    case ShapeType.ConvexHull:
                    case ShapeType.Mesh:
                        return;
                    default:
                        throw new UnimplementedShapeException((ShapeType)m_ShapeType.intValue);
                }
                EditorUtility.SetDirty(shape);
            }

            GUIUtility.ExitGUI();
        }

        void FitToRenderMeshesButton()
        {
            EditorGUI.BeginDisabledGroup(!m_HasGeometry || EditorUtility.IsPersistent(target));
            var rect = EditorGUI.IndentedRect(
                EditorGUILayout.GetControlRect(false, EditorGUIUtility.singleLineHeight, EditorStyles.miniButton)
            );
            if (GUI.Button(rect, Styles.FitToRenderMeshesLabel, EditorStyles.miniButton))
            {
                Undo.RecordObjects(targets, Styles.FitToRenderMeshesLabel.text);
                foreach (PhysicsShape shape in targets)
                {
                    shape.FitToEnabledRenderMeshes();
                    EditorUtility.SetDirty(shape);
                }
            }
            if (GUI.enabled && m_MatrixStatus > MessageType.Info)
                EditorGUILayout.HelpBox(Styles.GetFitToRenderMeshesWarning(targets.Length), m_MatrixStatus);
            EditorGUI.EndDisabledGroup();
        }

        void DisplayBoxControls()
        {
            EditorGUILayout.PropertyField(m_PrimitiveSize, Styles.SizeLabel, true);

            EditorGUILayout.PropertyField(m_PrimitiveCenter, Styles.CenterLabel, true);
            EditorGUILayout.PropertyField(m_PrimitiveOrientation, Styles.OrientationLabel, true);

            EditorGUILayout.PropertyField(m_ConvexRadius);

            FitToRenderMeshesButton();
        }

        void DisplayCapsuleControls()
        {
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(m_Capsule);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObjects(targets, Styles.GenericUndoMessage);
                foreach (PhysicsShape shape in targets)
                {
                    shape.GetCapsuleProperties(
                        out var center, out var height, out var radius, out EulerAngles orientation
                    );
                    shape.SetCapsule(center, height, radius, orientation);
                    EditorUtility.SetDirty(shape);
                }
            }

            EditorGUILayout.PropertyField(m_PrimitiveCenter, Styles.CenterLabel, true);
            EditorGUILayout.PropertyField(m_PrimitiveOrientation, Styles.OrientationLabel, true);

            FitToRenderMeshesButton();
        }

        void DisplaySphereControls()
        {
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(m_SphereRadius, Styles.RadiusLabel);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObjects(targets, Styles.GenericUndoMessage);
                foreach (PhysicsShape shape in targets)
                {
                    shape.GetSphereProperties(out var center, out var radius, out EulerAngles orientation);
                    shape.SetSphere(center, m_SphereRadius.floatValue, orientation);
                    EditorUtility.SetDirty(shape);
                }
            }

            EditorGUILayout.PropertyField(m_PrimitiveCenter, Styles.CenterLabel, true);
            EditorGUILayout.PropertyField(m_PrimitiveOrientation, Styles.OrientationLabel, true);

            FitToRenderMeshesButton();
        }

        void DisplayCylinderControls()
        {
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(m_Cylinder);
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObjects(targets, Styles.GenericUndoMessage);
                foreach (PhysicsShape shape in targets)
                {
                    shape.GetCylinderProperties(
                        out var center, out var height, out var radius, out EulerAngles orientation
                    );
                    shape.SetCylinder(center, height, radius, orientation);
                    EditorUtility.SetDirty(shape);
                }
            }

            EditorGUILayout.PropertyField(m_PrimitiveCenter, Styles.CenterLabel, true);
            EditorGUILayout.PropertyField(m_PrimitiveOrientation, Styles.OrientationLabel, true);

            EditorGUILayout.PropertyField(m_ConvexRadius);

            FitToRenderMeshesButton();
        }

        void DisplayPlaneControls()
        {
            EditorGUILayout.PropertyField(m_PrimitiveSize, Styles.SizeLabel, true);

            EditorGUILayout.PropertyField(m_PrimitiveCenter, Styles.CenterLabel, true);
            EditorGUILayout.PropertyField(m_PrimitiveOrientation, Styles.OrientationLabel, true);

            FitToRenderMeshesButton();
        }

        void DisplayMeshControls()
        {
            EditorGUILayout.PropertyField(m_CustomMesh);
            if (!m_HasGeometry && m_CustomMesh.objectReferenceValue == null)
                EditorGUILayout.HelpBox(Styles.GetNoGeometryWarning(targets.Length), MessageType.Error);
        }

        // TODO: implement interactive tool modes
        static readonly ConvexBoxBoundsHandle s_Box =
            new ConvexBoxBoundsHandle { handleColor = Color.clear };
        static readonly CapsuleBoundsHandle s_Capsule =
            new CapsuleBoundsHandle { handleColor = Color.clear, heightAxis = CapsuleBoundsHandle.HeightAxis.Z };
        static readonly ConvexCylinderBoundsHandle s_ConvexCylinder =
            new ConvexCylinderBoundsHandle { handleColor = Color.clear };
        static readonly SphereBoundsHandle s_Sphere =
            new SphereBoundsHandle { handleColor = Color.clear };
        static readonly BoxBoundsHandle s_Plane = new BoxBoundsHandle
        {
            handleColor = Color.clear,
            axes = PrimitiveBoundsHandle.Axes.X | PrimitiveBoundsHandle.Axes.Z
        };

        static readonly Color k_ShapeHandleColor = new Color32(145, 244, 139, 210);
        static readonly Color k_ShapeHandleColorDisabled = new Color32(84, 200, 77, 140);

        void OnSceneGUI()
        {
            if (Event.current.type != EventType.Repaint)
                return;

            var shape = target as PhysicsShape;

            var handleColor = shape.enabled ? k_ShapeHandleColor : k_ShapeHandleColorDisabled;
            var handleMatrix = shape.GetShapeToWorldMatrix();
            using (new Handles.DrawingScope(handleColor, handleMatrix))
            {
                switch (shape.ShapeType)
                {
                    case ShapeType.Box:
                        shape.GetBakedBoxProperties(
                            out var center, out var size, out var orientation, out var convexRadius
                        );
                        s_Box.ConvexRadius = convexRadius;
                        s_Box.center = float3.zero;
                        s_Box.size = size;
                        using (new Handles.DrawingScope(math.mul(Handles.matrix, float4x4.TRS(center, orientation, 1f))))
                            s_Box.DrawHandle();
                        break;
                    case ShapeType.Capsule:
                        s_Capsule.center = float3.zero;
                        s_Capsule.height = s_Capsule.radius = 0f;
                        shape.GetBakedCapsuleProperties(
                            out center, out var height, out var radius, out orientation, out var v0, out var v1
                        );
                        s_Capsule.height = height;
                        s_Capsule.radius = radius;
                        using (new Handles.DrawingScope(math.mul(Handles.matrix, float4x4.TRS(center, orientation, 1f))))
                            s_Capsule.DrawHandle();
                        break;
                    case ShapeType.Sphere:
                        shape.GetBakedSphereProperties(out center, out radius, out orientation);
                        s_Sphere.center = float3.zero;
                        s_Sphere.radius = radius;
                        using (new Handles.DrawingScope(math.mul(Handles.matrix, float4x4.TRS(center, orientation, 1f))))
                            s_Sphere.DrawHandle();
                        break;
                    case ShapeType.Cylinder:
                        shape.GetBakedCylinderProperties(out center, out height, out radius, out orientation, out convexRadius);
                        s_ConvexCylinder.center = float3.zero;
                        s_ConvexCylinder.Height = height;
                        s_ConvexCylinder.Radius = radius;
                        s_ConvexCylinder.ConvexRadius = convexRadius;
                        using (new Handles.DrawingScope(math.mul(Handles.matrix, float4x4.TRS(center, orientation, 1f))))
                            s_ConvexCylinder.DrawHandle();
                        break;
                    case ShapeType.Plane:
                        shape.GetPlaneProperties(out center, out var size2, out orientation);
                        s_Plane.center = float3.zero;
                        s_Plane.size = new float3(size2.x, 0f, size2.y);
                        var m = math.mul(shape.transform.localToWorldMatrix, float4x4.TRS(center, orientation, 1f));
                        using (new Handles.DrawingScope(m))
                            s_Plane.DrawHandle();
                        var right = math.mul(m, new float4 { x = 1f }).xyz;
                        var forward = math.mul(m, new float4 { z = 1f }).xyz;
                        var normal = math.cross(math.normalizesafe(forward), math.normalizesafe(right))
                            * 0.5f * math.lerp(math.length(right) * size2.x, math.length(forward) * size2.y, 0.5f);
                        using (new Handles.DrawingScope(float4x4.identity))
                            Handles.DrawLine(m.c3.xyz, m.c3.xyz + normal);
                        break;
                    case ShapeType.ConvexHull:
                    case ShapeType.Mesh:
                        if (Event.current.type != EventType.Repaint)
                            break;
                        var mesh = GetPreviewMesh(shape);
                        if (mesh == null || mesh.vertexCount == 0)
                            break;
                        DrawMesh(mesh, shape.transform.localToWorldMatrix);
                        break;
                    default:
                        throw new UnimplementedShapeException(shape.ShapeType);
                }
            }
        }

        void DrawMesh(UnityEngine.Mesh mesh, float4x4 localToWorld)
        {
            if (Event.current.type != EventType.Repaint)
                return;
            var wireFrame = GL.wireframe;
            GL.wireframe = true;
            var lighting = Handles.lighting;
            Handles.lighting = false;
            Shader.SetGlobalColor("_HandleColor", Handles.color);
            Shader.SetGlobalFloat("_HandleSize", 1f);
            Shader.SetGlobalMatrix("_ObjectToWorld", localToWorld);
            m_PreviewMeshMaterial.SetInt("_HandleZTest", (int)Handles.zTest);
            m_PreviewMeshMaterial.SetPass(0);
            Graphics.DrawMeshNow(mesh, localToWorld);
            Handles.lighting = lighting;
            GL.wireframe = wireFrame;
        }
    }
}
