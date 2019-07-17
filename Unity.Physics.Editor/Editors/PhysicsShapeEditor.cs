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
            static readonly string k_Plural =
                $"One or more selected {ObjectNames.NicifyVariableName(typeof(PhysicsShape).Name)}s";
            static readonly string k_Singular =
                $"This {ObjectNames.NicifyVariableName(typeof(PhysicsShape).Name)}";

            public static readonly string GenericUndoMessage = L10n.Tr("Change Shape");
            public static readonly string MultipleShapeTypesLabel =
                L10n.Tr("Multiple shape types in current selection.");

            static readonly GUIContent k_FitToRenderMeshesLabel =
                EditorGUIUtility.TrTextContent("Fit to Enabled Render Meshes");
            static readonly GUIContent k_FitToRenderMeshesWarningLabelSg = new GUIContent(
                k_FitToRenderMeshesLabel.text,
                EditorGUIUtility.Load("console.warnicon") as Texture,
                L10n.Tr($"{k_Singular} has non-uniform scale. Trying to fit the shape to render meshes might produce unexpected results.")
            );
            static readonly GUIContent k_FitToRenderMeshesWarningLabelPl = new GUIContent(
                k_FitToRenderMeshesLabel.text,
                EditorGUIUtility.Load("console.warnicon") as Texture,
                L10n.Tr($"{k_Plural} has non-uniform scale. Trying to fit the shape to render meshes might produce unexpected results.")
            );
            public static readonly GUIContent CenterLabel = EditorGUIUtility.TrTextContent("Center");
            public static readonly GUIContent SizeLabel = EditorGUIUtility.TrTextContent("Size");
            public static readonly GUIContent OrientationLabel = EditorGUIUtility.TrTextContent(
                "Orientation", "Euler orientation in the shape's local space (ZXY order)."
            );
            public static readonly GUIContent RadiusLabel = EditorGUIUtility.TrTextContent("Radius");
            public static readonly GUIContent MaterialLabel = EditorGUIUtility.TrTextContent("Material");

            public static GUIContent GetFitToRenderMeshesLabel(int numTargets, MessageType status) =>
                status >= MessageType.Warning
                    ? numTargets == 1 ? k_FitToRenderMeshesWarningLabelSg : k_FitToRenderMeshesWarningLabelPl
                    : k_FitToRenderMeshesLabel;

            static readonly string[] k_NoGeometryWarning =
            {
                L10n.Tr($"{k_Singular} has no enabled render meshes in its hierarchy and no custom mesh assigned."),
                L10n.Tr($"{k_Plural} has no enabled render meshes in their hierarchies and no custom mesh assigned.")
            };

            public static string GetNoGeometryWarning(int numTargets) =>
                numTargets == 1 ? k_NoGeometryWarning[0] : k_NoGeometryWarning[1];

            static readonly string[] k_NonReadableGeometryWarning =
            {
                L10n.Tr($"{k_Singular} has a non-readable mesh, but is not part of a sub-scene. Assign a custom mesh with Read/Write enabled in its import settings if it needs to be converted at run-time."),
                L10n.Tr($"{k_Plural} has a non-readable mesh, but is not part of a sub-scene. Assign a custom mesh with Read/Write enabled in its import settings if it needs to be converted at run-time.")
            };

            public static string GetNonReadableGeometryWarning(int numTargets) =>
                numTargets == 1 ? k_NonReadableGeometryWarning[0] : k_NonReadableGeometryWarning[1];

            static readonly string[] k_StaticColliderStatusMessage =
            {
                L10n.Tr($"{k_Singular} will be considered static. Add a {ObjectNames.NicifyVariableName(typeof(PhysicsBody).Name)} component if you will move it at run-time."),
                L10n.Tr($"{k_Plural} will be considered static. Add a {ObjectNames.NicifyVariableName(typeof(PhysicsBody).Name)} component if you will move them at run-time.")
            };

            public static string GetStaticColliderStatusMessage(int numTargets) =>
                numTargets == 1 ? k_StaticColliderStatusMessage[0] : k_StaticColliderStatusMessage[1];

            public static readonly string BoxCapsuleSuggestion =
                L10n.Tr($"Target {ShapeType.Box} has uniform size on its two short axes and a large convex radius. Consider using a {ShapeType.Capsule} instead.");
            public static readonly string BoxPlaneSuggestion =
                L10n.Tr($"Target {ShapeType.Box} is flat. Consider using a {ShapeType.Plane} instead.");
            public static readonly string BoxSphereSuggestion =
                L10n.Tr($"Target {ShapeType.Box} has uniform size and large convex radius. Consider using a {ShapeType.Sphere} instead.");
            public static readonly string CapsuleSphereSuggestion =
                L10n.Tr($"Target {ShapeType.Capsule}'s diameter is equal to its height. Consider using a {ShapeType.Sphere} instead.");
            public static readonly string CylinderCapsuleSuggestion =
                L10n.Tr($"Target {ShapeType.Cylinder} has a large convex radius. Consider using a {ShapeType.Capsule} instead.");
            public static readonly string CylinderSphereSuggestion =
                L10n.Tr($"Target {ShapeType.Cylinder} has a large convex radius and its diameter is equal to its height. Consider using a {ShapeType.Sphere} instead.");

            public static readonly GUIStyle Button =
                new GUIStyle(EditorStyles.miniButton) { padding = new RectOffset() };
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

        [Flags]
        enum GeometryState
        {
            Okay = 0,
            NoGeometry = 1,
            NonReadableGeometry = 2
        }

        GeometryState m_GeometryState;
        int m_NumImplicitStatic;

        protected override void OnEnable()
        {
            base.OnEnable();

            CheckGeometry();

            m_NumImplicitStatic = targets.Cast<PhysicsShape>().Count(
                shape => shape.GetPrimaryBody() == shape.gameObject
                    && shape.GetComponent<PhysicsBody>() == null
                    && shape.GetComponent<Rigidbody>() == null
            );

            var wireframeShader = Shader.Find("Hidden/Physics/ShapeHandle");
            m_PreviewMeshMaterial = new UnityEngine.Material(wireframeShader) { hideFlags = HideFlags.HideAndDontSave };

            Undo.postprocessModifications += CheckForMeshReassignment;
            Undo.undoRedoPerformed += CheckGeometry;
            Undo.undoRedoPerformed += Repaint;
        }

        void CheckGeometry()
        {
            m_GeometryState = GeometryState.Okay;
            var pointCloud = new NativeList<float3>(65535, Allocator.Temp);
            foreach (PhysicsShape shape in targets)
            {
                shape.GetConvexHullProperties(pointCloud, false);

                if (pointCloud.Length == 0)
                {
                    m_GeometryState |= GeometryState.NoGeometry;
                    continue;
                }

                var mesh = shape.GetMesh();
                if (mesh != null && !mesh.IsValidForConversion(shape.gameObject))
                    m_GeometryState |= GeometryState.NonReadableGeometry;
            }
            pointCloud.Dispose();
        }

        UndoPropertyModification[] CheckForMeshReassignment(UndoPropertyModification[] modifications)
        {
            foreach (var modification in modifications)
            {
                var targetType = modification.currentValue.target.GetType();
                if (
                    targetType == typeof(MeshFilter)
                    || targetType == typeof(MeshRenderer)
                    || targetType == typeof(SkinnedMeshRenderer)
                )
                {
                    CheckGeometry();
                    Repaint();
                    break;
                }
            }
            return modifications;
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

            Undo.postprocessModifications -= CheckForMeshReassignment;
            Undo.undoRedoPerformed -= CheckGeometry;
            Undo.undoRedoPerformed -= Repaint;
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
                    if (preview.SourceMesh != null && preview.SourceMesh.IsValidForConversion(shape.gameObject))
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
                EditorGUILayout.HelpBox(string.Join("\n\n", m_StatusMessages), m_Status);

            if (EditorGUI.EndChangeCheck())
                serializedObject.ApplyModifiedProperties();
        }

        MessageType m_GeometryStatus;
        List<string> m_GeometryStatusMessages = new List<string>();
        HashSet<string> m_ShapeSuggestions = new HashSet<string>();
        MessageType m_Status;
        List<string> m_StatusMessages = new List<string>(8);
        MessageType m_MatrixStatus;
        List<MatrixState> m_MatrixStates = new List<MatrixState>();

        void UpdateStatusMessages()
        {
            m_Status = MessageType.None;
            m_StatusMessages.Clear();

            if (m_NumImplicitStatic != 0)
                m_StatusMessages.Add(Styles.GetStaticColliderStatusMessage(targets.Length));

            m_ShapeSuggestions.Clear();
            foreach (PhysicsShape shape in targets)
            {
                switch (shape.ShapeType)
                {
                    case ShapeType.Box:
                        shape.GetBakedBoxProperties(out var c, out var s, out var o, out var cr);
                        var max = math.cmax(s);
                        var min = math.cmin(s);
                        if (min == 0f)
                            m_ShapeSuggestions.Add(Styles.BoxPlaneSuggestion);
                        else if (cr == min * 0.5f)
                        {
                            if (min == max)
                                m_ShapeSuggestions.Add(Styles.BoxSphereSuggestion);
                            else if (math.lengthsq(s - new float3(min)) == math.pow(max - min, 2f))
                                m_ShapeSuggestions.Add(Styles.BoxCapsuleSuggestion);
                        }
                        break;
                    case ShapeType.Capsule:
                        shape.GetBakedCapsuleProperties(out c, out var h, out var r, out o, out var v0, out var v1);
                        if (h == 2f * r)
                            m_ShapeSuggestions.Add(Styles.CapsuleSphereSuggestion);
                        break;
                    case ShapeType.Cylinder:
                        shape.GetBakedCylinderProperties(out c, out h, out r, out o, out cr);
                        if (cr == r)
                            m_ShapeSuggestions.Add(h == 2f * r ? Styles.CylinderSphereSuggestion : Styles.CylinderCapsuleSuggestion);
                        break;
                }
            }
            foreach (var suggestion in m_ShapeSuggestions)
                m_StatusMessages.Add(suggestion);

            var hierarchyStatus = StatusMessageUtility.GetHierarchyStatusMessage(targets, out var hierarchyStatusMessage);
            if (!string.IsNullOrEmpty(hierarchyStatusMessage))
            {
                m_StatusMessages.Add(hierarchyStatusMessage);
                m_Status = (MessageType)math.max((int)m_Status, (int)hierarchyStatus);
            }

            m_MatrixStates.Clear();
            foreach (var t in targets)
            {
                var localToWorld = (float4x4)(t as Component).transform.localToWorldMatrix;
                m_MatrixStates.Add(ManipulatorUtility.GetMatrixState(ref localToWorld));
            }

            m_MatrixStatus = StatusMessageUtility.GetMatrixStatusMessage(m_MatrixStates, out var matrixStatusMessage);
            if (m_MatrixStatus != MessageType.None)
            {
                m_StatusMessages.Add(matrixStatusMessage);
                m_Status = (MessageType)math.max((int)m_Status, (int)m_MatrixStatus);
            }

            m_GeometryStatus = MessageType.None;
            m_GeometryStatusMessages.Clear();
            if ((m_GeometryState & GeometryState.NoGeometry) == GeometryState.NoGeometry)
            {
                m_GeometryStatusMessages.Add(Styles.GetNoGeometryWarning(targets.Length));
                m_GeometryStatus = (MessageType)math.max((int)m_GeometryStatus, (int)MessageType.Error);
            }
            if ((m_GeometryState & GeometryState.NonReadableGeometry) == GeometryState.NonReadableGeometry)
            {
                m_GeometryStatusMessages.Add(Styles.GetNonReadableGeometryWarning(targets.Length));
                m_GeometryStatus = (MessageType)math.max((int)m_GeometryStatus, (int)MessageType.Warning);
            }
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
            EditorGUI.BeginDisabledGroup(
                (m_GeometryState & GeometryState.NoGeometry) == GeometryState.NoGeometry || EditorUtility.IsPersistent(target)
            );
            var rect = EditorGUI.IndentedRect(
                EditorGUILayout.GetControlRect(false, EditorGUIUtility.singleLineHeight, EditorStyles.miniButton)
            );
            var buttonLabel = Styles.GetFitToRenderMeshesLabel(targets.Length, m_MatrixStatus);
            if (GUI.Button(rect, buttonLabel, Styles.Button))
            {
                Undo.RecordObjects(targets, buttonLabel.text);
                foreach (PhysicsShape shape in targets)
                {
                    shape.FitToEnabledRenderMeshes();
                    EditorUtility.SetDirty(shape);
                }
            }
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
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(m_CustomMesh);
            if (EditorGUI.EndChangeCheck())
            {
                serializedObject.ApplyModifiedProperties();
                serializedObject.Update();
                CheckGeometry();
            }

            if (m_GeometryStatusMessages.Count > 0)
                EditorGUILayout.HelpBox(string.Join("\n\n", m_GeometryStatusMessages), m_GeometryStatus);
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
