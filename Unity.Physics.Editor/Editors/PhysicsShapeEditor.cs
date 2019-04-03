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
            public static readonly string StaticColliderStatusMessage = L10n.Tr(
                $"This {ObjectNames.NicifyVariableName(typeof(PhysicsShape).Name)} will be considered static. "
                + $"Add a {ObjectNames.NicifyVariableName(typeof(PhysicsBody).Name)} component if you will move it at run-time."
            );
            public static readonly string StaticCollidersStatusMessage = L10n.Tr(
                $"One or more selected {ObjectNames.NicifyVariableName(typeof(PhysicsShape).Name)}s will be considered static. "
                + $"Add a {ObjectNames.NicifyVariableName(typeof(PhysicsBody).Name)} component if you will move them at run-time."
            );

            public static readonly GUIContent FitToRenderMeshesLabel =
                EditorGUIUtility.TrTextContent("Fit to Render Meshes");
            public static readonly GUIContent CenterLabel = EditorGUIUtility.TrTextContent("Center");
            public static readonly GUIContent SizeLabel = EditorGUIUtility.TrTextContent("Size");
            public static readonly GUIContent OrientationLabel = EditorGUIUtility.TrTextContent(
                "Orientation", "Euler orientation in the shape's local space (ZXY order)."
            );
            public static readonly GUIContent RadiusLabel = EditorGUIUtility.TrTextContent("Radius");
            public static readonly GUIContent MaterialLabel = EditorGUIUtility.TrTextContent("Material");
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
        bool m_AtLeastOneStatic;

        protected override void OnEnable()
        {
            base.OnEnable();

            var pointCloud = new NativeList<float3>(65535, Allocator.Temp);
            (target as PhysicsShape).GetConvexHullProperties(pointCloud);
            m_HasGeometry = pointCloud.Length > 0;
            pointCloud.Dispose();

            m_AtLeastOneStatic = targets.Cast<PhysicsShape>()
                .Select(shape => shape.GetComponentInParent<PhysicsBody>())
                .Any(rb => rb == null);

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

        Dictionary<PhysicsShape, PreviewMeshData> m_PreviewMeshes = new Dictionary<PhysicsShape,PreviewMeshData>();
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
                    case ShapeType.Mesh:
                        DisplayMeshControls();
                        break;
                }
            }

            --EditorGUI.indentLevel;

            EditorGUILayout.LabelField(Styles.MaterialLabel);

            ++EditorGUI.indentLevel;

            EditorGUILayout.PropertyField(m_Material);

            --EditorGUI.indentLevel;

            DisplayStatusMessages();

            if (EditorGUI.EndChangeCheck())
                serializedObject.ApplyModifiedProperties();
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
                    default:
                        return;
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
                    shape.FitToGeometry();
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
            // TODO: warn if no render meshes and no custom mesh
            EditorGUILayout.PropertyField(m_CustomMesh);
        }

        void DisplayStatusMessages()
        {
            if (!m_AtLeastOneStatic)
                return;
            EditorGUILayout.HelpBox(
                targets.Length == 1 ? Styles.StaticColliderStatusMessage : Styles.StaticCollidersStatusMessage,
                MessageType.None
            );
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

            shape.GetBakeTransformation(out var linearScalar, out var radiusScalar);

            var handleColor = shape.enabled ? k_ShapeHandleColor : k_ShapeHandleColorDisabled;
            var handleMatrix = new float4x4(new RigidTransform(shape.transform.rotation, shape.transform.position));
            using (new Handles.DrawingScope(handleColor, handleMatrix))
            {
                switch (shape.ShapeType)
                {
                    case ShapeType.Box:
                        shape.GetBoxProperties(out var center, out var size, out EulerAngles orientation);
                        s_Box.ConvexRadius = shape.ConvexRadius * radiusScalar;
                        s_Box.center = float3.zero;
                        s_Box.size = size * linearScalar;
                        using (new Handles.DrawingScope(math.mul(Handles.matrix, float4x4.TRS(center * linearScalar, orientation, 1f))))
                            s_Box.DrawHandle();
                        break;
                    case ShapeType.Capsule:
                        s_Capsule.center = float3.zero;
                        s_Capsule.height = s_Capsule.radius = 0f;
                        shape.GetCapsuleProperties(out center, out var height, out var radius, out orientation);
                        shape.GetCapsuleProperties(out var v0, out var v1, out radius);
                        var ax = (v0 - v1) * linearScalar;
                        s_Capsule.height = math.length(ax) + radius * radiusScalar * 2f;
                        s_Capsule.radius = radius * radiusScalar;
                        ax = math.normalizesafe(ax, new float3(0f, 0f, 1f));
                        var up = math.mul(orientation, math.up());
                        var m = float4x4.TRS(center * linearScalar, quaternion.LookRotationSafe(ax, up), 1f);
                        using (new Handles.DrawingScope(math.mul(Handles.matrix, m)))
                            s_Capsule.DrawHandle();
                        break;
                    case ShapeType.Sphere:
                        shape.GetSphereProperties(out center, out radius, out orientation);
                        s_Sphere.center = float3.zero;
                        s_Sphere.radius = radius;
                        using (new Handles.DrawingScope(math.mul(Handles.matrix, float4x4.TRS(center * linearScalar, orientation, radiusScalar))))
                            s_Sphere.DrawHandle();
                        break;
                    case ShapeType.Cylinder:
                        shape.GetCylinderProperties(out center, out height, out radius, out orientation);
                        s_ConvexCylinder.ConvexRadius = shape.ConvexRadius * radiusScalar;
                        s_ConvexCylinder.center = float3.zero;
                        s_ConvexCylinder.Height = height;
                        s_ConvexCylinder.Radius = radius;
                        using (new Handles.DrawingScope(math.mul(Handles.matrix, float4x4.TRS(center * linearScalar, orientation, linearScalar))))
                            s_ConvexCylinder.DrawHandle();
                        break;
                    case ShapeType.Plane:
                        shape.GetPlaneProperties(out center, out var size2, out orientation);
                        s_Plane.center = float3.zero;
                        s_Plane.size = new float3(size2.x, 0, size2.y) * linearScalar;
                        using (new Handles.DrawingScope(math.mul(Handles.matrix, float4x4.TRS(center * linearScalar, orientation, 1f))))
                        {
                            Handles.DrawLine(
                                new float3(0f),
                                new float3(0f, math.lerp(math.cmax(size2), math.cmin(size2), 0.5f), 0) * 0.5f
                            );
                            s_Plane.DrawHandle();
                        }
                        break;
                    case ShapeType.ConvexHull:
                    case ShapeType.Mesh:
                        if (Event.current.type != EventType.Repaint)
                            break;
                        var mesh = GetPreviewMesh(shape);
                        if (mesh == null || mesh.vertexCount == 0)
                            break;
                        var localToWorld = new RigidTransform(shape.transform.rotation, shape.transform.position);
                        shape.GetBakeTransformation(out linearScalar, out radiusScalar);
                        DrawMesh(mesh, float4x4.TRS(localToWorld.pos, localToWorld.rot, linearScalar));
                        break;
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
