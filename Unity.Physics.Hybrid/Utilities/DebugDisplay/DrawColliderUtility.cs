using System.Collections.Generic;
using System.IO;
using Unity.Burst;
using Unity.Mathematics;
using UnityEngine;
using Unity.Collections;
using Unity.DebugDisplay;
using UnityEditor;

namespace Unity.Physics.Authoring
{
    internal static class DrawMeshUtility
    {
        internal static List<Matrix4x4> dynamicSpheres;
        internal static List<Matrix4x4> dynamicCapsules;
        internal static List<Matrix4x4> dynamicCylinders;
        internal static List<Matrix4x4> dynamicCubes;

        internal static List<Matrix4x4> staticSpheres;
        internal static List<Matrix4x4> staticCapsules;
        internal static List<Matrix4x4> staticCylinders;
        internal static List<Matrix4x4> staticCubes;

#if UNITY_EDITOR
        private static readonly UnityEngine.Material meshDynamicFacesMaterial =
            AssetDatabase.LoadAssetAtPath<UnityEngine.Material>(Path.Combine(Managed.debugDirName, "PhysicsDynamicDebugMaterial.mat"));

        private static readonly UnityEngine.Material meshStaticFacesMaterial =
            AssetDatabase.LoadAssetAtPath<UnityEngine.Material>(Path.Combine(Managed.debugDirName, "PhysicsStaticDebugMaterial.mat"));
#endif
        internal static void ClearTRS()
        {
            if (dynamicSpheres != null)
            {
                dynamicSpheres.Clear();
            }
            else
            {
                dynamicSpheres = new List<Matrix4x4>();
            }

            if (staticSpheres != null)
            {
                staticSpheres.Clear();
            }
            else
            {
                staticSpheres = new List<Matrix4x4>();
            }

            if (dynamicCubes != null)
            {
                dynamicCubes.Clear();
            }
            else
            {
                dynamicCubes = new List<Matrix4x4>();
            }

            if (staticCubes != null)
            {
                staticCubes.Clear();
            }
            else
            {
                staticCubes = new List<Matrix4x4>();
            }

            if (dynamicCylinders != null)
            {
                dynamicCylinders.Clear();
            }
            else
            {
                dynamicCylinders = new List<Matrix4x4>();
            }

            if (staticCylinders != null)
            {
                staticCylinders.Clear();
            }
            else
            {
                staticCylinders = new List<Matrix4x4>();
            }

            if (dynamicCapsules != null)
            {
                dynamicCapsules.Clear();
            }
            else
            {
                dynamicCapsules = new List<Matrix4x4>();
            }

            if (staticCapsules != null)
            {
                staticCapsules.Clear();
            }
            else
            {
                staticCapsules = new List<Matrix4x4>();
            }
        }

        // Takes the output from the primitive NativeList in the DisplayCollidersSystem and puts it into a C# List.
        // These Lists are used during the PhysicsDebugDisplaySystem.OnUpdate to render the collider faces
        internal static void SaveTRSList(NativeList<DisplayBodyColliders.PrimitiveInfo> primitives)
        {
            if (dynamicSpheres == null || dynamicCapsules == null || dynamicCylinders == null
                || dynamicCubes == null || staticSpheres == null || staticCapsules == null || staticCylinders == null || staticCubes == null)
            {
                return;
            }

            for (int i = 0; i < primitives.Length; i++)
            {
                DisplayBodyColliders.PrimitiveInfo primitive = primitives[i];
                Matrix4x4 trs = primitive.Trs;

                var isDynamic = (primitive.Flags & DisplayBodyColliders.PrimitiveInfo.PrimitiveFlags.Dynamic) != 0;
                if (trs.ValidTRS())
                {
                    if ((primitive.Flags & DisplayBodyColliders.PrimitiveInfo.PrimitiveFlags.Box) != 0)
                    {
                        if (isDynamic)
                        {
                            dynamicCubes.Add(trs);
                        }
                        else
                        {
                            staticCubes.Add(trs);
                        }
                    }
                    else if ((primitive.Flags & DisplayBodyColliders.PrimitiveInfo.PrimitiveFlags.Sphere) != 0)
                    {
                        if (isDynamic)
                        {
                            dynamicSpheres.Add(trs);
                        }
                        else
                        {
                            staticSpheres.Add(trs);
                        }
                    }
                    else if ((primitive.Flags & DisplayBodyColliders.PrimitiveInfo.PrimitiveFlags.Capsule) != 0)
                    {
                        if (isDynamic)
                        {
                            dynamicCapsules.Add(trs);
                        }
                        else
                        {
                            staticCapsules.Add(trs);
                        }
                    }
                    else if ((primitive.Flags & DisplayBodyColliders.PrimitiveInfo.PrimitiveFlags.Cylinder) != 0)
                    {
                        if (isDynamic)
                        {
                            dynamicCylinders.Add(trs);
                        }
                        else
                        {
                            staticCylinders.Add(trs);
                        }
                    }
                }
            }
        }

        internal static void DrawPrimitiveMeshes()
        {
            if (dynamicSpheres != null && dynamicSpheres.Count > 0) DrawBatches(PrimitiveType.Sphere, dynamicSpheres, true);
            if (dynamicCapsules != null && dynamicCapsules.Count > 0) DrawBatches(PrimitiveType.Capsule, dynamicCapsules, true);
            if (dynamicCylinders != null &&  dynamicCylinders.Count > 0) DrawBatches(PrimitiveType.Cylinder, dynamicCylinders, true);
            if (dynamicCubes != null && dynamicCubes.Count > 0) DrawBatches(PrimitiveType.Cube, dynamicCubes, true);

            if (staticSpheres != null && staticSpheres.Count > 0) DrawBatches(PrimitiveType.Sphere, staticSpheres, false);
            if (staticCapsules != null && staticCapsules.Count > 0) DrawBatches(PrimitiveType.Capsule, staticCapsules, false);
            if (staticCylinders != null &&  staticCylinders.Count > 0) DrawBatches(PrimitiveType.Cylinder, staticCylinders, false);
            if (staticCubes != null && staticCubes.Count > 0) DrawBatches(PrimitiveType.Cube, staticCubes, false);
        }

        // Use detailed meshes for the faces rendering
        private static void DrawBatches(PrimitiveType type, List<Matrix4x4> trsList, bool isDynamic)
        {
#if UNITY_EDITOR
            UnityEngine.Mesh primitiveMesh = DebugMeshCache.GetMesh(type);

            var debugMaterial = meshDynamicFacesMaterial;
            if (!isDynamic) debugMaterial = meshStaticFacesMaterial;

            const int maxBatchSize = 1023; //Batch size limit of DrawMeshInstanced
            var index = 0;
            var remaining = trsList.Count;

            while (remaining > 0)
            {
                var drawCount = math.min(remaining, maxBatchSize);
                Graphics.DrawMeshInstanced(primitiveMesh, 0, debugMaterial, trsList.GetRange(index, drawCount));
                index += maxBatchSize;
                remaining -= maxBatchSize;
            }
#endif
        }
    }

    static class DrawColliderUtility
    {
        private static readonly DebugDisplay.ColorIndex DebugDynamicColor = DebugDisplay.ColorIndex.OrangeRed;
        private static readonly DebugDisplay.ColorIndex DebugStaticColor = DebugDisplay.ColorIndex.Grey12;

        public static DebugDisplay.ColorIndex GetColorIndex(bool isDynamic)
        {
            if (isDynamic)
            {
                return DebugDynamicColor;
            }
            else //is static
            {
                return DebugStaticColor;
            }
        }

        [BurstDiscard]
        public static void DrawPrimitiveSphereEdges(float radius, float3 center, RigidTransform wfc)
        {
            var edgesColor = DebugDisplay.ColorIndex.Green;
            var shapeScale = new float3(1.0f, 1.0f, 1.0f);
            shapeScale = radius * shapeScale;

            var sphereVertexList = DisplayBodyColliderEdges.SphereVertexList;
            var sphereList = DisplayBodyColliderEdges.SphereList;

            var i = 0;
            while (i < sphereList.Count)
            {
                var a = math.transform(wfc, center + shapeScale * sphereVertexList[sphereList[i + 0]]);
                var b = math.transform(wfc, center + shapeScale * sphereVertexList[sphereList[i + 1]]);
                var c = math.transform(wfc, center + shapeScale * sphereVertexList[sphereList[i + 2]]);
                i += 3;

                PhysicsDebugDisplaySystem.Line(a, b, edgesColor);
                PhysicsDebugDisplaySystem.Line(b, c, edgesColor);
                PhysicsDebugDisplaySystem.Line(c, a, edgesColor);
            }
        }

        [BurstDiscard]
        public static void DrawPrimitiveCapsuleEdges(float radius, float height, float3 center, RigidTransform wfc)
        {
            var edgesColor = DebugDisplay.ColorIndex.Green;
            var shapeScale = new float3(2.0f * radius, height, 2.0f * radius);

            var capsuleEdgeList = DisplayBodyColliderEdges.CapsuleEdgeList;

            var i = 0;
            while (i < capsuleEdgeList.Count)
            {
                var a = center + shapeScale * capsuleEdgeList[i];
                var b = center + shapeScale * capsuleEdgeList[i + 1];
                i += 2;

                PhysicsDebugDisplaySystem.Line(math.transform(wfc, a), math.transform(wfc, b), edgesColor);
            }
        }

        private static void SetDiscSectionPoints(NativeArray<float3> dest, int count, float3 normalAxis, float3 from, float angle, float radius)
        {
            float3 startingPoint = math.normalize(from) * radius;
            var step = angle * math.PI / 180f;
            var r = quaternion.AxisAngle(normalAxis, step / (count - 1));
            float3 tangent = startingPoint;
            for (int i = 0; i < count; i++)
            {
                dest[i] = tangent;
                tangent = math.rotate(r, tangent);
            }
        }

        private static void GetWireArcSegments(ref NativeArray<float3> segmentArray, int segmentIndex, float3 center,
            float3 normal, float3 from, float angle, float radius)
        {
            const int kMaxArcPoints = 15;
            NativeArray<float3> sPoints = new NativeArray<float3>(kMaxArcPoints, Allocator.Temp);

            SetDiscSectionPoints(sPoints, kMaxArcPoints, normal, from, angle, radius); //scaled by radius

            for (var i = 0; i < kMaxArcPoints; ++i)
                sPoints[i] = center + sPoints[i];

            var j = segmentIndex;
            for (var i = 0; i < kMaxArcPoints - 1; ++i)
            {
                segmentArray[j++] = sPoints[i];
                segmentArray[j++] = sPoints[i + 1];
            }
            sPoints.Dispose();
        }

        private static readonly float3[] s_HeightAxes = new float3[3]
        {
            Vector3.right,
            Vector3.up,
            Vector3.forward
        };

        private static readonly int[] s_NextAxis = new int[3] {1, 2, 0};

        // Create a wireframe capsule with a default orientation along the y-axis. Use the general method of DrawWireArc
        // declared in GizmoUtil.cpp and with CapsuleBoundsHandle.DrawWireframe(). Output is an array that comprises
        // pairs of vertices to be used in the drawing of Capsule Collider Edges.
        public static NativeArray<float3> DrawCapsuleWireFrame()
        {
            const float radius = 0.5f;
            const float height = 2.0f;
            const int mHeightAxis = 1; //corresponds to the y-axis
            var center = new float3(0, 0, 0);

            var heightAx1 = s_HeightAxes[mHeightAxis];
            var heightAx2 = s_HeightAxes[s_NextAxis[mHeightAxis]];
            var heightAx3 = s_HeightAxes[s_NextAxis[s_NextAxis[mHeightAxis]]];

            var center1 = center + heightAx1 * (height * 0.5f - radius);
            var center2 = center - heightAx1 * (height * 0.5f - radius);

            // 15 segments * 2 float3s * 4 arcs + 4 lines * 2 float3s = 128 (+2 circles = 188)
            NativeArray<float3> capsuleEdges = new NativeArray<float3>(128, Allocator.Temp); //or 188 if want extra circles

            GetWireArcSegments(ref capsuleEdges, 0, center1, heightAx2, heightAx3, 180f, radius);
            GetWireArcSegments(ref capsuleEdges, 30, center2, heightAx2, heightAx3, -180f, radius);

            GetWireArcSegments(ref capsuleEdges, 60, center1, heightAx3, heightAx2, -180f, radius);
            GetWireArcSegments(ref capsuleEdges, 90, center2, heightAx3, heightAx2, 180f, radius);

            // Lines to connect the two hemispheres:
            capsuleEdges[120] = center1 + heightAx3 * radius;
            capsuleEdges[121] = center2 + heightAx3 * radius;

            capsuleEdges[122] = center1 - heightAx3 * radius;
            capsuleEdges[123] = center2 - heightAx3 * radius;


            capsuleEdges[124] = center1 + heightAx2 * radius;
            capsuleEdges[125] = center2 + heightAx2 * radius;

            capsuleEdges[126] = center1 - heightAx2 * radius;
            capsuleEdges[127] = center2 - heightAx2 * radius;

            // If want to add the [xz] circles along the y-axis edges (adds 30 lines per use)
            //GetWireArcSegments(ref capsuleEdges, 128, center1, heightAx1, heightAx2, -360f, radius);
            //GetWireArcSegments(ref capsuleEdges, 158, center2, heightAx1, heightAx2, 360f, radius);

            return capsuleEdges;
        }

        //wfc = worldFromCollider
        //Note: assuming that reading in vertices correctly and that vertex winding is correct
        public static NativeArray<float3> TransformAndCalculateNormal(float3 v0, float3 v1, float3 v2, RigidTransform wfc)
        {
            var verts = new NativeArray<float3>(4, Allocator.Temp);

            verts[0] = math.transform(wfc, v0);
            verts[1] = math.transform(wfc, v1);
            verts[2] = math.transform(wfc, v2);

            //Calculate normal to two vectors. Already in wfc-space
            verts[3] = math.normalize((math.cross(verts[1] - verts[0], verts[2] - verts[0])));

            return verts;
        }

        public static void DrawTriangle(float3 v0, float3 v1, float3 v2, RigidTransform worldFromCollider,
            DebugDisplay.ColorIndex colourIndex)
        {
            var v = TransformAndCalculateNormal(v0, v1, v2, worldFromCollider);
            PhysicsDebugDisplaySystem.Triangle(v[0], v[1], v[2], v[3], colourIndex);
        }
    }
}
