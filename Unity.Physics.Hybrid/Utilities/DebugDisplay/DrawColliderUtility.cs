using System;
using System.Collections.Generic;
using System.IO;
using Unity.Mathematics;
using UnityEngine;
using Unity.Collections;
using Unity.DebugDisplay;
using Unity.Entities;
using Unity.Transforms;
using UnityEditor;
using Matrix4x4 = UnityEngine.Matrix4x4;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

namespace Unity.Physics.Authoring
{
    internal static class DrawMeshUtility
    {
        static List<Matrix4x4> dynamicSpheres;
        static List<Matrix4x4> dynamicCapsules;
        static List<Matrix4x4> dynamicCylinders;
        static List<Matrix4x4> dynamicCubes;

        static List<Matrix4x4> staticSpheres;
        static List<Matrix4x4> staticCapsules;
        static List<Matrix4x4> staticCylinders;
        static List<Matrix4x4> staticCubes;

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

    internal readonly struct ColliderGeometry : IDisposable
    {
        internal readonly NativeArray<Vector3> VerticesArray;
        internal readonly NativeArray<int> IndicesArray;
        internal readonly NativeArray<Vector3> EdgesArray;

        internal ColliderGeometry(NativeArray<Vector3> vertices, NativeArray<int> indices, NativeArray<Vector3> edgesArray)
        {
            VerticesArray = vertices;
            IndicesArray = indices;
            EdgesArray = edgesArray;
        }

        public void Dispose()
        {
            VerticesArray.Dispose();
            IndicesArray.Dispose();
            EdgesArray.Dispose();
        }
    }

    internal struct PrimitiveColliderGeometries : IDisposable
    {
        internal ColliderGeometry CapsuleGeometry;
        internal ColliderGeometry BoxGeometry;
        internal ColliderGeometry CylinderGeometry;
        internal ColliderGeometry SphereGeometry;

        public void Dispose()
        {
            CapsuleGeometry.Dispose();
            BoxGeometry.Dispose();
            CylinderGeometry.Dispose();
            SphereGeometry.Dispose();
        }
    }

    static class DrawColliderUtility
    {
        private static readonly ColorIndex DebugDynamicColor = ColorIndex.DynamicMesh;
        private static readonly ColorIndex DebugStaticColor = ColorIndex.StaticMesh;
        private static readonly ColorIndex DebugKinematicColor = ColorIndex.KinematicMesh;

#if UNITY_EDITOR
        private static void CreateGeometryArray(MeshType meshType, out ColliderGeometry outGeometry)
        {
            var vertices = new List<Vector3>();
            var indices = new List<int>();
            DebugMeshCache.GetMesh(meshType).GetVertices(vertices);
            DebugMeshCache.GetMesh(meshType).GetIndices(indices, 0);

            // We want to simplify the capsule wireframe
            if (meshType == MeshType.Capsule)
            {
                outGeometry = new ColliderGeometry(
                    vertices.ToNativeArray(Allocator.Persistent),
                    indices.ToNativeArray(Allocator.Persistent),
                    DrawColliderUtility.CreateCapsuleWireFrame(Allocator.Persistent));
            }
            else
            {
                outGeometry = new ColliderGeometry(
                    vertices.ToNativeArray(Allocator.Persistent),
                    indices.ToNativeArray(Allocator.Persistent),
                    new NativeArray<Vector3>(0, Allocator.Persistent));
            }
        }

        public static void GetRigidBodiesFromQuery(ref SystemState state, ref EntityQuery query, ref NativeList<RigidBody> rigidBodiesList)
        {
            var entities = query.ToEntityArray(Allocator.Temp);
            foreach (var entity in entities)
            {
                var localToWorld = state.EntityManager.GetComponentData<LocalToWorld>(entity);
                var collider = state.EntityManager.GetComponentData<PhysicsCollider>(entity);
                var localTransform = state.EntityManager.GetComponentData<LocalTransform>(entity);
                CreateRigidBody(localToWorld.Value, localTransform.Scale,
                    collider.Value, out var rigidBody);
                rigidBodiesList.Add(rigidBody);
            }
        }

        public static void GetBodiesByMotionsFromQuery(ref SystemState state, ref EntityQuery rigidBodiesQuery, ref NativeList<RigidBody> rigidBodies,
            ref NativeList<BodyMotionType> bodyMotionTypes)
        {
            var dynamicEntities = rigidBodiesQuery.ToEntityArray(Allocator.Temp);
            foreach (var entity in dynamicEntities)
            {
                var localToWorld = state.EntityManager.GetComponentData<LocalToWorld>(entity);
                var collider = state.EntityManager.GetComponentData<PhysicsCollider>(entity);
                var localTransform = state.EntityManager.GetComponentData<LocalTransform>(entity);

                BodyMotionType motionType = BodyMotionType.Static;
                if (state.EntityManager.HasComponent<PhysicsMass>(entity))
                {
                    var physicsMass = state.EntityManager.GetComponentData<PhysicsMass>(entity);
                    motionType = physicsMass.IsKinematic ? BodyMotionType.Kinematic : BodyMotionType.Dynamic;
                }

                CreateRigidBody(localToWorld.Value, localTransform.Scale,
                    collider.Value, out var rigidBody);
                rigidBodies.Add(rigidBody);
                bodyMotionTypes.Add(motionType);
            }
        }

        public static void GetBodiesMotionTypesFromWorld(ref PhysicsWorld physicsWorld, ref NativeArray<BodyMotionType> bodyMotionTypes)
        {
            for (int index = 0; index < physicsWorld.NumBodies; ++index)
            {
                if (index < physicsWorld.NumDynamicBodies)
                {
                    bodyMotionTypes[index] = physicsWorld.MotionVelocities[index].IsKinematic
                        ? BodyMotionType.Kinematic
                        : BodyMotionType.Dynamic;
                }
                else
                {
                    bodyMotionTypes[index] = BodyMotionType.Static;
                }
            }
        }

        public static void CreateRigidBody(in float4x4 localToWorld, float scale, BlobAssetReference<Collider> collider, out RigidBody rigidBody)
        {
            var rigidTransform = Math.DecomposeRigidBodyTransform(localToWorld);
            rigidBody = new RigidBody
            {
                Collider = collider,
                WorldFromBody = rigidTransform,
                Scale = scale
            };
        }

        internal static void CreateGeometries(out PrimitiveColliderGeometries primitiveColliderGeometries)
        {
            CreateGeometryArray(MeshType.Capsule, out var capsuleGeometry);
            CreateGeometryArray(MeshType.Cube, out var boxGeometry);
            CreateGeometryArray(MeshType.Cylinder, out var cylinderGeometry);
            CreateGeometryArray(MeshType.Sphere, out var sphereGeometry);

            primitiveColliderGeometries = new PrimitiveColliderGeometries()
            {
                CapsuleGeometry = capsuleGeometry,
                BoxGeometry = boxGeometry,
                CylinderGeometry = cylinderGeometry,
                SphereGeometry = sphereGeometry
            };
        }

#endif

        public static ColorIndex GetColorIndex(BodyMotionType motionType)
        {
            if (motionType == BodyMotionType.Dynamic)
            {
                return DebugDynamicColor;
            }

            if (motionType == BodyMotionType.Kinematic)
            {
                return DebugKinematicColor;
            }

            return DebugStaticColor;
        }

        public static void DrawPrimitiveSphereEdges(float radius, float3 center, RigidTransform wfc, ref ColliderGeometry sphereGeometry, float uniformScale)
        {
            var edgesColor = DebugDisplay.ColorIndex.Green;
            var shapeScale = new float3(radius * 2.0f); // Radius to scale : Multiple by 2

            var sphereVerticesList = sphereGeometry.VerticesArray;
            var sphereIndicesList = sphereGeometry.IndicesArray;

            var i = 0;
            while (i < sphereIndicesList.Length)
            {
                var a = math.transform(wfc, center + uniformScale * shapeScale * sphereVerticesList[sphereIndicesList[i + 0]]);
                var b = math.transform(wfc, center + uniformScale * shapeScale * sphereVerticesList[sphereIndicesList[i + 1]]);
                var c = math.transform(wfc, center + uniformScale * shapeScale * sphereVerticesList[sphereIndicesList[i + 2]]);
                i += 3;

                BasePhysicsDebugDisplaySystem.Line(a, b, edgesColor);
                BasePhysicsDebugDisplaySystem.Line(b, c, edgesColor);
                BasePhysicsDebugDisplaySystem.Line(c, a, edgesColor);
            }
        }

        public static void DrawPrimitiveSphereFaces(float radius, float3 center, RigidTransform wfc, ref ColliderGeometry sphereGeometry, ColorIndex color, float uniformScale)
        {
            var shapeScale = new float3(radius * 2.0f) * uniformScale;

            var sphereVerticesList = sphereGeometry.VerticesArray;
            var sphereIndicesList = sphereGeometry.IndicesArray;

            var i = 0;
            while (i < sphereIndicesList.Length)
            {
                var a = math.transform(wfc, center + shapeScale * sphereVerticesList[sphereIndicesList[i + 0]]);
                var b = math.transform(wfc, center + shapeScale * sphereVerticesList[sphereIndicesList[i + 1]]);
                var c = math.transform(wfc, center + shapeScale * sphereVerticesList[sphereIndicesList[i + 2]]);
                i += 3;

                BasePhysicsDebugDisplaySystem.Triangle(a, b, c, math.cross(a, b), color);
            }
        }

        public static void DrawPrimitiveCapsuleEdges(float radius, float height, float3 center, Quaternion orientation, RigidTransform wfc, ref ColliderGeometry capsuleGeometry, float uniformScale)
        {
            var edgesColor = DebugDisplay.ColorIndex.Green;
            var shapeScale = new float3(2.0f * radius, height, 2.0f * radius) * uniformScale;

            var capsuleEdges = capsuleGeometry.EdgesArray;

            var i = 0;
            while (i < capsuleEdges.Length)
            {
                var a = math.transform(wfc, center + math.rotate(orientation, shapeScale * capsuleEdges[i + 0]));
                var b = math.transform(wfc, center + math.rotate(orientation, shapeScale * capsuleEdges[i + 1]));
                i += 2;

                BasePhysicsDebugDisplaySystem.Line(a, b, edgesColor);
            }
        }

        public static void DrawPrimitiveCapsuleFaces(float radius, float height, float3 center, Quaternion orientation, RigidTransform wfc, ref ColliderGeometry capsuleGeometry, ColorIndex color, float uniformScale)
        {
            var shapeScale = new float3(2.0f * radius, height, 2.0f * radius);

            var capsuleVerticesArray = capsuleGeometry.VerticesArray;
            var capsuleIndicesArray = capsuleGeometry.IndicesArray;

            var i = 0;
            while (i < capsuleIndicesArray.Length)
            {
                var a = math.transform(wfc, center + math.rotate(orientation, uniformScale * shapeScale * capsuleVerticesArray[capsuleIndicesArray[i]]));
                var b = math.transform(wfc, center + math.rotate(orientation, uniformScale * shapeScale * capsuleVerticesArray[capsuleIndicesArray[i + 1]]));
                var c = math.transform(wfc, center + math.rotate(orientation, uniformScale * shapeScale * capsuleVerticesArray[capsuleIndicesArray[i + 2]]));
                i += 3;

                BasePhysicsDebugDisplaySystem.Triangle(a, b, c, math.cross(a, b), color);
            }
        }

        public static void DrawPrimitiveBoxFaces(float3 size, float3 center, Quaternion orientation, RigidTransform wfc, ref ColliderGeometry boxGeometry, ColorIndex color, float uniformScale)
        {
            var boxVerticesArray = boxGeometry.VerticesArray;
            var boxIndicesArray = boxGeometry.IndicesArray;

            var i = 0;
            while (i < boxIndicesArray.Length)
            {
                var a = math.transform(wfc, center + math.rotate(orientation, uniformScale * size * boxVerticesArray[boxIndicesArray[i]]));
                var b = math.transform(wfc, center + math.rotate(orientation, uniformScale * size * boxVerticesArray[boxIndicesArray[i + 1]]));
                var c = math.transform(wfc, center + math.rotate(orientation, uniformScale * size * boxVerticesArray[boxIndicesArray[i + 2]]));
                i += 3;

                BasePhysicsDebugDisplaySystem.Triangle(a, b, c, math.cross(a, b), color);
            }
        }

        public static void DrawPrimitiveCylinderEdges(float radius, float height, float3 center, Quaternion orientation, RigidTransform wfc, ref ColliderGeometry cylinderGeometry, float uniformScale)
        {
            var edgesColor = DebugDisplay.ColorIndex.Green;
            var shapeScale = new float3(2.0f * radius, height * 0.5f, 2.0f * radius) * uniformScale;

            var cylinderVerticesArray = cylinderGeometry.VerticesArray;
            var cylinderIndicesArray = cylinderGeometry.IndicesArray;

            var i = 0;
            while (i < cylinderIndicesArray.Length)
            {
                var a = math.transform(wfc, center + math.rotate(orientation, shapeScale * cylinderVerticesArray[cylinderIndicesArray[i]]));
                var b = math.transform(wfc, center + math.rotate(orientation, shapeScale * cylinderVerticesArray[cylinderIndicesArray[i + 1]]));
                var c = math.transform(wfc, center + shapeScale * math.rotate(orientation, cylinderVerticesArray[cylinderIndicesArray[i + 2]]));
                i += 3;

                BasePhysicsDebugDisplaySystem.Line(a, b, edgesColor);
                BasePhysicsDebugDisplaySystem.Line(a, c, edgesColor);
                BasePhysicsDebugDisplaySystem.Line(b, c, edgesColor);
            }
        }

        public static void DrawPrimitiveCylinderFaces(float radius, float height, float3 center, Quaternion orientation, RigidTransform wfc, ref ColliderGeometry cylinderGeometry, ColorIndex color, float uniformScale)
        {
            var shapeScale = new float3(2.0f * radius, height * 0.5f, 2.0f * radius);

            var cylinderVerticesArray = cylinderGeometry.VerticesArray;
            var cylinderIndicesArray = cylinderGeometry.IndicesArray;

            var i = 0;
            while (i < cylinderIndicesArray.Length)
            {
                var a = math.transform(wfc, center + math.rotate(orientation, uniformScale * shapeScale * cylinderVerticesArray[cylinderIndicesArray[i]]));
                var b = math.transform(wfc, center + math.rotate(orientation, uniformScale * shapeScale * cylinderVerticesArray[cylinderIndicesArray[i + 1]]));
                var c = math.transform(wfc, center + shapeScale * math.rotate(orientation, uniformScale * cylinderVerticesArray[cylinderIndicesArray[i + 2]]));
                i += 3;

                BasePhysicsDebugDisplaySystem.Triangle(a, b, c, math.cross(a, b), color);
            }
        }

        private static void SetDiscSectionPoints(NativeArray<Vector3> dest, int count, Vector3 normalAxis, Vector3 from, float angle, float radius)
        {
            Vector3 startingPoint = math.normalize(from) * radius;
            var step = angle * math.PI / 180f;
            var r = quaternion.AxisAngle(normalAxis, step / (count - 1));
            Vector3 tangent = startingPoint;
            for (int i = 0; i < count; i++)
            {
                dest[i] = tangent;
                tangent = math.rotate(r, tangent);
            }
        }

        private static void GetWireArcSegments(ref NativeArray<Vector3> segmentArray, int segmentIndex, Vector3 center,
            Vector3 normal, Vector3 from, float angle, float radius)
        {
            const int kMaxArcPoints = 15;
            NativeArray<Vector3> sPoints = new NativeArray<Vector3>(kMaxArcPoints, Allocator.Temp);

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
        static NativeArray<Vector3> CreateCapsuleWireFrame(Allocator allocator)
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
            NativeArray<Vector3> capsuleEdges = new NativeArray<Vector3>(128, allocator); //or 188 if want extra circles

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
        static NativeArray<float3> TransformAndCalculateNormal(float3 v0, float3 v1, float3 v2, RigidTransform wfc)
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
            BasePhysicsDebugDisplaySystem.Triangle(v[0], v[1], v[2], v[3], colourIndex);
        }
    }
}
