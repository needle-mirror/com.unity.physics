using NUnit.Framework;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.PerformanceTesting;
using UnityEngine;
using Random = Unity.Mathematics.Random;
using TestUtils = Unity.Physics.Tests.Utils.TestUtils;

namespace Unity.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Test class containing tests for the <see cref="MeshCollider"/>
    /// </summary>
    class MeshColliderTests
    {
        /// <summary>
        /// Create <see cref="MeshCollider"/> with invalid triangle indices
        /// and ensure that the invalid index is detected
        /// </summary>
        [Test]
        public unsafe void TestMeshColliderCreateWithInvalidIndices()
        {
            int numTriangles = 10;
            var vertices = new NativeArray<float3>(numTriangles * 3, Allocator.Persistent);
            var triangles = new NativeArray<int3>(numTriangles, Allocator.Persistent);

            for (int i = 0; i < numTriangles; i++)
            {
                int firstVertexIndex = i * 3;

                vertices[firstVertexIndex] = new float3((float)firstVertexIndex, 1.0f * (float)(firstVertexIndex % 2), (float)(firstVertexIndex + 1));
                vertices[firstVertexIndex + 1] = new float3((float)(firstVertexIndex + 1), 1.0f * (float)((firstVertexIndex + 1) % 2), (float)(firstVertexIndex + 2));
                vertices[firstVertexIndex + 2] = new float3((float)(firstVertexIndex + 2), 1.0f * (float)((firstVertexIndex + 2 ) % 2), (float)(firstVertexIndex + 3));
                triangles[i] = new int3(firstVertexIndex, firstVertexIndex + 1, firstVertexIndex + 2);
            }

            Random rnd = new Random(0x12345678);

            for (int i = 0; i < 100; i++)
            {
                int indexToChange = rnd.NextInt(0, triangles.Length * 3 - 1);

                int triangleIndex = indexToChange / 3;
                int vertexInTriangle = indexToChange % 3;
                int invalidValue = rnd.NextInt() * (rnd.NextBool() ? -1 : 1);

                var triangle = triangles[triangleIndex];
                triangle[vertexInTriangle] = invalidValue;
                triangles[triangleIndex] = triangle;

                TestUtils.ThrowsException<System.ArgumentException>(
                        () => Unity.Physics.MeshCollider.Create(vertices, triangles)
                );

                triangle[vertexInTriangle] = indexToChange;
                triangles[triangleIndex] = triangle;
            }

            triangles.Dispose();
            vertices.Dispose();
        }

        /// <summary>
        /// Measure performance of creation of <see cref="MeshCollider"/>.
        /// </summary>
#if UNITY_2019_2_OR_NEWER
        [Test, Performance]
#else
        [PerformanceTest]
#endif
        [TestCase(TestName = "MeshBuilderPerfTest")]
        public void MeshBuilderPerfTest()
        {
            // Execute dummy job just to get Burst compilation out of the way.
            {
                var dummyVertices = new NativeArray<float3>(1, Allocator.TempJob);
                var dummyTriangles = new NativeArray<int3>(1, Allocator.TempJob);

                new TestMeshBuilderJob
                {
                    DummyRun = true,
                    Vertices = dummyVertices,
                    Triangles = dummyTriangles
                }.Run();

                dummyVertices.Dispose();
                dummyTriangles.Dispose();
            }

            UnityEngine.Mesh mesh = Resources.Load<UnityEngine.Mesh>("VolcanicTerrain_80000");

            // Vertices
            var vertices = new NativeArray<float3>(mesh.vertexCount, Allocator.TempJob);
            var verticesList = new List<Vector3>();
            mesh.GetVertices(verticesList);
            int vertexCount = 0;
            foreach (var v in verticesList)
            {
                vertices[vertexCount++] = v;
            }

            // Indices
            var finalIndicesList = new List<int>();
            var indicesList = new List<int>();
            for (var subMesh = 0; subMesh < mesh.subMeshCount; ++subMesh)
            {
                mesh.GetIndices(indicesList, subMesh);
                foreach (var i in indicesList)
                {
                    finalIndicesList.Add(i);
                }
                indicesList.Clear();
            }
            var indices = new NativeArray<int>(finalIndicesList.Count, Allocator.TempJob);
            int indexCount = 0;

            foreach (var i in finalIndicesList)
            {
                indices[indexCount++] = i;
            }

            NativeArray<int3> triangles;

            unsafe
            {
                triangles = indices.Reinterpret<int3>(UnsafeUtility.SizeOf<int>());
            }

            Measure.Method(() =>
            {
                new TestMeshBuilderJob
                {
                    DummyRun = false,
                    Vertices = vertices,
                    Triangles = triangles
                }.Run();
            }).Definition(sampleUnit: SampleUnit.Millisecond)
              .MeasurementCount(1)
              .Run();

            vertices.Dispose();
            indices.Dispose();
        }

        [BurstCompile(CompileSynchronously = true)]
        private struct TestMeshBuilderJob : IJob
        {
            public bool DummyRun;

            public NativeArray<float3> Vertices;
            public NativeArray<int3> Triangles;

            public unsafe void Execute()
            {
                if (DummyRun)
                {
                    return;
                }

                MeshCollider.Create(Vertices, Triangles);
            }
        }
    }
}

