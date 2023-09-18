using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Random = Unity.Mathematics.Random;

namespace Unity.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Test class containing tests for the <see cref="MeshCollider"/>
    /// </summary>
    class MeshColliderTests
    {
        static void GenerateMeshData(in int numTriangles, out NativeArray<float3> vertices, out NativeArray<int3> triangles)
        {
            vertices = new NativeArray<float3>(numTriangles * 3, Allocator.Temp);
            triangles = new NativeArray<int3>(numTriangles, Allocator.Temp);

            for (int i = 0; i < numTriangles; i++)
            {
                int firstVertexIndex = i * 3;

                vertices[firstVertexIndex]     = new float3(firstVertexIndex    , 1f * (firstVertexIndex % 2)      , firstVertexIndex + 1);
                vertices[firstVertexIndex + 1] = new float3(firstVertexIndex + 1, 1f * ((firstVertexIndex + 1) % 2), firstVertexIndex + 2);
                vertices[firstVertexIndex + 2] = new float3(firstVertexIndex + 2, 1f * ((firstVertexIndex + 2) % 2), firstVertexIndex + 3);
                triangles[i] = new int3(firstVertexIndex, firstVertexIndex + 1, firstVertexIndex + 2);
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            [GenerateTestsForBurstCompatibility]
            public void Execute()
            {
                GenerateMeshData(100, out var vertices, out var triangles);
                try
                {
                    using var collider = MeshCollider.Create(vertices, triangles);
                }
                finally
                {
                    vertices.Dispose();
                    triangles.Dispose();
                }
            }
        }

        [Test]
        public void MeshCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        struct PolygonCounter : ILeafColliderCollector
        {
            public int NumTriangles;
            public int NumQuads;

            public unsafe void AddLeaf(ColliderKey key, ref ChildCollider leaf)
            {
                var collider = leaf.Collider;
                if (collider->Type == ColliderType.Quad)
                {
                    ++NumQuads;
                }
                else if (collider->Type == ColliderType.Triangle)
                {
                    ++NumTriangles;
                }
            }

            public void PushCompositeCollider(ColliderKeyPath compositeKey, Math.MTransform parentFromComposite, out Math.MTransform worldFromParent)
            {
                worldFromParent = new Math.MTransform();

                // does nothing
            }

            public void PopCompositeCollider(uint numCompositeKeyBits, Math.MTransform worldFromParent)
            {
                // does nothing
            }
        }

        unsafe void ValidateMeshCollider(BlobAssetReference<Collider> collider, int expectedTriangleCount, int expectedQuadCount)
        {
            // manually created colliders are unique by design
            Assert.IsTrue(collider.Value.IsUnique);

            Assert.AreEqual(ColliderType.Mesh, collider.Value.Type);
            Assert.AreEqual(CollisionType.Composite, collider.Value.CollisionType);

            // make sure the mesh collider contains the correct number of triangles
            ref var meshCollider = ref UnsafeUtility.AsRef<MeshCollider>(collider.GetUnsafePtr());
            int quadCount = 0;
            int triangleCount = 0;
            ref Mesh mesh = ref meshCollider.Mesh;
            for (int sectionIndex = 0; sectionIndex < mesh.Sections.Length; sectionIndex++)
            {
                ref Mesh.Section section = ref mesh.Sections[sectionIndex];
                for (int primitiveIndex = 0; primitiveIndex < section.PrimitiveVertexIndices.Length; primitiveIndex++)
                {
                    Mesh.PrimitiveVertexIndices vertexIndices = section.PrimitiveVertexIndices[primitiveIndex];
                    Mesh.PrimitiveFlags flags = section.PrimitiveFlags[primitiveIndex];
                    bool isTrianglePair = (flags & Mesh.PrimitiveFlags.IsTrianglePair) != 0;
                    bool isQuad = (flags & Mesh.PrimitiveFlags.IsQuad) != 0;
                    if (isQuad)
                    {
                        ++quadCount;
                    }
                    else if (isTrianglePair)
                    {
                        triangleCount += 2;
                    }
                    else
                    {
                        ++triangleCount;
                    }
                }
            }
            Assert.AreEqual(expectedTriangleCount, triangleCount);
            Assert.AreEqual(expectedQuadCount, quadCount);

            // check the same with a leaf collector
            var counter = new PolygonCounter();
            meshCollider.GetLeaves(ref counter);
            Assert.AreEqual(expectedTriangleCount, counter.NumTriangles);
            Assert.AreEqual(expectedQuadCount, counter.NumQuads);
        }

        /// <summary>
        /// Create a <see cref="MeshCollider"/> and check that all attributes are set as expected
        /// </summary>
        [Test]
        public void MeshCollider_Create_ResultHasExpectedValues()
        {
            const int kNumTriangles = 10;
            GenerateMeshData(kNumTriangles, out var vertices, out var triangles);
            try
            {
                using var collider = MeshCollider.Create(vertices, triangles);
                const int kExpectedQuadCount = 0;
                ValidateMeshCollider(collider, kNumTriangles, kExpectedQuadCount);

                using var colliderClone = collider.Value.Clone();
                ValidateMeshCollider(colliderClone, kNumTriangles, kExpectedQuadCount);
            }
            finally
            {
                vertices.Dispose();
                triangles.Dispose();
            }
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        /// <summary>
        /// Create <see cref="MeshCollider"/> with invalid triangle indices
        /// and ensure that the invalid index is detected
        /// </summary>
        [Test]
        public void MeshCollider_Create_WhenTriangleIndexOutOfRange_Throws()
        {
            GenerateMeshData(10, out var vertices, out var triangles);

            try
            {
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

                    Assert.Throws<ArgumentException>(() => MeshCollider.Create(vertices, triangles));

                    triangle[vertexInTriangle] = indexToChange;
                    triangles[triangleIndex] = triangle;
                }
            }
            finally
            {
                triangles.Dispose();
                vertices.Dispose();
            }
        }

#endif
    }
}
