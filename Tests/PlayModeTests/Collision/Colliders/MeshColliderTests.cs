using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using TestUtils = Unity.Physics.Tests.Utils.TestUtils;
using Unity.Collections;
using Random = Unity.Mathematics.Random;

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
            var triangles = new NativeArray<int>(numTriangles * 3, Allocator.Persistent);

            for(int i = 0; i < numTriangles * 3; i++)
            {
                vertices[i] = new float3((float)i, 1.0f * (float)(i % 2), (float)(i + 1));
                triangles[i] = i;
            }

            Random rnd = new Random(0x12345678);

            for (int i = 0; i < 100; i++)
            {
                int indexToChange = rnd.NextInt(0, triangles.Length - 1);

                int invalidValue = rnd.NextInt() * (rnd.NextBool() ? -1 : 1);
                triangles[indexToChange] = invalidValue;

                TestUtils.ThrowsException<System.ArgumentException>(
                        () => Unity.Physics.MeshCollider.Create(vertices, triangles)
                );

                triangles[indexToChange] = indexToChange;
            }


            triangles.Dispose();
            vertices.Dispose();
        }
    }
}

