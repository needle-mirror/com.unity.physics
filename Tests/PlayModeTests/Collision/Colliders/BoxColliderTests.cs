using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;
using TestUtils = Unity.Physics.Tests.Utils.TestUtils;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Test class containing tests for the <see cref="BoxCollider"/>
    /// </summary>
    public class BoxColliderTests
    {
        #region Construction

        /// <summary>
        /// Create a <see cref="BoxCollider"/> and check that all attributes are set as expected
        /// </summary>
        [Test]
        public unsafe void TestBoxColliderCreate()
        {
            float3 center = new float3(-10.10f, 10.12f, 0.01f);
            quaternion orientation = quaternion.AxisAngle(math.normalize(new float3(1.4f, 0.2f, 1.1f)), 38.50f);
            float3 size = new float3(0.01f, 120.40f, 5.4f);
            float convexRadius = 0.0f;
            var collider = BoxCollider.Create(center, orientation, size, convexRadius);
            var boxCollider = UnsafeUtilityEx.AsRef<BoxCollider>(collider.GetUnsafePtr());

            Assert.AreEqual(center, boxCollider.Center);
            Assert.AreEqual(orientation, boxCollider.Orientation);
            Assert.AreEqual(size, boxCollider.Size);
            Assert.AreEqual(convexRadius, boxCollider.ConvexRadius);
            Assert.AreEqual(CollisionType.Convex, boxCollider.CollisionType);
            Assert.AreEqual(ColliderType.Box, boxCollider.Type);
        }

        /// <summary>
        /// Create <see cref="BoxCollider"/> with invalid center/orientation/size/convexRadius and
        /// check that exceptions are being thrown for invalid values
        /// </summary>
        [Test]
        public void TestBoxColliderCreateInvalid()
        {
            float3 center = new float3(1.0f, 0.0f, 0.0f);
            quaternion orientation = quaternion.AxisAngle(math.normalize(new float3(4.3f, 1.2f, 0.1f)), 1085.0f);
            float3 size = new float3(1.0f, 2.0f, 3.0f);
            float convexRadius = 0.45f;

            // Invalid center, positive infinity
            {
                float3 invalidCenter = new float3(float.PositiveInfinity, 1.0f, 1.0f);
                TestUtils.ThrowsException<System.ArgumentException>(
                        () => BoxCollider.Create(invalidCenter, orientation, size, convexRadius)
                );
            }

            // Invalid center, positive infinity
            {
                float3 invalidCenter = new float3(float.NegativeInfinity, 1.0f, 1.0f);
                TestUtils.ThrowsException<System.ArgumentException>(
                        () => BoxCollider.Create(invalidCenter, orientation, size, convexRadius)
                );
            }

            // Invalid center, nan
            {
                float3 invalidCenter = new float3(float.NaN, 1.0f, 1.0f);
                TestUtils.ThrowsException<System.ArgumentException>(
                        () => BoxCollider.Create(invalidCenter, orientation, size, convexRadius)
                );
            }

            // Negative size
            {
                float3 invalidSize = new float3(-1.0f, 1.0f, 1.0f);
                TestUtils.ThrowsException<System.ArgumentException>(
                        () => BoxCollider.Create(center, orientation, invalidSize, convexRadius)
                );
            }

            // Invalid size, positive inf
            {
                float3 invalidSize = new float3(float.PositiveInfinity, 1.0f, 1.0f);
                TestUtils.ThrowsException<System.ArgumentException>(
                        () => BoxCollider.Create(center, orientation, invalidSize, convexRadius)
                );
            }

            // Invalid size, negative inf
            {
                float3 invalidSize = new float3(float.NegativeInfinity, 1.0f, 1.0f);
                TestUtils.ThrowsException<System.ArgumentException>(
                        () => BoxCollider.Create(center, orientation, invalidSize, convexRadius)
                );
            }

            // Invalid size, nan
            {
                float3 invalidSize = new float3(float.NaN, 1.0f, 1.0f);
                TestUtils.ThrowsException<System.ArgumentException>(
                        () => BoxCollider.Create(center, orientation, invalidSize, convexRadius)
                );
            }

            // Negative convex radius
            {
                float invalidConvexRadius = -0.0001f;
                TestUtils.ThrowsException<System.ArgumentException>(
                        () => BoxCollider.Create(center, orientation, size, invalidConvexRadius)
                );
            }

            // Invalid convex radius, +inf
            {
                float invalidConvexRadius = float.PositiveInfinity;
                TestUtils.ThrowsException<System.ArgumentException>(
                        () => BoxCollider.Create(center, orientation, size, invalidConvexRadius)
                );
            }

            // Invalid convex radius, -inf
            {
                float invalidConvexRadius = float.NegativeInfinity;
                TestUtils.ThrowsException<System.ArgumentException>(
                        () => BoxCollider.Create(center, orientation, size, invalidConvexRadius)
                );
            }

            // Invalid convex radius, nan
            {
                float invalidConvexRadius = float.NaN;
                TestUtils.ThrowsException<System.ArgumentException>(
                        () => BoxCollider.Create(center, orientation, size, invalidConvexRadius)
                );
            }

        }

        #endregion

        #region IConvexCollider

        /// <summary>
        /// Test that a translated box collider generates the correct local AABB
        /// </summary>
        /// <remarks>
        /// The following code was used to produce reference data for Aabbs:
        /// <code>
        /// private Aabb CalculateBoxAabbNaive(float3 center, quaternion orientation, float3 size, quaternion bRotation, float3 bTranslation)
        ///{
        ///    float3[] points = {
        ///        0.5f * new float3(-size.x, -size.y, -size.z),
        ///        0.5f * new float3(-size.x, -size.y, size.z),
        ///        0.5f * new float3(-size.x, size.y, -size.z),
        ///        0.5f * new float3(-size.x, size.y, size.z),
        ///        0.5f * new float3(size.x, -size.y, -size.z),
        ///        0.5f * new float3(size.x, -size.y, size.z),
        ///        0.5f * new float3(size.x, size.y, -size.z),
        ///        0.5f * new float3(size.x, size.y, size.z)
        ///    };
        ///
        ///    for (int i = 0; i < 8; ++i)
        ///    {
        ///        points[i] = center + math.mul(orientation, points[i]);
        ///        points[i] = bTranslation + math.mul(bRotation, points[i]);
        ///    }
        ///
        ///    Aabb result = Aabb.CreateFromPoints(new float3x4(points[0], points[1], points[2], points[3]));
        ///    for (int i = 4; i < 8; ++i)
        ///    {
        ///        result.Include(points[i]);
        ///    }
        ///    return result;
        ///}
        /// </code>
        /// </remarks>
        [Test]
        public void TestBoxColliderCalculateAabbLocalTranslation()
        {
            // Expected values in this test were generated using CalculateBoxAabbNaive above
            {
                float3 center = new float3(-0.59f, 0.36f, 0.35f);
                quaternion orientation = quaternion.identity;
                float3 size = new float3(2.32f, 10.87f, 16.49f);
                float convexRadius = 0.25f;

                Aabb expectedAabb = new Aabb
                {
                    Min = new float3(-1.75f, -5.075f, -7.895f),
                    Max = new float3(0.57f, 5.795f, 8.595f)
                };

                var boxCollider = BoxCollider.Create(center, orientation, size, convexRadius);
                Aabb aabb = boxCollider.Value.CalculateAabb();
                TestUtils.AreEqual(expectedAabb.Min, aabb.Min, 1e-3f);
                TestUtils.AreEqual(expectedAabb.Max, aabb.Max, 1e-3f);
            }
        }

        /// <summary>
        /// Test that the created inertia tensor of the <see cref="BoxCollider"/> is correct
        /// </summary>
        /// <remarks>
        /// Formula for inertia tensor from here was used: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        /// </remarks>
        [Test]
        public void TestBoxColliderMassProperties()
        {
            float3 center = float3.zero;
            quaternion orientation = quaternion.identity;
            float3 size = new float3(1.0f, 250.0f, 2.0f);
            float convexRadius = 0.25f;
            var boxCollider = BoxCollider.Create(center, orientation, size, convexRadius);

            float3 expectedInertiaTensor = 1.0f / 12.0f * new float3(
                size.y * size.y + size.z * size.z,
                size.x * size.x + size.z * size.z,
                size.y * size.y + size.x * size.x);

            MassProperties massProperties = boxCollider.Value.MassProperties;
            float3 inertiaTensor = massProperties.MassDistribution.InertiaTensor;
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, 1e-3f);
        }

        #endregion
    }
}
