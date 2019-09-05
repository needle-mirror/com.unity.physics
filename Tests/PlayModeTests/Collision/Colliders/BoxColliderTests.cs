using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;
using TestUtils = Unity.Physics.Tests.Utils.TestUtils;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;

namespace Unity.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Test class containing tests for the <see cref="BoxCollider"/>
    /// </summary>
    class BoxColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute() =>
                BoxCollider.Create(new BoxGeometry { Orientation = quaternion.identity, Size = new float3(1f) }).Release();
        }

        [Test]
        public void BoxCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        /// <summary>
        /// Create a <see cref="BoxCollider"/> and check that all attributes are set as expected
        /// </summary>
        [Test]
        public unsafe void TestBoxColliderCreate()
        {
            var geometry = new BoxGeometry
            {
                Center = new float3(-10.10f, 10.12f, 0.01f),
                Orientation = quaternion.AxisAngle(math.normalize(new float3(1.4f, 0.2f, 1.1f)), 38.50f),
                Size = new float3(0.01f, 120.40f, 5.4f),
                BevelRadius = 0.0f
            };

            var collider = BoxCollider.Create(geometry);
            var boxCollider = UnsafeUtilityEx.AsRef<BoxCollider>(collider.GetUnsafePtr());

            Assert.AreEqual(geometry.Center, boxCollider.Center);
            Assert.AreEqual(geometry.Center, boxCollider.Geometry.Center);
            Assert.AreEqual(geometry.Orientation, boxCollider.Orientation);
            Assert.AreEqual(geometry.Orientation, boxCollider.Geometry.Orientation);
            Assert.AreEqual(geometry.Size, boxCollider.Size);
            Assert.AreEqual(geometry.Size, boxCollider.Geometry.Size);
            Assert.AreEqual(geometry.BevelRadius, boxCollider.BevelRadius);
            Assert.AreEqual(geometry.BevelRadius, boxCollider.Geometry.BevelRadius);
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
            var geometry = new BoxGeometry
            {
                Center = new float3(1.0f, 0.0f, 0.0f),
                Orientation = quaternion.AxisAngle(math.normalize(new float3(4.3f, 1.2f, 0.1f)), 1085.0f),
                Size = new float3(1.0f, 2.0f, 3.0f),
                BevelRadius = 0.45f
            };

            // Invalid center, positive infinity
            {
                var invalidGeometry = geometry;
                invalidGeometry.Center = new float3(float.PositiveInfinity, 1.0f, 1.0f);
                TestUtils.ThrowsException<ArgumentException>(() => BoxCollider.Create(invalidGeometry));
            }

            // Invalid center, positive infinity
            {
                var invalidGeometry = geometry;
                invalidGeometry.Center = new float3(float.NegativeInfinity, 1.0f, 1.0f);
                TestUtils.ThrowsException<ArgumentException>(() => BoxCollider.Create(invalidGeometry));
            }

            // Invalid center, nan
            {
                var invalidGeometry = geometry;
                invalidGeometry.Center = new float3(float.NaN, 1.0f, 1.0f);
                TestUtils.ThrowsException<ArgumentException>(() => BoxCollider.Create(invalidGeometry));
            }

            // Negative size
            {
                var invalidGeometry = geometry;
                invalidGeometry.Size = new float3(-1.0f, 1.0f, 1.0f);
                TestUtils.ThrowsException<ArgumentException>(() => BoxCollider.Create(invalidGeometry));
            }

            // Invalid size, positive inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.Size = new float3(float.PositiveInfinity, 1.0f, 1.0f);
                TestUtils.ThrowsException<ArgumentException>(() => BoxCollider.Create(invalidGeometry));
            }

            // Invalid size, negative inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.Size = new float3(float.NegativeInfinity, 1.0f, 1.0f);
                TestUtils.ThrowsException<ArgumentException>(() => BoxCollider.Create(invalidGeometry));
            }

            // Invalid size, nan
            {
                var invalidGeometry = geometry;
                invalidGeometry.Size = new float3(float.NaN, 1.0f, 1.0f);
                TestUtils.ThrowsException<ArgumentException>(() => BoxCollider.Create(invalidGeometry));
            }

            // Negative bevel radius
            {
                var invalidGeometry = geometry;
                invalidGeometry.BevelRadius = -0.0001f;
                TestUtils.ThrowsException<ArgumentException>(() => BoxCollider.Create(invalidGeometry));
            }

            // Invalid bevel radius, +inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.BevelRadius = float.PositiveInfinity;
                TestUtils.ThrowsException<ArgumentException>(() => BoxCollider.Create(invalidGeometry));
            }

            // Invalid bevel radius, -inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.BevelRadius = float.NegativeInfinity;
                TestUtils.ThrowsException<ArgumentException>(() => BoxCollider.Create(invalidGeometry));
            }

            // Invalid bevel radius, nan
            {
                var invalidGeometry = geometry;
                invalidGeometry.BevelRadius = float.NaN;
                TestUtils.ThrowsException<ArgumentException>(() => BoxCollider.Create(invalidGeometry));
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
                var geometry = new BoxGeometry
                {
                    Center = new float3(-0.59f, 0.36f, 0.35f),
                    Orientation = quaternion.identity,
                    Size = new float3(2.32f, 10.87f, 16.49f),
                    BevelRadius = 0.25f
                };

                Aabb expectedAabb = new Aabb
                {
                    Min = new float3(-1.75f, -5.075f, -7.895f),
                    Max = new float3(0.57f, 5.795f, 8.595f)
                };

                var boxCollider = BoxCollider.Create(geometry);
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
            var geometry = new BoxGeometry
            {
                Center = float3.zero,
                Orientation = quaternion.identity,
                Size = new float3(1.0f, 250.0f, 2.0f),
                BevelRadius = 0.25f
            };

            var boxCollider = BoxCollider.Create(geometry);

            float3 expectedInertiaTensor = 1.0f / 12.0f * new float3(
                geometry.Size.y * geometry.Size.y + geometry.Size.z * geometry.Size.z,
                geometry.Size.x * geometry.Size.x + geometry.Size.z * geometry.Size.z,
                geometry.Size.y * geometry.Size.y + geometry.Size.x * geometry.Size.x);

            MassProperties massProperties = boxCollider.Value.MassProperties;
            float3 inertiaTensor = massProperties.MassDistribution.InertiaTensor;
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, 1e-3f);
        }

        #endregion
    }
}
