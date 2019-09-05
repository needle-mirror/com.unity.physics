using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics.Tests.Utils;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;
using Random = Unity.Mathematics.Random;

namespace Unity.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Contains all <see cref="ConvexCollider"/> unit tests
    /// </summary>
    class ConvexColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute()
            {
                var points = new NativeArray<float3>(1024, Allocator.Temp);
                var random = new Random(1234);
                for (var i = 0; i < points.Length; ++i)
                    points[i] = random.NextFloat3(new float3(-1f), new float3(1f));
                ConvexCollider.Create(points, ConvexHullGenerationParameters.Default).Release();
            }
        }

        [Test]
        public void ConvexCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        /// <summary>
        /// Test that a <see cref="ConvexCollider"/> created with a point cloud has its attributes filled correctly
        /// </summary>
        [Test]
        public void TestConvexColliderCreate()
        {
            var points = new NativeArray<float3>(8, Allocator.TempJob)
            {
                [0] = new float3(1.45f, 8.67f, 3.45f),
                [1] = new float3(8.75f, 1.23f, 6.44f),
                [2] = new float3(100.34f, 5.33f, -2.55f),
                [3] = new float3(8.76f, 4.56f, -4.54f),
                [4] = new float3(9.75f, -0.45f, -8.99f),
                [5] = new float3(7.66f, 3.44f, 0.0f)
            };
            var collider = ConvexCollider.Create(
                points, new ConvexHullGenerationParameters { BevelRadius = 0.15f }, CollisionFilter.Default
            );
            points.Dispose();

            Assert.AreEqual(ColliderType.Convex, collider.Value.Type);
            Assert.AreEqual(CollisionType.Convex, collider.Value.CollisionType);
        }

        /// <summary>
        /// That that creating a <see cref="ConvexCollider"/> with invalid point clouds results in exceptions being thrown
        /// </summary>
        [Test]
        public void TestConvexColliderCreateInvalid()
        {
            // Invalid points
            {
                var validBevelRadius = new ConvexHullGenerationParameters { BevelRadius = 0.15f };

                // invalid point, +inf
                {
                    var invalidPoints = new NativeArray<float3>(6, Allocator.TempJob)
                    {
                        [0] = new float3(1.45f, 8.67f, 3.45f),
                        [1] = new float3(8.75f, 1.23f, 6.44f),
                        [2] = new float3(float.PositiveInfinity, 5.33f, -2.55f),
                        [3] = new float3(8.76f, 4.56f, -4.54f),
                        [4] = new float3(9.75f, -0.45f, -8.99f),
                        [5] = new float3(7.66f, 3.44f, 0.0f)
                    };
                    TestUtils.ThrowsException<ArgumentException>(
                        () => ConvexCollider.Create(invalidPoints, validBevelRadius, CollisionFilter.Default)
                    );
                    invalidPoints.Dispose();
                }

                // invalid point, -inf
                {
                    var invalidPoints = new NativeArray<float3>(6, Allocator.TempJob)
                    {
                        [0] = new float3(1.45f, 8.67f, 3.45f),
                        [1] = new float3(8.75f, 1.23f, 6.44f),
                        [2] = new float3(float.NegativeInfinity, 5.33f, -2.55f),
                        [3] = new float3(8.76f, 4.56f, -4.54f),
                        [4] = new float3(9.75f, -0.45f, -8.99f),
                        [5] = new float3(7.66f, 3.44f, 0.0f)
                    };
                    TestUtils.ThrowsException<ArgumentException>(
                        () => ConvexCollider.Create(invalidPoints, validBevelRadius, CollisionFilter.Default)
                    );
                    invalidPoints.Dispose();
                }

                // invalid point, NaN
                {
                    var invalidPoints = new NativeArray<float3>(6, Allocator.TempJob)
                    {
                        [0] = new float3(1.45f, 8.67f, 3.45f),
                        [1] = new float3(8.75f, 1.23f, 6.44f),
                        [2] = new float3(float.NaN, 5.33f, -2.55f),
                        [3] = new float3(8.76f, 4.56f, -4.54f),
                        [4] = new float3(9.75f, -0.45f, -8.99f),
                        [5] = new float3(7.66f, 3.44f, 0.0f)
                    };
                    TestUtils.ThrowsException<ArgumentException>(
                        () => ConvexCollider.Create(invalidPoints, validBevelRadius, CollisionFilter.Default)
                    );
                    invalidPoints.Dispose();
                }
            }

            // invalid convex radius
            {
                var points = new NativeArray<float3>(6, Allocator.TempJob)
                {
                    [0] = new float3(1.45f, 8.67f, 3.45f),
                    [1] = new float3(8.75f, 1.23f, 6.44f),
                    [2] = new float3(7.54f, 5.33f, -2.55f),
                    [3] = new float3(8.76f, 4.56f, -4.54f),
                    [4] = new float3(9.75f, -0.45f, -8.99f),
                    [5] = new float3(7.66f, 3.44f, 0.0f)
                };
                float3 scale = new float3(1.0f, 1.0f, 1.0f);
                var invalidBevelRadius = new ConvexHullGenerationParameters();

                // negative convex radius
                {
                    invalidBevelRadius.BevelRadius = -0.30f;
                    TestUtils.ThrowsException<ArgumentException>(
                       () => ConvexCollider.Create(points, invalidBevelRadius, CollisionFilter.Default)
                    );
                }

                // +inf convex radius
                {
                    invalidBevelRadius.BevelRadius = float.PositiveInfinity;
                    TestUtils.ThrowsException<ArgumentException>(
                       () => ConvexCollider.Create(points, invalidBevelRadius, CollisionFilter.Default)
                    );
                }

                // -inf convex radius
                {
                    invalidBevelRadius.BevelRadius = float.NegativeInfinity;
                    TestUtils.ThrowsException<ArgumentException>(
                       () => ConvexCollider.Create(points, invalidBevelRadius, CollisionFilter.Default)
                    );
                }

                // nan convex radius
                {
                    invalidBevelRadius.BevelRadius = float.NaN;
                    TestUtils.ThrowsException<ArgumentException>(
                       () => ConvexCollider.Create(points, invalidBevelRadius, CollisionFilter.Default)
                    );
                }

                points.Dispose();
            }
        }

        #endregion

        #region IConvexCollider

        /// <summary>
        /// Test that the local AABB is computed correctly for a <see cref="ConvexCollider"/> created with a point cloud
        /// </summary>
        [Test]
        public unsafe void TestConvexColliderCalculateAabbLocal([Values(0, 0.01f, 1.25f)] float maxShrinkMovement)
        {
            var points = new NativeArray<float3>(6, Allocator.TempJob)
            {
                [0] = new float3(1.45f, 8.67f, 3.45f),
                [1] = new float3(8.75f, 1.23f, 6.44f),
                [2] = new float3(100.34f, 5.33f, -2.55f),
                [3] = new float3(8.76f, 4.56f, -4.54f),
                [4] = new float3(9.75f, -0.45f, -8.99f),
                [5] = new float3(7.66f, 3.44f, 0.0f)
            };

            Aabb expectedAabb = Aabb.CreateFromPoints(new float3x4(points[0], points[1], points[2], points[3]));
            expectedAabb.Include(points[4]);
            expectedAabb.Include(points[5]);

            var collider = ConvexCollider.Create(
                points, new ConvexHullGenerationParameters { BevelRadius = maxShrinkMovement }, CollisionFilter.Default
            );
            points.Dispose();

            Aabb actualAabb = collider.Value.CalculateAabb();
            float convexRadius = ((ConvexCollider*)collider.GetUnsafePtr())->ConvexHull.ConvexRadius;
            float maxError = 1e-3f + maxShrinkMovement - convexRadius;
            TestUtils.AreEqual(expectedAabb.Min, actualAabb.Min, maxError);
            TestUtils.AreEqual(expectedAabb.Max, actualAabb.Max, maxError);
        }

        /// <summary>
        /// Test that the transformed AABB is computed correctly for a <see cref="ConvexCollider"/> created with a point cloud
        /// </summary>
        [Test]
        public unsafe void TestConvexColliderCalculateAabbTransformed([Values(0, 0.01f, 1.25f)] float maxShrinkMovement)
        {
            var points = new NativeArray<float3>(6, Allocator.TempJob)
            {
                [0] = new float3(1.45f, 8.67f, 3.45f),
                [1] = new float3(8.75f, 1.23f, 6.44f),
                [2] = new float3(100.34f, 5.33f, -2.55f),
                [3] = new float3(8.76f, 4.56f, -4.54f),
                [4] = new float3(9.75f, -0.45f, -8.99f),
                [5] = new float3(7.66f, 3.44f, 0.0f)
            };

            float3 translation = new float3(43.56f, -87.32f, -0.02f);
            quaternion rotation = quaternion.AxisAngle(math.normalize(new float3(8.45f, -2.34f, 0.82f)), 43.21f);

            float3[] transformedPoints = new float3[points.Length];
            for(int i = 0; i < points.Length; ++i)
            {
                transformedPoints[i] = translation + math.mul(rotation, points[i]);
            }

            Aabb expectedAabb = Aabb.CreateFromPoints(new float3x4(transformedPoints[0], transformedPoints[1], transformedPoints[2], transformedPoints[3]));
            expectedAabb.Include(transformedPoints[4]);
            expectedAabb.Include(transformedPoints[5]);

            var collider = ConvexCollider.Create(
                points, new ConvexHullGenerationParameters { BevelRadius = maxShrinkMovement }, CollisionFilter.Default
            );
            points.Dispose();

            Aabb actualAabb = collider.Value.CalculateAabb(new RigidTransform(rotation, translation));
            float convexRadius = ((ConvexCollider*)collider.GetUnsafePtr())->ConvexHull.ConvexRadius;
            float maxError = 1e-3f + maxShrinkMovement - convexRadius;

            TestUtils.AreEqual(expectedAabb.Min, actualAabb.Min, maxError);
            TestUtils.AreEqual(expectedAabb.Max, actualAabb.Max, maxError);
        }

        #endregion
    }
}

