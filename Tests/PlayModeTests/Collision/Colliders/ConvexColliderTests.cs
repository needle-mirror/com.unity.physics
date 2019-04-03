using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;
using TestUtils = Unity.Physics.Tests.Utils.TestUtils;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;

namespace Unity.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Contains all <see cref="ConvexCollider"/> unit tests
    /// </summary>
    public class ConvexColliderTests
    {
        #region Construction

        /// <summary>
        /// Test that a <see cref="ConvexCollider"/> created with a point cloud has its attributes filled correctly
        /// </summary>
        [Test]
        public void TestConvexColliderCreate()
        {
            var points = new NativeArray<float3>(8, Allocator.Temp)
            {
                [0] = new float3(1.45f, 8.67f, 3.45f),
                [1] = new float3(8.75f, 1.23f, 6.44f),
                [2] = new float3(100.34f, 5.33f, -2.55f),
                [3] = new float3(8.76f, 4.56f, -4.54f),
                [4] = new float3(9.75f, -0.45f, -8.99f),
                [5] = new float3(7.66f, 3.44f, 0.0f)
            };
            var collider = ConvexCollider.Create(points, convexRadius: 0.15f);
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
                float convexRadius = 0.15f;

                // invalid point, +inf
                {
                    var invalidPoints = new NativeArray<float3>(6, Allocator.Temp)
                    {
                        [0] = new float3(1.45f, 8.67f, 3.45f),
                        [1] = new float3(8.75f, 1.23f, 6.44f),
                        [2] = new float3(float.PositiveInfinity, 5.33f, -2.55f),
                        [3] = new float3(8.76f, 4.56f, -4.54f),
                        [4] = new float3(9.75f, -0.45f, -8.99f),
                        [5] = new float3(7.66f, 3.44f, 0.0f)
                    };
                    TestUtils.ThrowsException<System.ArgumentException>(
                        () => ConvexCollider.Create(invalidPoints, convexRadius)
                    );
                    invalidPoints.Dispose();
                }

                // invalid point, -inf
                {
                    var invalidPoints = new NativeArray<float3>(6, Allocator.Temp)
                    {
                        [0] = new float3(1.45f, 8.67f, 3.45f),
                        [1] = new float3(8.75f, 1.23f, 6.44f),
                        [2] = new float3(float.NegativeInfinity, 5.33f, -2.55f),
                        [3] = new float3(8.76f, 4.56f, -4.54f),
                        [4] = new float3(9.75f, -0.45f, -8.99f),
                        [5] = new float3(7.66f, 3.44f, 0.0f),
                    };
                    TestUtils.ThrowsException<System.ArgumentException>(
                        () => ConvexCollider.Create(invalidPoints, convexRadius)
                    );
                    invalidPoints.Dispose();
                }

                // invalid point, NaN
                {
                    var invalidPoints = new NativeArray<float3>(6, Allocator.Temp)
                    {
                        [0] = new float3(1.45f, 8.67f, 3.45f),
                        [1] = new float3(8.75f, 1.23f, 6.44f),
                        [2] = new float3(float.NaN, 5.33f, -2.55f),
                        [3] = new float3(8.76f, 4.56f, -4.54f),
                        [4] = new float3(9.75f, -0.45f, -8.99f),
                        [5] = new float3(7.66f, 3.44f, 0.0f)
                    };
                    TestUtils.ThrowsException<System.ArgumentException>(
                        () => ConvexCollider.Create(invalidPoints, convexRadius)
                    );
                    invalidPoints.Dispose();
                }
            }

            // invalid convex radius
            {
                var points = new NativeArray<float3>(6, Allocator.Temp)
                {
                    [0] = new float3(1.45f, 8.67f, 3.45f),
                    [1] = new float3(8.75f, 1.23f, 6.44f),
                    [2] = new float3(7.54f, 5.33f, -2.55f),
                    [3] = new float3(8.76f, 4.56f, -4.54f),
                    [4] = new float3(9.75f, -0.45f, -8.99f),
                    [5] = new float3(7.66f, 3.44f, 0.0f)
                };
                float3 scale = new float3(1.0f, 1.0f, 1.0f);

                // negative convex radius
                {
                    float invalidConvexRadius = -0.30f;
                    TestUtils.ThrowsException<System.ArgumentException>(
                       () => ConvexCollider.Create(points, invalidConvexRadius)
                    );
                }

                // +inf convex radius
                {
                    float invalidConvexRadius = float.PositiveInfinity;
                    TestUtils.ThrowsException<System.ArgumentException>(
                       () => ConvexCollider.Create(points, invalidConvexRadius)
                    );
                }

                // -inf convex radius
                {
                    float invalidConvexRadius = float.NegativeInfinity;
                    TestUtils.ThrowsException<System.ArgumentException>(
                       () => ConvexCollider.Create(points, invalidConvexRadius)
                    );
                }

                // nan convex radius
                {
                    float invalidConvexRadius = float.NaN;
                    TestUtils.ThrowsException<System.ArgumentException>(
                       () => ConvexCollider.Create(points, invalidConvexRadius)
                    );
                }

                points.Dispose();
            }
        }

        /// <summary>
        /// Test that the inertia tensor for the convex hull of a point cloud are calculated correctly
        /// </summary>
        /// <remarks>
        /// Code used to generate the reference inertia tensor:
        /// <code>
        /// hkArray<hkVector4> vertices;
        /// vertices.pushBack(hkVector4(-1.56f, 8.89f, -10.76f));
        /// vertices.pushBack(hkVector4(-4.74f, 80.11f, 10.56f));
        /// vertices.pushBack(hkVector4(-100.60f, -4.93f, -10.76f));
        /// vertices.pushBack(hkVector4(1.44f, 3.56f, 73.4f));
        /// vertices.pushBack(hkVector4(17.66f, 18.43f, 0.0f));
        /// vertices.pushBack(hkVector4(-1.32f, 9.99f, 80.4f));
        /// vertices.pushBack(hkVector4(-17.45f, 3.22f, -3.22f));
        /// vertices.pushBack(hkVector4(0.0f, 0.0f, 0.03f));
        /// hkStridedVertices stridedVertices; stridedVertices.set(vertices);
        /// float convexRadius = 0.15f;
        ///
        /// hknpConvexShape::BuildConfig buildConfig;
        /// buildConfig.m_massConfig.m_inertiaFactor = 1.0f;
        /// hknpShape* shape = hknpConvexShape::createFromVertices(stridedVertices, convexRadius, buildConfig);
        ///
        /// hkMassProperties massProps;
        /// shape->getMassProperties(massProps);
        ///
        /// hkVector4 inertiaTensor;
        /// hkRotation principleAxis;
        /// hkInertiaTensorComputer::convertInertiaTensorToPrincipleAxis(massProps.m_inertiaTensor, principleAxis);
        ///
        /// inertiaTensor.setMul(massProps.m_inertiaTensor.getColumn(0), hkVector4::getConstant<HK_QUADREAL_1000>());
        /// inertiaTensor.addMul(massProps.m_inertiaTensor.getColumn(1), hkVector4::getConstant<HK_QUADREAL_0100>());
        /// inertiaTensor.addMul(massProps.m_inertiaTensor.getColumn(2), hkVector4::getConstant<HK_QUADREAL_0010>());
        /// inertiaTensor.mul(hkSimdReal::fromFloat(1.0f / massProps.m_mass));
        /// </code>
        /// </remarks>
        [Test]
        public void TestConvexColliderMassProperties()
        {
            var points = new NativeArray<float3>(8, Allocator.Temp)
            {
                [0] = new float3(-1.56f, 8.89f, -10.76f),
                [1] = new float3(-4.74f, 80.11f, 10.56f),
                [2] = new float3(-100.60f, -4.93f, -10.76f),
                [3] = new float3(1.44f, 3.56f, 73.4f),
                [4] = new float3(17.66f, 18.43f, 0.0f),
                [5] = new float3(-1.32f, 9.99f, 80.4f),
                [6] = new float3(-17.45f, 3.22f, -3.22f),
                [7] = new float3(0.0f, 0.0f, 0.03f)
            };

            var collider = ConvexCollider.Create(points, convexRadius: 0.15f);
            points.Dispose();

            float3 expectedInertiaTensor = new float3(434.014862f, 824.963989f, 684.776672f);
            float3 inertiaTensor = collider.Value.MassProperties.MassDistribution.InertiaTensor;

            // Given the number of FP operations, we do a percentage comparison in this case
            Assert.IsTrue(math.abs(expectedInertiaTensor.x - inertiaTensor.x) / expectedInertiaTensor.x < 0.01f);
            Assert.IsTrue(math.abs(expectedInertiaTensor.y - inertiaTensor.y) / expectedInertiaTensor.y < 0.01f);
            Assert.IsTrue(math.abs(expectedInertiaTensor.z - inertiaTensor.z) / expectedInertiaTensor.z < 0.01f);
        }

        #endregion

        #region IConvexCollider

        /// <summary>
        /// Test that the local AABB is computed correctly for a <see cref="ConvexCollider"/> created with a point cloud
        /// </summary>
        [Test]
        public void TestConvexColliderCalculateAabbLocal()
        {
            var points = new NativeArray<float3>(6, Allocator.Temp)
            {
                [0] = new float3(1.45f, 8.67f, 3.45f),
                [1] = new float3(8.75f, 1.23f, 6.44f),
                [2] = new float3(100.34f, 5.33f, -2.55f),
                [3] = new float3(8.76f, 4.56f, -4.54f),
                [4] = new float3(9.75f, -0.45f, -8.99f),
                [5] = new float3(7.66f, 3.44f, 0.0f)
            };
            float convexRadius = 1.25f;

            Aabb expectedAabb = Aabb.CreateFromPoints(new float3x4(points[0], points[1], points[2], points[3]));
            expectedAabb.Include(points[4]);
            expectedAabb.Include(points[5]);

            // Currently the convex hull is not shrunk, so we have to expand by the convex radius
            expectedAabb.Expand(convexRadius);

            var collider = ConvexCollider.Create(points, convexRadius);
            points.Dispose();

            Aabb actualAabb = collider.Value.CalculateAabb();
            TestUtils.AreEqual(expectedAabb.Min, actualAabb.Min, 1e-3f);
            TestUtils.AreEqual(expectedAabb.Max, actualAabb.Max, 1e-3f);
        }

        /// <summary>
        /// Test that the transformed AABB is computed correctly for a <see cref="ConvexCollider"/> created with a point cloud
        /// </summary>
        [Test]
        public void TestConvexColliderCalculateAabbTransformed()
        {
            var points = new NativeArray<float3>(6, Allocator.Temp)
            {
                [0] = new float3(1.45f, 8.67f, 3.45f),
                [1] = new float3(8.75f, 1.23f, 6.44f),
                [2] = new float3(100.34f, 5.33f, -2.55f),
                [3] = new float3(8.76f, 4.56f, -4.54f),
                [4] = new float3(9.75f, -0.45f, -8.99f),
                [5] = new float3(7.66f, 3.44f, 0.0f)
            };

            float convexRadius = 1.25f;
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

            // Currently the convex hull is not shrunk, so we have to expand by the convex radius
            expectedAabb.Expand(convexRadius);

            var collider = ConvexCollider.Create(points, convexRadius);
            points.Dispose();

            Aabb actualAabb = collider.Value.CalculateAabb(new RigidTransform(rotation, translation));

            TestUtils.AreEqual(expectedAabb.Min, actualAabb.Min, 1e-3f);
            TestUtils.AreEqual(expectedAabb.Max, actualAabb.Max, 1e-3f);
        }

        #endregion
    }
}

