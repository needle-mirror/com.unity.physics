using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Mathematics;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;
using TestUtils = Unity.Physics.Tests.Utils.TestUtils;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;

namespace Unity.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Contains all tests for the <see cref="SphereCollider"/>
    /// </summary>
    class SphereColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute() => SphereCollider.Create(new SphereGeometry { Radius = 1f }).Release();
        }

        [Test]
        public void SphereCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        /// <summary>
        /// Tests if a created <see cref="SphereCollider"/> has its attributes set correctly
        /// </summary>
        [Test]
        unsafe public void TestSphereColliderCreate()
        {
            var sphere = new SphereGeometry
            {
                Center = new float3(-8.45f, 9.65f, -0.10f),
                Radius = 0.98f
            };

            var collider = SphereCollider.Create(sphere);
            var sphereCollider = UnsafeUtilityEx.AsRef<SphereCollider>(collider.GetUnsafePtr());

            TestUtils.AreEqual(sphere.Center, sphereCollider.Center, 1e-3f);
            TestUtils.AreEqual(sphere.Center, sphereCollider.Geometry.Center, 1e-3f);
            TestUtils.AreEqual(sphere.Radius, sphereCollider.Radius, 1e-3f);
            TestUtils.AreEqual(sphere.Radius, sphereCollider.Geometry.Radius, 1e-3f);
            Assert.AreEqual(ColliderType.Sphere, sphereCollider.Type);
            Assert.AreEqual(CollisionType.Convex, sphereCollider.CollisionType);
        }

        /// <summary>
        /// Create invalid <see cref="SphereCollider"/>s and check that appropriate exceptions are being thrown
        /// </summary>
        [Test]
        public void TestSphereColliderCreateInvalid()
        {
            var sphere = new SphereGeometry
            {
                Center = new float3(-10.34f, 0.0f, -1.54f),
                Radius = 1.25f
            };

            // positive inf center
            {
                var invalidSphere = sphere;
                invalidSphere.Center = new float3(float.PositiveInfinity, 0.0f, 0.0f);
                TestUtils.ThrowsException<System.ArgumentException>(() => SphereCollider.Create(invalidSphere));
            }

            // negative inf center
            {
                var invalidSphere = sphere;
                invalidSphere.Center = new float3(float.NegativeInfinity, 0.0f, 0.0f);
                TestUtils.ThrowsException<System.ArgumentException>(() => SphereCollider.Create(invalidSphere));
            }

            // nan center
            {
                var invalidSphere = sphere;
                invalidSphere.Center = new float3(float.NaN, 0.0f, 0.0f);
                TestUtils.ThrowsException<System.ArgumentException>(() => SphereCollider.Create(invalidSphere));
            }

            // negative radius
            {
                var invalidSphere = sphere;
                invalidSphere.Radius = -0.5f;
                TestUtils.ThrowsException<System.ArgumentException>(() => SphereCollider.Create(invalidSphere));
            }

            // positive inf radius
            {
                var invalidSphere = sphere;
                invalidSphere.Radius = float.PositiveInfinity;
                TestUtils.ThrowsException<System.ArgumentException>(() => SphereCollider.Create(invalidSphere));
            }

            // negative inf radius
            {
                var invalidSphere = sphere;
                invalidSphere.Radius = float.NegativeInfinity;
                TestUtils.ThrowsException<System.ArgumentException>(() => SphereCollider.Create(invalidSphere));
            }

            // nan radius
            {
                var invalidSphere = sphere;
                invalidSphere.Radius = float.NaN;
                TestUtils.ThrowsException<System.ArgumentException>(() => SphereCollider.Create(invalidSphere));
            }
        }

        #endregion

        #region IConvexCollider

        /// <summary>
        /// Test that the local AABB of a <see cref="SphereCollider"/> are calculated correctly
        /// </summary>
        [Test]
        public void TestSphereColliderCalculateAabbLocal()
        {
            float3 center = new float3(-8.4f, 5.63f, -7.2f);
            float radius = 2.3f;
            var sphereCollider = SphereCollider.Create(new SphereGeometry { Center = center, Radius = radius });

            Aabb expected = new Aabb();
            expected.Min = center - new float3(radius, radius, radius);
            expected.Max = center + new float3(radius, radius, radius);

            Aabb actual = sphereCollider.Value.CalculateAabb();
            TestUtils.AreEqual(expected.Min, actual.Min, 1e-3f);
            TestUtils.AreEqual(expected.Max, actual.Max, 1e-3f);
        }

        /// <summary>
        /// Test that the AABB of a transformed <see cref="SphereCollider"/> is calculated correctly
        /// </summary>
        [Test]
        public void TestSphereColliderCalculateAabbTransformed()
        {
            float3 center = new float3(-3.4f, 0.63f, -17.2f);
            float radius = 5.3f;
            var sphereCollider = SphereCollider.Create(new SphereGeometry { Center = center, Radius = radius });

            float3 translation = new float3(8.3f, -0.5f, 170.0f);
            quaternion rotation = quaternion.AxisAngle(math.normalize(new float3(1.1f, 4.5f, 0.0f)), 146.0f);

            Aabb expected = new Aabb();
            expected.Min = math.mul(rotation, center) + translation - new float3(radius, radius, radius);
            expected.Max = math.mul(rotation, center) + translation + new float3(radius, radius, radius);

            Aabb actual = sphereCollider.Value.CalculateAabb(new RigidTransform(rotation, translation));
            TestUtils.AreEqual(expected.Min, actual.Min, 1e-3f);
            TestUtils.AreEqual(expected.Max, actual.Max, 1e-3f);

        }

        /// <summary>
        /// Test that the inertia tensor of a <see cref="SphereCollider"/> is calculated correctly
        /// </summary>
        /// <remarks>
        /// Inertia tensor formula taken from here: https://en.wikipedia.org/wiki/List_of_moments_of_inertia
        /// </remarks>
        [Test]
        public void TestSphereColliderMassProperties()
        {
            float3 center = new float3(-8.4f, 5.63f, 77.2f);
            float radius = 2.3f;
            var sphereCollider = SphereCollider.Create(new SphereGeometry { Center = center, Radius = radius });

            float inertia = 2.0f / 5.0f * radius * radius;
            float3 expectedInertiaTensor = new float3(inertia, inertia, inertia);
            float3 inertiaTensor = sphereCollider.Value.MassProperties.MassDistribution.InertiaTensor;
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, 1e-3f);
        }

        #endregion
    }
}
