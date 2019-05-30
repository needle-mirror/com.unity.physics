using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;
using TestUtils = Unity.Physics.Tests.Utils.TestUtils;
using Unity.Collections.LowLevel.Unsafe;

namespace Unity.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Class collecting all tests for the <see cref="CapsuleCollider"/>
    /// </summary>
    public class CapsuleColliderTests
    {
        #region Construction

        /// <summary>
        /// Test if all attributes are set as expected when creating a new <see cref="CapsuleCollider"/>.
        /// </summary>
        [Test]
        unsafe public void TestCapsuleColliderCreate()
        {
            float3 v0 = new float3(1.45f, 0.34f, -8.65f);
            float3 v1 = new float3(100.45f, -80.34f, -8.65f);
            float radius = 1.45f;
            var collider = CapsuleCollider.Create(v0, v1, radius);
            var capsuleCollider = UnsafeUtilityEx.AsRef<CapsuleCollider>(collider.GetUnsafePtr());
            Assert.AreEqual(ColliderType.Capsule, capsuleCollider.Type);
            Assert.AreEqual(CollisionType.Convex, capsuleCollider.CollisionType);
            TestUtils.AreEqual(v0, capsuleCollider.Vertex0);
            TestUtils.AreEqual(v1, capsuleCollider.Vertex1);
            TestUtils.AreEqual(radius, capsuleCollider.Radius);
        }

        /// <summary>
        /// Test if <see cref="CapsuleCollider"/> throws expected exceptions when created with invalid arguments
        /// </summary>
        [Test]
        public void TestCapsuleColliderCreateInvalid()
        {
            float3 v0 = new float3(5.66f, -6.72f, 0.12f);
            float3 v1 = new float3(0.98f, 8.88f, 9.54f);
            float radius = 0.65f;

            // v0, +inf
            {
                float3 invalidV0 = new float3(float.PositiveInfinity, 0.0f, 0.0f);
                TestUtils.ThrowsException<System.ArgumentException>(() => CapsuleCollider.Create(invalidV0, v1, radius));
            }

            // v0, -inf
            {
                float3 invalidV0 = new float3(float.NegativeInfinity, 0.0f, 0.0f);
                TestUtils.ThrowsException<System.ArgumentException>(() => CapsuleCollider.Create(invalidV0, v1, radius));
            }

            // v0, nan
            {
                float3 invalidV0 = new float3(float.NaN, 0.0f, 0.0f);
                TestUtils.ThrowsException<System.ArgumentException>(() => CapsuleCollider.Create(invalidV0, v1, radius));
            }

            // v1, +inf
            {
                float3 invalidV1 = new float3(float.PositiveInfinity, 0.0f, 0.0f);
                TestUtils.ThrowsException<System.ArgumentException>(() => CapsuleCollider.Create(v0, invalidV1, radius));
            }

            // v1, -inf
            {
                float3 invalidV1 = new float3(float.NegativeInfinity, 0.0f, 0.0f);
                TestUtils.ThrowsException<System.ArgumentException>(() => CapsuleCollider.Create(v0, invalidV1, radius));
            }

            // v1, nan
            {
                float3 invalidV1 = new float3(float.NaN, 0.0f, 0.0f);
                TestUtils.ThrowsException<System.ArgumentException>(() => CapsuleCollider.Create(v0, invalidV1, radius));
            }

            // negative radius
            {
                float invalidRadius = -0.54f;
                TestUtils.ThrowsException<System.ArgumentException>(() => CapsuleCollider.Create(v0, v1, invalidRadius));
            }

            // radius, +inf
            {
                float invalidRadius = float.PositiveInfinity;
                TestUtils.ThrowsException<System.ArgumentException>(() => CapsuleCollider.Create(v0, v1, invalidRadius));
            }

            // radius, -inf
            {
                float invalidRadius = float.NegativeInfinity;
                TestUtils.ThrowsException<System.ArgumentException>(() => CapsuleCollider.Create(v0, v1, invalidRadius));
            }

            // radius, nan
            {
                float invalidRadius = float.NaN;
                TestUtils.ThrowsException<System.ArgumentException>(() => CapsuleCollider.Create(v0, v1, invalidRadius));
            }
        }

        #endregion

        #region IConvexCollider

        /// <summary>
        /// Test if the local AABB of the <see cref="CapsuleCollider"/> is calculated correctly.
        /// </summary>
        [Test]
        public void TestCapsuleColliderCalculateAabbLocal()
        {
            float radius = 2.3f;
            float length = 5.5f;
            float3 p0 = new float3(1.1f, 2.2f, 3.4f);
            float3 p1 = p0 + length * math.normalize(new float3(1, 1, 1));
            var capsuleCollider = CapsuleCollider.Create(p0, p1, radius);

            Aabb expectedAabb = new Aabb();
            expectedAabb.Min = math.min(p0, p1) - new float3(radius);
            expectedAabb.Max = math.max(p0, p1) + new float3(radius);

            Aabb aabb = capsuleCollider.Value.CalculateAabb();
            TestUtils.AreEqual(expectedAabb.Min, aabb.Min, 1e-3f);
            TestUtils.AreEqual(expectedAabb.Max, aabb.Max, 1e-3f);
        }

        /// <summary>
        /// Test whether the AABB of the transformed <see cref="CapsuleCollider"/> is calculated correctly.
        /// </summary>
        [Test]
        public void TestCapsuleColliderCalculateAabbTransformed()
        {
            float radius = 2.3f;
            float length = 5.5f;
            float3 p0 = new float3(1.1f, 2.2f, 3.4f);
            float3 p1 = p0 + length * math.normalize(new float3(1, 1, 1));
            var capsuleCollider = CapsuleCollider.Create(p0, p1, radius);

            float3 translation = new float3(-3.4f, 0.5f, 0.0f);
            quaternion rotation = quaternion.AxisAngle(math.normalize(new float3(0.4f, 0.0f, 150.0f)), 123.0f);

            Aabb expectedAabb = new Aabb();
            float3 p0Transformed = math.mul(rotation, p0) + translation;
            float3 p1Transformed = math.mul(rotation, p1) + translation;
            expectedAabb.Min = math.min(p0Transformed, p1Transformed) - new float3(radius);
            expectedAabb.Max = math.max(p0Transformed, p1Transformed) + new float3(radius);

            Aabb aabb = capsuleCollider.Value.CalculateAabb(new RigidTransform(rotation, translation));
            TestUtils.AreEqual(expectedAabb.Min, aabb.Min, 1e-3f);
            TestUtils.AreEqual(expectedAabb.Max, aabb.Max, 1e-3f);
        }

        /// <summary>
        /// Test whether the inertia tensor of the <see cref="CapsuleCollider"/> is calculated correctly.
        /// </summary>
        /// <remarks>
        /// Used the formula from the following article as reference: https://www.gamedev.net/articles/programming/math-and-physics/capsule-inertia-tensor-r3856/
        /// NOTE: There is an error in eq. 14 of the article: it should be H^2 / 4 instead of H^2 / 2 in Ixx and Izz.
        /// </remarks>
        [Test]
        public void TestCapsuleColliderMassProperties()
        {
            float radius = 2.3f;
            float length = 5.5f;
            float3 p0 = new float3(1.1f, 2.2f, 3.4f);
            float3 p1 = p0 + length * math.normalize(new float3(1, 1, 1));

            float hemisphereMass = 0.5f * 4.0f / 3.0f * (float)math.PI * radius * radius * radius;
            float cylinderMass = (float)math.PI * radius * radius * length;
            float totalMass = 2.0f * hemisphereMass + cylinderMass;
            hemisphereMass /= totalMass;
            cylinderMass /= totalMass;

            float itX = cylinderMass * (length * length / 12.0f + radius * radius / 4.0f) + 2.0f * hemisphereMass * (2.0f * radius * radius / 5.0f + length * length / 4.0f + 3.0f * length * radius / 8.0f);
            float itY = cylinderMass * radius * radius / 2.0f + 4.0f * hemisphereMass * radius * radius / 5.0f;
            float itZ = itX;
            float3 expectedInertiaTensor = new float3(itX, itY, itZ);

            var capsuleCollider = CapsuleCollider.Create(p0, p1, radius);
            float3 inertiaTensor = capsuleCollider.Value.MassProperties.MassDistribution.InertiaTensor;
            TestUtils.AreEqual(expectedInertiaTensor, inertiaTensor, 1e-3f);
        }

        #endregion
    }
}
