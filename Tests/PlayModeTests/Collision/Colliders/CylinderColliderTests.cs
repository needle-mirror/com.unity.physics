using System;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics.Extensions;
using Unity.Physics.Tests.Utils;

namespace Unity.Physics.Tests.Collision.Colliders
{
    class CylinderColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            [GenerateTestsForBurstCompatibility]
            public void Execute() => CylinderCollider.Create(new CylinderGeometry
            {
                Orientation = quaternion.identity,
                Height = 1f,
                Radius = 1f,
                SideCount = CylinderGeometry.MaxSideCount
            }).Dispose();
        }

        [Test]
        public void CylinderCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        unsafe void ValidateCylinderCollider(BlobAssetReference<Collider> collider, in CylinderGeometry geometry)
        {
            // manually created colliders are unique by design
            Assert.IsTrue(collider.Value.IsUnique);

            Assert.AreEqual(ColliderType.Cylinder, collider.Value.Type);
            Assert.AreEqual(CollisionType.Convex, collider.Value.CollisionType);

            ref var cylinderCollider = ref UnsafeUtility.AsRef<CylinderCollider>(collider.GetUnsafePtr());

            Assert.AreEqual(geometry.Center, cylinderCollider.Center);
            Assert.AreEqual(geometry.Center, cylinderCollider.Geometry.Center);
            Assert.AreEqual(geometry.Orientation, cylinderCollider.Orientation);
            Assert.AreEqual(geometry.Orientation, cylinderCollider.Geometry.Orientation);
            Assert.AreEqual(geometry.Height, cylinderCollider.Height);
            Assert.AreEqual(geometry.Height, cylinderCollider.Geometry.Height);
            Assert.AreEqual(geometry.Radius, cylinderCollider.Radius);
            Assert.AreEqual(geometry.Radius, cylinderCollider.Geometry.Radius);
            Assert.AreEqual(geometry.BevelRadius, cylinderCollider.BevelRadius);
            Assert.AreEqual(geometry.BevelRadius, cylinderCollider.Geometry.BevelRadius);
            Assert.AreEqual(CollisionType.Convex, cylinderCollider.CollisionType);
            Assert.AreEqual(ColliderType.Cylinder, cylinderCollider.Type);
        }

        [Test]
        public void CylinderCollider_Create_ResultHasExpectedValues()
        {
            var geometry = new CylinderGeometry
            {
                Center = new float3(-10.10f, 10.12f, 0.01f),
                Orientation = quaternion.AxisAngle(math.normalize(new float3(1.4f, 0.2f, 1.1f)), 38.50f),
                Height = 2f,
                Radius = 0.25f,
                BevelRadius = 0.05f,
                SideCount = 10
            };

            using var collider = CylinderCollider.Create(geometry);
            ValidateCylinderCollider(collider, geometry);
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void CylinderCollider_Create_WhenCenterInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN)] float errantValue
        )
        {
            var geometry = new CylinderGeometry
            {
                Center = new float3(errantValue),
                Orientation = quaternion.identity,
                SideCount = CylinderGeometry.MaxSideCount
            };

            var ex = Assert.Throws<ArgumentException>(() => CylinderCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(CylinderGeometry.Center)));
        }

        [Test]
        public void CylinderCollider_Create_WhenOrientationInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, 0f)] float errantValue
        )
        {
            var geometry = new CylinderGeometry
            {
                Orientation = new quaternion(0f, 0f, 0f, errantValue),
                SideCount = CylinderGeometry.MaxSideCount
            };

            var ex = Assert.Throws<ArgumentException>(() => CylinderCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(CylinderGeometry.Orientation)));
        }

        [Test]
        public void CylinderCollider_Create_WhenHeightInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f)] float errantValue
        )
        {
            var geometry = new CylinderGeometry
            {
                Height = errantValue,
                Orientation = quaternion.identity,
                SideCount = CylinderGeometry.MaxSideCount
            };

            var ex = Assert.Throws<ArgumentException>(() => CylinderCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(CylinderGeometry.Height)));
        }

        [Test]
        public void CylinderCollider_Create_WhenRadiusInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f)] float errantValue
        )
        {
            var geometry = new CylinderGeometry
            {
                Radius = errantValue,
                Orientation = quaternion.identity,
                SideCount = CylinderGeometry.MaxSideCount
            };

            var ex = Assert.Throws<ArgumentException>(() => CylinderCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(CylinderGeometry.Radius)));
        }

        [Test]
        public void CylinderCollider_Create_WhenBevelRadiusInvalid_Throws(
            [Values(float.PositiveInfinity, float.NegativeInfinity, float.NaN, -1f, 0.55f)] float errantValue
        )
        {
            var geometry = new CylinderGeometry
            {
                Height = 1f,
                Radius = 0.5f,
                Orientation = quaternion.identity,
                BevelRadius = errantValue,
                SideCount = CylinderGeometry.MaxSideCount
            };

            var ex = Assert.Throws<ArgumentException>(() => CylinderCollider.Create(geometry));
            Assert.That(ex.Message, Does.Match(nameof(CylinderGeometry.BevelRadius)));
        }

#endif

        #endregion

        #region Modification

        void ValidateGeometryEqual(in CylinderGeometry expectedGeometry, in CylinderGeometry actualGeometry)
        {
            const float kDelta = 1e-5f;
            TestUtils.AreEqual(expectedGeometry.Center, actualGeometry.Center, kDelta);
            Assert.That(expectedGeometry.Orientation, Is.OrientedEquivalentTo(actualGeometry.Orientation));
            TestUtils.AreEqual(expectedGeometry.Radius, actualGeometry.Radius, kDelta);
            TestUtils.AreEqual(expectedGeometry.Height, actualGeometry.Height, kDelta);
            TestUtils.AreEqual(expectedGeometry.BevelRadius, actualGeometry.BevelRadius, kDelta);
            TestUtils.AreEqual(expectedGeometry.SideCount, actualGeometry.SideCount);
        }

        /// <summary>
        /// Modify a <see cref="CylinderCollider"/> by baking a user-provided transformation matrix,
        /// containing translation, rotation and scale, into its geometry.
        /// </summary>
        [Test]
        public void TestCylinderColliderBakeTransformTRS([Values(0, 1, 2)] int axisID)
        {
            var geometry = new CylinderGeometry
            {
                Center = new float3(1, 2, 4.2f),
                Orientation = quaternion.LookRotationSafe(new float3 {[axisID] = 1f}, new float3 {[Math.Constants.PrevAxis[axisID]] = 1f}),
                Radius = 0.5f,
                Height = 4.2f,
                BevelRadius = 0.01f,
                SideCount = CylinderGeometry.MaxSideCount
            };

            using var collider = CylinderCollider.Create(geometry);
            ref var cylinderCollider = ref collider.As<CylinderCollider>();

            var translation = new float3(1, 2, 4.2f);
            var rotation = quaternion.Euler(math.radians(10), math.radians(30), math.radians(42));
            var scale = new float3(1.5f, 2.5f, 4.2f);
            var transform = new AffineTransform(translation, rotation, scale);
            collider.Value.BakeTransform(transform);

            var nonAxisScales = new float2(scale[Math.Constants.NextAxis[axisID]], scale[Math.Constants.PrevAxis[axisID]]);
            var expectedRadius = geometry.Radius * math.cmax(nonAxisScales);
            var expectedHeight = geometry.Height * scale[axisID];
            var expectedGeometry = new CylinderGeometry
            {
                Center = math.transform(transform, geometry.Center),
                Orientation = math.mul(rotation, geometry.Orientation),
                Radius = expectedRadius,
                Height = expectedHeight,
                BevelRadius = geometry.BevelRadius,
                SideCount = geometry.SideCount
            };

            ValidateGeometryEqual(expectedGeometry, cylinderCollider.Geometry);

            // Test that the mass properties are as expected
            using var expectedCollider = CylinderCollider.Create(expectedGeometry);
            TestUtils.AreEqual(expectedCollider.Value.MassProperties, cylinderCollider.MassProperties, 1e-4f);
        }

        /// <summary>
        /// Modify a <see cref="CylinderCollider"/> by baking a user-provided transformation matrix,
        /// containing translation and rotation, into its geometry.
        /// </summary>
        [Test]
        public void TestCylinderColliderBakeTransformTR()
        {
            var geometry = new CylinderGeometry
            {
                Center = new float3(1, 2, 4.2f),
                Orientation = quaternion.Euler(math.radians(20), math.radians(42), math.radians(50)),
                Radius = 0.5f,
                Height = 4.2f,
                BevelRadius = 0.01f,
                SideCount = CylinderGeometry.MaxSideCount
            };

            using var collider = CylinderCollider.Create(geometry);
            ref var cylinderCollider = ref collider.As<CylinderCollider>();

            var translation = new float3(1, 2, 4.2f);
            var rotation = quaternion.Euler(math.radians(10), math.radians(30), math.radians(42));
            var transform = new AffineTransform(translation, rotation);
            collider.Value.BakeTransform(transform);

            var expectedGeometry = new CylinderGeometry
            {
                Center = math.transform(transform, geometry.Center),
                Orientation = math.mul(rotation, geometry.Orientation),
                Radius = geometry.Radius,
                Height = geometry.Height,
                BevelRadius = geometry.BevelRadius,
                SideCount = geometry.SideCount
            };

            ValidateGeometryEqual(expectedGeometry, cylinderCollider.Geometry);

            // Test that the mass properties are as expected
            using var expectedCollider = CylinderCollider.Create(expectedGeometry);
            TestUtils.AreEqual(expectedCollider.Value.MassProperties, cylinderCollider.MassProperties, 1e-4f);
        }

        #endregion

        #region Utilities

        [Test]
        public void TestCylinderColliderToMesh()
        {
            var geometry = new CylinderGeometry()
            {
                Center = new float3(1, 2, 3),
                Orientation = quaternion.identity,
                Height = 4,
                Radius = 1,
                SideCount = CylinderGeometry.MaxSideCount
            };

            using var cylinderCollider = CylinderCollider.Create(geometry);

            var expectedSize = new float3(geometry.Radius * 2, geometry.Radius * 2, geometry.Height);
            var aabb = cylinderCollider.Value.CalculateAabb(RigidTransform.identity);
            TestUtils.AreEqual(geometry.Center, aabb.Center, math.EPSILON);
            TestUtils.AreEqual(expectedSize, aabb.Extents, math.EPSILON);

            var mesh = cylinderCollider.Value.ToMesh();
            TestUtils.AreEqual(geometry.Center, mesh.bounds.center, math.EPSILON);
            TestUtils.AreEqual(expectedSize, mesh.bounds.size, math.EPSILON);

            UnityEngine.Object.DestroyImmediate(mesh);
        }

        #endregion
    }
}
