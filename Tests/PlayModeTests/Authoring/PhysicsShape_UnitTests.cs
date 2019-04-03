using NUnit.Framework;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEngine;

namespace Unity.Physics.Tests.Authoring
{
    class PhysicsShape_UnitTests
    {
        const float k_Tolerance = 0.001f;

        PhysicsShape m_Shape;

        [SetUp]
        public void SetUp() => m_Shape = new GameObject("Shape").AddComponent<PhysicsShape>();

        [TearDown]
        public void TearDown()
        {
            if (m_Shape != null)
                GameObject.DestroyImmediate(m_Shape.gameObject);
        }

        [Test]
        public void SetBoxProperties_WithSizeLessThanZero_ClampsToZero()
        {
            m_Shape.SetBox(0f, -3f, quaternion.identity);

            m_Shape.GetBoxProperties(out var center, out var size, out quaternion orientation);

            Assert.That(size, Is.EqualTo(new float3(0f)));
        }

        [Test]
        public void GetCapsuleProperties_WhenShapeIsBox_HeightIsMaxDimension(
            [Values(0f, 1f, 2f, 3f)]float sizeX,
            [Values(0f, 1f, 2f, 3f)]float sizeY,
            [Values(0f, 1f, 2f, 3f)]float sizeZ
        )
        {
            var size = new float3(sizeX, sizeY, sizeZ);
            m_Shape.SetBox(0f, size, quaternion.identity);

            m_Shape.GetCapsuleProperties(out var center, out var height, out var radius, out quaternion orientation);

            Assert.That(height, Is.EqualTo(math.cmax(size)));
        }

        [Test]
        public void GetCapsuleProperties_WhenShapeIsBox_RadiusIsHalfSecondMaxDimension(
            [Values(0f, 1f, 2f, 3f)]float sizeX,
            [Values(0f, 1f, 2f, 3f)]float sizeY,
            [Values(0f, 1f, 2f, 3f)]float sizeZ
        )
        {
            var size = new float3(sizeX, sizeY, sizeZ);
            m_Shape.SetBox(0f, size, quaternion.identity);

            m_Shape.GetCapsuleProperties(out var center, out var height, out var radius, out quaternion orientation);

            var cmaxI = size.GetMaxAxis();
            var expectedRadius = 0.5f * math.cmax(cmaxI == 0 ? size.yz : cmaxI == 1 ? size.xz : size.xy);
            Assert.That(radius, Is.EqualTo(expectedRadius));
        }

        static readonly TestCaseData[] k_CapsuleOrientationTestCases =
        {
            new TestCaseData(new float3(2f, 1f, 1f), new float3(1f, 0f, 0f)).SetName("Aligned to x-axis"),
            new TestCaseData(new float3(1f, 2f, 1f), new float3(0f, 1f, 0f)).SetName("Aligned to y-axis"),
            new TestCaseData(new float3(1f, 1f, 2f), new float3(0f, 0f, 1f)).SetName("Aligned to z-axis")
        };
        [TestCaseSource(nameof(k_CapsuleOrientationTestCases))]
        public void GetCapsuleProperties_WhenShapeIsElongatedBox_OrientationPointsDownLongAxis(float3 boxSize, float3 expectedLookVector)
        {
            m_Shape.SetBox(0f, boxSize, quaternion.identity);

            m_Shape.GetCapsuleProperties(out var center, out var height, out var radius, out quaternion orientation);

            var lookVector = math.mul(orientation, new float3(0f, 0f, 1f));
            Assert.That(
                math.dot(lookVector, expectedLookVector), Is.EqualTo(1f).Within(k_Tolerance),
                $"Expected {expectedLookVector} but got {lookVector}"
            );
        }

        [Test]
        public void GetCylinderProperties_WhenShapeIsBox_HeightIsDeviantDimension(
            [Values(0f, 1f, 2f, 3f)]float sizeX,
            [Values(0f, 1f, 2f, 3f)]float sizeY,
            [Values(0f, 1f, 2f, 3f)]float sizeZ
        )
        {
            var size = new float3(sizeX, sizeY, sizeZ);
            m_Shape.SetBox(0f, size, quaternion.identity);

            m_Shape.GetCylinderProperties(out var center, out var height, out var radius, out quaternion orientation);

            var heightAxis = size.GetDeviantAxis();
            Assert.That(height, Is.EqualTo(size[heightAxis]));
        }

        [Test]
        public void GetCylinderProperties_WhenShapeIsBox_RadiusIsHalfMaxHomogenousDimension(
            [Values(0f, 1f, 2f, 3f)]float sizeX,
            [Values(0f, 1f, 2f, 3f)]float sizeY,
            [Values(0f, 1f, 2f, 3f)]float sizeZ
        )
        {
            var size = new float3(sizeX, sizeY, sizeZ);
            m_Shape.SetBox(0f, size, quaternion.identity);

            m_Shape.GetCylinderProperties(out var center, out var height, out var radius, out quaternion orientation);

            var heightAxis = size.GetDeviantAxis();
            var expectedRadius = 0.5f * math.cmax(heightAxis == 0 ? size.yz : heightAxis == 1 ? size.xz : size.xy);
            Assert.That(radius, Is.EqualTo(expectedRadius));
        }

        [Test]
        public void GetSphereProperties_WhenShapeIsBox_RadiusIsHalfMaxDimension(
            [Values(0f, 1f, 2f, 3f)] float sizeX,
            [Values(0f, 1f, 2f, 3f)] float sizeY,
            [Values(0f, 1f, 2f, 3f)] float sizeZ
        )
        {
            var size = new float3(sizeX, sizeY, sizeZ);
            m_Shape.SetBox(0f, size, quaternion.identity);

            m_Shape.GetSphereProperties(out var center, out var radius, out quaternion orientation);

            var expectedRadius = 0.5f * math.cmax(size);
            Assert.That(radius, Is.EqualTo(expectedRadius));
        }

        static readonly TestCaseData[] k_PlaneSizeTestCases =
        {
            new TestCaseData(new float3(2f, 3f, 1f), 0, 1).SetName("xy"),
            new TestCaseData(new float3(2f, 1f, 3f), 0, 2).SetName("xz"),
            new TestCaseData(new float3(1f, 2f, 3f), 1, 2).SetName("yz")
        };

        [TestCaseSource(nameof(k_PlaneSizeTestCases))]
        public void GetPlaneProperties_WhenShapeIsBox_SizeIsTwoGreatestDimensions(float3 boxSize, int ax1, int ax2)
        {
            m_Shape.SetBox(0f, boxSize, quaternion.identity);

            m_Shape.GetPlaneProperties(out var center, out var size, out quaternion orientation);

            Assert.That(
                new[] { size.x, size.y }, Is.EquivalentTo(new[] { boxSize[ax1], boxSize[ax2] }),
                "Plane dimensions did not match two greatest dimensions of original box"
            );
        }

        static readonly TestCaseData[] k_PlaneOrientationTestCases =
        {
            new TestCaseData(new float3(3f, 2f, 1f), quaternion.LookRotation(new float3(1f, 0f, 0f), new float3(0f, 0f, 1f))).SetName("look x, up z"),
            new TestCaseData(new float3(2f, 3f, 1f), quaternion.LookRotation(new float3(0f, 1f, 0f), new float3(0f, 0f, 1f))).SetName("look y, up z"),
            new TestCaseData(new float3(3f, 1f, 2f), quaternion.LookRotation(new float3(1f, 0f, 0f), new float3(0f, 1f, 0f))).SetName("look x, up y"),
            new TestCaseData(new float3(2f, 1f, 3f), quaternion.LookRotation(new float3(0f, 0f, 1f), new float3(0f, 1f, 0f))).SetName("look z, up y"),
            new TestCaseData(new float3(1f, 3f, 2f), quaternion.LookRotation(new float3(0f, 1f, 0f), new float3(1f, 0f, 0f))).SetName("look y, up x"),
            new TestCaseData(new float3(1f, 2f, 3f), quaternion.LookRotation(new float3(0f, 0f, 1f), new float3(1f, 0f, 0f))).SetName("look z, up x")
        };

        [TestCaseSource(nameof(k_PlaneOrientationTestCases))]
        public void GetPlaneProperties_WhenShapeIsBox_OrientationPointsDownLongAxisUpFlatAxis(float3 boxSize, quaternion expected)
        {
            m_Shape.SetBox(0f, boxSize, quaternion.identity);

            m_Shape.GetPlaneProperties(out var center, out var size, out quaternion orientation);

            Assert.That(
                math.abs(math.dot(orientation, expected)), Is.EqualTo(1f).Within(k_Tolerance),
                $"Expected {expected} but got {orientation}"
            );
        }
    }
}
