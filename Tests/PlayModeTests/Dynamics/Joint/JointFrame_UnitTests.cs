using NUnit.Framework;
using Unity.Mathematics;

namespace Unity.Physics.Tests.Joints
{
    class JointFrame_UnitTests
    {
        static readonly float3 k_XAxis = new float3(1f, 0f, 0f);
        static readonly float3 k_YAxis = new float3(0f, 1f, 0f);
        static readonly float3x2 k_DefaultAxes = new float3x2(k_XAxis, k_YAxis);

        static readonly TestCaseData[] k_ValidateAxesTestCases =
        {
            new TestCaseData(JointFrame.Identity)
                .Returns(k_DefaultAxes)
                .SetName("Identity => Default axes"),
            new TestCaseData(default(JointFrame))
                .Returns(k_DefaultAxes)
                .SetName("Both axes uninitialized => Default axes"),
            new TestCaseData(new JointFrame { Axis = k_XAxis, PerpendicularAxis = k_XAxis })
                .Returns(k_DefaultAxes)
                .SetName("Both axes default X => Default axes"),
            new TestCaseData(new JointFrame { Axis = k_XAxis, PerpendicularAxis = default })
                .Returns(k_DefaultAxes)
                .SetName("Axis default X, perpendicular uninitialized => Default axes"),
            new TestCaseData(new JointFrame { Axis = default, PerpendicularAxis = k_XAxis })
                .Returns(k_DefaultAxes)
                .SetName("Axis uninitialized, perpendicular default X => Default axes")
        };

        [TestCaseSource(nameof(k_ValidateAxesTestCases))]
        public float3x2 ValidateAxes_ReturnsExpectedValue(JointFrame jointFrame)
        {
            var validatedAxes = jointFrame.ValidateAxes();
            return new float3x2(validatedAxes.c0, validatedAxes.c1);
        }

        static readonly TestCaseData[] k_ValidateAxesPerpendicularTestCases =
        {
            new TestCaseData(default(float3)).SetName("Perpendicular input is zero"),
            new TestCaseData(k_XAxis).SetName("Perpendicular input is perpendicular to axis"),
            new TestCaseData(k_YAxis).SetName("Perpendicular input is parallel to axis")
        };

        [TestCaseSource(nameof(k_ValidateAxesPerpendicularTestCases))]
        public void ValidateAxes_WhenAxisDefaultY_PerpendicularAxisIsPerpendicular(float3 perpendicularAxis)
        {
            var jointFrame = new JointFrame { Axis = k_YAxis, PerpendicularAxis = perpendicularAxis };

            var validatedAxes = jointFrame.ValidateAxes();

            Assume.That(math.length(validatedAxes.c2), Is.EqualTo(1f).Within(0.0001f));
            var dot = math.dot(validatedAxes.c0, validatedAxes.c1);
            Assert.That(dot, Is.EqualTo(0f).Within(0.0001f));
        }
    }
}
