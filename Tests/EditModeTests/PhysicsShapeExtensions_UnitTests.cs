using System;
using NUnit.Framework;
using Unity.Mathematics;
using Unity.Physics.Authoring;

namespace Unity.Physics.Tests.Authoring
{
    class PhysicsShapeExtensions_UnitTests
    {
        static readonly TestCaseData[] k_DeviantAxisTestCases =
        {
            new TestCaseData(new float3( 0f,  1f, 1f)).Returns(0).SetName("Smallest axis, other axes identical"),
            new TestCaseData(new float3( 1f,  2f, 1f)).Returns(1).SetName("Largest axis, other axes identical"),
            new TestCaseData(new float3( 5f,  8f, 1f)).Returns(2).SetName("Smallest axis, other axes differ"),
            new TestCaseData(new float3( 9f,  2f, 3f)).Returns(0).SetName("Largest axis, other axes differ"),
            new TestCaseData(new float3(-1f, -1f, 1f)).Returns(2).SetName("Only positive axis, other axes identical"),
            new TestCaseData(new float3( 1f, -2f, 1f)).Returns(1).SetName("Only negative axis, other axes identical")
        };

        [TestCaseSource(nameof(k_DeviantAxisTestCases))]
        public int GetDeviantAxis_ReturnsTheMostDifferentAxis(float3 v)
        {
            return PhysicsShapeExtensions.GetDeviantAxis(v);
        }

        static readonly TestCaseData[] k_MaxAxisTestCases =
        {
            new TestCaseData(new float3( 3f,  2f,  1f)).Returns(0).SetName("X-axis (all positive)"),
            new TestCaseData(new float3( 3f,  2f, -4f)).Returns(0).SetName("X-axis (one negative with greater magnitude)"),
            new TestCaseData(new float3( 2f,  3f,  1f)).Returns(1).SetName("Y-axis (all positive)"),
            new TestCaseData(new float3(-4f,  3f,  2f)).Returns(1).SetName("Y-axis (one negative with greater magnitude)"),
            new TestCaseData(new float3( 1f,  2f,  3f)).Returns(2).SetName("Z-axis (all positive)"),
            new TestCaseData(new float3( 2f, -4f,  3f)).Returns(2).SetName("Z-axis (one negative with greater magnitude)"),
        };

        [TestCaseSource(nameof(k_MaxAxisTestCases))]
        public int GetMaxAxis_ReturnsLargestAxis(float3 v)
        {
            return PhysicsShapeExtensions.GetMaxAxis(v);
        }

        const float k_Small = math.FLT_MIN_NORMAL;
        const float k_Large = float.MaxValue;

        static readonly float4x4 k_Orthogonal = new float4x4(quaternion.Euler(math.PI / 8f, math.PI / 8f, math.PI / 8f), 0f);

        static readonly float3 k_NonUniformScale = new float3(1f, 2f, 3f);

        static readonly float4x4 k_Sheared = math.mul(float4x4.Scale(k_NonUniformScale), k_Orthogonal);

        static readonly TestCaseData[] k_ShearTestCases =
        {
            new TestCaseData(new float4x4()).Returns(false)
                .SetName("Orthogonal (Zero)"),
            new TestCaseData(float4x4.identity).Returns(false)
                .SetName("Orthogonal (Identity)"),
            new TestCaseData(k_Orthogonal).Returns(false)
                .SetName("Orthogonal (Rotated)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_NonUniformScale))).Returns(false)
                .SetName("Orthogonal (Rotated and scaled)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_NonUniformScale * new float3(0f, 1f, 1f)))).Returns(false)
                .SetName("Orthogonal (Rotated and zero scale on one axis)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_NonUniformScale * new float3(0f, 0f, 1f)))).Returns(false)
                .SetName("Orthogonal (Rotated and zero scale on two axes)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_Small))).Returns(false)
                .SetName("Orthogonal (Rotated and small scale)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_Large))).Returns(false)
                .SetName("Orthogonal (Rotated and large scale)"),
            new TestCaseData(math.mul(k_Orthogonal, float4x4.Scale(k_Small, 1f, k_Large))).Returns(false)
                .SetName("Orthogonal (Rotated and large difference between two axis scales)"),
            new TestCaseData(k_Sheared).Returns(true)
                .SetName("Sheared"),
            new TestCaseData(math.mul(k_Sheared, float4x4.Scale(k_Small))).Returns(true)
                .SetName("Sheared (Small scale)"),
            new TestCaseData(math.mul(k_Sheared, float4x4.Scale(k_Large / math.cmax(k_NonUniformScale)))).Returns(true)
                .SetName("Sheared (Large scale)"),
            new TestCaseData(math.mul(k_Sheared, float4x4.Scale(k_Small, 1f, k_Large / math.cmax(k_NonUniformScale)))).Returns(true)
                .SetName("Sheared (Large difference between two axis scales)")
        };

        [TestCaseSource(nameof(k_ShearTestCases))]
        public bool HasShear_ReturnsShearedState(float4x4 m) => m.HasShear();
    }
}