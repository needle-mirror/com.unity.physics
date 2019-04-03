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
    }
}