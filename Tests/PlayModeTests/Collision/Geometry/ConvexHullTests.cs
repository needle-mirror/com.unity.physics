using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using Assert = UnityEngine.Assertions.Assert;

namespace Unity.Physics.Tests.Collision.Geometry
{
    public class ConvexHullTests
    {
        [Test]
        public void BuildConvexHull2D()
        {
            // Build circle.
            using (var hull = new ConvexHullBuilder(8192, 8192 * 2))
            {
                var expectedCom = new float3(4, 5, 3);
                for (int n = 1024, i = 0; i < n; ++i)
                {
                    var angle = (float)(i / n * 2 * System.Math.PI);
                    hull.AddPoint(new float3(math.cos(angle), math.sin(angle), 0));
                }
                var massProperties = hull.ComputeMassProperties();
                Debug.Log($"COM: {massProperties.CenterOfMass}");
                Debug.Log($"Area: {massProperties.SurfaceArea}");
            }
        }
    }
}
