using System;
using NUnit.Framework;
using Unity.Mathematics;

namespace Unity.Physics.Tests.Collision.Queries
{
    class RayCastTests
    {
        [Test]
        public void RayVsTriangle()
        {
            // triangle
            var v1 = new float3(-1, -1, 0);
            var v2 = new float3(0, 1, 0);
            var v3 = new float3(1, -1, 0);

            {
                var origin = new float3(0, 0, -2);
                var direction = new float3(0, 0, 4);

                float fraction = 1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
                Assert.IsTrue(fraction == 0.5);
            }

            {
                var origin = new float3(0, 0, 2);
                var direction = new float3(0, 0, -4);

                float fraction = 1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
                Assert.IsTrue(fraction == 0.5);
            }

            {
                var origin = new float3(1, -1, -2);
                var direction = new float3(0, 0, 4);

                float fraction = 1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
                Assert.IsTrue(fraction == 0.5);
            }

            {
                var origin = new float3(2, 0, -2);
                var direction = new float3(0, 0, 4);

                float fraction = 1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsFalse(hit);
            }

            {
                var origin = new float3(2, 0, -2);
                var direction = new float3(0, 0, -4);

                float fraction = 1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsFalse(hit);
                Assert.IsTrue(math.all(normal == new float3(0, 0, 0)));
            }

            {
                v1 = new float3(-4, 0, 0);
                v2 = new float3(-5, 0, -1);
                v3 = new float3(-4, 0, -1);

                var origin = new float3(-4.497f, 0.325f, -0.613f);
                var direction = new float3(0f, -10f, 0f);

                float fraction = 1;
                bool hit = RaycastQueries.RayTriangle(origin, direction, v1, v2, v3, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
            }
        }

        [Test]
        public void RayVsCapsule([Values(0.0f, 1.0f)] float height)
        {
            // When height is 1, creates default capsule (radius 0.5, height 2)
            // When height is 0, creates sphere (radius 0.5)
            var vertex0 = new float3(0, -height / 2, 0);
            var vertex1 = new float3(0, height / 2, 0);
            float radius = 0.5f;

            var capsule = new CapsuleGeometry
            {
                Vertex0 = vertex0,
                Vertex1 = vertex1,
                Radius = radius
            };

            {
                // Ray from front hitting capsule/sphere
                var origin = new float3(0, 0, -2);
                var direction = new float3(0, 0, 4);

                float fraction = 1;
                bool hit = RaycastQueries.RayCapsule(origin, direction, capsule.Vertex0, capsule.Vertex1, capsule.Radius, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
                Assert.IsTrue(math.abs(fraction - 0.375f) < 1e-3f); // Should hit at distance 1.5 (fraction = 1.5/4)
            }

            {
                // Ray from back hitting capsule/sphere
                var origin = new float3(0, 0, 2);
                var direction = new float3(0, 0, -4);

                float fraction = 1;
                bool hit = RaycastQueries.RayCapsule(origin, direction, capsule.Vertex0, capsule.Vertex1, capsule.Radius, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
                Assert.IsTrue(math.abs(fraction - 0.375f) < 1e-3f);
            }

            {
                // Ray missing capsule/sphere (too far to the side)
                var origin = new float3(2, 0, -2);
                var direction = new float3(0, 0, 4);

                float fraction = 1;
                bool hit = RaycastQueries.RayCapsule(origin, direction, capsule.Vertex0, capsule.Vertex1, capsule.Radius, ref fraction, out float3 normal);
                Assert.IsFalse(hit);
            }

            {
                // Ray from above
                var origin = new float3(0, 2, 0);
                var direction = new float3(0, -4, 0);

                float fraction = 1;
                bool hit = RaycastQueries.RayCapsule(origin, direction, capsule.Vertex0, capsule.Vertex1, capsule.Radius, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
            }

            {
                // Ray from below
                var origin = new float3(0, -2, 0);
                var direction = new float3(0, 4, 0);

                float fraction = 1;
                bool hit = RaycastQueries.RayCapsule(origin, direction, capsule.Vertex0, capsule.Vertex1, capsule.Radius, ref fraction, out float3 normal);
                Assert.IsTrue(hit);
            }
        }
    }
}
