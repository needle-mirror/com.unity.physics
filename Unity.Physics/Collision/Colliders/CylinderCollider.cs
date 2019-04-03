using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.Physics
{
    public static class CylinderCollider
    {
        // Create a cylindrical convex hull with its height axis oriented along z-axis.
        public static unsafe BlobAssetReference<Collider> Create(
            float3 center, float height, float radius, quaternion orientation, float convexRadius,
            float3 scale, CollisionFilter filter, Material material
        )
        {
            // resolution of shape is limited by max number of coplanar contact points on each end of the shape
            const int k_NumPoints = ConvexConvexManifoldQueries.Manifold.k_MaxNumContacts * 2;
            var pointCloud = new NativeArray<float3>(k_NumPoints, Allocator.Temp);
            var arcStep = (float)(2f * math.PI / (k_NumPoints / 2));
            var halfHeight = height * 0.5f;
            for (var i = 0; i < k_NumPoints; i += 2)
            {
                var x = math.cos(arcStep * i) * radius;
                var y = math.sin(arcStep * i) * radius;
                pointCloud[i] = center + math.mul(orientation, new float3(x, y, -halfHeight));
                pointCloud[i + 1] = center + math.mul(orientation, new float3(x, y, halfHeight));
            }

            var collider = ConvexCollider.Create(pointCloud, convexRadius, scale, filter, material);
            pointCloud.Dispose();
            return collider;
        }
    }
}
