using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Unity.Mathematics;

namespace Unity.Physics
{
    // An axis aligned bounding box
    [DebuggerDisplay("{Min} - {Max}")]
    [Serializable]
    public struct Aabb
    {
        public float3 Min;
        public float3 Max;

        public float3 Extents => Max - Min;
        public float3 Center => (Max + Min) * 0.5f;
        public bool IsValid => math.all(Min <= Max);

        // Create an empty, invalid AABB
        public static readonly Aabb Empty = new Aabb { Min = Math.Constants.Max3F, Max = Math.Constants.Min3F };

        public float SurfaceArea
        {
            get
            {
                float3 diff = Max - Min;
                return 2 * math.dot(diff, diff.yzx);
            }
        }

        public static Aabb Union(Aabb a, Aabb b)
        {
            a.Include(b);
            return a;
        }

        [DebuggerStepThrough]
        public void Include(float3 point)
        {
            Min = math.min(Min, point);
            Max = math.max(Max, point);
        }

        [DebuggerStepThrough]
        public void Include(Aabb aabb)
        {
            Min = math.min(Min, aabb.Min);
            Max = math.max(Max, aabb.Max);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(float3 point) => math.all(point >= Min & point <= Max);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(Aabb aabb) => math.all((Min <= aabb.Min) & (Max >= aabb.Max));

        public void Expand(float distance)
        {
            Min -= distance;
            Max += distance;
        }

        public static Aabb CreateFromPoints(float3x4 points)
        {
            Aabb aabb;
            aabb.Min = points.c0;
            aabb.Max = aabb.Min;

            aabb.Min = math.min(aabb.Min, points.c1);
            aabb.Max = math.max(aabb.Max, points.c1);

            aabb.Min = math.min(aabb.Min, points.c2);
            aabb.Max = math.max(aabb.Max, points.c2);

            aabb.Min = math.min(aabb.Min, points.c3);
            aabb.Max = math.max(aabb.Max, points.c3);

            return aabb;
        }

        public bool Overlaps(Aabb other)
        {
            return math.all(Max >= other.Min & Min <= other.Max);
        }
    }

    // Helper functions
    public static partial class Math
    {
        // Transform an AABB into another space, expanding it as needed.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Aabb TransformAabb(RigidTransform transform, Aabb aabb)
        {
            float3 halfExtentsInA = aabb.Extents * 0.5f;
            float3 x = math.rotate(transform.rot, new float3(halfExtentsInA.x, 0, 0));
            float3 y = math.rotate(transform.rot, new float3(0, halfExtentsInA.y, 0));
            float3 z = math.rotate(transform.rot, new float3(0, 0, halfExtentsInA.z));

            float3 halfExtentsInB = math.abs(x) + math.abs(y) + math.abs(z);
            float3 centerInB = math.transform(transform, aabb.Center);

            return new Aabb
            {
                Min = centerInB - halfExtentsInB,
                Max = centerInB + halfExtentsInB
            };
        }

        // Transform an AABB into another space, expanding it as needed.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Aabb TransformAabb(MTransform transform, Aabb aabb)
        {
            float3 halfExtentsInA = aabb.Extents * 0.5f;
            float3 transformedX = math.abs(transform.Rotation.c0 * halfExtentsInA.x);
            float3 transformedY = math.abs(transform.Rotation.c1 * halfExtentsInA.y);
            float3 transformedZ = math.abs(transform.Rotation.c2 * halfExtentsInA.z);

            float3 halfExtentsInB = transformedX + transformedY + transformedZ;
            float3 centerInB = Math.Mul(transform, aabb.Center);

            return new Aabb
            {
                Min = centerInB - halfExtentsInB,
                Max = centerInB + halfExtentsInB
            };
        }
    }
}
