using System;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // A ray
    public struct Ray
    {
        public float3 Origin;
        private float3 m_Direction;
        public float3 ReciprocalDirection { get; private set; }

        public float3 Direction
        {
            get => m_Direction;
            set
            {
                m_Direction = value;
                Assert.IsFalse(math.all(m_Direction == float3.zero));
                ReciprocalDirection = math.rcp(m_Direction);
            }
        }

        public Ray(float3 origin, float3 direction)
        {
            Assert.IsFalse(math.all(direction == float3.zero));

            Origin = origin;
            m_Direction = direction;
            ReciprocalDirection = math.rcp(direction);
        }
    }

    // The input to ray cast queries
    public struct RaycastInput
    {
        public Ray Ray;
        public CollisionFilter Filter;
    }

    // A hit from a ray cast query
    public struct RaycastHit : IQueryResult
    {
        public float Fraction { get; set; }

        public float3 Position;
        public float3 SurfaceNormal;
        public int RigidBodyIndex;
        public ColliderKey ColliderKey;

        public void Transform(MTransform transform, uint numSubKeyBits, uint subKey)
        {
            Position = Mul(transform, Position);
            SurfaceNormal = math.mul(transform.Rotation, SurfaceNormal);
            ColliderKey.PushSubKey(numSubKeyBits, subKey);
        }

        public void Transform(MTransform transform, int rigidBodyIndex)
        {
            Position = Mul(transform, Position);
            SurfaceNormal = math.mul(transform.Rotation, SurfaceNormal);
            RigidBodyIndex = rigidBodyIndex;
        }
    }

    // Raycast query implementations
    public static class RaycastQueries
    {
        #region Ray vs primitives

        public static bool RaySphere(
            float3 rayOrigin, float3 rayDirection,
            float3 sphereCenter, float sphereRadius,
            ref float fraction, out float3 normal)
        {
            normal = float3.zero;

            // TODO.ma lots of float inaccuracy problems with this
            float3 diff = rayOrigin - sphereCenter;
            float a = math.dot(rayDirection, rayDirection);
            float b = 2.0f * math.dot(rayDirection, diff);
            float c = math.dot(diff, diff) - sphereRadius * sphereRadius;
            float discriminant = b * b - 4.0f * a * c;

            if (c < 0)
            {
                // Inside hit.
                fraction = 0;
                normal = math.normalize(-rayDirection);
                return true;
            }

            if (discriminant < 0)
            {
                return false;
            }

            float sqrtDiscriminant = math.sqrt(discriminant);
            float invDenom = 0.5f / a;

            float t0 = (sqrtDiscriminant - b) * invDenom;
            float t1 = (-sqrtDiscriminant - b) * invDenom;
            float tMin = math.min(t0, t1);

            if (tMin >= 0 && tMin < fraction)
            {
                fraction = tMin;
                normal = (rayOrigin + rayDirection * fraction - sphereCenter) / sphereRadius;

                return true;
            }

            return false;
        }

        public static bool RayCapsule(
            float3 rayOrigin, float3 rayDirection,
            float3 vertex0, float3 vertex1, float radius,
            ref float fraction, out float3 normal)
        {
            float axisLength = NormalizeWithLength(vertex1 - vertex0, out float3 axis);

            // Ray vs infinite cylinder
            {
                float directionDotAxis = math.dot(rayDirection, axis);
                float originDotAxis = math.dot(rayOrigin - vertex0, axis);
                float3 rayDirection2D = rayDirection - axis * directionDotAxis;
                float3 rayOrigin2D = rayOrigin - axis * originDotAxis;
                float cylinderFraction = fraction;

                if (RaySphere(rayOrigin2D, rayDirection2D, vertex0, radius, ref cylinderFraction, out normal))
                {
                    float t = originDotAxis + cylinderFraction * directionDotAxis; // distance of the hit from Vertex0 along axis
                    if (t >= 0.0f && t <= axisLength)
                    {
                        if (cylinderFraction == 0)
                        {
                            // Inside hit
                            normal = math.normalize(-rayDirection);
                        }

                        fraction = cylinderFraction;
                        return true;
                    }
                }
            }

            // Ray vs caps
            {
                bool hadHit = false;
                float3 capNormal;
                if (RaySphere(rayOrigin, rayDirection, vertex0, radius, ref fraction, out capNormal))
                {
                    hadHit = true;
                    normal = capNormal;
                }
                if (RaySphere(rayOrigin, rayDirection, vertex1, radius, ref fraction, out capNormal))
                {
                    hadHit = true;
                    normal = capNormal;
                }
                return hadHit;
            }
        }

        public static bool RayTriangle(
            float3 rayOrigin, float3 rayDirection,
            float3 a, float3 b, float3 c, // TODO: float3x3?
            ref float fraction, out float3 unnormalizedNormal)
        {
            float3 vAb = b - a;
            float3 vCa = a - c;

            float3 vN = math.cross(vAb, vCa);
            float3 vAp = rayOrigin - a;
            float3 end0 = vAp + rayDirection * fraction;

            float d = math.dot(vN, vAp);
            float e = math.dot(vN, end0);

            if (d * e >= 0)
            {
                unnormalizedNormal = float3.zero;
                return false;
            }

            float3 vBc = c - b;
            fraction *= d / (d - e);
            unnormalizedNormal = vN * math.sign(d);

            // edge normals
            float3 c0 = math.cross(vAb, rayDirection);
            float3 c1 = math.cross(vBc, rayDirection);
            float3 c2 = math.cross(vCa, rayDirection);

            float3 dots;
            {
                float3 o2 = rayOrigin + rayOrigin;
                float3 r0 = o2 - (a + b);
                float3 r1 = o2 - (b + c);
                float3 r2 = o2 - (c + a);

                dots.x = math.dot(r0, c0);
                dots.y = math.dot(r1, c1);
                dots.z = math.dot(r2, c2);
            }

            bool3 notOutSide = dots < 0;
            // hit if all bits have the same sign
            return math.all(notOutSide) || !math.any(notOutSide);
        }

        public static bool RayQuad(
            float3 rayOrigin, float3 rayDirection,
            float3 a, float3 b, float3 c, float3 d, // TODO: float3x4?
            ref float fraction, out float3 unnormalizedNormal)
        {
            float3 vAb = b - a;
            float3 vCa = a - c;

            float3 vN = math.cross(vAb, vCa);
            float3 vAp = rayOrigin - a;
            float3 end0 = vAp + rayDirection * fraction;

            float nDotAp = math.dot(vN, vAp);
            float e = math.dot(vN, end0);

            if (nDotAp * e >= 0)
            {
                unnormalizedNormal = float3.zero;
                return false;
            }

            float3 vBc = c - b;
            float3 vDa = a - d;
            float3 vCd = d - c;
            fraction *= nDotAp / (nDotAp - e);
            unnormalizedNormal = vN * math.sign(nDotAp);

            // edge normals
            float3 c0 = math.cross(vAb, rayDirection);
            float3 c1 = math.cross(vBc, rayDirection);
            float3 c2 = math.cross(vCd, rayDirection);
            float3 c3 = math.cross(vDa, rayDirection);

            float4 dots;
            {
                float3 o2 = rayOrigin + rayOrigin;
                float3 r0 = o2 - (a + b);
                float3 r1 = o2 - (b + c);
                float3 r2 = o2 - (c + d);
                float3 r3 = o2 - (d + a);

                dots.x = math.dot(r0, c0);
                dots.y = math.dot(r1, c1);
                dots.z = math.dot(r2, c2);
                dots.w = math.dot(r3, c3);
            }

            bool4 notOutSide = dots < 0;
            // hit if all bits have the same sign
            return math.all(notOutSide) || !math.any(notOutSide);
        }

        public static bool RayConvex(
            float3 rayOrigin, float3 rayDirection, ref ConvexHull hull,
            ref float fraction, out float3 normal)
        {
            // TODO: Call RaySphere/Capsule/Triangle() if num vertices <= 3 ?

            float convexRadius = hull.ConvexRadius;
            float fracEnter = -1.0f;
            float fracExit = 2.0f;
            float3 start = rayOrigin;
            float3 end = start + rayDirection * fraction;
            normal = new float3(1, 0, 0);
            for (int i = 0; i < hull.NumFaces; i++) // TODO.ma vectorize
            {
                // Calculate the plane's hit fraction
                Plane plane = hull.Planes[i];
                float startDistance = math.dot(start, plane.Normal) + plane.Distance - convexRadius;
                float endDistance = math.dot(end, plane.Normal) + plane.Distance - convexRadius;
                float newFraction = startDistance / (startDistance - endDistance);
                bool startInside = (startDistance < 0);
                bool endInside = (endDistance < 0);

                // If the ray is entirely outside of any plane, then it misses
                if (!(startInside || endInside))
                {
                    return false;
                }

                // If the ray crosses the plane, update the enter or exit fraction
                bool enter = !startInside && newFraction > fracEnter;
                bool exit = !endInside && newFraction < fracExit;
                fracEnter = math.select(fracEnter, newFraction, enter);
                normal = math.select(normal, plane.Normal, enter);
                fracExit = math.select(fracExit, newFraction, exit);
            }

            if (fracEnter < 0)
            {
                // Inside hit.
                fraction = 0;
                normal = math.normalize(-rayDirection);
                return true;
            }

            if (fracEnter < fracExit)
            {
                fraction *= fracEnter;
                return true;
            }

            // miss
            return false;
        }

        #endregion

        #region Ray vs colliders

        public static unsafe bool RayCollider<T>(RaycastInput input, Collider* collider, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Filter, collider->Filter))
            {
                return false;
            }

            float fraction = collector.MaxFraction;
            float3 normal;
            bool hadHit;
            switch (collider->Type)
            {
                case ColliderType.Sphere:
                    var sphere = (SphereCollider*)collider;
                    hadHit = RaySphere(input.Ray.Origin, input.Ray.Direction, sphere->Center, sphere->Radius, ref fraction, out normal);
                    break;
                case ColliderType.Capsule:
                    var capsule = (CapsuleCollider*)collider;
                    hadHit = RayCapsule(input.Ray.Origin, input.Ray.Direction, capsule->Vertex0, capsule->Vertex1, capsule->Radius, ref fraction, out normal);
                    break;
                case ColliderType.Triangle:
                    {
                        var triangle = (PolygonCollider*)collider;
                        hadHit = RayTriangle(input.Ray.Origin, input.Ray.Direction, triangle->Vertices[0], triangle->Vertices[1], triangle->Vertices[2], ref fraction, out float3 unnormalizedNormal);
                        normal = hadHit ? math.normalize(unnormalizedNormal) : float3.zero;
                        break;
                    }
                case ColliderType.Quad:
                    {
                        var quad = (PolygonCollider*)collider;
                        hadHit = RayQuad(input.Ray.Origin, input.Ray.Direction, quad->Vertices[0], quad->Vertices[1], quad->Vertices[2], quad->Vertices[3], ref fraction, out float3 unnormalizedNormal);
                        normal = hadHit ? math.normalize(unnormalizedNormal) : float3.zero;
                        break;
                    }
                case ColliderType.Box:
                case ColliderType.Convex:
                    hadHit = RayConvex(input.Ray.Origin, input.Ray.Direction, ref ((ConvexCollider*)collider)->ConvexHull, ref fraction, out normal);
                    break;
                case ColliderType.Mesh:
                    return RayMesh(input, (MeshCollider*)collider, ref collector);
                case ColliderType.Compound:
                    return RayCompound(input, (CompoundCollider*)collider, ref collector);
                default:
                    throw new NotImplementedException();
            }

            if (hadHit)
            {
                return collector.AddHit(new RaycastHit
                {
                    Fraction = fraction,
                    Position = input.Ray.Origin + input.Ray.Direction * fraction,
                    SurfaceNormal = normal,
                    RigidBodyIndex = -1,
                    ColliderKey = ColliderKey.Empty
                });
            }
            return false;
        }

        // Mesh
        private unsafe struct RayMeshLeafProcessor : BoundingVolumeHierarchy.IRaycastLeafProcessor
        {
            private readonly Mesh* m_Mesh;
            private readonly uint m_NumColliderKeyBits;

            public RayMeshLeafProcessor(MeshCollider* meshCollider)
            {
                m_Mesh = &meshCollider->Mesh;
                m_NumColliderKeyBits = meshCollider->NumColliderKeyBits;
            }

            public bool RayLeaf<T>(RaycastInput input, int primitiveKey, ref T collector) where T : struct, ICollector<RaycastHit>
            {
                m_Mesh->GetPrimitive(primitiveKey, out float3x4 vertices, out Mesh.PrimitiveFlags flags, out CollisionFilter filter);

                if (!CollisionFilter.IsCollisionEnabled(input.Filter, filter)) // TODO: could do this check within GetPrimitive()
                {
                    return false;
                }

                int numPolygons = Mesh.GetNumPolygonsInPrimitive(flags);
                bool isQuad = Mesh.IsPrimitveFlagSet(flags, Mesh.PrimitiveFlags.IsQuad);

                bool acceptHit = false;
                float3 unnormalizedNormal;

                for (int polygonIndex = 0; polygonIndex < numPolygons; polygonIndex++)
                {
                    float fraction = collector.MaxFraction;
                    bool hadHit;
                    if (isQuad)
                    {
                        hadHit = RayQuad(input.Ray.Origin, input.Ray.Direction, vertices[0], vertices[1], vertices[2], vertices[3], ref fraction, out unnormalizedNormal);
                    }
                    else
                    {
                        hadHit = RayTriangle(input.Ray.Origin, input.Ray.Direction, vertices[0], vertices[polygonIndex + 1], vertices[polygonIndex + 2], ref fraction, out unnormalizedNormal);
                    }

                    if (hadHit && fraction < collector.MaxFraction)
                    {
                        acceptHit |= collector.AddHit(new RaycastHit
                        {
                            Fraction = fraction,
                            Position = input.Ray.Origin + input.Ray.Direction * fraction,
                            SurfaceNormal = math.normalize(unnormalizedNormal),
                            RigidBodyIndex = -1,
                            ColliderKey = new ColliderKey(m_NumColliderKeyBits, (uint)(primitiveKey << 1 | polygonIndex))
                        });
                    }
                }

                return acceptHit;
            }
        }

        private static unsafe bool RayMesh<T>(RaycastInput input, MeshCollider* meshCollider, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            var leafProcessor = new RayMeshLeafProcessor(meshCollider);
            return meshCollider->Mesh.BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);
        }

        // Compound

        private unsafe struct RayCompoundLeafProcessor : BoundingVolumeHierarchy.IRaycastLeafProcessor
        {
            private readonly CompoundCollider* m_CompoundCollider;

            public RayCompoundLeafProcessor(CompoundCollider* compoundCollider)
            {
                m_CompoundCollider = compoundCollider;
            }

            public bool RayLeaf<T>(RaycastInput input, int leafData, ref T collector) where T : struct, ICollector<RaycastHit>
            {
                ref CompoundCollider.Child child = ref m_CompoundCollider->Children[leafData];

                if (!CollisionFilter.IsCollisionEnabled(input.Filter, child.Collider->Filter))
                {
                    return false;
                }

                MTransform compoundFromChild = new MTransform(child.CompoundFromChild);

                // Transform the ray into child space
                RaycastInput inputLs = input;
                {
                    MTransform childFromCompound = Inverse(compoundFromChild);
                    float3 originLs = Mul(childFromCompound, input.Ray.Origin);
                    float3 directionLs = math.mul(childFromCompound.Rotation, input.Ray.Direction);
                    inputLs.Ray = new Ray(originLs, directionLs);
                }

                int numHits = collector.NumHits;
                float fraction = collector.MaxFraction;

                if (child.Collider->CastRay(inputLs, ref collector))
                {
                    // Transform results back to compound space
                    collector.TransformNewHits(numHits, fraction, compoundFromChild, m_CompoundCollider->NumColliderKeyBits, (uint)leafData);
                    return true;
                }
                return false;
            }
        }

        private static unsafe bool RayCompound<T>(RaycastInput input, CompoundCollider* compoundCollider, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Filter, compoundCollider->Filter))
            {
                return false;
            }

            var leafProcessor = new RayCompoundLeafProcessor(compoundCollider);
            return compoundCollider->BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);
        }

        #endregion
    }
}
