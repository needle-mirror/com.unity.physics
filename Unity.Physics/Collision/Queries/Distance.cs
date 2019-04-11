using System;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Physics.BoundingVolumeHierarchy;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // The input to point distance queries
    public struct PointDistanceInput
    {
        public float3 Position;
        public float MaxDistance;
        public CollisionFilter Filter;
    }

    // The input to collider distance queries
    public unsafe struct ColliderDistanceInput
    {
        public Collider* Collider;
        public RigidTransform Transform;
        public float MaxDistance;
    }

    // A hit from a distance query
    public struct DistanceHit : IQueryResult
    {
        public float Fraction { get; set; }
        public float Distance => Fraction;
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

    // Distance query implementations
    public static class DistanceQueries
    {
        // Distance queries have edge cases where distance = 0, eg. consider choosing the correct normal for a point that is exactly on a triangle surface.
        // Additionally, with floating point numbers there are often numerical accuracy problems near distance = 0.  Some routines handle this with special
        // cases where distance^2 < distanceEpsSq, which is expected to be rare in normal usage.  distanceEpsSq is not an exact value, but chosen to be small
        // enough that at typical simulation scale the difference between distance = distanceEps and distance = 0 is negligible.
        private const float distanceEpsSq = 1e-8f;

        public struct Result
        {
            public float3 PositionOnAinA;
            public float3 NormalInA;
            public float Distance;

            public float3 PositionOnBinA => PositionOnAinA - NormalInA * Distance;
        }

        public static unsafe Result ConvexConvex(ref ConvexHull convexA, ref ConvexHull convexB, MTransform aFromB)
        {
            return ConvexConvex(
                convexA.VerticesPtr, convexA.NumVertices, convexA.ConvexRadius,
                convexB.VerticesPtr, convexB.NumVertices, convexB.ConvexRadius,
                aFromB);
        }

        public static unsafe Result ConvexConvex(
            float3* verticesA, int numVerticesA, float convexRadiusA,
            float3* verticesB, int numVerticesB, float convexRadiusB,
            MTransform aFromB)
        {
            ConvexConvexDistanceQueries.Result result = ConvexConvexDistanceQueries.ConvexConvex(
                verticesA, numVerticesA, verticesB, numVerticesB, aFromB, ConvexConvexDistanceQueries.PenetrationHandling.Exact3D);

            // Adjust for convex radii
            result.ClosestPoints.Distance -= (convexRadiusA + convexRadiusB);
            result.ClosestPoints.PositionOnAinA -= result.ClosestPoints.NormalInA * convexRadiusA;
            return result.ClosestPoints;
        }

        private static Result PointPoint(float3 pointB, float3 diff, float coreDistanceSq, float radiusA, float sumRadii)
        {
            bool distanceZero = coreDistanceSq == 0.0f;
            float invCoreDistance = math.select(math.rsqrt(coreDistanceSq), 0.0f, distanceZero);
            float3 normal = math.select(diff * invCoreDistance, new float3(0, 1, 0), distanceZero); // choose an arbitrary normal when the distance is zero
            float distance = coreDistanceSq * invCoreDistance;
            return new Result
            {
                NormalInA = normal,
                PositionOnAinA = pointB + normal * (distance - radiusA),
                Distance = distance - sumRadii
            };
        }

        public static Result PointPoint(float3 pointA, float3 pointB, float radiusA, float sumRadii)
        {
            float3 diff = pointA - pointB;
            float coreDistanceSq = math.lengthsq(diff);
            return PointPoint(pointB, diff, coreDistanceSq, radiusA, sumRadii);
        }

        public static unsafe Result SphereSphere(SphereCollider* sphereA, SphereCollider* sphereB, MTransform aFromB)
        {
            float3 posBinA = Mul(aFromB, sphereB->Center);
            return PointPoint(sphereA->Center, posBinA, sphereA->Radius, sphereA->Radius + sphereB->Radius);
        }

        public static unsafe Result BoxSphere(BoxCollider* boxA, SphereCollider* sphereB, MTransform aFromB)
        {
            MTransform aFromBoxA = new MTransform(boxA->Orientation, boxA->Center);
            float3 posBinA = Mul(aFromB, sphereB->Center);
            float3 posBinBoxA = Mul(Inverse(aFromBoxA), posBinA);
            float3 innerHalfExtents = boxA->Size * 0.5f - boxA->ConvexRadius;
            float3 normalInBoxA;
            float distance;
            {
                // from hkAabb::signedDistanceToPoint(), can optimize a lot
                float3 projection = math.min(posBinBoxA, innerHalfExtents);
                projection = math.max(projection, -innerHalfExtents);
                float3 difference = projection - posBinBoxA;
                float distanceSquared = math.lengthsq(difference);

                // Check if the sphere center is inside the box
                if (distanceSquared < 1e-6f)
                {
                    float3 projectionLocal = projection;
                    float3 absProjectionLocal = math.abs(projectionLocal);
                    float3 del = absProjectionLocal - innerHalfExtents;
                    int axis = IndexOfMaxComponent(new float4(del, -float.MaxValue));
                    switch (axis)
                    {
                        case 0: normalInBoxA = new float3(projectionLocal.x < 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f); break;
                        case 1: normalInBoxA = new float3(0.0f, projectionLocal.y < 0.0f ? 1.0f : -1.0f, 0.0f); break;
                        case 2: normalInBoxA = new float3(0.0f, 0.0f, projectionLocal.z < 0.0f ? 1.0f : -1.0f); break;
                        default:
                            normalInBoxA = new float3(1, 0, 0);
                            Assert.IsTrue(false);
                            break;
                    }
                    distance = math.max(del.x, math.max(del.y, del.z));
                }
                else
                {
                    float invDistance = math.rsqrt(distanceSquared);
                    normalInBoxA = difference * invDistance;
                    distance = distanceSquared * invDistance;
                }
            }

            float3 normalInA = math.mul(aFromBoxA.Rotation, normalInBoxA);
            return new Result
            {
                NormalInA = normalInA,
                PositionOnAinA = posBinA + normalInA * (distance - boxA->ConvexRadius),
                Distance = distance - (sphereB->Radius + boxA->ConvexRadius)
            };
        }

        public static Result CapsuleSphere(
            float3 capsuleVertex0, float3 capsuleVertex1, float capsuleRadius,
            float3 sphereCenter, float sphereRadius,
            MTransform aFromB)
        {
            // Transform the sphere into capsule space
            float3 centerB = Mul(aFromB, sphereCenter);

            // Point-segment distance
            float3 edgeA = capsuleVertex1 - capsuleVertex0;
            float dot = math.dot(edgeA, centerB - capsuleVertex0);
            float edgeLengthSquared = math.lengthsq(edgeA);
            dot = math.max(dot, 0.0f);
            dot = math.min(dot, edgeLengthSquared);
            float invEdgeLengthSquared = 1.0f / edgeLengthSquared;
            float frac = dot * invEdgeLengthSquared;
            float3 pointOnA = capsuleVertex0 + edgeA * frac;
            return PointPoint(pointOnA, centerB, capsuleRadius, capsuleRadius + sphereRadius);
        }

        // Find the closest points on a pair of line segments
        private static void SegmentSegment(float3 pointA, float3 edgeA, float3 pointB, float3 edgeB, out float3 closestAOut, out float3 closestBOut)
        {
            // Find the closest point on edge A to the line containing edge B
            float3 diff = pointB - pointA;

            float r = math.dot(edgeA, edgeB);
            float s1 = math.dot(edgeA, diff);
            float s2 = math.dot(edgeB, diff);
            float lengthASq = math.lengthsq(edgeA);
            float lengthBSq = math.lengthsq(edgeB);

            float invDenom, invLengthASq, invLengthBSq;
            {
                float denom = lengthASq * lengthBSq - r * r;
                float3 inv = 1.0f / new float3(denom, lengthASq, lengthBSq);
                invDenom = inv.x;
                invLengthASq = inv.y;
                invLengthBSq = inv.z;
            }

            float fracA = (s1 * lengthBSq - s2 * r) * invDenom;
            fracA = math.clamp(fracA, 0.0f, 1.0f);

            // Find the closest point on edge B to the point on A just found
            float fracB = fracA * (invLengthBSq * r) - invLengthBSq * s2;
            fracB = math.clamp(fracB, 0.0f, 1.0f);

            // If the point on B was clamped then there may be a closer point on A to the edge
            fracA = fracB * (invLengthASq * r) + invLengthASq * s1;
            fracA = math.clamp(fracA, 0.0f, 1.0f);

            closestAOut = pointA + fracA * edgeA;
            closestBOut = pointB + fracB * edgeB;
        }

        public static unsafe Result CapsuleCapsule(CapsuleCollider* capsuleA, CapsuleCollider* capsuleB, MTransform aFromB)
        {
            // Transform capsule B into A-space
            float3 pointB = Mul(aFromB, capsuleB->Vertex0);
            float3 edgeB = math.mul(aFromB.Rotation, capsuleB->Vertex1 - capsuleB->Vertex0);

            // Get point and edge of A
            float3 pointA = capsuleA->Vertex0;
            float3 edgeA = capsuleA->Vertex1 - capsuleA->Vertex0;

            // Get the closest points on the capsules
            SegmentSegment(pointA, edgeA, pointB, edgeB, out float3 closestA, out float3 closestB);
            float3 diff = closestA - closestB;
            float coreDistanceSq = math.lengthsq(diff);
            if (coreDistanceSq < distanceEpsSq)
            {
                // Special case for extremely small distances, should be rare
                float3 normal = math.cross(edgeA, edgeB);
                if (math.lengthsq(normal) < 1e-5f)
                {
                    float3 edge = math.normalizesafe(edgeA, math.normalizesafe(edgeB, new float3(1, 0, 0))); // edges are parallel or one of the capsules is a sphere
                    Math.CalculatePerpendicularNormalized(edge, out normal, out float3 unused); // normal is anything perpendicular to edge
                }
                else
                {
                    normal = math.normalize(normal); // normal is cross of edges, sign doesn't matter
                }
                return new Result
                {
                    NormalInA = normal,
                    PositionOnAinA = pointA - normal * capsuleA->Radius,
                    Distance = -capsuleA->Radius - capsuleB->Radius
                };
            }
            return PointPoint(closestA, closestB, capsuleA->Radius, capsuleA->Radius + capsuleB->Radius);
        }

        private static void CalcTrianglePlanes(float3 v0, float3 v1, float3 v2, float3 normalDirection,
            out FourTransposedPoints verts, out FourTransposedPoints edges, out FourTransposedPoints perps)
        {
            verts = new FourTransposedPoints(v0, v1, v2, v0);
            edges = verts.V1230 - verts;
            perps = edges.Cross(new FourTransposedPoints(normalDirection));
        }

        // Checks if the closest point on the triangle is on its face.  If so returns true and sets signedDistance to the distance along the normal, otherwise returns false
        private static bool PointTriangleFace(float3 point, float3 v0, float3 normal,
            FourTransposedPoints verts, FourTransposedPoints edges, FourTransposedPoints perps, FourTransposedPoints rels, out float signedDistance)
        {
            float4 dots = perps.Dot(rels);
            float4 dotsSq = dots * math.abs(dots);
            float4 perpLengthSq = perps.Dot(perps);
            if (math.all(dotsSq <= perpLengthSq * distanceEpsSq))
            {
                // Closest point on face
                signedDistance = math.dot(v0 - point, normal);
                return true;
            }

            signedDistance = float.MaxValue;
            return false;
        }

        public static Result TriangleSphere(
            float3 vertex0, float3 vertex1, float3 vertex2, float3 normal,
            float3 sphereCenter, float sphereRadius,
            MTransform aFromB)
        {
            // Sphere center in A-space (TODO.ma should probably work in sphere space, typical for triangle verts to be far from the origin in its local space)
            float3 pointB = Mul(aFromB, sphereCenter);

            // Calculate triangle edges and edge planes
            FourTransposedPoints vertsA;
            FourTransposedPoints edgesA;
            FourTransposedPoints perpsA;
            CalcTrianglePlanes(vertex0, vertex1, vertex2, normal, out vertsA, out edgesA, out perpsA);

            // Check if the closest point is on the triangle face
            FourTransposedPoints rels = new FourTransposedPoints(pointB) - vertsA;
            if (PointTriangleFace(pointB, vertex0, normal, vertsA, edgesA, perpsA, rels, out float signedDistance))
            {
                return new Result
                {
                    PositionOnAinA = pointB + normal * signedDistance,
                    NormalInA = math.select(normal, -normal, signedDistance < 0),
                    Distance = math.abs(signedDistance) - sphereRadius
                };
            }

            // Find the closest point on the triangle edges - project point onto the line through each edge, then clamp to the edge
            float4 nums = rels.Dot(edgesA);
            float4 dens = edgesA.Dot(edgesA);
            float4 sols = math.clamp(nums / dens, 0.0f, 1.0f); // fraction along the edge TODO.ma see how it handles inf/nan from divide by zero
            FourTransposedPoints projs = edgesA.MulT(sols) - rels;
            float4 distancesSq = projs.Dot(projs);

            float3 proj0 = projs.GetPoint(0);
            float3 proj1 = projs.GetPoint(1);
            float3 proj2 = projs.GetPoint(2);

            // Find the closest projected point
            bool less1 = distancesSq.x < distancesSq.y;
            float3 direction = math.select(proj1, proj0, less1);
            float distanceSq = math.select(distancesSq.y, distancesSq.x, less1);

            bool less2 = distanceSq < distancesSq.z;
            direction = math.select(proj2, direction, less2);
            distanceSq = math.select(distancesSq.z, distanceSq, less2);

            const float triangleConvexRadius = 0.0f;
            return PointPoint(pointB, direction, distanceSq, triangleConvexRadius, sphereRadius);
        }

        public static Result QuadSphere(
            float3 vertex0, float3 vertex1, float3 vertex2, float3 vertex3, float3 normalDirection,
            float3 sphereCenter, float sphereRadius,
            MTransform aFromB)
        {
            // TODO: Do this in one pass
            Result result1 = TriangleSphere(vertex0, vertex1, vertex2, normalDirection, sphereCenter, sphereRadius, aFromB);
            Result result2 = TriangleSphere(vertex0, vertex2, vertex3, normalDirection, sphereCenter, sphereRadius, aFromB);
            return result1.Distance < result2.Distance ? result1 : result2;
        }

        // given two (normal, distance) pairs, select the one with smaller distance
        private static void SelectMin(ref float3 dirInOut, ref float distInOut, ref float3 posInOut, float3 newDir, float newDist, float3 newPos)
        {
            bool less = newDist < distInOut;
            dirInOut = math.select(dirInOut, newDir, less);
            distInOut = math.select(distInOut, newDist, less);
            posInOut = math.select(posInOut, newPos, less);
        }

        public static unsafe Result CapsuleTriangle(CapsuleCollider* capsuleA, PolygonCollider* triangleB, MTransform aFromB)
        {
            // Get vertices
            float3 c0 = capsuleA->Vertex0;
            float3 c1 = capsuleA->Vertex1;
            float3 t0 = Mul(aFromB, triangleB->ConvexHull.Vertices[0]);
            float3 t1 = Mul(aFromB, triangleB->ConvexHull.Vertices[1]);
            float3 t2 = Mul(aFromB, triangleB->ConvexHull.Vertices[2]);

            float3 direction;
            float distanceSq;
            float3 pointCapsule;
            float sign = 1.0f; // negated if penetrating
            {
                // Calculate triangle edges and edge planes
                float3 faceNormal = math.mul(aFromB.Rotation, triangleB->ConvexHull.Planes[0].Normal);
                FourTransposedPoints vertsB;
                FourTransposedPoints edgesB;
                FourTransposedPoints perpsB;
                CalcTrianglePlanes(t0, t1, t2, faceNormal, out vertsB, out edgesB, out perpsB);

                // c0 against triangle face
                {
                    FourTransposedPoints rels = new FourTransposedPoints(c0) - vertsB;
                    PointTriangleFace(c0, t0, faceNormal, vertsB, edgesB, perpsB, rels, out float signedDistance);
                    distanceSq = signedDistance * signedDistance;
                    if (distanceSq > distanceEpsSq)
                    {
                        direction = -faceNormal * signedDistance;
                    }
                    else
                    {
                        direction = math.select(faceNormal, -faceNormal, math.dot(c1 - c0, faceNormal) < 0); // rare case, capsule point is exactly on the triangle face
                    }
                    pointCapsule = c0;
                }

                // c1 against triangle face
                {
                    FourTransposedPoints rels = new FourTransposedPoints(c1) - vertsB;
                    PointTriangleFace(c1, t0, faceNormal, vertsB, edgesB, perpsB, rels, out float signedDistance);
                    float distanceSq1 = signedDistance * signedDistance;
                    float3 direction1;
                    if (distanceSq1 > distanceEpsSq)
                    {
                        direction1 = -faceNormal * signedDistance;
                    }
                    else
                    {
                        direction1 = math.select(faceNormal, -faceNormal, math.dot(c0 - c1, faceNormal) < 0); // rare case, capsule point is exactly on the triangle face
                    }
                    SelectMin(ref direction, ref distanceSq, ref pointCapsule, direction1, distanceSq1, c1);
                }

                // axis against triangle edges
                float3 axis = c1 - c0;
                for (int i = 0; i < 3; i++)
                {
                    float3 closestOnCapsule, closestOnTriangle;
                    SegmentSegment(c0, axis, vertsB.GetPoint(i), edgesB.GetPoint(i), out closestOnCapsule, out closestOnTriangle);
                    float3 edgeDiff = closestOnCapsule - closestOnTriangle;
                    float edgeDistanceSq = math.lengthsq(edgeDiff);
                    edgeDiff = math.select(edgeDiff, perpsB.GetPoint(i), edgeDistanceSq < distanceEpsSq); // use edge plane if the capsule axis intersects the edge
                    SelectMin(ref direction, ref distanceSq, ref pointCapsule, edgeDiff, edgeDistanceSq, closestOnCapsule);
                }

                // axis against triangle face
                {
                    // Find the intersection of the axis with the triangle plane
                    float axisDot = math.dot(axis, faceNormal);
                    float dist0 = math.dot(t0 - c0, faceNormal); // distance from c0 to the plane along the normal
                    float t = dist0 * math.select(math.rcp(axisDot), 0.0f, axisDot == 0.0f);
                    if (t > 0.0f && t < 1.0f)
                    {
                        // If they intersect, check if the intersection is inside the triangle
                        FourTransposedPoints rels = new FourTransposedPoints(c0 + axis * t) - vertsB;
                        float4 dots = perpsB.Dot(rels);
                        if (math.all(dots <= float4.zero))
                        {
                            // Axis intersects the triangle, choose the separating direction
                            float dist1 = axisDot - dist0;
                            bool use1 = math.abs(dist1) < math.abs(dist0);
                            float dist = math.select(-dist0, dist1, use1);
                            float3 closestOnCapsule = math.select(c0, c1, use1);
                            SelectMin(ref direction, ref distanceSq, ref pointCapsule, dist * faceNormal, dist * dist, closestOnCapsule);

                            // Even if the edge is closer than the face, we now know that the edge hit was penetrating
                            sign = -1.0f;
                        }
                    }
                }
            }

            float invDistance = math.rsqrt(distanceSq);
            float distance;
            float3 normal;
            if (distanceSq < distanceEpsSq)
            {
                normal = math.normalize(direction); // rare case, capsule axis almost exactly touches the triangle
                distance = 0.0f;
            }
            else
            {
                normal = direction * invDistance * sign; // common case, distanceSq = lengthsq(direction)
                distance = distanceSq * invDistance * sign;
            }
            return new Result
            {
                NormalInA = normal,
                PositionOnAinA = pointCapsule - normal * capsuleA->Radius,
                Distance = distance - capsuleA->Radius
            };
        }

        // Dispatch any pair of convex colliders
        public static unsafe Result ConvexConvex(Collider* convexA, Collider* convexB, MTransform aFromB)
        {
            Result result;
            bool flip = false;
            switch (convexA->Type)
            {
                case ColliderType.Sphere:
                    SphereCollider* sphereA = (SphereCollider*)convexA;
                    switch (convexB->Type)
                    {
                        case ColliderType.Sphere:
                            result = SphereSphere(sphereA, (SphereCollider*)convexB, aFromB);
                            break;
                        case ColliderType.Capsule:
                            CapsuleCollider* capsuleB = (CapsuleCollider*)convexB;
                            result = CapsuleSphere(capsuleB->Vertex0, capsuleB->Vertex1, capsuleB->Radius, sphereA->Center, sphereA->Radius, Inverse(aFromB));
                            flip = true;
                            break;
                        case ColliderType.Triangle:
                            PolygonCollider* triangleB = (PolygonCollider*)convexB;
                            result = TriangleSphere(
                                triangleB->Vertices[0], triangleB->Vertices[1], triangleB->Vertices[2], triangleB->Planes[0].Normal,
                                sphereA->Center, sphereA->Radius, Inverse(aFromB));
                            flip = true;
                            break;
                        case ColliderType.Quad:
                            PolygonCollider* quadB = (PolygonCollider*)convexB;
                            result = QuadSphere(
                                quadB->Vertices[0], quadB->Vertices[1], quadB->Vertices[2], quadB->Vertices[3], quadB->Planes[0].Normal,
                                sphereA->Center, sphereA->Radius, Inverse(aFromB));
                            flip = true;
                            break;
                        case ColliderType.Box:
                            result = BoxSphere((BoxCollider*)convexB, sphereA, Inverse(aFromB));
                            flip = true;
                            break;
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            result = ConvexConvex(ref sphereA->ConvexHull, ref ((ConvexCollider*)convexB)->ConvexHull, aFromB);
                            break;
                        default:
                            throw new NotImplementedException();
                    }
                    break;
                case ColliderType.Capsule:
                    CapsuleCollider* capsuleA = (CapsuleCollider*)convexA;
                    switch (convexB->Type)
                    {
                        case ColliderType.Sphere:
                            SphereCollider* sphereB = (SphereCollider*)convexB;
                            result = CapsuleSphere(capsuleA->Vertex0, capsuleA->Vertex1, capsuleA->Radius, sphereB->Center, sphereB->Radius, aFromB);
                            break;
                        case ColliderType.Capsule:
                            result = CapsuleCapsule(capsuleA, (CapsuleCollider*)convexB, aFromB);
                            break;
                        case ColliderType.Triangle:
                            result = CapsuleTriangle(capsuleA, (PolygonCollider*)convexB, aFromB);
                            break;
                        case ColliderType.Box:
                        case ColliderType.Quad:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            result = ConvexConvex(ref capsuleA->ConvexHull, ref ((ConvexCollider*)convexB)->ConvexHull, aFromB);
                            break;
                        default:
                            throw new NotImplementedException();
                    }
                    break;
                case ColliderType.Triangle:
                    PolygonCollider* triangleA = (PolygonCollider*)convexA;
                    switch (convexB->Type)
                    {
                        case ColliderType.Sphere:
                            SphereCollider* sphereB = (SphereCollider*)convexB;
                            result = TriangleSphere(
                                triangleA->Vertices[0], triangleA->Vertices[1], triangleA->Vertices[2], triangleA->Planes[0].Normal,
                                sphereB->Center, sphereB->Radius, aFromB);
                            break;
                        case ColliderType.Capsule:
                            result = CapsuleTriangle((CapsuleCollider*)convexB, triangleA, Inverse(aFromB));
                            flip = true;
                            break;
                        case ColliderType.Box:
                        case ColliderType.Triangle:
                        case ColliderType.Quad:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            result = ConvexConvex(ref triangleA->ConvexHull, ref ((ConvexCollider*)convexB)->ConvexHull, aFromB);
                            break;
                        default:
                            throw new NotImplementedException();
                    }
                    break;
                case ColliderType.Box:
                    BoxCollider* boxA = (BoxCollider*)convexA;
                    switch (convexB->Type)
                    {
                        case ColliderType.Sphere:
                            result = BoxSphere(boxA, (SphereCollider*)convexB, aFromB);
                            break;
                        case ColliderType.Capsule:
                        case ColliderType.Box:
                        case ColliderType.Triangle:
                        case ColliderType.Quad:
                        case ColliderType.Cylinder:
                        case ColliderType.Convex:
                            result = ConvexConvex(ref boxA->ConvexHull, ref ((ConvexCollider*)convexB)->ConvexHull, aFromB);
                            break;
                        default:
                            throw new NotImplementedException();
                    }
                    break;
                case ColliderType.Quad:
                case ColliderType.Cylinder:
                case ColliderType.Convex:
                    result = ConvexConvex(ref ((ConvexCollider*)convexA)->ConvexHull, ref ((ConvexCollider*)convexB)->ConvexHull, aFromB);
                    break;
                default:
                    throw new NotImplementedException();
            }

            if (flip)
            {
                result.PositionOnAinA = Mul(aFromB, result.PositionOnBinA);
                result.NormalInA = math.mul(aFromB.Rotation, -result.NormalInA);
            }

            return result;
        }

        public static unsafe bool PointCollider<T>(PointDistanceInput input, Collider* target, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Filter, target->Filter))
            {
                return false;
            }

            Result result;
            switch (target->Type)
            {
                case ColliderType.Sphere:
                    var sphere = (SphereCollider*)target;
                    result = PointPoint(sphere->Center, input.Position, sphere->Radius, sphere->Radius);
                    break;
                case ColliderType.Capsule:
                    var capsule = (CapsuleCollider*)target;
                    result = CapsuleSphere(capsule->Vertex0, capsule->Vertex1, capsule->Radius, input.Position, 0.0f, MTransform.Identity);
                    break;
                case ColliderType.Triangle:
                    var triangle = (PolygonCollider*)target;
                    result = TriangleSphere(
                        triangle->Vertices[0], triangle->Vertices[1], triangle->Vertices[2], triangle->Planes[0].Normal,
                        input.Position, 0.0f, MTransform.Identity);
                    break;
                case ColliderType.Quad:
                    var quad = (PolygonCollider*)target;
                    result = QuadSphere(
                        quad->Vertices[0], quad->Vertices[1], quad->Vertices[2], quad->Vertices[3], quad->Planes[0].Normal,
                        input.Position, 0.0f, MTransform.Identity);
                    break;
                case ColliderType.Convex:
                case ColliderType.Box:
                case ColliderType.Cylinder:
                    ref ConvexHull hull = ref ((ConvexCollider*)target)->ConvexHull;
                    result = ConvexConvex(hull.VerticesPtr, hull.NumVertices, hull.ConvexRadius, &input.Position, 1, 0.0f, MTransform.Identity);
                    break;
                case ColliderType.Mesh:
                    return PointMesh(input, (MeshCollider*)target, ref collector);
                case ColliderType.Compound:
                    return PointCompound(input, (CompoundCollider*)target, ref collector);
                default:
                    throw new NotImplementedException();
            }

            if (result.Distance < collector.MaxFraction)
            {
                collector.AddHit(new DistanceHit
                {
                    Fraction = result.Distance,
                    SurfaceNormal = -result.NormalInA,
                    Position = result.PositionOnAinA,
                    ColliderKey = ColliderKey.Empty
                });
                return true;
            }
            return false;
        }

        public static unsafe bool ColliderCollider<T>(ColliderDistanceInput input, Collider* target, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Collider->Filter, target->Filter))
            {
                return false;
            }

            switch (input.Collider->CollisionType)
            {
                case CollisionType.Convex:
                    switch (target->Type)
                    {
                        case ColliderType.Convex:
                        case ColliderType.Sphere:
                        case ColliderType.Capsule:
                        case ColliderType.Triangle:
                        case ColliderType.Quad:
                        case ColliderType.Box:
                        case ColliderType.Cylinder:
                            MTransform targetFromQuery = new MTransform(input.Transform);
                            Result result = ConvexConvex(target, input.Collider, targetFromQuery);
                            if (result.Distance < collector.MaxFraction)
                            {
                                collector.AddHit(new DistanceHit
                                {
                                    Fraction = result.Distance,
                                    SurfaceNormal = -result.NormalInA,
                                    Position = result.PositionOnAinA,
                                    ColliderKey = ColliderKey.Empty
                                });
                                return true;
                            }
                            return false;
                        case ColliderType.Mesh:
                            return ConvexMesh(input, (MeshCollider*)target, ref collector);
                        case ColliderType.Compound:
                            return ConvexCompound(input, (CompoundCollider*)target, ref collector);
                        default:
                            throw new NotImplementedException();
                    }
                case CollisionType.Composite:
                    // no support for composite query shapes
                    throw new NotImplementedException();
                default:
                    throw new NotImplementedException();
            }
        }

        private unsafe struct ConvexMeshLeafProcessor : IPointDistanceLeafProcessor, IColliderDistanceLeafProcessor
        {
            private readonly Mesh* m_Mesh;
            private readonly uint m_NumColliderKeyBits;

            public ConvexMeshLeafProcessor(MeshCollider* meshCollider)
            {
                m_Mesh = &meshCollider->Mesh;
                m_NumColliderKeyBits = meshCollider->NumColliderKeyBits;
            }

            public bool DistanceLeaf<T>(PointDistanceInput input, int primitiveKey, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                m_Mesh->GetPrimitive(primitiveKey, out float3x4 vertices, out Mesh.PrimitiveFlags flags, out CollisionFilter filter);

                if (!CollisionFilter.IsCollisionEnabled(input.Filter, filter)) // TODO: could do this check within GetPrimitive()
                {
                    return false;
                }

                int numPolygons = Mesh.GetNumPolygonsInPrimitive(flags);
                bool isQuad = Mesh.IsPrimitveFlagSet(flags, Mesh.PrimitiveFlags.IsQuad);

                float3 normalDirection = math.cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
                bool acceptHit = false;

                for (int polygonIndex = 0; polygonIndex < numPolygons; polygonIndex++)
                {
                    Result result;
                    if (isQuad)
                    {
                        result = QuadSphere(
                            vertices[0], vertices[1], vertices[2], vertices[3], normalDirection,
                            input.Position, 0.0f, MTransform.Identity);
                    }
                    else
                    {
                        result = TriangleSphere(
                            vertices[0], vertices[1], vertices[2], normalDirection,
                            input.Position, 0.0f, MTransform.Identity);
                    }

                    if (result.Distance < collector.MaxFraction)
                    {
                        acceptHit |= collector.AddHit(new DistanceHit
                        {
                            Fraction = result.Distance,
                            Position = result.PositionOnAinA,
                            SurfaceNormal = -result.NormalInA,
                            ColliderKey = new ColliderKey(m_NumColliderKeyBits, (uint)(primitiveKey << 1 | polygonIndex))
                        });
                    }
                }

                return acceptHit;
            }

            public bool DistanceLeaf<T>(ColliderDistanceInput input, int primitiveKey, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                m_Mesh->GetPrimitive(primitiveKey, out float3x4 vertices, out Mesh.PrimitiveFlags flags, out CollisionFilter filter);

                if (!CollisionFilter.IsCollisionEnabled(input.Collider->Filter, filter)) // TODO: could do this check within GetPrimitive()
                {
                    return false;
                }

                int numPolygons = Mesh.GetNumPolygonsInPrimitive(flags);
                bool isQuad = Mesh.IsPrimitveFlagSet(flags, Mesh.PrimitiveFlags.IsQuad);

                float3* v = stackalloc float3[4];
                bool acceptHit = false;

                ref ConvexHull inputHull = ref ((ConvexCollider*)input.Collider)->ConvexHull;
                MTransform targetFromQuery = new MTransform(input.Transform);

                for (int polygonIndex = 0; polygonIndex < numPolygons; polygonIndex++)
                {
                    int numVertices;
                    if (isQuad)
                    {
                        v[0] = vertices[0];
                        v[1] = vertices[1];
                        v[2] = vertices[2];
                        v[3] = vertices[3];
                        numVertices = 4;
                    }
                    else
                    {
                        v[0] = vertices[0];
                        v[1] = vertices[1 + polygonIndex];
                        v[2] = vertices[2 + polygonIndex];
                        numVertices = 3;
                    }

                    Result result = ConvexConvex(v, numVertices, 0.0f, inputHull.VerticesPtr, inputHull.NumVertices, inputHull.ConvexRadius, targetFromQuery);
                    if (result.Distance < collector.MaxFraction)
                    {
                        acceptHit |= collector.AddHit(new DistanceHit
                        {
                            Fraction = result.Distance,
                            Position = result.PositionOnAinA,
                            SurfaceNormal = -result.NormalInA,
                            ColliderKey = new ColliderKey(m_NumColliderKeyBits, (uint)(primitiveKey << 1 | polygonIndex))
                        });
                    }
                }

                return acceptHit;
            }
        }

        public static unsafe bool PointMesh<T>(PointDistanceInput input, MeshCollider* meshCollider, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            var leafProcessor = new ConvexMeshLeafProcessor(meshCollider);
            return meshCollider->Mesh.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);
        }

        public static unsafe bool ConvexMesh<T>(ColliderDistanceInput input, MeshCollider* meshCollider, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            var leafProcessor = new ConvexMeshLeafProcessor(meshCollider);
            return meshCollider->Mesh.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);
        }

        private unsafe struct ConvexCompoundLeafProcessor : IPointDistanceLeafProcessor, IColliderDistanceLeafProcessor
        {
            private readonly CompoundCollider* m_CompoundCollider;

            public ConvexCompoundLeafProcessor(CompoundCollider* compoundCollider)
            {
                m_CompoundCollider = compoundCollider;
            }

            public bool DistanceLeaf<T>(PointDistanceInput input, int leafData, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                ref CompoundCollider.Child child = ref m_CompoundCollider->Children[leafData];

                if (!CollisionFilter.IsCollisionEnabled(input.Filter, child.Collider->Filter))
                {
                    return false;
                }

                // Transform the point into child space
                MTransform compoundFromChild = new MTransform(child.CompoundFromChild);
                PointDistanceInput inputLs = input;
                {
                    MTransform childFromCompound = Inverse(compoundFromChild);
                    inputLs.Position = Math.Mul(childFromCompound, input.Position);
                }

                int numHits = collector.NumHits;
                float fraction = collector.MaxFraction;

                if (child.Collider->CalculateDistance(inputLs, ref collector))
                {
                    // Transform results back to compound space
                    collector.TransformNewHits(numHits, fraction, compoundFromChild, m_CompoundCollider->NumColliderKeyBits, (uint)leafData);
                    return true;
                }
                return false;
            }

            public bool DistanceLeaf<T>(ColliderDistanceInput input, int leafData, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                ref CompoundCollider.Child child = ref m_CompoundCollider->Children[leafData];

                if (!CollisionFilter.IsCollisionEnabled(input.Collider->Filter, child.Collider->Filter))
                {
                    return false;
                }

                // Transform the query into child space
                ColliderDistanceInput inputLs = input;
                inputLs.Transform = math.mul(math.inverse(child.CompoundFromChild), input.Transform);

                int numHits = collector.NumHits;
                float fraction = collector.MaxFraction;

                if (child.Collider->CalculateDistance(inputLs, ref collector))
                {
                    // Transform results back to compound space
                    collector.TransformNewHits(numHits, fraction, new MTransform(child.CompoundFromChild), m_CompoundCollider->NumColliderKeyBits, (uint)leafData);
                    return true;
                }
                return false;
            }
        }

        public static unsafe bool PointCompound<T>(PointDistanceInput input, CompoundCollider* compoundCollider, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            var leafProcessor = new ConvexCompoundLeafProcessor(compoundCollider);
            return compoundCollider->BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);
        }

        public static unsafe bool ConvexCompound<T>(ColliderDistanceInput input, CompoundCollider* compoundCollider, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            var leafProcessor = new ConvexCompoundLeafProcessor(compoundCollider);
            return compoundCollider->BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);
        }
    }
}
