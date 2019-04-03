using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // Low level convex-convex contact manifold query implementations
    internal static class ConvexConvexManifoldQueries
    {
        // The output of convex-convex manifold queries
        public unsafe struct Manifold
        {
            public int NumContacts;
            public float3 Normal;

            public const int k_MaxNumContacts = 32;
            private fixed float m_ContactPositions[k_MaxNumContacts * 3];
            private fixed float m_Distances[k_MaxNumContacts];

            // Create a single point manifold from a distance query result
            public Manifold(DistanceQueries.Result convexDistance, MTransform worldFromA)
            {
                NumContacts = 1;
                Normal = math.mul(worldFromA.Rotation, convexDistance.NormalInA);
                this[0] = new ContactPoint
                {
                    Distance = convexDistance.Distance,
                    Position = Mul(worldFromA, convexDistance.PositionOnBinA)
                };
            }

            public ContactPoint this[int contactIndex]
            {
                get
                {
                    Assert.IsTrue(contactIndex >= 0 && contactIndex < k_MaxNumContacts);

                    int offset = contactIndex * 3;
                    var contact = new ContactPoint();

                    fixed (float* positions = m_ContactPositions)
                    {
                        contact.Position.x = positions[offset];
                        contact.Position.y = positions[offset + 1];
                        contact.Position.z = positions[offset + 2];
                    }

                    fixed (float* distances = m_Distances)
                    {
                        contact.Distance = distances[contactIndex];
                    }

                    return contact;
                }
                set
                {
                    Assert.IsTrue(contactIndex >= 0 && contactIndex < k_MaxNumContacts);

                    int offset = contactIndex * 3;
                    fixed (float* positions = m_ContactPositions)
                    {
                        positions[offset] = value.Position.x;
                        positions[offset + 1] = value.Position.y;
                        positions[offset + 2] = value.Position.z;
                    }

                    fixed (float* distances = m_Distances)
                    {
                        distances[contactIndex] = value.Distance;
                    }
                }
            }

            public void Flip()
            {
                for (int i = 0; i < NumContacts; i++)
                {
                    ContactPoint contact = this[i];
                    contact.Position += Normal * contact.Distance;
                    this[i] = contact;
                }
                Normal = -Normal;
            }
        }

        #region Convex vs convex

        // Create a contact point for a pair of spheres in world space.
        public static unsafe void SphereSphere(
            SphereCollider* sphereA, SphereCollider* sphereB,
            MTransform worldFromA, MTransform aFromB, float maxDistance,
            out Manifold manifold)
        {
            DistanceQueries.Result convexDistance = DistanceQueries.SphereSphere(sphereA, sphereB, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create a contact point for a box and a sphere in world space.
        public static unsafe void BoxSphere(
            BoxCollider* boxA, SphereCollider* sphereB,
            MTransform worldFromA, MTransform aFromB, float maxDistance,
            out Manifold manifold)
        {
            DistanceQueries.Result convexDistance = DistanceQueries.BoxSphere(boxA, sphereB, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create contact points for a pair of boxes in world space.
        public static unsafe void BoxBox(
            BoxCollider* boxA, BoxCollider* boxB,
            MTransform worldFromA, MTransform aFromB, float maxDistance,
            out Manifold manifold)
        {
            manifold = new Manifold();

            // Get transforms with box center at origin
            MTransform bFromBoxB = new MTransform(boxB->Orientation, boxB->Center);
            MTransform aFromBoxA = new MTransform(boxA->Orientation, boxA->Center);
            MTransform boxAFromBoxB = Mul(Inverse(aFromBoxA), Mul(aFromB, bFromBoxB));
            MTransform boxBFromBoxA = Inverse(boxAFromBoxB);

            float3 halfExtentsA = boxA->Size * 0.5f;
            float3 halfExtentsB = boxB->Size * 0.5f;

            // Test planes of each box against the other's vertices
            float3 normal; // in BoxA-space
            float distance;
            {
                float3 normalA = new float3(1, 0, 0);
                float3 normalB = new float3(1, 0, 0);
                float distA = 0.0f;
                float distB = 0.0f;
                if (!PointPlanes(boxAFromBoxB, halfExtentsA, halfExtentsB, maxDistance, ref normalA, ref distA) ||
                    !PointPlanes(boxBFromBoxA, halfExtentsB, halfExtentsA, maxDistance, ref normalB, ref distB))
                {
                    return;
                }

                normalB = math.mul(boxAFromBoxB.Rotation, normalB);
                bool aGreater = distA > distB;
                normal = math.select(-normalB, normalA, (bool3)aGreater);
                distance = math.select(distB, distA, aGreater);
            }

            // Test edge pairs
            {
                float3 edgeA = new float3(1.0f, 0.0f, 0.0f);
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        float3 edgeB;
                        switch (j)
                        {
                            case 0: edgeB = boxAFromBoxB.Rotation.c0; break;
                            case 1: edgeB = boxAFromBoxB.Rotation.c1; break;
                            case 2: edgeB = boxAFromBoxB.Rotation.c2; break;
                            default: edgeB = new float3(0.0f); break;
                        }
                        float3 dir = math.cross(edgeA, edgeB);

                        // hack around parallel edges
                        if (math.all(math.abs(dir) < new float3(1e-5f)))
                        {
                            continue;
                        }

                        float3 edgeNormal = math.normalize(dir);
                        float3 supportA = math.select(halfExtentsA, -halfExtentsA, dir < new float3(0.0f));
                        float maxA = math.abs(math.dot(supportA, edgeNormal));
                        float minA = -maxA;
                        float3 dirInB = math.mul(boxBFromBoxA.Rotation, dir);
                        float3 supportBinB = math.select(halfExtentsB, -halfExtentsB, dirInB < new float3(0.0f));
                        float3 supportB = math.mul(boxAFromBoxB.Rotation, supportBinB);
                        float offsetB = math.abs(math.dot(supportB, edgeNormal));
                        float centerB = math.dot(boxAFromBoxB.Translation, edgeNormal);
                        float maxB = centerB + offsetB;
                        float minB = centerB - offsetB;

                        float2 diffs = new float2(minB - maxA, minA - maxB); // positive normal, negative normal
                        if (math.all(diffs > new float2(maxDistance)))
                        {
                            return;
                        }

                        if (diffs.x > distance)
                        {
                            distance = diffs.x;
                            normal = -edgeNormal;
                        }

                        if (diffs.y > distance)
                        {
                            distance = diffs.y;
                            normal = edgeNormal;
                        }
                    }

                    edgeA = edgeA.zxy;
                }
            }

            if (distance < maxDistance)
            {
                // Get the normal and supporting faces
                float3 normalInA = math.mul(boxA->Orientation, normal);
                manifold.Normal = math.mul(worldFromA.Rotation, normalInA);
                int faceIndexA = boxA->ConvexHull.GetSupportingFace(-normalInA);
                int faceIndexB = boxB->ConvexHull.GetSupportingFace(math.mul(math.transpose(aFromB.Rotation), normalInA));

                // Build manifold
                if (!FaceFace(ref boxA->ConvexHull, ref boxB->ConvexHull, faceIndexA, faceIndexB, worldFromA, aFromB, normalInA, distance, ref manifold))
                {
                    // TODO.ma - the closest points are vertices, we need GJK to find them.
                    // We could call GJK here, or always use GJK instead of BoxTriangle(), or do nothing and live with possible tunneling.
                    // This method is ~2.5x faster than GJK, so if it gets the right result without calling GJK most of the time, it's worth keeping even if we call GJK when it fails.
                }
            }
        }

        // Create a single point manifold between a capsule and a sphere in world space.
        public static unsafe void CapsuleSphere(
            CapsuleCollider* capsuleA, SphereCollider* sphereB,
            MTransform worldFromA, MTransform aFromB, float maxDistance,
            out Manifold manifold)
        {
            DistanceQueries.Result convexDistance = DistanceQueries.CapsuleSphere(
                capsuleA->Vertex0, capsuleA->Vertex1, capsuleA->Radius, sphereB->Center, sphereB->Radius, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create a contact point for a pair of capsules in world space.
        public static unsafe void CapsuleCapsule(
            CapsuleCollider* capsuleA, CapsuleCollider* capsuleB,
            MTransform worldFromA, MTransform aFromB, float maxDistance,
            out Manifold manifold)
        {
            // TODO: Should produce a multi-point manifold
            DistanceQueries.Result convexDistance = DistanceQueries.CapsuleCapsule(capsuleA, capsuleB, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create contact points for a box and triangle in world space.
        public static unsafe void BoxTriangle(
            BoxCollider* boxA, PolygonCollider* triangleB,
            MTransform worldFromA, MTransform aFromB, float maxDistance,
            out Manifold manifold)
        {
            Assert.IsTrue(triangleB->Vertices.Length == 3);

            // Get triangle in box space
            MTransform aFromBoxA = new MTransform(boxA->Orientation, boxA->Center);
            MTransform boxAFromB = Mul(Inverse(aFromBoxA), aFromB);
            float3 t0 = Mul(boxAFromB, triangleB->ConvexHull.Vertices[0]);
            float3 t1 = Mul(boxAFromB, triangleB->ConvexHull.Vertices[1]);
            float3 t2 = Mul(boxAFromB, triangleB->ConvexHull.Vertices[2]);

            Plane triPlane = triangleB->ConvexHull.Planes[0];
            float3 triangleNormal = math.mul(boxAFromB.Rotation, triPlane.Normal);
            FourTransposedPoints vertsB;
            FourTransposedPoints edgesB;
            FourTransposedPoints perpsB;
            CalcTrianglePlanes(t0, t1, t2, triangleNormal, out vertsB, out edgesB, out perpsB);

            float3 halfExtents = boxA->Size * 0.5f + maxDistance;

            // find the closest minkowski plane
            float4 plane;
            {
                // Box face vs triangle vertex
                float4 planeFaceVertex;
                {
                    // get aabb of minkowski diff
                    float3 tMin = math.min(math.min(t0, t1), t2) - halfExtents;
                    float3 tMax = math.max(math.max(t0, t1), t2) + halfExtents;

                    // find the aabb face closest to the origin
                    float3 axis0 = new float3(1, 0, 0);
                    float3 axis1 = axis0.zxy; // 010
                    float3 axis2 = axis0.yzx; // 001

                    float4 planeX = SelectMaxW(new float4(axis0, -tMax.x), new float4(-axis0, tMin.x));
                    float4 planeY = SelectMaxW(new float4(axis1, -tMax.y), new float4(-axis1, tMin.y));
                    float4 planeZ = SelectMaxW(new float4(axis2, -tMax.z), new float4(-axis2, tMin.z));

                    planeFaceVertex = SelectMaxW(planeX, planeY);
                    planeFaceVertex = SelectMaxW(planeFaceVertex, planeZ);
                }

                // Box vertex vs triangle face
                float4 planeVertexFace;
                {
                    // Calculate the triangle normal 
                    float triangleOffset = math.dot(triangleNormal, t0);
                    float expansionOffset = math.dot(math.abs(triangleNormal), halfExtents);
                    planeVertexFace = SelectMaxW(
                        new float4(triangleNormal, -triangleOffset - expansionOffset),
                        new float4(-triangleNormal, triangleOffset - expansionOffset));
                }

                // Edge planes
                float4 planeEdgeEdge = new float4(0, 0, 0, -float.MaxValue);
                {
                    // Test the planes from crossing axis i with each edge of the triangle, for example if i = 1 then n0 is from (0, 1, 0) x (t1 - t0).
                    for (int i = 0, j = 1, k = 2; i < 3; j = k, k = i, i++)
                    {
                        // Normalize the cross product and flip it to point outward from the edge
                        float4 lengthsSq = edgesB.GetComponent(j) * edgesB.GetComponent(j) + edgesB.GetComponent(k) * edgesB.GetComponent(k);
                        float4 invLengths = math.rsqrt(lengthsSq);
                        float4 dots = edgesB.GetComponent(j) * perpsB.GetComponent(k) - edgesB.GetComponent(k) * perpsB.GetComponent(j);
                        float4 factors = invLengths * math.sign(dots);

                        float4 nj = -edgesB.GetComponent(k) * factors;
                        float4 nk = edgesB.GetComponent(j) * factors;
                        float4 distances = -nj * vertsB.GetComponent(j) - nk * vertsB.GetComponent(k) - math.abs(nj) * halfExtents[j] - math.abs(nk) * halfExtents[k];

                        // If the box edge is parallel to the triangle face then skip it, the plane is redundant with a vertex-face plane
                        bool4 valid = dots != float4.zero;
                        distances = math.select(Constants.Min4F, distances, valid);

                        float3 n0 = new float3(); n0[i] = 0.0f; n0[j] = nj[0]; n0[k] = nk[0];
                        float3 n1 = new float3(); n1[i] = 0.0f; n1[j] = nj[1]; n1[k] = nk[1];
                        float3 n2 = new float3(); n2[i] = 0.0f; n2[j] = nj[2]; n2[k] = nk[2];
                        float4 temp = SelectMaxW(SelectMaxW(new float4(n0, distances.x), new float4(n1, distances.y)), new float4(n2, distances.z));
                        planeEdgeEdge = SelectMaxW(planeEdgeEdge, temp);
                    }
                }

                plane = SelectMaxW(SelectMaxW(planeFaceVertex, planeVertexFace), planeEdgeEdge);
            }

            manifold = new Manifold();

            // Check for a separating plane TODO.ma could early out as soon as any plane with w>0 is found
            if (plane.w <= 0.0f)
            {
                // Get the normal and supporting faces
                float3 normalInA = math.mul(boxA->Orientation, plane.xyz);
                manifold.Normal = math.mul(worldFromA.Rotation, normalInA);
                int faceIndexA = boxA->ConvexHull.GetSupportingFace(-normalInA);
                int faceIndexB = triangleB->ConvexHull.GetSupportingFace(math.mul(math.transpose(aFromB.Rotation), normalInA));

                // Build manifold
                if (!FaceFace(ref boxA->ConvexHull, ref triangleB->ConvexHull, faceIndexA, faceIndexB, worldFromA, aFromB, normalInA, float.MaxValue, ref manifold))
                {
                    // TODO.ma - the closest points are vertices, we need GJK to find them.
                    // We could call GJK here, or always use GJK instead of BoxTriangle(), or do nothing and live with possible tunneling.
                    // This method is ~2x faster than GJK, so if it gets the right result without calling GJK most of the time, it's worth keeping even if we call GJK when it fails.
                }
            }
        }

        // Create a single point manifold between a triangle and sphere in world space.
        public static unsafe void TriangleSphere(
            PolygonCollider* triangleA, SphereCollider* sphereB,
            MTransform worldFromA, MTransform aFromB, float maxDistance,
            out Manifold manifold)
        {
            Assert.IsTrue(triangleA->Vertices.Length == 3);

            DistanceQueries.Result convexDistance = DistanceQueries.TriangleSphere(
                triangleA->Vertices[0], triangleA->Vertices[1], triangleA->Vertices[2], triangleA->Planes[0].Normal,
                sphereB->Center, sphereB->Radius, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                manifold = new Manifold(convexDistance, worldFromA);
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create contact points for a capsule and triangle in world space.
        public static unsafe void CapsuleTriangle(
            CapsuleCollider* capsuleA, PolygonCollider* triangleB,
            MTransform worldFromA, MTransform aFromB, float maxDistance,
            out Manifold manifold)
        {
            Assert.IsTrue(triangleB->Vertices.Length == 3);

            DistanceQueries.Result convexDistance = DistanceQueries.CapsuleTriangle(capsuleA, triangleB, aFromB);
            if (convexDistance.Distance < maxDistance)
            {
                // Build manifold
                manifold = new Manifold
                {
                    Normal = math.mul(worldFromA.Rotation, -convexDistance.NormalInA) // negate the normal because we are temporarily flipping to triangle A capsule B
                };
                MTransform worldFromB = Mul(worldFromA, aFromB);
                MTransform bFromA = Inverse(aFromB);
                float3 normalInB = math.mul(bFromA.Rotation, convexDistance.NormalInA);
                int faceIndexB = triangleB->ConvexHull.GetSupportingFace(normalInB);
                if (FaceEdge(ref triangleB->ConvexHull, ref capsuleA->ConvexHull, faceIndexB, worldFromB, bFromA, -normalInB, convexDistance.Distance + capsuleA->Radius, ref manifold))
                {
                    manifold.Flip();
                }
                else
                {
                    manifold = new Manifold(convexDistance, worldFromA);
                }
            }
            else
            {
                manifold = new Manifold();
            }
        }

        // Create contact points for a pair of generic convex hulls in world space.
        public static unsafe void ConvexConvex(
            ref ConvexHull hullA, ref ConvexHull hullB,
            MTransform worldFromA, MTransform aFromB, float maxDistance,
            out Manifold manifold)
        {
            // Get closest points on the hulls
            ConvexConvexDistanceQueries.Result result = ConvexConvexDistanceQueries.ConvexConvex(
                hullA.VerticesPtr, hullA.NumVertices, hullB.VerticesPtr, hullB.NumVertices, aFromB, ConvexConvexDistanceQueries.PenetrationHandling.Exact3D);

            float sumRadii = hullB.ConvexRadius + hullA.ConvexRadius;
            if (result.ClosestPoints.Distance < maxDistance + sumRadii)
            {
                float3 normal = result.ClosestPoints.NormalInA;

                manifold = new Manifold
                {
                    Normal = math.mul(worldFromA.Rotation, normal)
                };

                if (hullA.NumFaces > 0)
                {
                    int faceIndexA = hullA.GetSupportingFace(-normal, result.SimplexVertexA(0));
                    if (hullB.NumFaces > 0)
                    {
                        // Convex vs convex
                        int faceIndexB = hullB.GetSupportingFace(math.mul(math.transpose(aFromB.Rotation), normal), result.SimplexVertexB(0));
                        if (FaceFace(ref hullA, ref hullB, faceIndexA, faceIndexB, worldFromA, aFromB, normal, result.ClosestPoints.Distance, ref manifold))
                        {
                            return;
                        }
                    }
                    else if (hullB.NumVertices == 2)
                    {
                        // Convex vs capsule
                        if (FaceEdge(ref hullA, ref hullB, faceIndexA, worldFromA, aFromB, normal, result.ClosestPoints.Distance, ref manifold))
                        {
                            return;
                        }
                    } // Else convex vs sphere
                }
                else if (hullA.NumVertices == 2)
                {
                    if (hullB.NumFaces > 0)
                    {
                        // Capsule vs convex
                        manifold.Normal = math.mul(worldFromA.Rotation, -normal); // negate the normal because we are temporarily flipping to triangle A capsule B
                        MTransform worldFromB = Mul(worldFromA, aFromB);
                        MTransform bFromA = Inverse(aFromB);
                        float3 normalInB = math.mul(bFromA.Rotation, normal);
                        int faceIndexB = hullB.GetSupportingFace(normalInB, result.SimplexVertexB(0));
                        bool foundClosestPoint = FaceEdge(ref hullB, ref hullA, faceIndexB, worldFromB, bFromA, -normalInB, result.ClosestPoints.Distance, ref manifold);
                        manifold.Flip();
                        if (foundClosestPoint)
                        {
                            return;
                        }
                    } // Else capsule vs capsule or sphere
                } // Else sphere vs something

                // Either one of the shapes is a sphere, or both of the shapes are capsules, or both of the closest features are nearly perpendicular to the contact normal,
                // or FaceFace()/FaceEdge() missed the closest point due to numerical error.  In these cases, add the closest point directly to the manifold.
                if (manifold.NumContacts < Manifold.k_MaxNumContacts)
                {
                    DistanceQueries.Result convexDistance = result.ClosestPoints;
                    manifold[manifold.NumContacts++] = new ContactPoint
                    {
                        Position = Mul(worldFromA, convexDistance.PositionOnAinA) - manifold.Normal * (convexDistance.Distance - hullB.ConvexRadius),
                        Distance = convexDistance.Distance - sumRadii
                    };
                }
            }
            else
            {
                manifold = new Manifold();
            }
        }

        #endregion

        #region Helpers

        // BoxBox() helper
        private static bool PointPlanes(MTransform aFromB, float3 halfExtA, float3 halfExtB, float maxDistance, ref float3 normalOut, ref float distanceOut)
        {
            // Calculate the AABB of box B in A-space
            Aabb aabbBinA;
            {
                Aabb aabbBinB = new Aabb { Min = -halfExtB, Max = halfExtB };
                aabbBinA = Math.TransformAabb(aFromB, aabbBinB);
            }

            // Check for a miss
            float3 toleranceHalfExt = halfExtA + maxDistance;
            bool3 miss = (aabbBinA.Min > toleranceHalfExt) | (-toleranceHalfExt > aabbBinA.Max);
            if (math.any(miss))
            {
                return false;
            }

            // Return the normal with minimum separating distance
            float3 diff0 = aabbBinA.Min - halfExtA; // positive normal
            float3 diff1 = -aabbBinA.Max - halfExtA; // negative normal
            bool3 greater01 = diff0 > diff1;
            float3 max01 = math.select(diff1, diff0, greater01);
            distanceOut = math.cmax(max01);

            int axis = IndexOfMaxComponent(max01);
            if (axis == 0)
            {
                normalOut = new float3(1.0f, 0.0f, 0.0f);
            }
            else if (axis == 1)
            {
                normalOut = new float3(0.0f, 1.0f, 0.0f);
            }
            else
            {
                normalOut = new float3(0.0f, 0.0f, 1.0f);
            }
            normalOut = math.select(normalOut, -normalOut, greater01);

            return true;
        }

        // returns the argument with greater w component
        private static float4 SelectMaxW(float4 a, float4 b)
        {
            return math.select(b, a, a.w > b.w);
        }

        private static void CalcTrianglePlanes(float3 v0, float3 v1, float3 v2, float3 normalDirection,
            out FourTransposedPoints verts, out FourTransposedPoints edges, out FourTransposedPoints perps)
        {
            verts = new FourTransposedPoints(v0, v1, v2, v0);
            edges = verts.V1230 - verts;
            perps = edges.Cross(new FourTransposedPoints(normalDirection));
        }

        #endregion

        #region Multiple contact generation

        // Iterates over the edges of a face
        private unsafe struct EdgeIterator
        {
            // Current edge
            public float3 Vertex0 { get; private set; }
            public float3 Vertex1 { get; private set; }
            public float3 Edge { get; private set; }
            public float3 Perp { get; private set; }
            public float Offset { get; private set; }
            public int Index { get; private set; }

            // Face description
            private float3* vertices;
            private byte* indices;
            private float3 normal;
            private int count;

            public static unsafe EdgeIterator Begin(float3* vertices, byte* indices, float3 normal, int count)
            {
                EdgeIterator iterator = new EdgeIterator();
                iterator.vertices = vertices;
                iterator.indices = indices;
                iterator.normal = normal;
                iterator.count = count;

                iterator.Vertex1 = (indices == null) ? vertices[count - 1] : vertices[indices[count - 1]];
                iterator.update();
                return iterator;
            }

            public bool Valid()
            {
                return Index < count;
            }

            public void Advance()
            {
                Index++;
                update();
            }

            private void update()
            {
                Vertex0 = Vertex1;
                Vertex1 = (indices == null) ? vertices[Index] : vertices[indices[Index]];

                Edge = Vertex1 - Vertex0;
                Perp = math.cross(Edge, normal); // points outwards from face
                Offset = math.dot(Perp, Vertex1);
            }
        }

        // Cast ray originA, directionA against plane normalB, offsetB and update the ray hit fractions
        private static void castRayPlane(float3 originA, float3 directionA, float3 normalB, float offsetB, ref float fracEnter, ref float fracExit)
        {
            // Cast edge A against plane B
            float start = math.dot(originA, normalB) - offsetB;
            float diff = math.dot(directionA, normalB);
            float end = start + diff;
            float frac = math.select(-start / diff, 0.0f, diff == 0.0f);

            bool startInside = (start <= 0.0f);
            bool endInside = (end <= 0.0f);

            bool enter = !startInside & (frac > fracEnter);
            fracEnter = math.select(fracEnter, frac, enter);

            bool exit = !endInside & (frac < fracExit);
            fracExit = math.select(fracExit, frac, exit);

            bool hit = startInside | endInside;
            fracEnter = math.select(fracExit, fracEnter, hit); // mark invalid with enter <= exit in case of a miss
        }

        // If the rejections of the faces from the contact normal are just barely touching, then FaceFace() might miss the closest points because of numerical error.
        // FaceFace() and FaceEdge() check if they found a point as close as the closest, and if not they return false so that the caller can add it.
        private const float closestDistanceTolerance = 1e-4f;

        // Tries to generate a manifold between a pair of faces.  It can fail in some cases due to numerical accuracy:
        // 1) both faces are nearly perpendicular to the normal
        // 2) the closest features on the shapes are vertices, so that the intersection of the projection of the faces to the plane perpendicular to the normal contains only one point
        // In those cases, FaceFace() returns false and the caller should generate a contact from the closest points on the shapes.
        private static unsafe bool FaceFace(
            ref ConvexHull convexA, ref ConvexHull convexB, int faceIndexA, int faceIndexB, MTransform worldFromA, MTransform aFromB,
            float3 normal, float distance, ref Manifold manifold)
        {
            // Get the plane of each face
            Plane planeA = convexA.Planes[faceIndexA];
            Plane planeB = TransformPlane(aFromB, convexB.Planes[faceIndexB]);

            // Handle cases where one of the faces is nearly perpendicular to the contact normal
            // This gets around divide by zero / numerical problems from dividing collider planes which often contain some error by a very small number, amplifying that error
            const float cosMaxAngle = 0.05f;
            float dotA = math.dot(planeA.Normal, normal);
            float dotB = math.dot(planeB.Normal, normal);
            bool acceptB = true; // true if vertices of B projected onto the face of A are accepted
            if (dotA > -cosMaxAngle)
            {
                // Handle cases where both faces are nearly perpendicular to the contact normal.
                if (dotB < cosMaxAngle)
                {
                    // Both faces are nearly perpendicular to the contact normal, let the caller generate a single contact
                    return false;
                }

                // Face of A is nearly perpendicular to the contact normal, don't try to project vertices onto it
                acceptB = false;
            }
            else if (dotB < cosMaxAngle)
            {
                // Face of B is nearly perpendicular to the normal, so we need to clip the edges of B against face A instead
                MTransform bFromA = Inverse(aFromB);
                float3 normalInB = math.mul(bFromA.Rotation, -normal);
                MTransform worldFromB = Mul(worldFromA, aFromB);
                bool result = FaceFace(ref convexB, ref convexA, faceIndexB, faceIndexA, worldFromB, bFromA, normalInB, distance, ref manifold);
                manifold.Normal = -manifold.Normal;
                manifold.Flip();
                return result;
            }

            // Check if the manifold gets a point roughly as close as the closest
            distance += closestDistanceTolerance;
            bool foundClosestPoint = false;

            // Transform vertices of B into A-space
            // Initialize validB, which is true for each vertex of B that is inside face A
            ConvexHull.Face faceA = convexA.Faces[faceIndexA];
            ConvexHull.Face faceB = convexB.Faces[faceIndexB];
            bool* validB = stackalloc bool[faceB.NumVertices];
            float3* verticesBinA = stackalloc float3[faceB.NumVertices];
            {
                byte* indicesB = convexB.FaceVertexIndicesPtr + faceB.FirstIndex;
                float3* verticesB = convexB.VerticesPtr;
                for (int i = 0; i < faceB.NumVertices; i++)
                {
                    validB[i] = acceptB;
                    verticesBinA[i] = Mul(aFromB, verticesB[indicesB[i]]);
                }
            }

            // For each edge of A
            float invDotB = math.rcp(dotB);
            float sumRadii = convexA.ConvexRadius + convexB.ConvexRadius;
            byte* indicesA = convexA.FaceVertexIndicesPtr + faceA.FirstIndex;
            float3* verticesA = convexA.VerticesPtr;
            for (EdgeIterator edgeA = EdgeIterator.Begin(verticesA, indicesA, -normal, faceA.NumVertices); edgeA.Valid(); edgeA.Advance())
            {
                float fracEnterA = 0.0f;
                float fracExitA = 1.0f;

                // For each edge of B
                for (EdgeIterator edgeB = EdgeIterator.Begin(verticesBinA, null, normal, faceB.NumVertices); edgeB.Valid(); edgeB.Advance())
                {
                    // Cast edge A against plane B and test if vertex B is inside plane A
                    castRayPlane(edgeA.Vertex0, edgeA.Edge, edgeB.Perp, edgeB.Offset, ref fracEnterA, ref fracExitA);
                    validB[edgeB.Index] &= (math.dot(edgeB.Vertex1, edgeA.Perp) < edgeA.Offset);
                }

                // If edge A hits B, add a contact points
                if (fracEnterA < fracExitA)
                {
                    float distance0 = (math.dot(edgeA.Vertex0, planeB.Normal) + planeB.Distance) * invDotB;
                    float deltaDistance = math.dot(edgeA.Edge, planeB.Normal) * invDotB;
                    float3 vertexAOnB = edgeA.Vertex0 - normal * distance0;
                    float3 edgeAOnB = edgeA.Edge - normal * deltaDistance;
                    foundClosestPoint |= AddEdgeContact(vertexAOnB, edgeAOnB, distance0, deltaDistance, fracEnterA, normal, convexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
                    if (fracExitA < 1.0f) // If the exit fraction is 1, then the next edge has the same contact point with enter fraction 0
                    {
                        foundClosestPoint |= AddEdgeContact(vertexAOnB, edgeAOnB, distance0, deltaDistance, fracExitA, normal, convexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
                    }
                }
            }

            // For each vertex of B
            float invDotA = math.rcp(dotA);
            for (int i = 0; i < faceB.NumVertices; i++)
            {
                if (validB[i] && manifold.NumContacts < Manifold.k_MaxNumContacts)
                {
                    float3 vertexB = verticesBinA[i];
                    float distanceB = (math.dot(vertexB, planeA.Normal) + planeA.Distance) * -invDotA;
                    manifold[manifold.NumContacts++] = new ContactPoint
                    {
                        Position = Mul(worldFromA, vertexB) + manifold.Normal * convexB.ConvexRadius,
                        Distance = distanceB - sumRadii
                    };
                    foundClosestPoint |= distanceB <= distance;
                }
            }

            return foundClosestPoint;
        }

        // Tries to generate a manifold between a face and an edge.  It can fail for the same reasons as FaceFace().
        // In those cases, FaceEdge() returns false and the caller should generate a contact from the closest points on the shapes.
        private static unsafe bool FaceEdge(
            ref ConvexHull faceConvexA, ref ConvexHull edgeConvexB, int faceIndexA, MTransform worldFromA, MTransform aFromB,
            float3 normal, float distance, ref Manifold manifold)
        {
            // Check if the face is nearly perpendicular to the normal
            const float cosMaxAngle = 0.05f;
            Plane planeA = faceConvexA.Planes[faceIndexA];
            float dotA = math.dot(planeA.Normal, normal);
            if (math.abs(dotA) < cosMaxAngle)
            {
                return false;
            }

            // Check if the manifold gets a point roughly as close as the closest
            distance += closestDistanceTolerance;
            bool foundClosestPoint = false;

            // Get the supporting face on A
            ConvexHull.Face faceA = faceConvexA.Faces[faceIndexA];
            byte* indicesA = faceConvexA.FaceVertexIndicesPtr + faceA.FirstIndex;

            // Get edge in B
            float3 vertexB0 = Math.Mul(aFromB, edgeConvexB.Vertices[0]);
            float3 edgeB = math.mul(aFromB.Rotation, edgeConvexB.Vertices[1] - edgeConvexB.Vertices[0]);

            // For each edge of A
            float3* verticesA = faceConvexA.VerticesPtr;
            float fracEnterB = 0.0f;
            float fracExitB = 1.0f;
            for (EdgeIterator edgeA = EdgeIterator.Begin(verticesA, indicesA, -normal, faceA.NumVertices); edgeA.Valid(); edgeA.Advance())
            {
                // Cast edge B against plane A
                castRayPlane(vertexB0, edgeB, edgeA.Perp, edgeA.Offset, ref fracEnterB, ref fracExitB);
            }

            // If edge B hits A, add a contact points
            if (fracEnterB < fracExitB)
            {
                float invDotA = math.rcp(dotA);
                float sumRadii = faceConvexA.ConvexRadius + edgeConvexB.ConvexRadius;
                float distance0 = (math.dot(vertexB0, planeA.Normal) + planeA.Distance) * -invDotA;
                float deltaDistance = math.dot(edgeB, planeA.Normal) * -invDotA;
                foundClosestPoint |= AddEdgeContact(vertexB0, edgeB, distance0, deltaDistance, fracEnterB, normal, edgeConvexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
                foundClosestPoint |= AddEdgeContact(vertexB0, edgeB, distance0, deltaDistance, fracExitB, normal, edgeConvexB.ConvexRadius, sumRadii, worldFromA, distance, ref manifold);
            }

            return foundClosestPoint;
        }

        // Adds a contact to the manifold from an edge and fraction
        private static bool AddEdgeContact(float3 vertex0, float3 edge, float distance0, float deltaDistance, float fraction, float3 normalInA, float radiusB, float sumRadii,
            MTransform worldFromA, float distanceThreshold, ref Manifold manifold)
        {
            if (manifold.NumContacts < Manifold.k_MaxNumContacts)
            {
                float3 position = vertex0 + fraction * edge;
                float distance = distance0 + fraction * deltaDistance;

                manifold[manifold.NumContacts++] = new ContactPoint
                {
                    Position = Mul(worldFromA, position + normalInA * radiusB),
                    Distance = distance - sumRadii
                };

                return distance <= distanceThreshold;
            }
            return false;
        }

        #endregion
    }
}
