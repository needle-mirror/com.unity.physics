using NUnit.Framework;
using Unity.Mathematics;
using Unity.Collections;
using Random = Unity.Mathematics.Random;
using static Unity.Physics.Math;
using Unity.Physics.Tests.Utils;

namespace Unity.Physics.Tests.Collision.Queries
{
    public class QueryTests
    {
        // These tests mostly work by comparing the results of different methods of calculating the same thing.
        // The results will not be exactly the same due to floating point inaccuracy, approximations in methods like convex-convex collider cast, etc.
        const float tolerance = 1e-3f;

        //
        // Query result validation
        //

        static unsafe float3 getSupport(ref ConvexHull hull, float3 direction)
        {
            float4 best = new float4(hull.Vertices[0], math.dot(hull.Vertices[0], direction));
            for (int i = 1; i < hull.Vertices.Length; i++)
            {
                float dot = math.dot(hull.Vertices[i], direction);
                best = math.select(best, new float4(hull.Vertices[i], dot), dot > best.w);
            }
            return best.xyz;
        }

        static void ValidateDistanceResult(DistanceQueries.Result result, ref ConvexHull a, ref ConvexHull b, MTransform aFromB, float referenceDistance, string failureMessage)
        {
            float adjustedTolerance = tolerance;
            if (result.Distance < a.ConvexRadius + b.ConvexRadius)
            {
                // Core shape penetration distances are less accurate, and error scales with the number of vertices (as well as shape size). See stopThreshold in ConvexConvexDistanceQueries.
                // This is not usually noticeable in rigid body simulation because accuracy improves as the penetration resolves.
                // The tolerance is tuned for these tests, it might require further tuning as the tests change.
                adjustedTolerance = 1e-2f + 1e-3f * (a.NumVertices + b.NumVertices);
            }

            // Check that the distances is correct
            Assert.AreEqual(result.Distance, referenceDistance, adjustedTolerance, failureMessage + ": incorrect distance");

            // Check that the separating normal and closest point are consistent with the distance
            float3 tempA = getSupport(ref a, -result.NormalInA);
            float3 supportQuery = tempA - result.NormalInA * a.ConvexRadius;
            float3 tempB = Math.Mul(aFromB, getSupport(ref b, math.mul(aFromB.InverseRotation, result.NormalInA)));
            float3 supportTarget = tempB + result.NormalInA * b.ConvexRadius;

            float supportQueryDot = math.dot(supportQuery, result.NormalInA);
            float supportTargetDot = math.dot(supportTarget, result.NormalInA);
            float supportDistance = supportQueryDot - supportTargetDot;
            Assert.AreEqual(result.Distance, supportDistance, adjustedTolerance, failureMessage + ": incorrect normal");

            float positionDot = math.dot(result.PositionOnAinA, result.NormalInA);
            Assert.AreEqual(supportQueryDot, positionDot, adjustedTolerance, failureMessage + ": incorrect position");
        }

        //
        // Reference implementations of queries using simple brute-force methods
        //

        static unsafe float RefConvexConvexDistance(ref ConvexHull a, ref ConvexHull b, MTransform aFromB)
        {
            bool success = false;
            if (a.NumVertices + b.NumVertices < 64) // TODO - work around hull builder asserts
            {
                // Build the minkowski difference in a-space
                int maxNumVertices = a.NumVertices * b.NumVertices;
                ConvexHullBuilder diff = new ConvexHullBuilder(maxNumVertices, 2 * maxNumVertices, Allocator.Temp);
                Aabb aabb = Aabb.Empty;
                for (int iB = 0; iB < b.NumVertices; iB++)
                {
                    float3 vertexB = Math.Mul(aFromB, b.Vertices[iB]);
                    for (int iA = 0; iA < a.NumVertices; iA++)
                    {
                        float3 vertexA = a.Vertices[iA];
                        aabb.Include(vertexA - vertexB);
                    }
                }
                diff.IntegerSpaceAabb = aabb;
                success = true;
                for (int iB = 0; iB < b.NumVertices; iB++)
                {
                    float3 vertexB = Math.Mul(aFromB, b.Vertices[iB]);
                    for (int iA = 0; iA < a.NumVertices; iA++)
                    {
                        float3 vertexA = a.Vertices[iA];
                        if (!diff.AddPoint(vertexA - vertexB, (uint)(iA | iB << 16)))
                        {
                            // TODO - coplanar vertices are tripping up ConvexHullBuilder, we should fix it but for now fall back to DistanceQueries.ConvexConvex()
                            success = false;
                        }
                    }
                }

                float distance = 0.0f;
                if (success && diff.Triangles.GetFirstIndex() != -1)
                {
                    // Find the closest triangle to the origin
                    distance = float.MaxValue;
                    bool penetrating = true;
                    for (int t = diff.Triangles.GetFirstIndex(); t != -1; t = diff.Triangles.GetNextIndex(t))
                    {
                        ConvexHullBuilder.Triangle triangle = diff.Triangles[t];
                        float3 v0 = diff.Vertices[triangle.GetVertex(0)].Position;
                        float3 v1 = diff.Vertices[triangle.GetVertex(1)].Position;
                        float3 v2 = diff.Vertices[triangle.GetVertex(2)].Position;
                        float3 n = diff.ComputePlane(t).Normal;
                        DistanceQueries.Result result = DistanceQueries.TriangleSphere(v0, v1, v2, n, float3.zero, 0.0f, MTransform.Identity);
                        if (result.Distance < distance)
                        {
                            distance = result.Distance;
                        }
                        penetrating = penetrating & (math.dot(n, -result.NormalInA) < 0.0f); // only penetrating if inside of all planes
                    }

                    if (penetrating)
                    {
                        distance = -distance;
                    }

                    distance -= a.ConvexRadius + b.ConvexRadius;
                }
                else
                {
                    success = false;
                }

                diff.Dispose();

                if (success)
                {
                    return distance;
                }
            }

            // Fall back in case hull isn't 3D or hull builder fails
            // Most of the time this happens for cases like sphere-sphere, capsule-capsule, etc. which have special implementations,
            // so comparing those to GJK still validates the results of different API queries against each other.
            return DistanceQueries.ConvexConvex(ref a, ref b, aFromB).Distance;
        }

        private static unsafe void TestConvexConvexDistance(ConvexCollider* target, ConvexCollider* query, MTransform queryFromTarget, string failureMessage)
        {
            // Do the query, API version and reference version, then validate the result
            DistanceQueries.Result result = DistanceQueries.ConvexConvex((Collider*)query, (Collider*)target, queryFromTarget);
            float referenceDistance = RefConvexConvexDistance(ref query->ConvexHull, ref target->ConvexHull, queryFromTarget);
            ValidateDistanceResult(result, ref query->ConvexHull, ref target->ConvexHull, queryFromTarget, referenceDistance, failureMessage);
        }

        // This test generates random shape pairs, queries the distance between them, and validates some properties of the results:
        // - Distance is compared against a slow reference implementation of the closest distance query
        // - Closest points are on the plane through the support vertices in the normal direction
        // If the test fails, it will report a seed.  Set dbgTest to the seed to run the failing case alone.
        [Test]
        public unsafe void ConvexConvexDistanceTest()
        {
            Random rnd = new Random(0x12345678);
            uint dbgTest = 0;

            int numTests = 5000;
            if (dbgTest > 0)
            {
                numTests = 1;
            }

            for (int i = 0; i < numTests; i++)
            {
                // Save state to repro this query without doing everything that came before it
                if (dbgTest > 0)
                {
                    rnd.state = dbgTest;
                }
                uint state = rnd.state;

                // Generate random query inputs
                var target = TestUtils.GenerateRandomConvex(ref rnd);
                var query = TestUtils.GenerateRandomConvex(ref rnd);
                MTransform queryFromTarget = new MTransform(
                    (rnd.NextInt(10) > 0) ? rnd.NextQuaternionRotation() : quaternion.identity,
                    rnd.NextFloat3(-3.0f, 3.0f));
                TestConvexConvexDistance((ConvexCollider*)target.GetUnsafePtr(), (ConvexCollider*)query.GetUnsafePtr(), queryFromTarget, "ConvexConvexDistanceTest failed " + i + " (" + state.ToString() + ")");
            }
        }

        // This test generates random shapes and queries distance to themselves using small or identity transforms.  This hits edge
        // cases in collision detection routines where the two shapes being tested have equal or nearly equal features.  The results
        // are validated in the same way as those in ConvexConvexDistanceTest().
        // If the test fails, it will report a pair of seeds.  Set dbgShape to the first and dbgTest to the second to run the failing case alone.
        [Test]
        public unsafe void ConvexConvexDistanceEdgeCaseTest()
        {
            Random rnd = new Random(0x90456148);
            uint dbgShape = 0;
            uint dbgTest = 0;

            int numShapes = 500;
            int numTests = 50;
            if (dbgShape > 0)
            {
                numShapes = 1;
                numTests = 1;
            }

            for (int iShape = 0; iShape < numShapes; iShape++)
            {
                if (dbgShape > 0)
                {
                    rnd.state = dbgShape;
                }
                uint shapeState = rnd.state;

                // Generate a random collider
                var collider = TestUtils.GenerateRandomConvex(ref rnd);

                for (int iTest = 0; iTest < numTests; iTest++)
                {
                    if (dbgTest > 0)
                    {
                        rnd.state = dbgTest;
                    }
                    uint testState = rnd.state;

                    // Generate random transform
                    float distance = math.pow(10.0f, rnd.NextFloat(-15.0f, -1.0f));
                    float angle = math.pow(10.0f, rnd.NextFloat(-15.0f, 0.0f));
                    MTransform queryFromTarget = new MTransform(
                        (rnd.NextInt(10) > 0) ? quaternion.AxisAngle(rnd.NextFloat3Direction(), angle) : quaternion.identity,
                        (rnd.NextInt(10) > 0) ? rnd.NextFloat3Direction() * distance : float3.zero);
                    TestConvexConvexDistance((ConvexCollider*)collider.GetUnsafePtr(), (ConvexCollider*)collider.GetUnsafePtr(), queryFromTarget, "ConvexConvexDistanceEdgeCaseTest failed " + iShape + ", " + iTest + " (" + shapeState+ ", " + testState + ")");
                }
            }
        }

        static DistanceQueries.Result DistanceResultFromDistanceHit(DistanceHit hit, MTransform queryFromWorld)
        {
            return new DistanceQueries.Result
            {
                PositionOnAinA = Math.Mul(queryFromWorld, hit.Position + hit.SurfaceNormal * hit.Distance),
                NormalInA = math.mul(queryFromWorld.Rotation, hit.SurfaceNormal),
                Distance = hit.Distance
            };
        }

        static unsafe void GetHitLeaf(ref Physics.PhysicsWorld world, int rigidBodyIndex, ColliderKey colliderKey, MTransform queryFromWorld, out ChildCollider leaf, out MTransform queryFromTarget)
        {
            Physics.RigidBody body = world.Bodies[rigidBodyIndex];
            Collider.GetLeafCollider(body.Collider, body.WorldFromBody, colliderKey, out leaf);
            MTransform worldFromLeaf = new MTransform(leaf.TransformFromChild);
            queryFromTarget = Math.Mul(queryFromWorld, worldFromLeaf);
        }

        // Does distance queries and checks some properties of the results:
        // - Closest hit returned from the all hits query has the same fraction as the hit returned from the closest hit query
        // - Any hit and closest hit queries return a hit if and only if the all hits query does
        // - Hit distance is the same as the support distance in the hit normal direction
        // - Fetching the shapes from any world query hit and querying them directly gives a matching result
        static unsafe void WorldCalculateDistanceTest(ref Physics.PhysicsWorld world, ColliderDistanceInput input, ref NativeList<DistanceHit> hits, string failureMessage)
        {
            // Do an all-hits query
            hits.Clear();
            world.CalculateDistance(input, ref hits);

            // Check each hit and find the closest
            float closestDistance = float.MaxValue;
            MTransform queryFromWorld = Math.Inverse(new MTransform(input.Transform));
            for (int iHit = 0; iHit < hits.Length; iHit++)
            {
                DistanceHit hit = hits[iHit];
                closestDistance = math.min(closestDistance, hit.Distance);

                // Fetch the leaf collider and query it directly
                ChildCollider leaf;
                MTransform queryFromTarget;
                GetHitLeaf(ref world, hit.RigidBodyIndex, hit.ColliderKey, queryFromWorld, out leaf, out queryFromTarget);
                float referenceDistance = DistanceQueries.ConvexConvex(input.Collider, leaf.Collider, queryFromTarget).Distance;

                // Compare to the world query result
                DistanceQueries.Result result = DistanceResultFromDistanceHit(hit, queryFromWorld);
                ValidateDistanceResult(result, ref ((ConvexCollider*)input.Collider)->ConvexHull, ref ((ConvexCollider*)leaf.Collider)->ConvexHull, queryFromTarget, referenceDistance,
                    failureMessage + ", hits[" + iHit + "]");
            }

            // Do a closest-hit query and check that the distance matches
            DistanceHit closestHit;
            bool hasClosestHit = world.CalculateDistance(input, out closestHit);
            if (hits.Length == 0)
            {
                Assert.IsFalse(hasClosestHit, failureMessage + ", closestHit: no matching result in hits");
            }
            else
            {
                ChildCollider leaf;
                MTransform queryFromTarget;
                GetHitLeaf(ref world, closestHit.RigidBodyIndex, closestHit.ColliderKey, queryFromWorld, out leaf, out queryFromTarget);

                DistanceQueries.Result result = DistanceResultFromDistanceHit(closestHit, queryFromWorld);
                ValidateDistanceResult(result, ref ((ConvexCollider*)input.Collider)->ConvexHull, ref ((ConvexCollider*)leaf.Collider)->ConvexHull, queryFromTarget, closestDistance,
                    failureMessage + ", closestHit");
            }

            // Do an any-hit query and check that it is consistent with the others
            bool hasAnyHit = world.CalculateDistance(input);
            Assert.AreEqual(hasAnyHit, hasClosestHit, failureMessage + ": any hit result inconsistent with the others");

            // TODO - this test can't catch false misses.  We could do brute-force broadphase / midphase search to cover those.
        }

        static unsafe void CheckColliderCastHit(ref Physics.PhysicsWorld world, ColliderCastInput input, ColliderCastHit hit, string failureMessage)
        {
            // Fetch the leaf collider and convert the shape cast result into a distance result at the hit transform 
            ChildCollider leaf;
            MTransform queryFromWorld = Math.Inverse(new MTransform(input.Orientation, math.lerp(input.Start, input.End, hit.Fraction)));
            GetHitLeaf(ref world, hit.RigidBodyIndex, hit.ColliderKey, queryFromWorld, out leaf, out MTransform queryFromTarget);
            DistanceQueries.Result result = new DistanceQueries.Result
            {
                PositionOnAinA = Math.Mul(queryFromWorld, hit.Position),
                NormalInA = math.mul(queryFromWorld.Rotation, hit.SurfaceNormal),
                Distance = 0.0f
            };

            // If the fraction is zero then the shapes should penetrate, otherwise they should have zero distance
            if (hit.Fraction == 0.0f)
            {
                // Do a distance query to verify initial penetration
                result.Distance = DistanceQueries.ConvexConvex(input.Collider, leaf.Collider, queryFromTarget).Distance;
                Assert.Less(result.Distance, tolerance, failureMessage + ": zero fraction with positive distance");
            }

            // Verify the distance at the hit transform
            ValidateDistanceResult(result, ref ((ConvexCollider*)input.Collider)->ConvexHull, ref ((ConvexCollider*)leaf.Collider)->ConvexHull, queryFromTarget, result.Distance, failureMessage);
        }

        // Does collider casts and checks some properties of the results:
        // - Closest hit returned from the all hits query has the same fraction as the hit returned from the closest hit query
        // - Any hit and closest hit queries return a hit if and only if the all hits query does
        // - Distance between the shapes at the hit fraction is zero
        static unsafe void WorldColliderCastTest(ref Physics.PhysicsWorld world, ColliderCastInput input, ref NativeList<ColliderCastHit> hits, string failureMessage)
        {
            // Do an all-hits query
            hits.Clear();
            world.CastCollider(input, ref hits);

            // Check each hit and find the earliest
            float minFraction = float.MaxValue;
            RigidTransform worldFromQuery = new RigidTransform(input.Orientation, input.Start);
            for (int iHit = 0; iHit < hits.Length; iHit++)
            {
                ColliderCastHit hit = hits[iHit];
                minFraction = math.min(minFraction, hit.Fraction);
                CheckColliderCastHit(ref world, input, hit, failureMessage + ", hits[" + iHit + "]");
            }

            // Do a closest-hit query and check that the fraction matches
            ColliderCastHit closestHit;
            bool hasClosestHit = world.CastCollider(input, out closestHit);
            if (hits.Length == 0)
            {
                Assert.IsFalse(hasClosestHit, failureMessage + ", closestHit: no matching result in hits");
            }
            else
            {
                Assert.AreEqual(closestHit.Fraction, minFraction, tolerance * math.length(input.Ray.Displacement), failureMessage + ", closestHit: fraction does not match");
                CheckColliderCastHit(ref world, input, closestHit, failureMessage + ", closestHit");
            }

            // Do an any-hit query and check that it is consistent with the others
            bool hasAnyHit = world.CastCollider(input);
            Assert.AreEqual(hasAnyHit, hasClosestHit, failureMessage + ": any hit result inconsistent with the others");

            // TODO - this test can't catch false misses.  We could do brute-force broadphase / midphase search to cover those.
        }

        static unsafe void CheckRaycastHit(ref Physics.PhysicsWorld world, RaycastInput input, RaycastHit hit, string failureMessage)
        {
            // Fetch the leaf collider
            ChildCollider leaf;
            {
                Physics.RigidBody body = world.Bodies[hit.RigidBodyIndex];
                Collider.GetLeafCollider(body.Collider, body.WorldFromBody, hit.ColliderKey, out leaf);
            }

            // Check that the hit position matches the fraction
            float3 hitPosition = math.lerp(input.Start, input.End, hit.Fraction);
            Assert.Less(math.length(hitPosition - hit.Position), tolerance, failureMessage + ": inconsistent fraction and position");

            // Query the hit position and check that it's on the surface of the shape
            PointDistanceInput pointInput = new PointDistanceInput
            {
                Position = math.transform(math.inverse(leaf.TransformFromChild), hit.Position),
                MaxDistance = float.MaxValue
            };
            DistanceHit distanceHit;
            leaf.Collider->CalculateDistance(pointInput, out distanceHit);
            if (((ConvexCollider*)leaf.Collider)->ConvexHull.ConvexRadius > 0.0f)
            {
                // Convex raycast approximates radius, so it's possible that the hit position is not exactly on the shape, but must at least be outside
                Assert.Greater(distanceHit.Distance, -tolerance, failureMessage);
            }
            else
            {
                Assert.AreEqual(distanceHit.Distance, 0.0f, tolerance, failureMessage);
            }
        }

        // Does raycasts and checks some properties of the results:
        // - Closest hit returned from the all hits query has the same fraction as the hit returned from the closest hit query
        // - Any hit and closest hit queries return a hit if and only if the all hits query does
        // - All hits are on the surface of the hit shape
        static unsafe void WorldRaycastTest(ref Physics.PhysicsWorld world, RaycastInput input, ref NativeList<RaycastHit> hits, string failureMessage)
        {
            // Do an all-hits query
            hits.Clear();
            world.CastRay(input, ref hits);

            // Check each hit and find the earliest
            float minFraction = float.MaxValue;
            for (int iHit = 0; iHit < hits.Length; iHit++)
            {
                RaycastHit hit = hits[iHit];
                minFraction = math.min(minFraction, hit.Fraction);
                CheckRaycastHit(ref world, input, hit, failureMessage + ", hits[" + iHit + "]");
            }

            // Do a closest-hit query and check that the fraction matches
            RaycastHit closestHit;
            bool hasClosestHit = world.CastRay(input, out closestHit);
            if (hits.Length == 0)
            {
                Assert.IsFalse(hasClosestHit, failureMessage + ", closestHit: no matching result in hits");
            }
            else
            {
                Assert.AreEqual(closestHit.Fraction, minFraction, tolerance * math.length(input.Ray.Displacement), failureMessage + ", closestHit: fraction does not match");
                CheckRaycastHit(ref world, input, closestHit, failureMessage + ", closestHit");
            }

            // Do an any-hit query and check that it is consistent with the others
            bool hasAnyHit = world.CastRay(input);
            Assert.AreEqual(hasAnyHit, hasClosestHit, failureMessage + ": any hit result inconsistent with the others");

            // TODO - this test can't catch false misses.  We could do brute-force broadphase / midphase search to cover those.
        }

        // This test generates random worlds, queries them, and validates some properties of the query results.
        // See WorldCalculateDistanceTest, WorldColliderCastTest, and WorldRaycastTest for details about each query.
        // If the test fails, it will report a pair of seeds.  Set dbgWorld to the first and dbgTest to the second to run the failing case alone.
        [Test]
        public unsafe void WorldQueryTest()
        {
            const uint seed = 0x12345678;
            uint dbgWorld = 0; // set dbgWorld, dbgTest to the seed reported from a failure message to repeat the failing case alone
            uint dbgTest = 0;

            int numWorlds = 200;
            int numTests = 5000;
            if (dbgWorld > 0)
            {
                numWorlds = 1;
                numTests = 1;
            }

            Random rnd = new Random(seed);
            NativeList<DistanceHit> distanceHits = new NativeList<DistanceHit>(Allocator.Temp);
            NativeList<ColliderCastHit> colliderCastHits = new NativeList<ColliderCastHit>(Allocator.Temp);
            NativeList<RaycastHit> raycastHits = new NativeList<RaycastHit>(Allocator.Temp);

            for (int iWorld = 0; iWorld < numWorlds; iWorld++)
            {
                // Save state to repro this query without doing everything that came before it
                if (dbgWorld > 0)
                {
                    rnd.state = dbgWorld;
                }
                uint worldState = rnd.state;
                Physics.PhysicsWorld world = TestUtils.GenerateRandomWorld(ref rnd, rnd.NextInt(1, 20), 10.0f);

                for (int iTest = 0; iTest < (numTests / numWorlds); iTest++)
                {
                    if (dbgTest > 0)
                    {
                        rnd.state = dbgTest;
                    }
                    uint testState = rnd.state;
                    string failureMessage = iWorld + ", " + iTest + " (" + worldState.ToString() + ", " + testState.ToString() + ")";

                    // Generate common random query inputs
                    var collider = TestUtils.GenerateRandomConvex(ref rnd);
                    RigidTransform transform = new RigidTransform
                    {
                        pos = rnd.NextFloat3(-10.0f, 10.0f),
                        rot = (rnd.NextInt(10) > 0) ? rnd.NextQuaternionRotation() : quaternion.identity,
                    };
                    var startPos = transform.pos;
                    var endPos = startPos + rnd.NextFloat3(-5.0f, 5.0f);

                    // Distance test
                    {
                        ColliderDistanceInput input = new ColliderDistanceInput
                        {
                            Collider = (Collider*)collider.GetUnsafePtr(),
                            Transform = transform,
                            MaxDistance = (rnd.NextInt(4) > 0) ? rnd.NextFloat(5.0f) : 0.0f
                        };
                        WorldCalculateDistanceTest(ref world, input, ref distanceHits, "WorldQueryTest failed CalculateDistance " + failureMessage);
                    }

                    // Collider cast test
                    {
                        ColliderCastInput input = new ColliderCastInput
                        {
                            Collider = (Collider*)collider.GetUnsafePtr(),
                            Start = startPos,
                            End = endPos,
                            Orientation = transform.rot,
                        };
                        WorldColliderCastTest(ref world, input, ref colliderCastHits, "WorldQueryTest failed ColliderCast " + failureMessage);
                    }

                    // Ray cast test
                    {
                        RaycastInput input = new RaycastInput
                        {
                            Start = startPos,
                            End = endPos,
                            Filter = CollisionFilter.Default
                        };
                        WorldRaycastTest(ref world, input, ref raycastHits, "WorldQueryTest failed Raycast " + failureMessage);
                    }
                }

                world.Dispose(); // TODO leaking memory if the test fails
            }

            distanceHits.Dispose(); // TODO leaking memory if the test fails
            colliderCastHits.Dispose();
            raycastHits.Dispose();
        }

        // Tests that a contact point is on the surface of its shape
        static unsafe void CheckPointOnSurface(ref ChildCollider leaf, float3 position, string failureMessage)
        {
            float3 positionLocal = math.transform(math.inverse(leaf.TransformFromChild), position);
            leaf.Collider->CalculateDistance(new PointDistanceInput { Position = positionLocal, MaxDistance = float.MaxValue, Filter = Physics.CollisionFilter.Default }, out DistanceHit hit);
            Assert.Less(hit.Distance, tolerance, failureMessage + ": contact point outside of shape");
            Assert.Greater(hit.Distance, -((ConvexCollider*)leaf.Collider)->ConvexHull.ConvexRadius - tolerance, failureMessage + ": contact point inside of shape");
        }

        // Tests that the points of a manifold are all coplanar
        static unsafe void CheckManifoldFlat(ref ConvexConvexManifoldQueries.Manifold manifold, float3 normal, string failureMessage)
        {
            float3 point0 = manifold[0].Position + normal * manifold[0].Distance;
            float3 point1 = manifold[1].Position + normal * manifold[1].Distance;
            for (int i = 2; i < manifold.NumContacts; i++)
            {
                // Try to calculate a plane from points 0, 1, iNormal
                float3 point = manifold[i].Position + normal * manifold[i].Distance;
                float3 cross = math.cross(point - point0, point - point1);
                if (math.lengthsq(cross) > 1e-6f)
                {
                    // Test that each point in the manifold is on the plane
                    float3 faceNormal = math.normalize(cross);
                    float dot = math.dot(point0, faceNormal);
                    for (int j = 2; j < manifold.NumContacts; j++)
                    {
                        float3 testPoint = manifold[j].Position + normal * manifold[j].Distance;
                        Assert.AreEqual(dot, math.dot(faceNormal, testPoint), tolerance, failureMessage + " contact " + j);
                    }
                    break;
                }
            }
        }

        // This test generates random worlds, generates manifolds for every pair of bodies in the world, and validates some properties of the manifolds:
        // - should contain the closest point
        // - each body's contact points should be on that body's surface
        // - each body's contact points should all be coplanar
        // If the test fails, it will report a seed.  Set dbgWorld to that seed to run the failing case alone.
        [Test]
        public unsafe void ManifoldQueryTest()
        {
            const uint seed = 0x98765432;
            Random rnd = new Random(seed);
            int numWorlds = 1000;

            uint dbgWorld = 0;
            if (dbgWorld > 0)
            {
                numWorlds = 1;
            }

            for (int iWorld = 0; iWorld < numWorlds; iWorld++)
            {
                // Save state to repro this query without doing everything that came before it
                if (dbgWorld > 0)
                {
                    rnd.state = dbgWorld;
                }
                uint worldState = rnd.state;
                Physics.PhysicsWorld world = TestUtils.GenerateRandomWorld(ref rnd, rnd.NextInt(1, 20), 3.0f);

                // Manifold test
                // TODO would be nice if we could change the world collision tolerance
                for (int iBodyA = 0; iBodyA < world.NumBodies; iBodyA++)
                {
                    for (int iBodyB = iBodyA + 1; iBodyB < world.NumBodies; iBodyB++)
                    {
                        Physics.RigidBody bodyA = world.Bodies[iBodyA];
                        Physics.RigidBody bodyB = world.Bodies[iBodyB];
                        if (bodyA.Collider->Type == ColliderType.Mesh && bodyB.Collider->Type == ColliderType.Mesh)
                        {
                            continue; // TODO - no mesh-mesh manifold support yet
                        }

                        // Build manifolds
                        BlockStream contacts = new BlockStream(1, 0, Allocator.Temp);
                        BlockStream.Writer contactWriter = contacts;
                        contactWriter.BeginForEachIndex(0);
                        ManifoldQueries.BodyBody(ref world, new BodyIndexPair { BodyAIndex = iBodyA, BodyBIndex = iBodyB }, 1.0f, ref contactWriter);
                        contactWriter.EndForEachIndex();

                        // Read each manifold
                        BlockStream.Reader contactReader = contacts;
                        contactReader.BeginForEachIndex(0);
                        int manifoldIndex = 0;
                        while (contactReader.RemainingItemCount > 0)
                        {
                            string failureMessage = iWorld + " (" + worldState + ") " + iBodyA + " vs " + iBodyB + " #" + manifoldIndex;
                            manifoldIndex++;

                            // Read the manifold header
                            ContactHeader header = contactReader.Read<ContactHeader>();
                            ConvexConvexManifoldQueries.Manifold manifold = new ConvexConvexManifoldQueries.Manifold();
                            manifold.NumContacts = header.NumContacts;
                            manifold.Normal = header.Normal;

                            // Get the leaf shapes
                            ChildCollider leafA, leafB;
                            {
                                Collider.GetLeafCollider(bodyA.Collider, bodyA.WorldFromBody, header.ColliderKeys.ColliderKeyA, out leafA);
                                Collider.GetLeafCollider(bodyB.Collider, bodyB.WorldFromBody, header.ColliderKeys.ColliderKeyB, out leafB);
                            }

                            // Read each contact point
                            int minIndex = 0;
                            for (int iContact = 0; iContact < header.NumContacts; iContact++)
                            {
                                // Read the contact and find the closest
                                ContactPoint contact = contactReader.Read<ContactPoint>();
                                manifold[iContact] = contact;
                                if (contact.Distance < manifold[minIndex].Distance)
                                {
                                    minIndex = iContact;
                                }

                                // Check that the contact point is on or inside the shape
                                CheckPointOnSurface(ref leafA, contact.Position + manifold.Normal * contact.Distance, failureMessage + " contact " + iContact + " leaf A");
                                CheckPointOnSurface(ref leafB, contact.Position, failureMessage + " contact " + iContact + " leaf B");
                            }

                            // Check the closest point
                            // TODO - Box-box and box-triangle manifolds have special manifold generation code that trades some accuracy for performance, see comments in 
                            // ConvexConvexManifoldQueries.BoxBox() and BoxTriangle(). It may change later, until then they get an exception from the closest point distance test.
                            ColliderType typeA = leafA.Collider->Type;
                            ColliderType typeB = leafB.Collider->Type;
                            bool skipClosestPointTest =
                                (typeA == ColliderType.Box && (typeB == ColliderType.Box || typeB == ColliderType.Triangle)) ||
                                (typeB == ColliderType.Box && typeA == ColliderType.Triangle);
                            if (!skipClosestPointTest)
                            {
                                ContactPoint closestPoint = manifold[minIndex];
                                RigidTransform aFromWorld = math.inverse(leafA.TransformFromChild);
                                DistanceQueries.Result result = new DistanceQueries.Result
                                {
                                    PositionOnAinA = math.transform(aFromWorld, closestPoint.Position + manifold.Normal * closestPoint.Distance),
                                    NormalInA = math.mul(aFromWorld.rot, manifold.Normal),
                                    Distance = closestPoint.Distance
                                };

                                MTransform aFromB = new MTransform(math.mul(aFromWorld, leafB.TransformFromChild));
                                float referenceDistance = DistanceQueries.ConvexConvex(leafA.Collider, leafB.Collider, aFromB).Distance;
                                ValidateDistanceResult(result, ref ((ConvexCollider*)leafA.Collider)->ConvexHull, ref ((ConvexCollider*)leafB.Collider)->ConvexHull, aFromB, referenceDistance, failureMessage + " closest point");
                            }

                            // Check that the manifold is flat
                            CheckManifoldFlat(ref manifold, manifold.Normal, failureMessage + ": non-flat A");
                            CheckManifoldFlat(ref manifold, float3.zero, failureMessage + ": non-flat B");
                        }

                        contacts.Dispose();
                    }
                }

                world.Dispose(); // TODO leaking memory if the test fails
            }
        }
    }
}
