using System;
using System.Collections.Generic;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics.Extensions;
using Unity.Physics.Tests.Utils;

namespace Unity.Physics.Tests.Collision.Colliders
{
    /// <summary>
    /// Test class containing tests for the <see cref="CompoundCollider"/>
    /// </summary>
    class CompoundColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public BlobAssetReference<Collider> BoxCollider;

            [GenerateTestsForBurstCompatibility]
            public void Execute()
            {
                using var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(3, Allocator.Temp)
                    {
                        [0] = new CompoundCollider.ColliderBlobInstance { Collider = BoxCollider, CompoundFromChild = new RigidTransform(quaternion.identity, new float3(0)) },
                        [1] = new CompoundCollider.ColliderBlobInstance { Collider = BoxCollider, CompoundFromChild = new RigidTransform(quaternion.identity, new float3(1)) },
                        [2] = new CompoundCollider.ColliderBlobInstance { Collider = BoxCollider, CompoundFromChild = new RigidTransform(quaternion.identity, new float3(2)) },
                    };
                CompoundCollider.Create(children).Dispose();
            }
        }

        [Test]
        public void CreateCompound_WhenCalledFromBurstJob_DoesNotThrow()
        {
            // Create a unit box as child collider in the compound
            using var box = BoxCollider.Create(new BoxGeometry {Orientation = quaternion.identity});

            // create a compound collider from a job
            new CreateFromBurstJob
            {
                BoxCollider = box
            }.Run();
        }

        [Test]
        public void MassProperties_BuiltFromChildren_MatchesExpected()
        {
            void TestCompoundBox(RigidTransform transform)
            {
                // Create a unit box
                using var box = BoxCollider.Create(new BoxGeometry
                {
                    Center = transform.pos,
                    Orientation = transform.rot,
                    Size = new float3(1),
                    BevelRadius = 0.0f
                });

                // Create a compound of mini boxes, matching the volume of the single box
                using var miniBox = BoxCollider.Create(new BoxGeometry
                {
                    Center = float3.zero,
                    Orientation = quaternion.identity,
                    Size = new float3(0.5f, 0.5f, 0.5f),
                    BevelRadius = 0.0f
                });
                var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(8, Allocator.Temp)
                {
                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+0.25f, +0.25f, +0.25f))) },
                    [1] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-0.25f, +0.25f, +0.25f))) },
                    [2] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+0.25f, -0.25f, +0.25f))) },
                    [3] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+0.25f, +0.25f, -0.25f))) },
                    [4] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-0.25f, -0.25f, +0.25f))) },
                    [5] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(+0.25f, -0.25f, -0.25f))) },
                    [6] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-0.25f, +0.25f, -0.25f))) },
                    [7] = new CompoundCollider.ColliderBlobInstance { Collider = miniBox, CompoundFromChild = math.mul(transform, new RigidTransform(quaternion.identity, new float3(-0.25f, -0.25f, -0.25f))) }
                };
                using var compound = CompoundCollider.Create(children);
                children.Dispose();

                var boxMassProperties = box.Value.MassProperties;
                var compoundMassProperties = compound.Value.MassProperties;

                TestUtils.AreEqual(compoundMassProperties.Volume, boxMassProperties.Volume, 1e-3f);
                TestUtils.AreEqual(compoundMassProperties.AngularExpansionFactor, boxMassProperties.AngularExpansionFactor, 1e-3f);
                TestUtils.AreEqual(compoundMassProperties.MassDistribution.Transform.pos, boxMassProperties.MassDistribution.Transform.pos, 1e-3f);
                //TestUtils.AreEqual(compoundMassProperties.MassDistribution.Orientation, boxMassProperties.MassDistribution.Orientation, 1e-3f);   // TODO: Figure out why this differs, and if that is a problem
                TestUtils.AreEqual(compoundMassProperties.MassDistribution.InertiaTensor, boxMassProperties.MassDistribution.InertiaTensor, 1e-3f);
            }

            // Compare box with compound at various transforms
            TestCompoundBox(RigidTransform.identity);
            TestCompoundBox(new RigidTransform(quaternion.identity, new float3(1.0f, 2.0f, 3.0f)));
            TestCompoundBox(new RigidTransform(quaternion.EulerXYZ(0.5f, 1.0f, 1.5f), float3.zero));
            TestCompoundBox(new RigidTransform(quaternion.EulerXYZ(0.5f, 1.0f, 1.5f), new float3(1.0f, 2.0f, 3.0f)));
        }

        unsafe void ValidateCompoundCollider(BlobAssetReference<Collider> collider, int expectedChildCount)
        {
            // manually created colliders are unique by design
            Assert.IsTrue(collider.Value.IsUnique);

            Assert.AreEqual(ColliderType.Compound, collider.Value.Type);
            Assert.AreEqual(CollisionType.Composite, collider.Value.CollisionType);

            var compound = (CompoundCollider*)collider.GetUnsafePtr();
            var uniqueChildren = new HashSet<long>();
            for (var i = 0; i < compound->Children.Length; i++)
                uniqueChildren.Add((long)compound->Children[i].Collider);
            Assert.That(uniqueChildren.Count, Is.EqualTo(expectedChildCount));
        }

        [Test]
        public void CreateCompound_WithRepeatedInputs_ChildrenAreInstances()
        {
            BlobAssetReference<Collider> boxBlob = default;
            BlobAssetReference<Collider> capsuleBlob = default;
            BlobAssetReference<Collider> sphereBlob = default;
            BlobAssetReference<Collider> compoundBlob = default;
            BlobAssetReference<Collider> cloneBlob = default;
            try
            {
                // 3 unique instance inputs
                boxBlob = BoxCollider.Create(new BoxGeometry { Orientation = quaternion.identity, Size = new float3(1) });
                capsuleBlob = CapsuleCollider.Create(new CapsuleGeometry { Radius = 0.5f, Vertex0 = new float3(1f), Vertex1 = new float3(-1f) });
                sphereBlob = SphereCollider.Create(new SphereGeometry { Radius = 0.5f });
                var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(8, Allocator.Temp)
                {
                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3(0f)) },
                    [1] = new CompoundCollider.ColliderBlobInstance { Collider = capsuleBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3(1f)) },
                    [2] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3(2f)) },
                    [3] = new CompoundCollider.ColliderBlobInstance { Collider = sphereBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3(3f)) },
                    [4] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3(4f)) },
                    [5] = new CompoundCollider.ColliderBlobInstance { Collider = capsuleBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3(5f)) },
                    [6] = new CompoundCollider.ColliderBlobInstance { Collider = boxBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3(6f)) },
                    [7] = new CompoundCollider.ColliderBlobInstance { Collider = sphereBlob, CompoundFromChild = new RigidTransform(quaternion.identity, new float3(7f)) }
                };

                const int kExpectedChildCount = 3;
                compoundBlob = CompoundCollider.Create(children);
                ValidateCompoundCollider(compoundBlob, kExpectedChildCount);

                cloneBlob = compoundBlob.Value.Clone();
                ValidateCompoundCollider(cloneBlob, kExpectedChildCount);
            }
            finally
            {
                boxBlob.Dispose();
                capsuleBlob.Dispose();
                sphereBlob.Dispose();
                if (compoundBlob.IsCreated)
                    compoundBlob.Dispose();

                if (cloneBlob.IsCreated)
                    cloneBlob.Dispose();
            }
        }

#if ENABLE_UNITY_COLLECTIONS_CHECKS
        [Test]
        public void CompoundCollider_Create_WhenChildrenNotCreated_Throws() =>
            Assert.Throws<ArgumentException>(() => CompoundCollider.Create(default));

        [Test]
        public void CompoundCollider_Create_WhenChildrenEmpty_Throws() =>
            Assert.Throws<ArgumentException>(() => CompoundCollider.Create(new NativeArray<CompoundCollider.ColliderBlobInstance>(0, Allocator.Temp)));

        [Test]
        public void CompoundCollider_Create_WhenNestingLevelBreached_Throws()
        {
            var CreatedColliders = new NativeList<BlobAssetReference<Collider>>(Allocator.Temp);
            Assert.Throws<ArgumentException>(
                () =>
                {
                    int numSpheres = 17;
                    float sphereRadius = 0.5f;
                    float ringRadius = 2f;
                    BlobAssetReference<Collider> compound = default;
                    for (int i = 0; i < numSpheres; ++i)
                    {
                        var t = i / (float)numSpheres * 2f * math.PI;
                        var p = ringRadius * new float3(math.cos(t), 0f, math.sin(t));
                        var sphere = SphereCollider.Create(new SphereGeometry { Center = p, Radius = sphereRadius });
                        CreatedColliders.Add(sphere);
                        if (compound.IsCreated)
                        {
                            compound = CompoundCollider.Create(
                                new NativeArray<CompoundCollider.ColliderBlobInstance>(2, Allocator.Temp)
                                {
                                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = sphere, CompoundFromChild = RigidTransform.identity },
                                    [1] = new CompoundCollider.ColliderBlobInstance { Collider = compound, CompoundFromChild = RigidTransform.identity }
                                });
                        }
                        else
                        {
                            compound = CompoundCollider.Create(
                                new NativeArray<CompoundCollider.ColliderBlobInstance>(1, Allocator.Temp)
                                {
                                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = sphere, CompoundFromChild = RigidTransform.identity }
                                });
                        }
                        CreatedColliders.Add(compound);
                    }
                });

            for (int i = CreatedColliders.Length - 1; i >= 0; i--)
            {
                CreatedColliders[i].Dispose();
            }

            CreatedColliders.Dispose();
        }

#endif
        #endregion

        #region Modification

        /// <summary>
        /// Modify a <see cref="CompoundCollider"/> by manually modifying its child colliders' shape and
        /// transformation, then update it and expect the bounding volume and mass properties to have updated accordingly.
        /// </summary>
        [Test]
        public unsafe void TestCompoundColliderUpdate()
        {
            using var points = new NativeArray<float3>(ConvexColliderTests.k_TestPoints, Allocator.Temp);
            float origSphereRadius1 = 1.0f;
            float origSphereRadius2 = 1.5f;
            float expectedSphereRadiusScale = 2.0f;
            using var childCollider1 = SphereCollider.Create(new SphereGeometry {Radius = origSphereRadius1});
            using var childCollider2 = SphereCollider.Create(new SphereGeometry {Radius = origSphereRadius2});

            using var expectedChildCollider1 = SphereCollider.Create(new SphereGeometry
                {Radius = origSphereRadius1 * expectedSphereRadiusScale});
            using var expectedChildCollider2 = SphereCollider.Create(new SphereGeometry
                {Radius = origSphereRadius2 * expectedSphereRadiusScale});
            var origTransform1 =
                new RigidTransform(
                    quaternion.AxisAngle(math.normalize(new float3(1.1f, 4.5f, 2.0f)), math.radians(42f)),
                    new float3(1, -2, 3));
            var origTransform2 =
                new RigidTransform(
                    quaternion.AxisAngle(math.normalize(new float3(2.1f, 3.5f, 1.0f)), math.radians(24f)),
                    new float3(-4, -5, 7));

            var expectedTransformDelta1 = new RigidTransform(
                quaternion.AxisAngle(math.normalize(new float3(3.1f, 4.5f, -2.0f)), math.radians(18f)),
                new float3(1.0f, 2.0f, 3.0f));
            var expectedTransformDelta2 = new RigidTransform(
                quaternion.AxisAngle(math.normalize(new float3(-2.5f, 2.3f, 4.2f)), math.radians(67f)),
                new float3(-4.0f, -5.0f, 7.0f));

            var expectedTransform1 = math.mul(expectedTransformDelta1, origTransform1);
            var expectedTransform2 = math.mul(expectedTransformDelta2, origTransform2);

            using var origChildren = new NativeArray<CompoundCollider.ColliderBlobInstance>(2, Allocator.Temp)
                {
                    [0] = new CompoundCollider.ColliderBlobInstance
                    {Collider = childCollider1, CompoundFromChild = origTransform1},
                    [1] = new CompoundCollider.ColliderBlobInstance
                    {Collider = childCollider2, CompoundFromChild = origTransform2}
                };
            using var origCollider = CompoundCollider.Create(origChildren);

            using var expectedChildren = new NativeArray<CompoundCollider.ColliderBlobInstance>(2, Allocator.Temp)
                {
                    [0] = new CompoundCollider.ColliderBlobInstance
                    {Collider = expectedChildCollider1, CompoundFromChild = expectedTransform1},
                    [1] = new CompoundCollider.ColliderBlobInstance
                    {Collider = expectedChildCollider2, CompoundFromChild = expectedTransform2}
                };
            using var expectedCollider = CompoundCollider.Create(expectedChildren);

            // modify the original collider: change the radius and transformation of the spheres

            ref var compoundCollider = ref origCollider.As<CompoundCollider>();
            var children = compoundCollider.Children;
            ref var c = ref children[0];
            c.CompoundFromChild = math.mul(expectedTransformDelta1, c.CompoundFromChild);
            var sphere = (SphereCollider*)c.Collider;
            sphere->Geometry = new SphereGeometry { Radius = sphere->Radius * expectedSphereRadiusScale };

            c = ref children[1];
            c.CompoundFromChild = math.mul(expectedTransformDelta2, c.CompoundFromChild);
            sphere = (SphereCollider*)c.Collider;
            sphere->Geometry = new SphereGeometry { Radius = sphere->Radius * expectedSphereRadiusScale };

            // update the collider
            compoundCollider.Update();

            // ensure the updated collider matches the expected collider
            var modifiedAABB = origCollider.Value.CalculateAabb();
            var expectedAABB = expectedCollider.Value.CalculateAabb();
            const float kEps = 1e-4f;
            TestUtils.AreEqual(expectedAABB.Center, modifiedAABB.Center, kEps);
            TestUtils.AreEqual(expectedAABB.Extents, modifiedAABB.Extents, kEps);

            var modifiedMassProperties = origCollider.Value.MassProperties;
            var expectedMassProperties = expectedCollider.Value.MassProperties;
            TestUtils.AreEqual(expectedMassProperties.Volume, modifiedMassProperties.Volume, kEps);
            TestUtils.AreEqual(expectedMassProperties.AngularExpansionFactor, modifiedMassProperties.AngularExpansionFactor, kEps);
            TestUtils.AreEqual(expectedMassProperties.MassDistribution.Transform.pos, modifiedMassProperties.MassDistribution.Transform.pos, kEps);
            TestUtils.AreEqual(expectedMassProperties.MassDistribution.Transform, modifiedMassProperties.MassDistribution.Transform, kEps);
            TestUtils.AreEqual(expectedMassProperties.MassDistribution.InertiaTensor, modifiedMassProperties.MassDistribution.InertiaTensor, kEps);
        }

        /// <summary>
        /// Modify a <see cref="CompoundCollider"/> by baking a user-provided affine transformation,
        /// containing translation, rotation, scale and shear, into its geometry.
        /// </summary>
        [Test]
        public unsafe void TestCompoundColliderBakeTransformAffine()
        {
            using var points = new NativeArray<float3>(ConvexColliderTests.k_TestPoints, Allocator.Temp);
            using var childCollider1 = ConvexCollider.Create(points, ConvexHullGenerationParameters.Default);
            using var childCollider2 = childCollider1.Value.Clone();

            using var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(2, Allocator.Temp)
                {
                    [0] = new CompoundCollider.ColliderBlobInstance { Collider = childCollider1, CompoundFromChild = new RigidTransform(quaternion.AxisAngle(math.normalize(new float3(1.1f, 4.5f, 2.0f)), math.radians(42f)), new float3(1, -2, 3)) },
                    [1] = new CompoundCollider.ColliderBlobInstance { Collider = childCollider2, CompoundFromChild = new RigidTransform(quaternion.AxisAngle(math.normalize(new float3(2.1f, 3.5f, 1.0f)), math.radians(24f)), new float3(-4, -5, 7)) }
                };
            using var collider = CompoundCollider.Create(children);

            var translation = new float3(3.4f, 2.5f, -1.1f);
            var rotation = quaternion.AxisAngle(math.normalize(new float3(1.1f, 10.1f, -3.4f)), 78.0f);
            var scale = new float3(1.5f, 2.5f, 4.2f);
            var transform = new AffineTransform(translation, rotation, scale);

            // add some shear deformation
            var shearXY = float3x3.zero;
            var shearXZ = float3x3.zero;
            var shearYZ = float3x3.zero;

            shearXY[2][0] = shearXY[2][1] = 0.5f;
            shearXZ[1][0] = shearXZ[1][2] = 0.42f;
            shearYZ[0][1] = shearYZ[0][2] = 0.3f;

            transform = math.mul(shearXY, math.mul(shearXZ, math.mul(shearYZ, transform)));

            // obtain original vertices from both convex collider children and transform them into expected vertices
            ref var compoundCollider = ref collider.As<CompoundCollider>();
            ref var convexCollider1 = ref childCollider1.As<ConvexCollider>();
            ref var convexCollider2 = ref childCollider1.As<ConvexCollider>();

            BlobArray.Accessor<float3>[] originalVertices = { convexCollider1.ConvexHull.Vertices, convexCollider2.ConvexHull.Vertices };
            var vertexCount = originalVertices[0].Length + originalVertices[1].Length;
            var expectedVertices = new NativeList<float3>(vertexCount, Allocator.Temp);
            for (int i = 0; i < originalVertices.Length; ++i)
            {
                var childVertices = originalVertices[i];
                var childTM = children[i].CompoundFromChild;
                for (int j = 0; j < childVertices.Length; ++j)
                {
                    expectedVertices.Add(math.transform(transform, math.transform(childTM, childVertices[j])));
                }
            }

            // apply the transformation to the compound collider
            collider.Value.BakeTransform(transform);

            // validate the resultant compound collider
            var childColliders = compoundCollider.Children;
            Assert.AreEqual(childColliders.Length, 2);

            var actualConvexColliders = new[] { (ConvexCollider*)childColliders[0].Collider, (ConvexCollider*)childColliders[1].Collider };
            var actualVertexCount = actualConvexColliders[0]->ConvexHull.Vertices.Length + actualConvexColliders[1]->ConvexHull.Vertices.Length;
            var actualVertices = new NativeList<float3>(actualVertexCount, Allocator.Temp);
            for (int i = 0; i < actualConvexColliders.Length; ++i)
            {
                var childVertices = actualConvexColliders[i]->ConvexHull.Vertices;
                var childTM = childColliders[i].CompoundFromChild;
                for (int j = 0; j < childVertices.Length; ++j)
                {
                    actualVertices.Add(math.transform(childTM, childVertices[j]));
                }
            }

            Assert.AreEqual(expectedVertices.Length, actualVertices.Length);
            for (int i = 0; i < expectedVertices.Length; ++i)
            {
                TestUtils.AreEqual(expectedVertices[i], actualVertices[i], 1e-5f);
            }
        }

        #endregion

        #region Utilities

        [Test]
        public void TestCompoundColliderToMesh()
        {
            // create a compound made of 2 boxes placed diagonally from each other and make sure we get the
            // expected mesh center and bounds

            var size = new float3(1, 2, 3);
            var geometry = new BoxGeometry
            {
                Center = float3.zero,
                Orientation = quaternion.identity,
                Size = size
            };
            using var boxCollider = BoxCollider.Create(geometry);

            var children = new NativeArray<CompoundCollider.ColliderBlobInstance>(2, Allocator.Temp);
            try
            {
                for (int i = 0; i < 2; ++i)
                {
                    children[i] = new CompoundCollider.ColliderBlobInstance {Collider = boxCollider, CompoundFromChild = RigidTransform.Translate(i * size)};
                }

                var expectedBoundsSize = 2 * size;
                var expectedBoundsCenter = size / 2;

                using var compoundCollider = CompoundCollider.Create(children);
                var aabb = compoundCollider.Value.CalculateAabb(RigidTransform.identity);
                TestUtils.AreEqual(expectedBoundsCenter, aabb.Center, math.EPSILON);
                TestUtils.AreEqual(expectedBoundsSize, aabb.Extents, math.EPSILON);

                var mesh = compoundCollider.Value.ToMesh();
                TestUtils.AreEqual(expectedBoundsCenter, mesh.bounds.center, math.EPSILON);
                TestUtils.AreEqual(expectedBoundsSize, mesh.bounds.size, math.EPSILON);

                UnityEngine.Object.DestroyImmediate(mesh);
            }
            finally
            {
                children.Dispose();
            }
        }

        #endregion
    }
}
