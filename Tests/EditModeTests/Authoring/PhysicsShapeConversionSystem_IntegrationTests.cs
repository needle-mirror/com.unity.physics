using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEditor;
using UnityEngine;
#if !UNITY_EDITOR
using UnityEngine.TestTools;
#endif
#if LEGACY_PHYSICS
using LegacyBox = UnityEngine.BoxCollider;
using LegacyCapsule = UnityEngine.CapsuleCollider;
using LegacyCollider = UnityEngine.Collider;
using LegacyMesh = UnityEngine.MeshCollider;
using LegacySphere = UnityEngine.SphereCollider;
using LegacyRigidBody = UnityEngine.Rigidbody;
#endif
using UnityMesh = UnityEngine.Mesh;

namespace Unity.Physics.Tests.Authoring
{
    class PhysicsShapeConversionSystem_IntegrationTests : BaseHierarchyConversionTest
    {
        [OneTimeSetUp]
        public void OneTimeSetUp()
        {
            ReadableMesh = Resources.GetBuiltinResource<UnityMesh>("New-Cylinder.fbx");
            Assume.That(ReadableMesh.isReadable, Is.True, $"{ReadableMesh} was not readable.");

            NonReadableMesh = UnityMesh.Instantiate(ReadableMesh);
            NonReadableMesh.UploadMeshData(true);
            Assume.That(NonReadableMesh.isReadable, Is.False, $"{NonReadableMesh} was readable.");

            MeshWithMultipleSubMeshes = new UnityMesh
            {
                name = nameof(MeshWithMultipleSubMeshes),
                vertices = new[]
                {
                    new Vector3(0f, 1f, 0f),
                    new Vector3(1f, 1f, 0f),
                    new Vector3(1f, 0f, 0f),
                    new Vector3(0f, 0f, 0f)
                },
                normals = new[]
                {
                    Vector3.back,
                    Vector3.back,
                    Vector3.back,
                    Vector3.back
                },
                subMeshCount = 2
            };
            MeshWithMultipleSubMeshes.SetTriangles(new[] { 0, 1, 2 }, 0);
            MeshWithMultipleSubMeshes.SetTriangles(new[] { 2, 3, 0 }, 1);
            Assume.That(MeshWithMultipleSubMeshes.isReadable, Is.True, $"{MeshWithMultipleSubMeshes} was not readable.");
        }

        [OneTimeTearDown]
        public void OneTimeTearDown()
        {
            if (NonReadableMesh != null)
                UnityMesh.DestroyImmediate(NonReadableMesh);
            if (MeshWithMultipleSubMeshes != null)
                UnityMesh.DestroyImmediate(MeshWithMultipleSubMeshes);
        }

        UnityMesh NonReadableMesh { get; set; }
        UnityMesh ReadableMesh { get; set; }
        UnityMesh MeshWithMultipleSubMeshes { get; set; }

        readonly PhysicsWorldIndex k_DefaultWorldIndex = new PhysicsWorldIndex();

        [Test]
        public void PhysicsShapeConversionSystem_WhenBodyHasOneSiblingShape_CreatesPrimitive()
        {
            CreateHierarchy(
                new[] { typeof(PhysicsBodyAuthoring), typeof(PhysicsShapeAuthoring) },
                Array.Empty<Type>(),
                Array.Empty<Type>()
            );
            Root.GetComponent<PhysicsShapeAuthoring>().SetBox(new BoxGeometry { Size = 1f, Orientation = quaternion.identity });

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(c => Assert.That(c.Value.Value.Type, Is.EqualTo(ColliderType.Box)), k_DefaultWorldIndex);
        }

        [Test]
        public void PhysicsShapeConversionSystem_WhenBodyHasOneDescendentShape_CreatesCompound()
        {
            CreateHierarchy(
                new[] { typeof(PhysicsBodyAuthoring) },
                new[] { typeof(PhysicsShapeAuthoring) },
                Array.Empty<Type>()
            );
            Parent.GetComponent<PhysicsShapeAuthoring>().SetBox(new BoxGeometry { Size = 1f, Orientation = quaternion.identity });

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(
                c =>
                {
                    Assert.That(c.Value.Value.Type, Is.EqualTo(ColliderType.Compound));
                    unsafe
                    {
                        var compoundCollider = (CompoundCollider*)c.Value.GetUnsafePtr();
                        Assert.That(compoundCollider->Children, Has.Length.EqualTo(1));
                        Assert.That(compoundCollider->Children[0].Collider->Type, Is.EqualTo(ColliderType.Box));
                    }
                },
                k_DefaultWorldIndex
            );
        }

        [Test]
        public void PhysicsShapeConversionSystem_WhenBodyHasOneDescendentShape_CreatesCompoundWithFiniteMass()
        {
            CreateHierarchy(
                new[] { typeof(PhysicsBodyAuthoring) },
                new[] { typeof(PhysicsShapeAuthoring) },
                Array.Empty<Type>()
            );
            Parent.GetComponent<PhysicsShapeAuthoring>().SetBox(new BoxGeometry { Size = 1f, Orientation = quaternion.identity });
            Parent.GetComponent<PhysicsShapeAuthoring>().CollisionResponse = CollisionResponsePolicy.RaiseTriggerEvents;

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(
                c =>
                {
                    Assert.That(c.Value.Value.Type, Is.EqualTo(ColliderType.Compound));
                    unsafe
                    {
                        var compoundCollider = (CompoundCollider*)c.Value.GetUnsafePtr();
                        Assume.That(compoundCollider->Children, Has.Length.EqualTo(1));
                        Assume.That(compoundCollider->Children[0].Collider->Type, Is.EqualTo(ColliderType.Box));

                        // Make sure compound mass properties are calculated properly
                        Assert.That(compoundCollider->MassProperties.Volume > 0.0f);
                        Assert.That(math.all(math.isfinite(compoundCollider->MassProperties.MassDistribution.Transform.pos)));
                        Assert.That(math.all(math.isfinite(compoundCollider->MassProperties.MassDistribution.Transform.rot.value)));
                        Assert.That(math.all(math.isfinite(compoundCollider->MassProperties.MassDistribution.InertiaTensor)));
                        Assert.That(math.all(math.isfinite(compoundCollider->MassProperties.MassDistribution.InertiaMatrix.c0)));
                        Assert.That(math.all(math.isfinite(compoundCollider->MassProperties.MassDistribution.InertiaMatrix.c1)));
                        Assert.That(math.all(math.isfinite(compoundCollider->MassProperties.MassDistribution.InertiaMatrix.c2)));
                    }
                },
                k_DefaultWorldIndex
            );
        }

        [Test]
        public void PhysicsShapeConversionSystem_WhenBodyHasMultipleDescendentShapes_CreatesCompound()
        {
            CreateHierarchy(
                new[] { typeof(PhysicsBodyAuthoring) },
                new[] { typeof(PhysicsShapeAuthoring) },
                new[] { typeof(PhysicsShapeAuthoring) }
            );
            Parent.GetComponent<PhysicsShapeAuthoring>().SetBox(new BoxGeometry { Size = 1f, Orientation = quaternion.identity });
            Child.GetComponent<PhysicsShapeAuthoring>().SetSphere(new SphereGeometry { Radius = 1f }, quaternion.identity);

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(
                c =>
                {
                    Assert.That(c.Value.Value.Type, Is.EqualTo(ColliderType.Compound));
                    unsafe
                    {
                        var compoundCollider = (CompoundCollider*)c.Value.GetUnsafePtr();

                        var childTypes = Enumerable.Range(0, compoundCollider->NumChildren)
                            .Select(i => compoundCollider->Children[i].Collider->Type)
                            .ToArray();
                        Assert.That(childTypes, Is.EquivalentTo(new[] { ColliderType.Box, ColliderType.Sphere }));
                    }
                },
                k_DefaultWorldIndex
            );
        }

        static readonly Regex k_NonReadableMeshPattern = new Regex(@"\b((un)?readable|Read\/Write|(non-)?accessible)\b");

        [Test]
        public void PhysicsShapeConversionSystem_WhenShapeHasNonReadableConvex_ThrowsException()
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { typeof(PhysicsShapeAuthoring) });
            Child.GetComponent<PhysicsShapeAuthoring>().SetConvexHull(default, NonReadableMesh);

            VerifyLogsException<InvalidOperationException>(k_NonReadableMeshPattern);
        }

        [Test]
        public void PhysicsShapeConversionSystem_WhenShapeHasNonReadableMesh_ThrowsException()
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { typeof(PhysicsShapeAuthoring) });
            Child.GetComponent<PhysicsShapeAuthoring>().SetMesh(NonReadableMesh);

            VerifyLogsException<InvalidOperationException>(k_NonReadableMeshPattern);
        }

        [Test]
        public unsafe void ConversionSystems_WhenMeshCollider_MultipleSubMeshes_AllSubMeshesIncluded(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyMesh),
#endif
                typeof(PhysicsShapeAuthoring)
             )]
            Type shapeType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { shapeType });
#if LEGACY_PHYSICS
            if (Child.GetComponent(shapeType) is LegacyMesh meshCollider)
                meshCollider.sharedMesh = MeshWithMultipleSubMeshes;
            else
                Child.GetComponent<PhysicsShapeAuthoring>().SetMesh(MeshWithMultipleSubMeshes);
#endif

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(c =>
            {
                ref var mesh = ref ((MeshCollider*)c.ColliderPtr)->Mesh;
                Assume.That(mesh.Sections.Length, Is.EqualTo(1), "Expected a single section on mesh collider.");
                ref var section = ref mesh.Sections[0];
                Assume.That(section.PrimitiveFlags.Length, Is.EqualTo(1), "Expected a single primitive on mesh collider.");
                Assert.That(section.PrimitiveFlags[0] & Mesh.PrimitiveFlags.IsQuad, Is.EqualTo(Mesh.PrimitiveFlags.IsQuad), "Expected single quad primitive on mesh collider.");
            }, k_DefaultWorldIndex);
        }

        [Test]
        public void ConversionSystems_WhenGOHasShape_GOIsActive_AuthoringComponentEnabled_AuthoringDataConverted(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox), typeof(LegacyCapsule), typeof(LegacySphere), typeof(LegacyMesh),
#endif
                typeof(PhysicsShapeAuthoring)
             )]
            Type shapeType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { shapeType });
#if LEGACY_PHYSICS
            if (Child.GetComponent(shapeType) is LegacyMesh meshCollider)
                meshCollider.sharedMesh = ReadableMesh;
#endif

            // conversion presumed to create valid PhysicsCollider under default conditions
            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(c => Assert.That(c.IsValid, Is.True), k_DefaultWorldIndex);
        }

        [Test]
        public void ConversionSystems_WhenGOHasShape_AuthoringComponentDisabled_AuthoringDataNotConverted(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox), typeof(LegacyCapsule), typeof(LegacySphere), typeof(LegacyMesh),
#endif
                typeof(PhysicsShapeAuthoring)
             )]
            Type shapeType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { shapeType });
#if LEGACY_PHYSICS
            if (Child.GetComponent(shapeType) is LegacyMesh meshCollider)
                meshCollider.sharedMesh = ReadableMesh;
#endif
            var c = Child.GetComponent(shapeType);
#if LEGACY_PHYSICS
            if (c is LegacyCollider collider)
                collider.enabled = false;
            else
#endif
            (c as PhysicsShapeAuthoring).enabled = false;

            // conversion presumed to create valid PhysicsCollider under default conditions
            // covered by corresponding test ConversionSystems_WhenGOHasShape_GOIsActive_AuthoringComponentEnabled_AuthoringDataConverted
            VerifyNoDataProduced<PhysicsCollider>();
        }

        [Test]
        public void ConversionSystems_WhenGOHasShape_GOIsInactive_BodyIsNotConverted(
            [Values] Node inactiveNode,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox), typeof(LegacyCapsule), typeof(LegacySphere), typeof(LegacyMesh),
#endif
                typeof(PhysicsShapeAuthoring)
             )]
            Type shapeType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { shapeType });
#if LEGACY_PHYSICS
            if (Child.GetComponent(shapeType) is LegacyMesh meshCollider)
                meshCollider.sharedMesh = ReadableMesh;
#endif
            GetNode(inactiveNode).SetActive(false);
            var numInactiveNodes = Root.GetComponentsInChildren<Transform>(true).Count(t => t.gameObject.activeSelf);
            Assume.That(numInactiveNodes, Is.EqualTo(2));

            // conversion presumed to create valid PhysicsCollider under default conditions
            // covered by corresponding test ConversionSystems_WhenGOHasShape_GOIsActive_AuthoringComponentEnabled_AuthoringDataConverted
            VerifyNoDataProduced<PhysicsCollider>();
        }

        static void SetDefaultShape(PhysicsShapeAuthoring shape, ShapeType type)
        {
            switch (type)
            {
                case ShapeType.Box:
                    shape.SetBox(default);
                    break;
                case ShapeType.Capsule:
                    shape.SetCapsule(new CapsuleGeometryAuthoring { OrientationEuler = EulerAngles.Default });
                    break;
                case ShapeType.Sphere:
                    shape.SetSphere(default, quaternion.identity);
                    break;
                case ShapeType.Cylinder:
                    shape.SetCylinder(new CylinderGeometry { SideCount = CylinderGeometry.MaxSideCount });
                    break;
                case ShapeType.Plane:
                    shape.SetPlane(default, default, quaternion.identity);
                    break;
                case ShapeType.ConvexHull:
                    shape.SetConvexHull(ConvexHullGenerationParameters.Default);
                    break;
                case ShapeType.Mesh:
                    shape.SetMesh();
                    break;
            }

            shape.FitToEnabledRenderMeshes();
        }

        static readonly RigidTransform k_SharedDataChildTransformation =
            new RigidTransform(quaternion.EulerZXY(math.PI / 4), new float3(1f, 2f, 3f));

        [Test]
        public unsafe void ConversionSystems_WhenMultipleShapesShareInputs_CollidersShareTheSameData(
            [Values(ShapeType.ConvexHull, ShapeType.Mesh)] ShapeType shapeType
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                new[] { typeof(PhysicsShapeAuthoring), typeof(PhysicsBodyAuthoring), typeof(MeshFilter), typeof(MeshRenderer) },
                new[] { typeof(PhysicsShapeAuthoring), typeof(PhysicsBodyAuthoring), typeof(MeshFilter), typeof(MeshRenderer) }
            );
            foreach (var meshFilter in Root.GetComponentsInChildren<MeshFilter>())
                meshFilter.sharedMesh = ReadableMesh;
            foreach (var shape in Root.GetComponentsInChildren<PhysicsShapeAuthoring>())
            {
                SetDefaultShape(shape, shapeType);
                shape.ForceUnique = false;
            }
            Child.transform.localPosition = k_SharedDataChildTransformation.pos;
            Child.transform.localRotation = k_SharedDataChildTransformation.rot;

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(colliders =>
            {
                var uniqueColliders = new HashSet<int>();
                foreach (var c in colliders)
                    uniqueColliders.Add((int)c.ColliderPtr);
                var numUnique = uniqueColliders.Count;
                Assert.That(numUnique, Is.EqualTo(1), $"Expected colliders to reference the same data, but found {numUnique} different colliders.");
            }, 2, k_DefaultWorldIndex);
        }

        static readonly TestCaseData[] k_MultipleAuthoringComponentsTestCases =
        {
            new TestCaseData(
                new[] { typeof(PhysicsBodyAuthoring), typeof(PhysicsShapeAuthoring), typeof(PhysicsShapeAuthoring) },
                new[] { ColliderType.Box, ColliderType.Box }
            ).SetName("Multiple PhysicsShapeAuthoring"),
#if LEGACY_PHYSICS
            new TestCaseData(
                new[] { typeof(PhysicsBodyAuthoring), typeof(LegacyBox), typeof(LegacyCapsule), typeof(LegacySphere), typeof(PhysicsShapeAuthoring) },
                new[] { ColliderType.Box, ColliderType.Box, ColliderType.Capsule, ColliderType.Sphere }
            ).SetName("Mix of classic and new shape types")
#endif
        };

        [TestCaseSource(nameof(k_MultipleAuthoringComponentsTestCases))]
        public void ConversionSystems_WhenBodyHasMultipleShapeComponents_CreatesCompound(
            Type[] componentTypes, ColliderType[] expectedColliderTypes
        )
        {
            CreateHierarchy(componentTypes, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsShapeAuthoring>().SetBox(new BoxGeometry { Size = 1f });

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(
                c =>
                {
                    Assert.That(c.Value.Value.Type, Is.EqualTo(ColliderType.Compound));
                    unsafe
                    {
                        var compoundCollider = (CompoundCollider*)c.Value.GetUnsafePtr();

                        var childTypes = Enumerable.Range(0, compoundCollider->NumChildren)
                            .Select(i => compoundCollider->Children[i].Collider->Type)
                            .ToArray();
                        Assert.That(childTypes, Is.EquivalentTo(expectedColliderTypes));
                    }
                },
                k_DefaultWorldIndex
            );
        }

#if LEGACY_PHYSICS
        [Test]
        [Ignore("Behavior is inconsistent on some platforms.")]
        public void LegacyMeshColliderConversionSystem_WhenMeshColliderHasNonReadableMesh_ThrowsException(
            [Values] bool convex
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { typeof(LegacyMesh) });
#if !UNITY_EDITOR
            // legacy components log error messages in the player for non-readable meshes once for each property access
            LogAssert.Expect(LogType.Error, k_NonReadableMeshPattern);
            LogAssert.Expect(LogType.Error, k_NonReadableMeshPattern);
#endif
            Child.GetComponent<LegacyMesh>().sharedMesh = NonReadableMesh;
            Child.GetComponent<LegacyMesh>().convex = convex;

            VerifyLogsException<InvalidOperationException>(k_NonReadableMeshPattern);
        }

        [Test]
        public unsafe void LegacyMeshColliderConversionSystem_WhenMultipleShapesShareInputs_CollidersShareTheSameData(
            [Values] bool convex
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                new[] { typeof(LegacyMesh), typeof(LegacyRigidBody) },
                new[] { typeof(LegacyMesh), typeof(LegacyRigidBody) }
            );
            foreach (var shape in Root.GetComponentsInChildren<LegacyMesh>())
            {
                shape.convex = convex;
                shape.sharedMesh = ReadableMesh;
            }
            Child.transform.localPosition = k_SharedDataChildTransformation.pos;
            Child.transform.localRotation = k_SharedDataChildTransformation.rot;

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(colliders =>
            {
                var uniqueColliders = new HashSet<int>();
                foreach (var c in colliders)
                    uniqueColliders.Add((int)c.ColliderPtr);
                var numUnique = uniqueColliders.Count;
                Assert.That(numUnique, Is.EqualTo(1), $"Expected colliders to reference the same data, but found {numUnique} different colliders.");
            }, 2, k_DefaultWorldIndex);
        }

#endif

        [Test]
        public unsafe void ConversionSystems_WhenMultipleShapesShareMeshes_WithDifferentOffsets_CollidersDoNotShareTheSameData(
            [Values(ShapeType.ConvexHull, ShapeType.Mesh)] ShapeType shapeType
        )
        {
            CreateHierarchy(
                new[] { typeof(PhysicsShapeAuthoring), typeof(PhysicsBodyAuthoring) },
                new[] { typeof(MeshFilter), typeof(MeshRenderer) },
                new[] { typeof(PhysicsShapeAuthoring), typeof(PhysicsBodyAuthoring), typeof(MeshFilter), typeof(MeshRenderer) }
            );
            foreach (var meshFilter in Root.GetComponentsInChildren<MeshFilter>())
                meshFilter.sharedMesh = ReadableMesh;
            foreach (var shape in Root.GetComponentsInChildren<PhysicsShapeAuthoring>())
            {
                SetDefaultShape(shape, shapeType);
                shape.ForceUnique = false;
            }
            // Root will get mesh from Parent (with offset) and Child will get mesh from itself (no offset)
            Parent.transform.localPosition = k_SharedDataChildTransformation.pos;
            Parent.transform.localRotation = k_SharedDataChildTransformation.rot;

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(colliders =>
            {
                var uniqueColliders = new HashSet<int>();
                foreach (var c in colliders)
                    uniqueColliders.Add((int)c.ColliderPtr);
                var numUnique = uniqueColliders.Count;
                Assert.That(numUnique, Is.EqualTo(2), $"Expected colliders to reference unique data, but found {numUnique} different colliders.");
            }, 2, k_DefaultWorldIndex);
        }

        [Test]
        public unsafe void ConversionSystems_WhenMultipleShapesShareMeshes_WithDifferentInheritedScale_CollidersDoNotShareTheSameData(
            [Values(ShapeType.ConvexHull, ShapeType.Mesh)] ShapeType shapeType
        )
        {
            CreateHierarchy(
                new[] { typeof(PhysicsShapeAuthoring), typeof(PhysicsBodyAuthoring), typeof(MeshFilter), typeof(MeshRenderer) },
                Array.Empty<Type>(),
                new[] { typeof(PhysicsShapeAuthoring), typeof(PhysicsBodyAuthoring), typeof(MeshFilter), typeof(MeshRenderer) }
            );
            foreach (var meshFilter in Root.GetComponentsInChildren<MeshFilter>())
                meshFilter.sharedMesh = ReadableMesh;
            foreach (var shape in Root.GetComponentsInChildren<PhysicsShapeAuthoring>())
            {
                SetDefaultShape(shape, shapeType);
                shape.ForceUnique = false;
            }
            // affects Child scale
            Parent.transform.localScale = new Vector3(2f, 2f, 2f);

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(colliders =>
            {
                var uniqueColliders = new HashSet<int>();
                foreach (var c in colliders)
                    uniqueColliders.Add((int)c.ColliderPtr);
                var numUnique = uniqueColliders.Count;
                Assert.That(numUnique, Is.EqualTo(2), $"Expected colliders to reference unique data, but found {numUnique} different colliders.");
            }, 2, k_DefaultWorldIndex);
        }

        [Test]
        public unsafe void ConversionSystems_WhenMultipleShapesShareInputs_AndShapeIsForcedUnique_CollidersDoNotShareTheSameData(
            [Values] ShapeType shapeType
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                new[] { typeof(PhysicsShapeAuthoring), typeof(PhysicsBodyAuthoring), typeof(MeshFilter), typeof(MeshRenderer) },
                new[] { typeof(PhysicsShapeAuthoring), typeof(PhysicsBodyAuthoring), typeof(MeshFilter), typeof(MeshRenderer) }
            );
            foreach (var meshFilter in Root.GetComponentsInChildren<MeshFilter>())
                meshFilter.sharedMesh = ReadableMesh;
            foreach (var shape in Root.GetComponentsInChildren<PhysicsShapeAuthoring>())
            {
                SetDefaultShape(shape, shapeType);
                shape.ForceUnique = true;
            }

            TestConvertedSharedData<PhysicsCollider, PhysicsWorldIndex>(colliders =>
            {
                var uniqueColliders = new HashSet<int>();
                foreach (var c in colliders)
                    uniqueColliders.Add((int)c.ColliderPtr);

                var numUnique = uniqueColliders.Count;
                Assert.That(numUnique, Is.EqualTo(2), $"Expected colliders to reference unique data, but found {numUnique} different colliders.");
            }, 2, k_DefaultWorldIndex);
        }
    }
}
