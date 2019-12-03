using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEngine;
#if !UNITY_EDITOR
using UnityEngine.TestTools;
#endif
using PxBox = UnityEngine.BoxCollider;
using PxCapsule = UnityEngine.CapsuleCollider;
using PxSphere = UnityEngine.SphereCollider;
using PxMesh = UnityEngine.MeshCollider;
using UnityMesh = UnityEngine.Mesh;

namespace Unity.Physics.Tests.Authoring
{
    class PhysicsShapeConversionSystem_IntegrationTests : BaseHierarchyConversionTest
    {
        [OneTimeSetUp]
        public void OneTimeSetUp()
        {
            NonReadableMesh = Resources.LoadAll<UnityMesh>("not-readable").Single();
            Assume.That(NonReadableMesh.isReadable, Is.False, $"{NonReadableMesh} was readable.");
            ReadableMesh = Resources.GetBuiltinResource<UnityMesh>("New-Cylinder.fbx");
            Assume.That(ReadableMesh.isReadable, Is.True, $"{ReadableMesh} was not readable.");
        }

        UnityMesh NonReadableMesh { get; set; }
        UnityMesh ReadableMesh { get; set; }

        [Test]
        public void PhysicsShapeConversionSystem_WhenBodyHasOneSiblingShape_CreatesPrimitive()
        {
            CreateHierarchy(
                new[] { typeof(ConvertToEntity), typeof(PhysicsBodyAuthoring), typeof(PhysicsShapeAuthoring) },
                new[] { typeof(ConvertToEntity) },
                new[] { typeof(ConvertToEntity) }
            );
            Root.GetComponent<PhysicsShapeAuthoring>().SetBox(new BoxGeometry { Size = 1f, Orientation = quaternion.identity });

            TestConvertedData<PhysicsCollider>(c => Assert.That(c.Value.Value.Type, Is.EqualTo(ColliderType.Box)));
        }

        [Test]
        public void PhysicsShapeConversionSystem_WhenBodyHasOneDescendentShape_CreatesCompound()
        {
            CreateHierarchy(
                new[] { typeof(ConvertToEntity), typeof(PhysicsBodyAuthoring) },
                new[] { typeof(ConvertToEntity), typeof(PhysicsShapeAuthoring) },
                new[] { typeof(ConvertToEntity) }
            );
            Parent.GetComponent<PhysicsShapeAuthoring>().SetBox(new BoxGeometry { Size = 1f, Orientation = quaternion.identity });

            TestConvertedData<PhysicsCollider>(
                c =>
                {
                    Assert.That(c.Value.Value.Type, Is.EqualTo(ColliderType.Compound));
                    unsafe
                    {
                        var compoundCollider = (CompoundCollider*)c.Value.GetUnsafePtr();
                        Assert.That(compoundCollider->Children, Has.Length.EqualTo(1));
                        Assert.That(compoundCollider->Children[0].Collider->Type, Is.EqualTo(ColliderType.Box));
                    }
                }
            );
        }

        [Test]
        public void PhysicsShapeConversionSystem_WhenBodyHasMultipleDescendentShapes_CreatesCompound()
        {
            CreateHierarchy(
                new[] { typeof(ConvertToEntity), typeof(PhysicsBodyAuthoring) },
                new[] { typeof(ConvertToEntity), typeof(PhysicsShapeAuthoring) },
                new[] { typeof(ConvertToEntity), typeof(PhysicsShapeAuthoring) }
            );
            Parent.GetComponent<PhysicsShapeAuthoring>().SetBox(new BoxGeometry { Size = 1f, Orientation = quaternion.identity });
            Child.GetComponent<PhysicsShapeAuthoring>().SetSphere(new SphereGeometry { Radius = 1f }, quaternion.identity);

            TestConvertedData<PhysicsCollider>(
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
                }
            );
        }

        [Test]
        public void PhysicsShapeConversionSystem_WhenBodyHasMultipleDifferentSiblingShapes_CreatesCompound_WithFlatHierarchy()
        {
            CreateHierarchy(
                new[] { typeof(ConvertToEntity), typeof(PhysicsBodyAuthoring), typeof(PxBox), typeof(PxCapsule), typeof(PxSphere), typeof(PhysicsShapeAuthoring) },
                new[] { typeof(ConvertToEntity) },
                new[] { typeof(ConvertToEntity) }
            );
            Root.GetComponent<PhysicsShapeAuthoring>().SetBox(new BoxGeometry { Size = 1f });

            TestConvertedData<PhysicsCollider>(
                c =>
                {
                    Assert.That(c.Value.Value.Type, Is.EqualTo(ColliderType.Compound));
                    unsafe
                    {
                        var compoundCollider = (CompoundCollider*)c.Value.GetUnsafePtr();

                        var childTypes = Enumerable.Range(0, compoundCollider->NumChildren)
                            .Select(i => compoundCollider->Children[i].Collider->Type)
                            .ToArray();
                        Assert.That(childTypes, Is.EquivalentTo(new[] { ColliderType.Box, ColliderType.Box, ColliderType.Capsule, ColliderType.Sphere }));
                    }
                }
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
        [Ignore("Behavior is inconsistent on some platforms.")]
        public void LegacyMeshColliderConversionSystem_WhenMeshColliderHasNonReadableMesh_ThrowsException(
            [Values]bool convex
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { typeof(PxMesh) });
#if !UNITY_EDITOR
            // legacy components log error messages in the player for non-readable meshes once for each property access
            LogAssert.Expect(LogType.Error, k_NonReadableMeshPattern);
            LogAssert.Expect(LogType.Error, k_NonReadableMeshPattern);
#endif
            Child.GetComponent<PxMesh>().sharedMesh = NonReadableMesh;
            Child.GetComponent<PxMesh>().convex = convex;

            VerifyLogsException<InvalidOperationException>(k_NonReadableMeshPattern);
        }

        [Test]
        public void ConversionSystems_WhenGOHasShape_GOIsActive_AuthoringComponentEnabled_AuthoringDataConverted(
            [Values(typeof(PhysicsShapeAuthoring), typeof(PxBox), typeof(PxCapsule), typeof(PxSphere), typeof(PxMesh))]
            Type shapeType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { shapeType });
            if (Child.GetComponent(shapeType) is PxMesh meshCollider)
                meshCollider.sharedMesh = Resources.GetBuiltinResource<UnityEngine.Mesh>("New-Cylinder.fbx");

            // conversion presumed to create valid PhysicsCollider under default conditions
            TestConvertedData<PhysicsCollider>(c => Assert.That(c.IsValid, Is.True));
        }

        [Test]
        public void ConversionSystems_WhenGOHasShape_AuthoringComponentDisabled_AuthoringDataNotConverted(
            [Values(typeof(PhysicsShapeAuthoring), typeof(PxBox), typeof(PxCapsule), typeof(PxSphere), typeof(PxMesh))]
            Type shapeType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { shapeType });
            if (Child.GetComponent(shapeType) is PxMesh meshCollider)
                meshCollider.sharedMesh = Resources.GetBuiltinResource<UnityEngine.Mesh>("New-Cylinder.fbx");
            var c = Child.GetComponent(shapeType);
            if (c is UnityEngine.Collider collider)
                collider.enabled = false;
            else (c as PhysicsShapeAuthoring).enabled = false;

            // conversion presumed to create valid PhysicsCollider under default conditions
            // covered by corresponding test ConversionSystems_WhenGOHasShape_GOIsActive_AuthoringComponentEnabled_AuthoringDataConverted
            VerifyNoDataProduced<PhysicsCollider>();
        }

        [Test]
        public void ConversionSystems_WhenGOHasShape_GOIsInactive_BodyIsNotConverted(
            [Values]Node inactiveNode,
            [Values(typeof(PhysicsShapeAuthoring), typeof(PxBox), typeof(PxCapsule), typeof(PxSphere), typeof(PxMesh))]
            Type shapeType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { shapeType });
            if (Child.GetComponent(shapeType) is PxMesh meshCollider)
                meshCollider.sharedMesh = Resources.GetBuiltinResource<UnityEngine.Mesh>("New-Cylinder.fbx");
            GetNode(inactiveNode).SetActive(false);
            var numInactiveNodes = Root.GetComponentsInChildren<Transform>(true).Count(t => t.gameObject.activeSelf);
            Assume.That(numInactiveNodes, Is.EqualTo(2));

            // conversion presumed to create valid PhysicsCollider under default conditions
            // covered by corresponding test ConversionSystems_WhenGOHasShape_GOIsActive_AuthoringComponentEnabled_AuthoringDataConverted
            VerifyNoDataProduced<PhysicsCollider>();
        }

        void SetDefaultShape(PhysicsShapeAuthoring shape, ShapeType type)
        {
            switch (type)
            {
                case ShapeType.Box:
                    shape.SetBox(default);
                    break;
                case ShapeType.Capsule:
                    shape.SetCapsule(default, quaternion.identity);
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

            TestConvertedData<PhysicsCollider>(colliders =>
            {
                var uniqueColliders = new HashSet<int>();
                foreach (var c in colliders)
                    uniqueColliders.Add((int)c.ColliderPtr);
                var numUnique = uniqueColliders.Count;
                Assert.That(numUnique, Is.EqualTo(1), $"Expected colliders to reference the same data, but found {numUnique} different colliders.");
            }, 2);
        }

        [Test]
        public unsafe void LegacyMeshColliderConversionSystem_WhenMultipleShapesShareInputs_CollidersShareTheSameData(
            [Values]bool convex
        )
        {
            CreateHierarchy(
                Array.Empty<Type>(),
                new[] { typeof(MeshCollider), typeof(Rigidbody) },
                new[] { typeof(MeshCollider), typeof(Rigidbody) }
            );
            foreach (var shape in Root.GetComponentsInChildren<PxMesh>())
            {
                shape.convex = convex;
                shape.sharedMesh = ReadableMesh;
            }
            Child.transform.localPosition = k_SharedDataChildTransformation.pos;
            Child.transform.localRotation = k_SharedDataChildTransformation.rot;

            TestConvertedData<PhysicsCollider>(colliders =>
            {
                var uniqueColliders = new HashSet<int>();
                foreach (var c in colliders)
                    uniqueColliders.Add((int)c.ColliderPtr);
                var numUnique = uniqueColliders.Count;
                Assert.That(numUnique, Is.EqualTo(1), $"Expected colliders to reference the same data, but found {numUnique} different colliders.");
            }, 2);
        }

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

            TestConvertedData<PhysicsCollider>(colliders =>
            {
                var uniqueColliders = new HashSet<int>();
                foreach (var c in colliders)
                    uniqueColliders.Add((int)c.ColliderPtr);
                var numUnique = uniqueColliders.Count;
                Assert.That(numUnique, Is.EqualTo(2), $"Expected colliders to reference unique data, but found {numUnique} different colliders.");
            }, 2);
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

            TestConvertedData<PhysicsCollider>(colliders =>
            {
                var uniqueColliders = new HashSet<int>();
                foreach (var c in colliders)
                    uniqueColliders.Add((int)c.ColliderPtr);
                var numUnique = uniqueColliders.Count;
                Assert.That(numUnique, Is.EqualTo(2), $"Expected colliders to reference unique data, but found {numUnique} different colliders.");
            }, 2);
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

            TestConvertedData<PhysicsCollider>(colliders =>
            {
                var uniqueColliders = new HashSet<int>();
                foreach (var c in colliders)
                    uniqueColliders.Add((int)c.ColliderPtr);

                var numUnique = uniqueColliders.Count;
                Assert.That(numUnique, Is.EqualTo(2), $"Expected colliders to reference unique data, but found {numUnique} different colliders.");
            }, 2);
        }
    }
}
