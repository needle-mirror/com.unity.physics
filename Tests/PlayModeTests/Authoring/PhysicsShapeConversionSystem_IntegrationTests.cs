using System;
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
        }

        UnityMesh NonReadableMesh { get; set; }

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
                new[] { typeof(ConvertToEntity), typeof(PhysicsBodyAuthoring), typeof(PxBox), typeof(PxCapsule), typeof(PxSphere) },
                new[] { typeof(ConvertToEntity) },
                new[] { typeof(ConvertToEntity) }
            );

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
                        Assert.That(childTypes, Is.EquivalentTo(new[] { ColliderType.Box, ColliderType.Capsule, ColliderType.Sphere }));
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
    }
}
