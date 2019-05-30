using System;
using System.Linq;
using NUnit.Framework;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEngine;
using PxBox = UnityEngine.BoxCollider;
using PxCapsule = UnityEngine.CapsuleCollider;
using PxSphere = UnityEngine.SphereCollider;
using PxMesh = UnityEngine.MeshCollider;

namespace Unity.Physics.Tests.Authoring
{
    class PhysicsShapeConversionSystem_IntegrationTests : BaseHierarchyConversionTest
    {
        [Test]
        public void PhysicsShapeConversionSystem_WhenBodyHasOneSiblingShape_CreatesPrimitive()
        {
            CreateHierarchy(
                new[] { typeof(ConvertToEntity), typeof(PhysicsBody), typeof(PhysicsShape) },
                new[] { typeof(ConvertToEntity) },
                new[] { typeof(ConvertToEntity) }
            );
            Root.GetComponent<PhysicsShape>().SetBox(float3.zero, new float3(1, 1, 1), quaternion.identity);

            TestConvertedData<PhysicsCollider>(c => Assert.That(c.Value.Value.Type, Is.EqualTo(ColliderType.Box)));
        }

        [Test]
        public void PhysicsShapeConversionSystem_WhenBodyHasOneDescendentShape_CreatesCompound()
        {
            CreateHierarchy(
                new[] { typeof(ConvertToEntity), typeof(PhysicsBody) },
                new[] { typeof(ConvertToEntity), typeof(PhysicsShape) },
                new[] { typeof(ConvertToEntity) }
            );
            Parent.GetComponent<PhysicsShape>().SetBox(float3.zero, new float3(1, 1, 1), quaternion.identity);

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
                new[] { typeof(ConvertToEntity), typeof(PhysicsBody) },
                new[] { typeof(ConvertToEntity), typeof(PhysicsShape) },
                new[] { typeof(ConvertToEntity), typeof(PhysicsShape) }
            );
            Parent.GetComponent<PhysicsShape>().SetBox(float3.zero, new float3(1, 1, 1), quaternion.identity);
            Child.GetComponent<PhysicsShape>().SetSphere(float3.zero, 1.0f, quaternion.identity);

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

        [Ignore("GameObjectConversionUtility does not yet support multiples of the same component type.")]
        [TestCase(2)]
        [TestCase(5)]
        [TestCase(10)]
        public void PhysicsShapeConversionSystem_WhenBodyHasMultipleSiblingShapes_CreatesCompound(int shapeCount)
        {
            CreateHierarchy(
                new[] { typeof(ConvertToEntity), typeof(PhysicsBody) },
                new[] { typeof(ConvertToEntity) },
                new[] { typeof(ConvertToEntity) }
            );
            for (int i = 0; i < shapeCount; ++i)
            {
                Root.AddComponent<PhysicsShape>().SetBox(float3.zero, new float3(1, 1, 1), quaternion.identity);
            }

            TestConvertedData<PhysicsCollider>(
                c =>
                {
                    Assert.That(c.Value.Value.Type, Is.EqualTo(ColliderType.Compound));
                    unsafe
                    {
                        var compoundCollider = (CompoundCollider*)c.Value.GetUnsafePtr();
                        Assert.That(compoundCollider->Children, Has.Length.EqualTo(shapeCount));
                        for (int i = 0; i < compoundCollider->Children.Length; i++)
                            Assert.That(compoundCollider->Children[i].Collider->Type, Is.EqualTo(ColliderType.Box));
                    }
                }
            );
        }

        [Test]
        public void ConversionSystems_WhenGOHasShape_GOIsActive_AuthoringComponentEnabled_AuthoringDataConverted(
            [Values(typeof(PhysicsShape), typeof(PxBox), typeof(PxCapsule), typeof(PxSphere), typeof(PxMesh))]
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
            [Values(typeof(PhysicsShape), typeof(PxBox), typeof(PxCapsule), typeof(PxSphere), typeof(PxMesh))]
            Type shapeType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { shapeType });
            if (Child.GetComponent(shapeType) is PxMesh meshCollider)
                meshCollider.sharedMesh = Resources.GetBuiltinResource<UnityEngine.Mesh>("New-Cylinder.fbx");
            var c = Child.GetComponent(shapeType);
            if (c is UnityEngine.Collider collider)
                collider.enabled = false;
            else (c as PhysicsShape).enabled = false;

            // conversion presumed to create valid PhysicsCollider under default conditions
            // covered by corresponding test ConversionSystems_WhenGOHasShape_GOIsActive_AuthoringComponentEnabled_AuthoringDataConverted
            VerifyNoDataProduced<PhysicsCollider>();
        }

        [Test]
        public void ConversionSystems_WhenGOHasShape_GOIsInactive_BodyIsNotConverted(
            [Values]Node inactiveNode,
            [Values(typeof(PhysicsShape), typeof(PxBox), typeof(PxCapsule), typeof(PxSphere), typeof(PxMesh))]
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
