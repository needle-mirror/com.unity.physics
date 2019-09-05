using System;
using NUnit.Framework;
using Unity.Physics.Authoring;
using Unity.Physics.Editor;
using UnityEngine;
using Unity.Physics.Tests.Authoring;
using PxBox = UnityEngine.BoxCollider;
using PxCapsule = UnityEngine.CapsuleCollider;
using PxMesh = UnityEngine.MeshCollider;
using PxSphere = UnityEngine.SphereCollider;

namespace Unity.Physics.Tests.Editor
{
    class StatusMessageUtility_IntegrationTests : BaseHierarchyConversionTest
    {
        [Test]
        public void GetHierarchyStatusMessage_WhenRoot_MessageNullOrEmpty()
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), Array.Empty<Type>());

            StatusMessageUtility.GetHierarchyStatusMessage(new[] { Root.transform }, out var msg);

            Assert.That(msg, Is.Null.Or.Empty);
        }

        [Test]
        public void GetHierarchyStatusMessage_WhenChild_AndChildIsNotPrimaryBody_MessageNullOrEmpty(
            [Values(typeof(PhysicsBodyAuthoring), typeof(PhysicsShapeAuthoring), typeof(PxBox), typeof(Rigidbody))]
            Type parentComponentType,
            [Values(typeof(PhysicsShapeAuthoring), typeof(PxBox), typeof(PxCapsule), typeof(PxMesh), typeof(PxSphere))]
            Type childComponentType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), new[] { parentComponentType }, new[] { childComponentType });
            Assume.That(PhysicsShapeExtensions.GetPrimaryBody(Child), Is.EqualTo(Parent));

            StatusMessageUtility.GetHierarchyStatusMessage(new[] { Child.GetComponent(childComponentType) }, out var msg);

            Assert.That(msg, Is.Null.Or.Empty);
        }

        [Test]
        public void GetHierarchyStatusMessage_WhenChild_AndChildIsPrimaryBody_MessageNotNullOrEmpty(
            [Values(typeof(PhysicsBodyAuthoring), typeof(PhysicsShapeAuthoring), typeof(PxBox), typeof(PxCapsule), typeof(PxMesh), typeof(PxSphere), typeof(Rigidbody))]
            Type childComponentType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { childComponentType });
            Assume.That(PhysicsShapeExtensions.GetPrimaryBody(Child), Is.EqualTo(Child));

            StatusMessageUtility.GetHierarchyStatusMessage(new[] { Child.GetComponent(childComponentType) }, out var msg);

            Assert.That(msg, Is.Not.Null.Or.Empty);
        }

        [Test]
        public void GetHierarchyStatusMessage_WhenChild_AndChildHasBodyAndShape_QueryingBodyReturnsMessage(
            [Values(typeof(PhysicsBodyAuthoring), typeof(Rigidbody))]
            Type bodyType,
            [Values(typeof(PhysicsShapeAuthoring), typeof(PxBox), typeof(PxCapsule), typeof(PxMesh), typeof(PxSphere))]
            Type shapeType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { bodyType, shapeType });
            Assume.That(PhysicsShapeExtensions.GetPrimaryBody(Child), Is.EqualTo(Child));

            StatusMessageUtility.GetHierarchyStatusMessage(new[] { Child.GetComponent(bodyType) }, out var msg);

            Assert.That(msg, Is.Not.Null.Or.Empty);
        }

        [Test]
        public void GetHierarchyStatusMessage_WhenChild_AndChildHasBodyAndShape_QueryingShapeReturnsNullOrEmpty(
            [Values(typeof(PhysicsBodyAuthoring), typeof(Rigidbody))]
            Type bodyType,
            [Values(typeof(PhysicsShapeAuthoring), typeof(PxBox), typeof(PxCapsule), typeof(PxMesh), typeof(PxSphere))]
            Type shapeType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { bodyType, shapeType });
            Assume.That(PhysicsShapeExtensions.GetPrimaryBody(Child), Is.EqualTo(Child));

            StatusMessageUtility.GetHierarchyStatusMessage(new[] { Child.GetComponent(shapeType) }, out var msg);

            Assert.That(msg, Is.Null.Or.Empty);
        }
    }
}
