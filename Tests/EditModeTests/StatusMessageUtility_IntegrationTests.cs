using System;
using NUnit.Framework;
using Unity.Physics.Authoring;
using Unity.Physics.Editor;
using Unity.Physics.Tests.Authoring;
using UnityEngine;
#if LEGACY_PHYSICS
using LegacyBox = UnityEngine.BoxCollider;
using LegacyCapsule = UnityEngine.CapsuleCollider;
using LegacyMesh = UnityEngine.MeshCollider;
using LegacySphere = UnityEngine.SphereCollider;
using LegacyRigidBody = UnityEngine.Rigidbody;
#endif

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
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody), typeof(LegacyBox),
#endif
                typeof(PhysicsBodyAuthoring), typeof(PhysicsShapeAuthoring)
            )]
            Type parentComponentType,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox), typeof(LegacyCapsule), typeof(LegacyMesh), typeof(LegacySphere),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
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
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody), typeof(LegacyBox), typeof(LegacyCapsule), typeof(LegacyMesh), typeof(LegacySphere),
#endif
                typeof(PhysicsBodyAuthoring), typeof(PhysicsShapeAuthoring)
            )]
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
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring)
            )]
            Type bodyType,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox), typeof(LegacyCapsule), typeof(LegacyMesh), typeof(LegacySphere),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
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
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring)
            )]
            Type bodyType,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox), typeof(LegacyCapsule), typeof(LegacyMesh), typeof(LegacySphere),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
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
