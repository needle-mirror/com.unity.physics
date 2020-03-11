using System;
using NUnit.Framework;
using Unity.Entities;
using Unity.Physics.Authoring;
using UnityEngine;
#if LEGACY_PHYSICS
using LegacyBox = UnityEngine.BoxCollider;
using LegacyRigidBody = UnityEngine.Rigidbody;
#endif

namespace Unity.Physics.Tests.Authoring
{
    class PhysicsShapeExtensions_IntegrationTests : BaseHierarchyConversionTest
    {
        [Test]
        public void GetPrimaryBody_WhenHierarchyContainsMultipleBodies_ReturnsFirstParent(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring)
            )]
            Type rootBodyType,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring)
            )]
            Type parentBodyType
        )
        {
            CreateHierarchy(new[] { rootBodyType }, new[] { parentBodyType }, Array.Empty<Type>());

            var primaryBody = PhysicsShapeExtensions.GetPrimaryBody(Child);

            Assert.That(primaryBody, Is.EqualTo(Parent));
        }

        [Test]
        public void GetPrimaryBody_WhenFirstParentPhysicsBodyIsDisabled_ReturnsFirstEnabledAncestor(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring)
            )]
            Type rootBodyType,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring)
            )]
            Type parentBodyType
        )
        {
            CreateHierarchy(new[] { rootBodyType }, new[] { parentBodyType }, new[] { typeof(PhysicsBodyAuthoring) });
            // if root is PhysicsBodyAuthoring, test assumes it is enabled; Rigidbody is Component and cannot be disabled
            Assume.That(Root.GetComponent<PhysicsBodyAuthoring>()?.enabled ?? true, Is.True);
            Child.GetComponent<PhysicsBodyAuthoring>().enabled = false;

            var primaryBody = PhysicsShapeExtensions.GetPrimaryBody(Child);

            Assert.That(primaryBody, Is.EqualTo(Parent));
        }

        [Test]
        public void GetPrimaryBody_WhenHierarchyContainsBody_AndIsStaticOptimized_ReturnsBody(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring)
            )]
            Type parentBodyType,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
            Type childShapeType
        )
        {
            CreateHierarchy(new[] { typeof(StaticOptimizeEntity) }, new[] { parentBodyType }, new[] { childShapeType });

            var primaryBody = PhysicsShapeExtensions.GetPrimaryBody(Child);

            Assert.That(primaryBody, Is.EqualTo(Parent));
        }

        [Test]
        public void GetPrimaryBody_WhenHierarchyContainsNoBodies_ReturnsTopMostShape(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
            Type rootShapeType,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
            Type parentShapeType,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
            Type childShapeType
        )
        {
            CreateHierarchy(new[] { rootShapeType }, new[] { parentShapeType }, new[] { childShapeType });

            var primaryBody = PhysicsShapeExtensions.GetPrimaryBody(Child);

            Assert.That(primaryBody, Is.EqualTo(Root));
        }

        [Test]
        public void GetPrimaryBody_WhenHierarchyContainsNoBodies_IsStaticOptimized_ReturnsStaticOptimizeEntity(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyBox),
#endif
                typeof(PhysicsShapeAuthoring)
            )]
            Type childShapeType
        )
        {
            CreateHierarchy(new[] { typeof(StaticOptimizeEntity) }, Array.Empty<Type>(), new[] { childShapeType });

            var primaryBody = PhysicsShapeExtensions.GetPrimaryBody(Child);

            Assert.That(primaryBody, Is.EqualTo(Root));
        }
    }
}
