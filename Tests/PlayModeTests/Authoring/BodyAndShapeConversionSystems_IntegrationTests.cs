using System;
using System.Linq;
using NUnit.Framework;
using Unity.Physics.Authoring;
using UnityEngine;
#if LEGACY_PHYSICS
using LegacyRigidBody = UnityEngine.Rigidbody;
#endif

namespace Unity.Physics.Tests.Authoring
{
    class BodyAndShapeConversionSystems_IntegrationTests : BaseHierarchyConversionTest
    {
#if LEGACY_PHYSICS
        [Test]
        public void ConversionSystems_WhenGOHasPhysicsBodyAndLegacyRigidBody_EntityUsesPhysicsBodyMass()
        {
            CreateHierarchy(new[] { typeof(LegacyRigidBody), typeof(PhysicsBodyAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBodyAuthoring>().Mass = 100f;
            Root.GetComponent<LegacyRigidBody>().mass = 50f;

            TestConvertedData<PhysicsMass>(mass => Assert.That(mass.InverseMass, Is.EqualTo(0.01f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasPhysicsBodyAndLegacyRigidBody_EntityUsesPhysicsBodyDamping()
        {
            CreateHierarchy(new[] { typeof(LegacyRigidBody), typeof(PhysicsBodyAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBodyAuthoring>().LinearDamping = 1f;
            Root.GetComponent<LegacyRigidBody>().drag = 0.5f;

            TestConvertedData<PhysicsDamping>(damping => Assert.That(damping.Linear, Is.EqualTo(1f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasDynamicPhysicsBodyWithCustomGravity_AndKinematicLegacyRigidBody_EntityUsesPhysicsBodyGravityFactor()
        {
            CreateHierarchy(new[] { typeof(LegacyRigidBody), typeof(PhysicsBodyAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBodyAuthoring>().MotionType = BodyMotionType.Dynamic;
            Root.GetComponent<PhysicsBodyAuthoring>().GravityFactor = 2f;
            Root.GetComponent<LegacyRigidBody>().isKinematic = true;

            TestConvertedData<PhysicsGravityFactor>(gravity => Assert.That(gravity.Value, Is.EqualTo(2f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasKinematicPhysicsBody_AndDynamicLegacyRigidBody_EntityUsesPhysicsBodyGravityFactor()
        {
            CreateHierarchy(new[] { typeof(LegacyRigidBody), typeof(PhysicsBodyAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBodyAuthoring>().MotionType = BodyMotionType.Kinematic;
            Root.GetComponent<LegacyRigidBody>().isKinematic = false;

            TestConvertedData<PhysicsGravityFactor>(gravity => Assert.That(gravity.Value, Is.EqualTo(0f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasDynamicPhysicsBodyWithDefaultGravity_AndDynamicLegacyRigidBodyWithCustomGravity_EntityHasNoGravityFactor()
        {
            CreateHierarchy(new[] { typeof(LegacyRigidBody), typeof(PhysicsBodyAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBodyAuthoring>().MotionType = BodyMotionType.Dynamic;
            Root.GetComponent<LegacyRigidBody>().isKinematic = true;

            VerifyNoDataProduced<PhysicsGravityFactor>();
        }

        [Test]
        public void ConversionSystems_WhenGOHasDynamicPhysicsBodyWithNoPhysicsShape_AndDynamicLegacyRigidBodyWithNoCollider_EntityHasNoPhysicsCollider()
        {
            CreateHierarchy(new[] { typeof(LegacyRigidBody), typeof(PhysicsBodyAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBodyAuthoring>().MotionType = BodyMotionType.Dynamic;
            Root.GetComponent<PhysicsBodyAuthoring>().Mass = 100f;
            Root.GetComponent<LegacyRigidBody>().isKinematic = false;
            Root.GetComponent<LegacyRigidBody>().mass = 50f;

            VerifyNoDataProduced<PhysicsCollider>();
        }

        [Test]
        public void ConversionSystems_WhenGOHasDynamicPhysicsBody_AndKinematicLegacyRigidBody_EntityUsesPhysicsBodyMass()
        {
            CreateHierarchy(new[] { typeof(LegacyRigidBody), typeof(PhysicsBodyAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBodyAuthoring>().MotionType = BodyMotionType.Dynamic;
            Root.GetComponent<PhysicsBodyAuthoring>().Mass = 100f;
            Root.GetComponent<LegacyRigidBody>().isKinematic = true;
            Root.GetComponent<LegacyRigidBody>().mass = 50f;

            TestConvertedData<PhysicsMass>(mass => Assert.That(mass.InverseMass, Is.EqualTo(0.01f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasKinematicPhysicsBody_AndDynamicLegacyRigidBody_EntityUsesPhysicsBodyMass()
        {
            CreateHierarchy(new[] { typeof(LegacyRigidBody), typeof(PhysicsBodyAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBodyAuthoring>().MotionType = BodyMotionType.Kinematic;
            Root.GetComponent<PhysicsBodyAuthoring>().Mass = 100f;
            Root.GetComponent<LegacyRigidBody>().isKinematic = false;
            Root.GetComponent<LegacyRigidBody>().mass = 50f;

            TestConvertedData<PhysicsMass>(mass => Assert.That(mass.InverseMass, Is.EqualTo(0f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasStaticPhysicsBody_AndDynamicLegacyRigidBody_EntityHasNoGravityFactor()
        {
            CreateHierarchy(new[] { typeof(LegacyRigidBody), typeof(PhysicsBodyAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBodyAuthoring>().MotionType = BodyMotionType.Static;
            Root.GetComponent<LegacyRigidBody>().isKinematic = false;

            VerifyNoDataProduced<PhysicsGravityFactor>();
        }
#endif

        [Test]
        public void ConversionSystems_WhenGOHasBody_GOIsActive_BodyIsConverted(
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring)
            )]
            Type bodyType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { bodyType });

            // conversion presumed to create PhysicsVelocity under default conditions
            TestConvertedData<PhysicsVelocity>(v => Assert.That(v, Is.EqualTo(default(PhysicsVelocity))));
        }

        [Test]
        public void ConversionSystems_WhenGOHasBody_AuthoringComponentDisabled_AuthoringDataNotConverted()
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { typeof(PhysicsBodyAuthoring) });
            Child.GetComponent<PhysicsBodyAuthoring>().enabled = false;

            // conversion presumed to create PhysicsVelocity under default conditions
            // covered by corresponding test ConversionSystems_WhenGOHasBody_GOIsActive_BodyIsConverted
            VerifyNoDataProduced<PhysicsVelocity>();
        }

        [Test]
        public void ConversionSystems_WhenGOHasBody_GOIsInactive_BodyIsNotConverted(
            [Values]Node inactiveNode,
            [Values(
#if LEGACY_PHYSICS
                typeof(LegacyRigidBody),
#endif
                typeof(PhysicsBodyAuthoring)
            )]
            Type bodyType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { bodyType });
            GetNode(inactiveNode).SetActive(false);
            var numInactiveNodes = Root.GetComponentsInChildren<Transform>(true).Count(t => t.gameObject.activeSelf);
            Assume.That(numInactiveNodes, Is.EqualTo(2));

            // conversion presumed to create PhysicsVelocity under default conditions
            // covered by corresponding test ConversionSystems_WhenGOHasBody_GOIsActive_BodyIsConverted
            VerifyNoDataProduced<PhysicsVelocity>();
        }
    }
}
