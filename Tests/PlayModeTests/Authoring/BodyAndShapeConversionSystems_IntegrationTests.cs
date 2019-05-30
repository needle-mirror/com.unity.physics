using System;
using System.Linq;
using NUnit.Framework;
using Unity.Physics.Authoring;
using UnityEngine;

namespace Unity.Physics.Tests.Authoring
{
    class BodyAndShapeConversionSystems_IntegrationTests : BaseHierarchyConversionTest
    {
        [Test]
        public void ConversionSystems_WhenGOHasPhysicsBodyAndRigidbody_EntityUsesPhysicsBodyMass()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsBody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBody>().Mass = 100f;
            Root.GetComponent<Rigidbody>().mass = 50f;

            TestConvertedData<PhysicsMass>(mass => Assert.That(mass.InverseMass, Is.EqualTo(0.01f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasPhysicsBodyAndRigidbody_EntityUsesPhysicsBodyDamping()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsBody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBody>().LinearDamping = 1f;
            Root.GetComponent<Rigidbody>().drag = 0.5f;

            TestConvertedData<PhysicsDamping>(damping => Assert.That(damping.Linear, Is.EqualTo(1f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasDynamicPhysicsBodyWithCustomGravity_AndKinematicRigidbody_EntityUsesPhysicsBodyGravityFactor()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsBody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBody>().MotionType = BodyMotionType.Dynamic;
            Root.GetComponent<PhysicsBody>().GravityFactor = 2f;
            Root.GetComponent<Rigidbody>().isKinematic = true;

            TestConvertedData<PhysicsGravityFactor>(gravity => Assert.That(gravity.Value, Is.EqualTo(2f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasKinematicPhysicsBody_AndDynamicRigidbody_EntityUsesPhysicsBodyGravityFactor()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsBody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBody>().MotionType = BodyMotionType.Kinematic;
            Root.GetComponent<Rigidbody>().isKinematic = false;

            TestConvertedData<PhysicsGravityFactor>(gravity => Assert.That(gravity.Value, Is.EqualTo(0f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasDynamicPhysicsBodyWithDefaultGravity_AndDynamicRigidbodyWithCustomGravity_EntityHasNoGravityFactor()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsBody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBody>().MotionType = BodyMotionType.Dynamic;
            Root.GetComponent<Rigidbody>().isKinematic = true;

            VerifyNoDataProduced<PhysicsGravityFactor>();
        }

        [Test]
        public void ConversionSystems_WhenGOHasDynamicPhysicsBodyWithNoPhysicsShape_AndDynamicRigidbodyWithNoCollider_EntityHasNoPhysicsCollider()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsBody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBody>().MotionType = BodyMotionType.Dynamic;
            Root.GetComponent<PhysicsBody>().Mass = 100f;
            Root.GetComponent<Rigidbody>().isKinematic = false;
            Root.GetComponent<Rigidbody>().mass = 50f;

            VerifyNoDataProduced<PhysicsCollider>();
        }

        [Test]
        public void ConversionSystems_WhenGOHasDynamicPhysicsBody_AndKinematicRigidbody_EntityUsesPhysicsBodyMass()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsBody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBody>().MotionType = BodyMotionType.Dynamic;
            Root.GetComponent<PhysicsBody>().Mass = 100f;
            Root.GetComponent<Rigidbody>().isKinematic = true;
            Root.GetComponent<Rigidbody>().mass = 50f;

            TestConvertedData<PhysicsMass>(mass => Assert.That(mass.InverseMass, Is.EqualTo(0.01f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasKinematicPhysicsBody_AndDynamicRigidbody_EntityUsesPhysicsBodyMass()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsBody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBody>().MotionType = BodyMotionType.Kinematic;
            Root.GetComponent<PhysicsBody>().Mass = 100f;
            Root.GetComponent<Rigidbody>().isKinematic = false;
            Root.GetComponent<Rigidbody>().mass = 50f;

            TestConvertedData<PhysicsMass>(mass => Assert.That(mass.InverseMass, Is.EqualTo(0f)));
        }

        [Test]
        public void ConversionSystems_WhenGOHasStaticPhysicsBody_AndDynamicRigidbody_EntityHasNoGravityFactor()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsBody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsBody>().MotionType = BodyMotionType.Static;
            Root.GetComponent<Rigidbody>().isKinematic = false;

            VerifyNoDataProduced<PhysicsGravityFactor>();
        }

        [Test]
        public void ConversionSystems_WhenGOHasBody_GOIsActive_BodyIsConverted(
            [Values(typeof(PhysicsBody), typeof(Rigidbody))]Type bodyType
        )
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { bodyType });

            // conversion presumed to create PhysicsVelocity under default conditions
            TestConvertedData<PhysicsVelocity>(v => Assert.That(v, Is.EqualTo(default(PhysicsVelocity))));
        }

        [Test]
        public void ConversionSystems_WhenGOHasBody_AuthoringComponentDisabled_AuthoringDataNotConverted()
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { typeof(PhysicsBody) });
            Child.GetComponent<PhysicsBody>().enabled = false;

            // conversion presumed to create PhysicsVelocity under default conditions
            // covered by corresponding test ConversionSystems_WhenGOHasBody_GOIsActive_BodyIsConverted
            VerifyNoDataProduced<PhysicsVelocity>();
        }

        [Test]
        public void ConversionSystems_WhenGOHasBody_GOIsInactive_BodyIsNotConverted(
            [Values]Node inactiveNode,
            [Values(typeof(PhysicsBody), typeof(Rigidbody))]Type bodyType
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
