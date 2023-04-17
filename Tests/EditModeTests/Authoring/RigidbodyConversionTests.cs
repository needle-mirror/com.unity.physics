using System;
using System.Linq;
using NUnit.Framework;
using Unity.Physics.Authoring;
using UnityEngine;

namespace Unity.Physics.Tests.Authoring
{
    class RigidbodyConversionTests : BaseHierarchyConversionTest
    {
        // Make sure the Rigidbody mass property is converted to a PhysicsMass component
        [Test]
        public void RigidbodyConversion_Mass()
        {
            CreateHierarchy(new[] { typeof(Rigidbody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<Rigidbody>().mass = 50f;

            TestConvertedData<PhysicsMass>(mass => Assert.That(mass.InverseMass, Is.EqualTo(0.02f)));
        }

        // Make sure the Rigidbody drag property is converted to PhysicsDamping.Linear
        [Test]
        public void RigidbodyConversion_Damping()
        {
            CreateHierarchy(new[] { typeof(Rigidbody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<Rigidbody>().drag = 0.5f;

            TestConvertedData<PhysicsDamping>(damping => Assert.That(damping.Linear, Is.EqualTo(0.5f)));
        }

        // Make sure a kinematic body does contain a PhysicsGravityFactor component with factor zero by default
        [Test]
        public void RigidbodyConversion_KinematicProducesGravityFactor()
        {
            CreateHierarchy(new[] { typeof(Rigidbody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<Rigidbody>().isKinematic = true;

            TestConvertedData<PhysicsGravityFactor>(gravity => Assert.That(gravity.Value, Is.EqualTo(0.0f)));
        }

        // Make sure a non-kinematic body does not contain a PhysicsGravityFactor component by default
        [Test]
        public void RigidbodyConversion_NotKinematicDoesNotProduceGravityFactor()
        {
            CreateHierarchy(new[] { typeof(Rigidbody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<Rigidbody>().isKinematic = false;

            VerifyNoDataProduced<PhysicsGravityFactor>();
        }

        // Make sure by default a rigid body does not contain a collider
        [Test]
        public void RigidbodyConversion_NoCollider()
        {
            CreateHierarchy(new[] { typeof(Rigidbody) }, Array.Empty<Type>(), Array.Empty<Type>());

            VerifyNoDataProduced<PhysicsCollider>();
        }

        // Make sure we get the correct mass (infinite) with kinematic bodies
        [Test]
        public void RigidbodyConversion_KinematicCausesInfiniteMass()
        {
            CreateHierarchy(new[] { typeof(Rigidbody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<Rigidbody>().isKinematic = true;
            Root.GetComponent<Rigidbody>().mass = 50f;

            // zero inverse mass corresponds to infinite mass
            TestConvertedData<PhysicsMass>(mass => Assert.That(mass.InverseMass, Is.EqualTo(0.0f)));
        }

        // Make sure we get the correct mass with non-kinematic bodies
        [Test]
        public void RigidbodyConversion_NotKinematicCausesFiniteMass()
        {
            CreateHierarchy(new[] { typeof(Rigidbody) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<Rigidbody>().isKinematic = false;
            Root.GetComponent<Rigidbody>().mass = 50f;

            TestConvertedData<PhysicsMass>(mass => Assert.That(mass.InverseMass, Is.EqualTo(0.02f)));
        }

        // Make sure we get the correct mass with non-kinematic bodies
        [Test]
        public void RigidbodyConversion_GOIsActive_BodyIsConverted()
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { typeof(Rigidbody) });

            // conversion presumed to create PhysicsVelocity under default conditions
            TestConvertedData<PhysicsVelocity>(v => Assert.That(v, Is.EqualTo(default(PhysicsVelocity))));
        }

        [Test]
        public void RigidbodyConversion_GOIsInactive_BodyIsNotConverted([Values] Node inactiveNode)
        {
            CreateHierarchy(Array.Empty<Type>(), Array.Empty<Type>(), new[] { typeof(Rigidbody) });
            GetNode(inactiveNode).SetActive(false);
            var numInactiveNodes = Root.GetComponentsInChildren<Transform>(true).Count(t => t.gameObject.activeSelf);
            Assume.That(numInactiveNodes, Is.EqualTo(2));

            // conversion presumed to create PhysicsVelocity under default conditions
            // covered by corresponding test RigidbodyConversion_GOIsActive_BodyIsConverted
            VerifyNoDataProduced<PhysicsVelocity>();
        }

        // Make sure we get the default physics world index in a default rigid body
        [Test]
        public void RigidbodyConversion_DefaultPhysicsWorldIndex()
        {
            CreateHierarchy(new[] { typeof(Rigidbody) }, Array.Empty<Type>(), Array.Empty<Type>());

            // Note: testing for presence of PhysicsVelocity component which is expected for a default rigid body
            TestConvertedSharedData<PhysicsVelocity, PhysicsWorldIndex>(
                null,
                k_DefaultWorldIndex);
        }

        // Make sure we get the default physics world index when using a default PhysicsWorldIndexAuthoring component
        [Test]
        public void RigidbodyConversion_WithPhysicsWorldIndexAuthoring_DefaultPhysicsWorldIndex()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsWorldIndexAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());

            // Note: testing for presence of PhysicsVelocity component which is expected for a default rigid body
            TestConvertedSharedData<PhysicsVelocity, PhysicsWorldIndex>(
                null,
                k_DefaultWorldIndex);
        }

        // Make sure we get the correct physics world index when using the PhysicsWorldIndexAuthoring component
        [Test]
        public void RigidbodyConversion_WithPhysicsWorldIndexAuthoring_NonDefaultPhysicsWorldIndex()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsWorldIndexAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());
            Root.GetComponent<PhysicsWorldIndexAuthoring>().WorldIndex = 3;

            // Note: testing for presence of PhysicsVelocity component which is expected for a default rigid body
            TestConvertedSharedData<PhysicsVelocity, PhysicsWorldIndex>(
                null,
                new PhysicsWorldIndex(3));
        }

        // Make sure there is no leftover baking data when using the PhysicsWorldIndexAuthoring component
        [Test]
        public void RigidbodyConversion_WithPhysicsWorldIndexAuthoring_NoBakingDataRemainsAfterBaking()
        {
            CreateHierarchy(new[] { typeof(Rigidbody), typeof(PhysicsWorldIndexAuthoring) }, Array.Empty<Type>(), Array.Empty<Type>());
            VerifyNoDataProduced<PhysicsWorldIndexBakingData>();
        }
    }
}
