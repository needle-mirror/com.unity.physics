using System;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
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

        void TestConvertedData<T>(Action<T> checkValue) where T : struct, IComponentData
        {
            var world = new World("Test world");

            try
            {
                GameObjectConversionUtility.ConvertGameObjectHierarchy(Root, world);

                using (var group = world.EntityManager.CreateComponentGroup(typeof(T)))
                {
                    using (var bodies = group.ToComponentDataArray<T>(Allocator.Persistent))
                    {
                        Assume.That(bodies, Has.Length.EqualTo(1));
                        var componentData = bodies[0];

                        checkValue(componentData);
                    }
                }
            }
            finally
            {
                world.Dispose();
            }
        }

        void VerifyNoDataProduced<T>() where T : struct, IComponentData
        {
            var world = new World("Test world");

            try
            {
                GameObjectConversionUtility.ConvertGameObjectHierarchy(Root, world);

                using (var group = world.EntityManager.CreateComponentGroup(typeof(T)))
                using (var bodies = group.ToComponentDataArray<T>(Allocator.Persistent))
                    Assert.That(bodies.Length, Is.EqualTo(0), $"Conversion pipeline produced {typeof(T).Name}");
            }
            finally
            {
                world.Dispose();
            }
        }
    }
}
