using System;
using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;
using Unity.Physics.Extensions;

namespace Unity.Physics.Tests.Dynamics.Motion
{
    class MotionTests
    {
        [Test]
        public void MassPropertiesUnitSphereTest()
        {
            var unitSphere = MassProperties.UnitSphere;

            Assert.AreEqual(float3.zero, unitSphere.MassDistribution.Transform.pos);
            Assert.AreEqual(quaternion.identity, unitSphere.MassDistribution.Transform.rot);
            Assert.AreEqual(new float3(0.4f), unitSphere.MassDistribution.InertiaTensor);
            Assert.AreEqual(0.0f, unitSphere.AngularExpansionFactor);
        }

        [Test]
        public void MotionVelocityApplyLinearImpulseTest()
        {
            var motionVelocity = new MotionVelocity()
            {
                LinearVelocity = new float3(3.0f, 4.0f, 5.0f),
                InverseInertia = float3.zero,
                InverseMass = 2.0f
            };
            motionVelocity.ApplyLinearImpulse(new float3(1.0f, 2.0f, 3.0f));

            Assert.AreEqual(new float3(5.0f, 8.0f, 11.0f), motionVelocity.LinearVelocity);
        }

        [Test]
        public void MotionVelocityApplyAngularImpulseTest()
        {
            var motionVelocity = new MotionVelocity()
            {
                AngularVelocity = new float3(3.0f, 4.0f, 5.0f),
                InverseInertia = new float3(2.0f, 3.0f, 4.0f),
                InverseMass = 0.42f
            };
            motionVelocity.ApplyAngularImpulse(new float3(1.0f, 2.0f, 3.0f));

            Assert.AreEqual(new float3(5.0f, 10.0f, 17.0f), motionVelocity.AngularVelocity);
        }

        [Test]
        public void MotionVelocityCalculateExpansionTest()
        {
            var motionVelocity = new MotionVelocity()
            {
                LinearVelocity = new float3(2.0f, 1.0f, 5.0f),
                AngularVelocity = new float3(3.0f, 4.0f, 5.0f),
                InverseInertia = new float3(2.0f, 3.0f, 4.0f),
                InverseMass = 2.0f,
                AngularExpansionFactor = 1.2f
            };
            var motionExpansion = motionVelocity.CalculateExpansion(1.0f / 60.0f);

            Assert.AreEqual(new float3(1.0f / 30.0f, 1.0f / 60.0f, 1.0f / 12.0f), motionExpansion.Linear);
            Assert.That((float)math.SQRT2 / 10.0f, Is.PrettyCloseTo(motionExpansion.Uniform));
        }

        [Test]
        public void MotionExpansionMaxDistanceTest()
        {
            var motionExpansion = new MotionExpansion()
            {
                Linear = new float3(2.0f, 3.0f, 4.0f),
                Uniform = 5.0f
            };

            Assert.AreEqual(math.sqrt(29.0f) + 5.0f, motionExpansion.MaxDistance);
        }

        [Test]
        public void MotionExpansionSweepAabbTest()
        {
            var motionExpansion = new MotionExpansion()
            {
                Linear = new float3(2.0f, 3.0f, 4.0f),
                Uniform = 5.0f
            };
            var aabb = motionExpansion.ExpandAabb(new Aabb() { Min = new float3(-10.0f, -10.0f, -10.0f), Max = new float3(10.0f, 10.0f, 10.0f) });

            Assert.AreEqual(new float3(-15.0f, -15.0f, -15.0f), aabb.Min);
            Assert.AreEqual(new float3(17.0f, 18.0f, 19.0f), aabb.Max);
        }

        static void ValidatePhysicsVelocity(in PhysicsVelocity expectedVelocity, in PhysicsVelocity actualVelocity,
            float angularEps = Float3PrettyCloseConstraint.DefaultTolerance,
            float linearEps = Float3PrettyCloseConstraint.DefaultTolerance)
        {
            Assert.That(expectedVelocity.Angular, Is.PrettyCloseTo(actualVelocity.Angular).Within(angularEps));
            Assert.That(expectedVelocity.Linear, Is.PrettyCloseTo(actualVelocity.Linear).Within(linearEps));
        }

        [Test]
        public void PhysicsVelocityApplyAngularImpulseTest([Values(0.42f, 1.0f, 2.123f)] float scale)
        {
            var w = new float3(-1.2f, 3.4f, -5.6f);
            var invI = new float3(2.2f, 3.3f, 4.4f);
            var scaledInvI = scale != 1 ? invI * math.rcp(math.pow(scale, 5)) : invI;
            var mass = 1.42f;
            var inertiaTM = new RigidTransform(quaternion.Euler(-1.123f, 2.456f, -3.1f), new float3(-0.42f, 1.321f, -2.142f));
            var bodyTM = new RigidTransform(quaternion.Euler(0.2f, 1.2f, -0.8f), new float3(1.123f, -2.234f, 3.345f));
            var angularImpulseLocal = new float3(4.2f, -1.42f, -2.042f);
            // bring impulse from inertia space to world space
            var angularImpulseWorld = math.mul(bodyTM.rot, math.mul(inertiaTM.rot, angularImpulseLocal));

            var massProperties = new PhysicsMass
            {
                Transform = inertiaTM,
                InverseInertia = invI,
                InverseMass = math.rcp(mass)
            };

            var initialVelocity = new PhysicsVelocity
            {
                Angular = w,
                Linear = new float3(6.7f, -7.8f, 8.9f)
            };
            var expectedVelocity = initialVelocity;
            expectedVelocity.Angular += angularImpulseLocal * scaledInvI;

            var actualVelocity = initialVelocity;
            var actualVelocityImpulseInWorldSpace = initialVelocity;

            actualVelocity.ApplyAngularImpulse(massProperties, scale, angularImpulseLocal);
            actualVelocityImpulseInWorldSpace.ApplyAngularImpulseWorldSpace(massProperties, bodyTM.pos, bodyTM.rot, scale, angularImpulseWorld);

            ValidatePhysicsVelocity(expectedVelocity, actualVelocity);
            ValidatePhysicsVelocity(expectedVelocity, actualVelocityImpulseInWorldSpace, angularEps: 1e-3f);

            if (scale == 1.0f)
            {
                var actualVelocityUnscaled = initialVelocity;
                var actualVelocityUnscaledImpulseInWorldSpace = initialVelocity;
                actualVelocityUnscaled.ApplyAngularImpulse(massProperties, angularImpulseLocal);
                actualVelocityUnscaledImpulseInWorldSpace.ApplyAngularImpulseWorldSpace(massProperties, bodyTM.pos, bodyTM.rot, scale, angularImpulseWorld);

                ValidatePhysicsVelocity(expectedVelocity, actualVelocityUnscaled);
                ValidatePhysicsVelocity(expectedVelocity, actualVelocityUnscaledImpulseInWorldSpace, angularEps: 1e-3f);
            }
        }
    }
}
