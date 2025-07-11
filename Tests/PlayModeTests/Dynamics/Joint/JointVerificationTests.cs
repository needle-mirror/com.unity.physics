using NUnit.Framework;
using Unity.Mathematics;
using UnityEngine;

namespace Unity.Physics.Tests.Dynamics.Joints
{
    /// <summary>
    /// Specific verification tests for specific joints, designed to ensure we obtain the expected physical behavior.
    /// </summary>
    [TestFixture]
    class JointVerificationTests
    {
        PhysicsWorld m_World;
        SimulationStepInput m_StepInput;
        int m_DynamicBodyIndex;
        int m_StaticBodyIndex;
        int m_JointIndex;

        void Simulate(PhysicsJoint physicsJoint, int numSteps = 480)
        {
            var joint = new Joint
            {
                BodyPair = new BodyIndexPair { BodyIndexA = m_DynamicBodyIndex, BodyIndexB = m_StaticBodyIndex },
                AFromJoint = physicsJoint.BodyAFromJoint.AsMTransform(),
                BFromJoint = physicsJoint.BodyBFromJoint.AsMTransform(),
                Constraints = physicsJoint.m_Constraints,
            };

            var joints = m_World.DynamicsWorld.Joints;
            joints[m_JointIndex] = joint;

            var context = new SimulationContext();
            try
            {
                context.Reset(m_StepInput);
                for (var i = 0; i < numSteps; ++i)
                {
                    Unity.Physics.Simulation.StepImmediate(m_StepInput, ref context);
#if DEBUG_PRINT_BODY_POSITION
                    if (i % 10 == 0)
                        Debug.Log(m_World.DynamicsWorld.MotionDatas[m_DynamicBodyIndex].WorldFromMotion.pos);
#endif
                }
            }
            finally
            {
                context.Dispose();
            }
        }

        [TearDown]
        public void TearDown()
        {
            m_World.Dispose();
        }

        [SetUp]
        public void Setup()
        {
            m_World = new PhysicsWorld(1, 1, 1);

            m_DynamicBodyIndex = 0;
            m_StaticBodyIndex = 1;
            m_JointIndex = 0;

            // Simulation step input
            m_StepInput = new SimulationStepInput()
            {
                World = m_World,
                Gravity = new float3(0, -9.81f, 0),
                NumSubsteps = 1,
                NumSolverIterations = 4,
                TimeStep = 1 / 60f,
                SynchronizeCollisionWorld = false // we don't need any collisions in these tests
            };

            // position the dynamic and static bodies at origin
            var body = new RigidBody
            {
                WorldFromBody = new RigidTransform(quaternion.identity, float3.zero),
                Scale = 1.0f,
            };

            // mass properties of a unit sphere
            var massProperties = MassProperties.UnitSphere;
            var mass = 1.5f;

            var velocity = new MotionVelocity
            {
                InverseInertia = math.rcp(massProperties.MassDistribution.InertiaTensor * mass),
                InverseMass = math.rcp(mass),
                GravityFactor = 1f
            };

            var bodies = m_World.CollisionWorld.Bodies;
            bodies[m_DynamicBodyIndex] = body;
            bodies[m_StaticBodyIndex] = body;

            var motionDatas = m_World.MotionDatas;
            motionDatas[m_DynamicBodyIndex] = MotionData.Zero;

            var motionVelocities = m_World.MotionVelocities;
            motionVelocities[m_DynamicBodyIndex] = velocity;
        }

        /// <summary>
        /// Creates a spring joint between a dynamic and static body with a spring stiffness that should
        /// result in a target equilibrium position of a given distance unit from the anchor point.
        /// </summary>
        [Test]
        public void Spring_ReachesCorrectEquilibriumExtension([Values(0.3f, 0.5f, 0.8f, 1.0f)] float dampingRatio)
        {
            var equilibriumSpringExtension = 0.75f;
            var dynMotionVelocity = m_World.DynamicsWorld.MotionVelocities[m_DynamicBodyIndex];
            var mass = math.rcp(dynMotionVelocity.InverseMass);
            var restoringForce = -m_StepInput.Gravity.y * mass;
            var requiredStiffnessCoefficient = restoringForce / equilibriumSpringExtension;

            // convert spring stiffness to spring frequency:
            var springFrequency = JacobianUtilities.CalculateSpringFrequencyFromSpringConstant(requiredStiffnessCoefficient, mass);

            var physicsJoint = PhysicsJoint.CreateLimitedDistance(float3.zero, float3.zero, new Math.FloatRange(0, 0), float.MaxValue,
                springFrequency, dampingRatio);
            Simulate(physicsJoint);

            var dynMotionData = m_World.DynamicsWorld.MotionDatas[m_DynamicBodyIndex];
            Assert.That(dynMotionData.WorldFromMotion.pos.y, Is.EqualTo(-equilibriumSpringExtension).Within(1e-3));
        }

        ///<summary>
        /// Creates a hinge joint with a locked spring and a distinct arm length,
        /// and checks if the hinged body settles at the expected target angle.
        /// </summary>
        [Test]
        public void Hinge_AngularSpring_ReachesCorrectEquilibriumAngle([Values(0.05f, 0.1f, 0.3f)] float dampingRatio)
        {
            var armLength = 0.5f;
            var jointAnchor = new BodyFrame(new RigidTransform(quaternion.identity, new float3(0, 0, armLength)));

            // Create hinge joint and set it up as an angular spring with target at 0 and a specific expected equilibrium angle
            var equilibriumAngleInDegrees = 15.0f;
            var equilibriumAngle = math.radians(equilibriumAngleInDegrees);
            var targetAngle = 0.0f;

            // Required restoring torque under the assumption that the body is at 1 distance unit away from
            // the hinge joint anchor point.
            var dynMotionVelocity = m_World.DynamicsWorld.MotionVelocities[m_DynamicBodyIndex];
            var restoringTorque = armLength * -m_StepInput.Gravity.y * math.rcp(dynMotionVelocity.InverseMass) * math.sin(math.PIHALF - equilibriumAngle);

            // spring stiffness to achieve the required restoring torque at the target angle
            var requiredStiffnessCoefficient = restoringTorque / equilibriumAngle;

            // convert spring stiffness to spring frequency:
            var inertia = math.rcp(dynMotionVelocity.InverseInertia);
            var springFrequency = JacobianUtilities.CalculateSpringFrequencyFromSpringConstant(requiredStiffnessCoefficient, inertia.x);

            var rotationalMotor = PhysicsJoint.CreateRotationalMotor(jointAnchor, jointAnchor,
                targetAngle, float.MaxValue, springFrequency, dampingRatio);
            Simulate(rotationalMotor, 600);

            // check that body is at expected angle
            var dynMotionData = m_World.DynamicsWorld.MotionDatas[m_DynamicBodyIndex];
            var dynRot = dynMotionData.WorldFromMotion.rot;
            ((Quaternion)dynRot).ToAngleAxis(out var angle, out var axis);
            Assert.That((float3)axis, Is.PrettyCloseTo(new float3(-1, 0, 0)));
            Assert.That(angle, Is.EqualTo(equilibriumAngleInDegrees).Within(1));
        }

        ///<summary>
        /// Creates a hinge joint with limits locked by a soft angular spring, and checks if the hinged body hits the limit.
        /// </summary>
        [Test]
        public void Hinge_HitsLimit([Values(0.0f, 0.1f, 0.3f)] float dampingRatio, [Values(0.1f, 0.5f, 1f)] float springFrequency)
        {
            var armLength = 0.5f;
            var jointAnchor = new BodyFrame(new RigidTransform(quaternion.identity, new float3(0, 0, armLength)));

            // create hinge joint with limits
            var limitAngleInDegrees = 10.0f;
            var limitAngle = math.radians(limitAngleInDegrees);
            var rotationalMotor = PhysicsJoint.CreateRotationalMotor(jointAnchor, jointAnchor, 0, new Math.FloatRange(-limitAngle, limitAngle), float.MaxValue,
                springFrequency, dampingRatio);

            Simulate(rotationalMotor, 600);

            // check that body is at expected angle
            var dynMotionData = m_World.DynamicsWorld.MotionDatas[m_DynamicBodyIndex];
            var dynRot = dynMotionData.WorldFromMotion.rot;
            ((Quaternion)dynRot).ToAngleAxis(out var angle, out var axis);
            Assert.That((float3)axis, Is.PrettyCloseTo(new float3(-1, 0, 0)));
            Assert.That(angle, Is.EqualTo(limitAngleInDegrees).Within(1));
        }
    }
}
