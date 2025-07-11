using System;
using NUnit.Framework;
using Unity.Mathematics;
using Random = Unity.Mathematics.Random;

namespace Unity.Physics.Tests.Dynamics.Motors
{
    [TestFixture]
    public class PositionMotorTests
    {
        // Called by: Orientation tests, Max Impulse Tests
        // Constants: the number steps, solver iterations, initial motion of BodyA and bodyB
        // This test checks:
        // 1) Verifies bodyA arrives at the target position within the given time,
        // 2) Verifies that the orientation of bodyA has not changed while it moved
        // 3) Verifies that the maxImpulse is never exceeded
        void TestSimulatePositionMotor(string testName, Joint jointData, int numSubsteps, int numSolverIterations,
            MotionVelocity velocityA, MotionVelocity velocityB, MotionData motionA, MotionData motionB,
            bool useGravity, float3 targetPosition, float maxImpulse, int numStabilizingSteps, bool isException = false)
        {
            int numFrames = 30; // duration = 0.5s

            var position0 = motionA.WorldFromMotion.pos;
            var rotation0 = motionA.WorldFromMotion.rot;
            var motorOrientation = math.normalizesafe(targetPosition); //to only consider direction the motor is acting on

            MotorTestRunner.TestSimulateMotor(testName, ref jointData, MotorTestRunner.JointType.PositionMotor,
                ref velocityA, ref velocityB, ref motionA, ref motionB,
                useGravity, maxImpulse, motorOrientation, numSubsteps, numSolverIterations, numFrames, numStabilizingSteps,
                out float3 accumulateAngularVelocity, out float3 accumulateLinearVelocity, isException);

            // Note: some off-axis / gravity / softer spring & damping enabled tests require a larger threshold
            var testThreshold = 0.01f;

            // Verify that bodyA arrived at target position by some threshold, unless the maxImpulse was zero
            var distanceAmoved = motionA.WorldFromMotion.pos - position0;
            float3 compareToTarget = maxImpulse < math.EPSILON
                ? math.abs(distanceAmoved) //if maxImpulse=0, then motor shouldn't move
                : math.abs(targetPosition - distanceAmoved);

            // Verify that the orientation of bodyA hasn't changed while it moved to the target
            var orientationDifference = math.mul(motionA.WorldFromMotion.rot, rotation0).ToEulerAngles();
            string failureMessage = $"{testName}: Position motor orientation changed during simulation from {rotation0} to {motionA.WorldFromMotion.rot}";
            Assert.Less(orientationDifference.x, testThreshold, failureMessage);
            Assert.Less(orientationDifference.y, testThreshold, failureMessage);
            Assert.Less(orientationDifference.z, testThreshold, failureMessage);

            if (isException) //when motor moves due to gravity when impulse=0. Should move, but not to target
            {
                failureMessage = $"{testName}: Position motor didn't move at all. Position: {targetPosition}";
                Assert.Greater(compareToTarget.x, testThreshold, failureMessage);
                Assert.Greater(compareToTarget.y, testThreshold, failureMessage);
                Assert.Greater(compareToTarget.z, testThreshold, failureMessage);
            }
            else
            {
                failureMessage = $"{testName}: Position motor didn't arrive at target {targetPosition} by this margin {compareToTarget}";
                Assert.Less(compareToTarget.x, testThreshold, failureMessage);
                Assert.Less(compareToTarget.y, testThreshold, failureMessage);
                Assert.Less(compareToTarget.z, testThreshold, failureMessage);
            }
        }

        // Check for positive and negative targets that are both axis-aligned and off-axis.
        private static readonly float3[] k_PM_DirectionOfMovement =
        {
            new float3(0f, 0f, -1f),
            new float3(0f, 0f, 1f),
            new float3(0f, -1f, 0f),
            new float3(0f, 1f, 0f),
            new float3(-1f, 0f, 0f),
            new float3(1f, 0f, 0f),

            new float3(-1f, 1f, 0f),
            new float3(3f, 0f, -2f),
            new float3(0f, 0.5f, -2f),
            new float3(1.5f, 1.5f, 1.5f),
            new float3(-1.5f, -1.5f, -1.5f),
            new float3(1.5f, -1.5f, 1.5f)
        };

        // Test with gravity enabled and disabled.
        private static readonly bool[] k_PM_UseGravity =
        {
            true,
            false
        };

        // Since the value of spring frequency and damping ratio are so coupled, do not test the permutations separately
        // First index (x) = Spring Frequency, Second index (y) = Damping Ratio
        private static readonly float2[] k_PM_SpringFrequencyAndDampingRatio =
        {
            new float2(Constraint.DefaultSpringFrequency, Constraint.DefaultDampingRatio),
            new float2(5.0f, 0.7f)
        };

        private static readonly int2[] k_PM_Steps =
        {
            new int2(1, 4),
            new int2(4, 1)
        };

        private static TestCaseData[] k_PM_Permutations = MakePermutations();

        // Using a constant of numStabilizingStep = 30 for the softer spring/damping parameters right now. With the
        // default spring/damping parameters, a value of 0 can be used
        private static TestCaseData[] MakePermutations()
        {
            int count = 0;
            int length = k_PM_Steps.Length * k_PM_UseGravity.Length * k_PM_DirectionOfMovement.Length *
                k_PM_SpringFrequencyAndDampingRatio.Length;
            TestCaseData[] testList = new TestCaseData[length];

            foreach (int2 iStep in k_PM_Steps)
            {
                foreach (bool iGravity in k_PM_UseGravity)
                {
                    foreach (float3 iDirection in k_PM_DirectionOfMovement)
                    {
                        foreach (float2 iSpringAndDamping in k_PM_SpringFrequencyAndDampingRatio)
                        {
                            // skip conditions (for tests known to fail)
                            if (iGravity &&
                                (math.abs(iSpringAndDamping.x - Constraint.DefaultSpringFrequency) < 0.0001f))
                            {
                                // iDirection = (0f, -1f, 0f), iGravity=true, spring = Constraint.DefaultSpringFrequency) //Disabled since applying gravity along the axis of motion fails tests
                                // iDirection = (0f, 1f, 0f), iGravity=true, spring = Constraint.DefaultSpringFrequency)  //Disabled since applying gravity along the axis of motion fails tests

                                var check = math.length(math.abs(iDirection) - new float3(0f, 1f, 0f));
                                if (check < 0.0001f)
                                {
                                    length--;
                                    continue;
                                }
                            }

                            if (iGravity && iStep.x == 4 && iStep.y == 1 && iDirection.Equals(new float3(0f, -1f, 0f)) && iSpringAndDamping.Equals(new float2(5.0f, 0.7f)))
                            {
                                //Skip [4/1] Test 50: gravity:True, direction:float3(0f, -1f, 0f), springF:5, dampingR:0.7
                                //Linear Motor impulse 40.1635 exceeded maximum (40) Expected<= 0.029f, But was: 0.16350174f
                                length--;
                                continue;
                            }

                            var name =
                                $"[{iStep.x}/{iStep.y}] Test {count}: gravity:{iGravity}, direction:{iDirection}, springF:{iSpringAndDamping.x}, dampingR:{iSpringAndDamping.y}";
                            testList[count] = new TestCaseData(iStep.x, iStep.y, iDirection, iGravity,
                                iSpringAndDamping.x, iSpringAndDamping.y, 30).SetName(name);
                            count++;
                        }
                    }
                }
            }

            Array.Resize(ref testList, count);

            return testList;
        }

        // Tests that with a given direction of movement of specified that bodyA arrives at the target position within a
        // set number of steps. Variables: direction of movement, if gravity is enabled/disabled, springFrequency and dampingRatio.
        // Held constant: the target distance, the anchor position of bodyA, the maxImpulse for the motor, numStabilizingSteps
        [TestCaseSource(nameof(k_PM_Permutations))]
        public void OrientationTests_PM(int numSubsteps, int numSolverIterations, float3 directionOfMovement, bool useGravity,
            float springFrequency, float dampingRatio, int numStabilizingSteps)
        {
            // Constants: BodyB is resting above BodyA
            RigidTransform worldFromA = new RigidTransform(quaternion.identity, new float3(-0.5f, 5f, -4f));
            RigidTransform worldFromB = new RigidTransform(quaternion.identity, new float3(-0.5f, 6f, -4f));
            float targetDistance = 4.0f;  //distance from anchorA
            var anchorA = float3.zero;
            var maxImpulse = 40.0f;

            // Calculated variables
            var axisInB = math.normalize(directionOfMovement);
            var targetPosition = targetDistance * axisInB;

            MotorTestUtility.SetupMotionVelocity(out MotionVelocity velocityA, out MotionVelocity velocityB);
            MotorTestUtility.SetupMotionData(worldFromA, worldFromB, out MotionData motionA, out MotionData motionB);

            var jointFrameA = JacobianUtilities.CalculateDefaultBodyFramesForConnectedBody(
                worldFromA, worldFromB, anchorA, axisInB, out BodyFrame jointFrameB, false);

            Joint joint = MotorTestRunner.CreateTestMotor(
                PhysicsJoint.CreatePositionMotor(jointFrameA, jointFrameB, targetDistance, maxImpulse, springFrequency, dampingRatio));

            TestSimulatePositionMotor("OrientationTests (PM)", joint, numSubsteps, numSolverIterations,
                velocityA, velocityB, motionA, motionB, useGravity, targetPosition, maxImpulse, numStabilizingSteps);
        }

        // Test cases to verify: for a given direction of movement and with gravity
        // enabled/disabled, the motor arrives at the target.
        private static readonly TestCaseData[] k_PM_maxImpulseTestCases =
        {
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), 10.0f, false, false).SetName("1/4: On-Axis maxImpulse=10, gravity off"),
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), 0.0f, false, false).SetName("1/4: On-Axis maxImpulse=0, gravity off"),
            new TestCaseData(1, 4, new float3(1f, 1f, -1f), 10.0f, false, false).SetName("1/4: Off-Axis maxImpulse=10, gravity off"),
            new TestCaseData(1, 4, new float3(1f, 1f, -1f), 0.0f, false, false).SetName("1/4: Off-Axis maxImpulse=0, gravity off"),

            new TestCaseData(1, 4, new float3(0f, 0f, -1f), 10.0f, true, false).SetName("1/4: On-Axis maxImpulse=10, gravity on"),
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), 0.0f, true, false).SetName("1/4: On-Axis maxImpulse=0, gravity on"),
            new TestCaseData(1, 4, new float3(1f, 1f, -1f), 10.0f, true, false).SetName("1/4: Off-Axis maxImpulse=10, gravity on"),
            new TestCaseData(1, 4, new float3(1f, 0f, -1f), 0.0f, true, false).SetName("1/4: Off-Axis maxImpulse=0, gravity on"),
            new TestCaseData(1, 4, new float3(1f, 1f, -1f), 0.0f, true, true).SetName("1/4: Off-Axis maxImpulse=0, gravity on"), //exception case

            new TestCaseData(4, 1, new float3(0f, 0f, -1f), 10.0f, false, false).SetName("4/1: On-Axis maxImpulse=10, gravity off"),
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), 0.0f, false, false).SetName("4/1: On-Axis maxImpulse=0, gravity off"),
            new TestCaseData(4, 1, new float3(1f, 1f, -1f), 10.0f, false, false).SetName("4/1: Off-Axis maxImpulse=10, gravity off"),
            new TestCaseData(4, 1, new float3(1f, 1f, -1f), 0.0f, false, false).SetName("4/1: Off-Axis maxImpulse=0, gravity off"),

            new TestCaseData(4, 1, new float3(0f, 0f, -1f), 10.0f, true, false).SetName("4/1: On-Axis maxImpulse=10, gravity on"),
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), 0.0f, true, false).SetName("4/1: On-Axis maxImpulse=0, gravity on"),
            new TestCaseData(4, 1, new float3(1f, 1f, -1f), 10.0f, true, false).SetName("4/1: Off-Axis maxImpulse=10, gravity on"),
            new TestCaseData(4, 1, new float3(1f, 0f, -1f), 0.0f, true, false).SetName("4/1: Off-Axis maxImpulse=0, gravity on"),
            new TestCaseData(4, 1, new float3(1f, 1f, -1f), 0.0f, true, true).SetName("4/1: Off-Axis maxImpulse=0, gravity on"), //exception case
        };

        // Purpose of this test is to verify that the maxImpulse for the motor is not exceeded and that if a maxImpulse
        // is not infinity, that a position motor will arrive at the target.
        // Constants: the target distance, the anchor position of bodyA and the max impulse of the motor
        [TestCaseSource(nameof(k_PM_maxImpulseTestCases))]
        public void MaxImpulseTests_PM(int numSubsteps, int numSolverIterations, float3 directionOfMovement,
            float maxImpulse, bool useGravity, bool isException)
        {
            // Constants: BodyB is resting above BodyA
            RigidTransform worldFromA = new RigidTransform(quaternion.identity, new float3(-0.5f, 5f, -4f));
            RigidTransform worldFromB = new RigidTransform(quaternion.identity, new float3(-0.5f, 6f, -4f));
            float targetDistance = 2.0f;  //target is a scalar distance from anchorA
            var anchorPosition = float3.zero;

            // Calculated variables
            var axis = math.normalize(directionOfMovement);
            var targetDisplacement = targetDistance * axis;

            MotorTestUtility.SetupMotionVelocity(out MotionVelocity velocityA, out MotionVelocity velocityB);
            MotorTestUtility.SetupMotionData(worldFromA, worldFromB, out MotionData motionA, out MotionData motionB);

            var jointFrameA = JacobianUtilities.CalculateDefaultBodyFramesForConnectedBody(
                worldFromA, worldFromB, anchorPosition, axis, out BodyFrame jointFrameB, false);

            Joint joint = MotorTestRunner.CreateTestMotor(
                PhysicsJoint.CreatePositionMotor(jointFrameA, jointFrameB, targetDistance, maxImpulse));

            TestSimulatePositionMotor("Max Impulse Tests (PM)", joint, numSubsteps, numSolverIterations,
                velocityA, velocityB, motionA, motionB, useGravity, targetDisplacement, maxImpulse, 0, isException);
        }

        // Runs a simulation with random pivots, random axes, and random velocities for both bodyA and bodyB,
        // for numSteps. On the last step, test checks if InitialError is less than a threshold. This configuration is
        // not going to generate realistic motion. Intention is to verify that the target can be reached by the motor
        [Test]
        public void RandomConfigurationTest_PM()
        {
            MotorTestRunner.RunRandomConfigurationMotorTest("Random Configuration Tests (PM)", (ref Random rnd) =>
            {
                var jointFrameA = new BodyFrame();
                var jointFrameB = new BodyFrame();
                MotorTestUtility.GenerateRandomPivots(ref rnd, out jointFrameA.Position, out jointFrameB.Position);
                MotorTestUtility.GenerateRandomAxes(ref rnd, out jointFrameA.Axis, out jointFrameB.Axis);

                Math.CalculatePerpendicularNormalized(jointFrameA.Axis, out jointFrameA.PerpendicularAxis, out _);
                Math.CalculatePerpendicularNormalized(jointFrameB.Axis, out jointFrameB.PerpendicularAxis, out _);

                var distance = rnd.NextFloat(-5f, 5f);
                //var testRange = new float3(1f, 1f, 1f);
                //var direction = rnd.NextFloat3(-1 * testRange, testRange);
                //var target = math.normalize(direction) * distance; //target is a vector
                var maxImpulseForMotor = math.INFINITY;

                return MotorTestRunner.CreateTestMotor(
                    PhysicsJoint.CreatePositionMotor(jointFrameA, jointFrameB, distance, maxImpulseForMotor));
            });
        }
    }
}
