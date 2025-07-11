using NUnit.Framework;
using Unity.Mathematics;

namespace Unity.Physics.Tests.Dynamics.Motors
{
    [TestFixture]
    public class LinearVelocityMotorTests
    {
        // Called by Constant Velocity Tests. These tests require a larger test threshold. A smaller threshold is possible if
        // more frequency and numSteps are increased, don't want tests to take too long.
        // This test checks:
        // 1) Verifies bodyA target velocity has been achieved within the given time
        // 2) Verifies that the orientation of bodyA after one rotation matches the initial orientation
        // 3) Verifies that the position of bodyA after one rotation matches the initial position
        // 4) Verifies that the maxImpulse is never exceeded
        // 5) Verifies that the variance in the average angular velocity is within a threshold
        void TestSimulateLinearVelocityMotor_ConstantSpeed(string testName, Joint jointData, int numSubsteps, int numSolverIterations,
            MotionVelocity velocityA, MotionVelocity velocityB, MotionData motionA, MotionData motionB,
            bool useGravity, float3 targetVelocity, float maxImpulse)
        {
            string failureMessage;

            int numFrames = 61; //duration 1.0s to get full rotation
            int numStabilizingSteps = 0; //initializing test at target velocity

            var motorOrientation = math.normalizesafe(targetVelocity); //to only consider direction the motor is acting on

            var position0 = motionA.WorldFromMotion.pos;
            var rotation0 = motionA.WorldFromMotion.rot;

            MotorTestRunner.TestSimulateMotor(testName, ref jointData, MotorTestRunner.JointType.LinearVelocityMotor,
                ref velocityA, ref velocityB, ref motionA, ref motionB,
                useGravity, maxImpulse, motorOrientation, numSubsteps, numSolverIterations, numFrames, numStabilizingSteps,
                out float3 accumulateAngularVelocity, out float3 accumulateLinearVelocity);

            const float threshold = 0.0001f;

            // After simulation, Angular speed should be zero:
            var compareToTarget = math.abs(velocityA.AngularVelocity);
            failureMessage = $"{testName}: Final angular velocity failed test with angular velocity {velocityA.AngularVelocity}. Target: 0";
            Assert.Less(compareToTarget.x, threshold, failureMessage);
            Assert.Less(compareToTarget.y, threshold, failureMessage);
            Assert.Less(compareToTarget.z, threshold, failureMessage);

            // After simulation, the position should be at d2 = d1 + vt:
            var expectedPosition = position0 + targetVelocity * numFrames * MotorTestRunner.Timestep;
            var positionChange = math.abs(motionA.WorldFromMotion.pos - expectedPosition);
            failureMessage = $"{testName}: Position after simulation {motionA.WorldFromMotion.pos} doesn't match expected position: {expectedPosition}";
            Assert.Less(positionChange.x, threshold, failureMessage);
            Assert.Less(positionChange.y, threshold, failureMessage);
            Assert.Less(positionChange.z, threshold, failureMessage);

            // After simulation, the rotation should be unchanged:
            var orientationDifference = math.mul(motionA.WorldFromMotion.rot, rotation0).ToEulerAngles();
            failureMessage = $"{testName}: Rotation after simulation {motionA.WorldFromMotion.rot.ToEulerAngles()} should be unchanged: {rotation0.ToEulerAngles()}";
            Assert.Less(orientationDifference.x, threshold, failureMessage);
            Assert.Less(orientationDifference.y, threshold, failureMessage);
            Assert.Less(orientationDifference.z, threshold, failureMessage);

            // [Units rad/s] The motor should maintain a constant velocity over many iterations:
            var meanAngularVelocity = accumulateAngularVelocity / numFrames;
            compareToTarget = math.abs(meanAngularVelocity); //target = 0.0 rad/s
            failureMessage = $"{testName}: Averaged angular velocity failed test with mean {meanAngularVelocity}. Target: 0";
            Assert.Less(compareToTarget.x, threshold, failureMessage);
            Assert.Less(compareToTarget.y, threshold, failureMessage);
            Assert.Less(compareToTarget.z, threshold, failureMessage);

            // After one full rotation, averaged linear velocity should be zero
            var meanLinearVelocity = accumulateLinearVelocity / numFrames;
            compareToTarget = math.abs(meanLinearVelocity - targetVelocity);
            failureMessage = $"{testName}: Averaged linear velocity failed test with mean {meanLinearVelocity}. Target: {targetVelocity}";
            Assert.Less(compareToTarget.x, threshold, failureMessage);
            Assert.Less(compareToTarget.y, threshold, failureMessage);
            Assert.Less(compareToTarget.z, threshold, failureMessage);
        }

        // Vary the direction of motion with/without gravity
        private static readonly TestCaseData[] k_LVM_ConstantVelocityTestCases =
        {
            // Arguments: direction of motion, using gravity
            new TestCaseData(1, 4, new float3(-1f, 0f, 0f), false).SetName("1/4: Axis Aligned -x, gravity off"),
            new TestCaseData(1, 4, new float3(0f, -1f, 0f), false).SetName("1/4: Axis Aligned -y, gravity off"),
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), false).SetName("1/4: Axis Aligned -z, gravity off"),

            new TestCaseData(1, 4, new float3(1f, 0f, 0f), false).SetName("1/4: Axis Aligned +x, gravity off"),
            new TestCaseData(1, 4, new float3(0f, 1f, 0f), false).SetName("1/4: Axis Aligned +y, gravity off"),
            new TestCaseData(1, 4, new float3(0f, 0f, 1f), false).SetName("1/4: Axis Aligned +z, gravity off"),

            new TestCaseData(1, 4, new float3(-1f, 0f, 0f), true).SetName("1/4: Axis Aligned -x, gravity on"),
            new TestCaseData(1, 4, new float3(0f, -1f, 0f), true).SetName("1/4: Axis Aligned -y, gravity on"),
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), true).SetName("1/4: Axis Aligned -z, gravity on"),

            new TestCaseData(1, 4, new float3(1f, 0f, 0f), true).SetName("1/4: Axis Aligned +x, gravity on"),
            new TestCaseData(1, 4, new float3(0f, 1f, 0f), true).SetName("1/4: Axis Aligned +y, gravity on"),
            new TestCaseData(1, 4, new float3(0f, 0f, 1f), true).SetName("1/4: Axis Aligned +z, gravity on"),

            new TestCaseData(1, 4, new float3(-1f, -1f, 0f), false).SetName("1/4: Off-Axis -x/-y, gravity off"),
            new TestCaseData(1, 4, new float3(-1f, -1f, -1f), false).SetName("1/4: Off-Axis -x/-y/-z, gravity off"),
            new TestCaseData(1, 4, new float3(0f, -1f, 1f), false).SetName("1/4: Off-Axis -y/+z, gravity off"),
            new TestCaseData(1, 4, new float3(1f, 0f, 1f), false).SetName("1/4: Off-Axis +x/+z, gravity off"),

            new TestCaseData(1, 4, new float3(-1f, -1f, 0f), true).SetName("1/4: Off-Axis -x/-y, gravity on"),
            new TestCaseData(1, 4, new float3(-1f, -1f, -1f), true).SetName("1/4: Off-Axis -x/-y/-z, gravity on"),
            new TestCaseData(1, 4, new float3(0f, -1f, 1f), true).SetName("1/4: Off-Axis -y/+z, gravity on"),
            new TestCaseData(1, 4, new float3(1f, 0f, 1f), true).SetName("1/4: Off-Axis +x/+z, gravity on"),

            new TestCaseData(4, 1, new float3(-1f, 0f, 0f), false).SetName("4/1: Axis Aligned -x, gravity off"),
            new TestCaseData(4, 1, new float3(0f, -1f, 0f), false).SetName("4/1: Axis Aligned -y, gravity off"),
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), false).SetName("4/1: Axis Aligned -z, gravity off"),

            new TestCaseData(4, 1, new float3(1f, 0f, 0f), false).SetName("4/1: Axis Aligned +x, gravity off"),
            new TestCaseData(4, 1, new float3(0f, 1f, 0f), false).SetName("4/1: Axis Aligned +y, gravity off"),
            new TestCaseData(4, 1, new float3(0f, 0f, 1f), false).SetName("4/1: Axis Aligned +z, gravity off"),

            new TestCaseData(4, 1, new float3(-1f, 0f, 0f), true).SetName("4/1: Axis Aligned -x, gravity on"),
            new TestCaseData(4, 1, new float3(0f, -1f, 0f), true).SetName("4/1: Axis Aligned -y, gravity on"),
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), true).SetName("4/1: Axis Aligned -z, gravity on"),

            new TestCaseData(4, 1, new float3(1f, 0f, 0f), true).SetName("4/1: Axis Aligned +x, gravity on"),
            new TestCaseData(4, 1, new float3(0f, 1f, 0f), true).SetName("4/1: Axis Aligned +y, gravity on"),
            new TestCaseData(4, 1, new float3(0f, 0f, 1f), true).SetName("4/1: Axis Aligned +z, gravity on"),

            new TestCaseData(4, 1, new float3(-1f, -1f, 0f), false).SetName("4/1: Off-Axis -x/-y, gravity off"),
            new TestCaseData(4, 1, new float3(-1f, -1f, -1f), false).SetName("4/1: Off-Axis -x/-y/-z, gravity off"),
            new TestCaseData(4, 1, new float3(0f, -1f, 1f), false).SetName("4/1: Off-Axis -y/+z, gravity off"),
            new TestCaseData(4, 1, new float3(1f, 0f, 1f), false).SetName("4/1: Off-Axis +x/+z, gravity off"),

            new TestCaseData(4, 1, new float3(-1f, -1f, 0f), true).SetName("4/1: Off-Axis -x/-y, gravity on"),
            new TestCaseData(4, 1, new float3(-1f, -1f, -1f), true).SetName("4/1: Off-Axis -x/-y/-z, gravity on"),
            new TestCaseData(4, 1, new float3(0f, -1f, 1f), true).SetName("4/1: Off-Axis -y/+z, gravity on"),
            new TestCaseData(4, 1, new float3(1f, 0f, 1f), true).SetName("4/1: Off-Axis +x/+z, gravity on"),
        };

        // Purpose of this test is for bodyA to make a full rotation in the expected amount of time and arrive back at its
        // starting point after one full rotation.
        [TestCaseSource(nameof(k_LVM_ConstantVelocityTestCases))]
        public void ConstantVelocityTest_LVM(int numSubsteps, int numSolverIterations, float3 directionOfMovement, bool useGravity)
        {
            // Constants: BodyB is resting below BodyA
            RigidTransform worldFromA = new RigidTransform(quaternion.identity, float3.zero);
            RigidTransform worldFromB = new RigidTransform(quaternion.identity, new float3(0.0f, -1.0f, 0.0f));
            float targetSpeed = 1.0f; //m/s
            float maxImpulseForMotor = math.INFINITY;

            // Calculated variables
            var direction = math.normalize(directionOfMovement);
            var targetVelocity = targetSpeed * direction;

            MotorTestUtility.SetupMotionVelocity(out MotionVelocity velocityA, out MotionVelocity velocityB);
            velocityA.LinearVelocity = targetVelocity; //initialize at target velocity
            MotorTestUtility.SetupMotionData(worldFromA, worldFromB, out MotionData motionA, out MotionData motionB);

            var jointFrameA = JacobianUtilities.CalculateDefaultBodyFramesForConnectedBody(
                worldFromA, worldFromB, float3.zero, direction, out BodyFrame jointFrameB, false);

            Joint joint = MotorTestRunner.CreateTestMotor(
                PhysicsJoint.CreateLinearVelocityMotor(jointFrameA, jointFrameB, targetSpeed, maxImpulseForMotor));

            TestSimulateLinearVelocityMotor_ConstantSpeed("Constant Velocity Tests (LVM)", joint, numSubsteps, numSolverIterations,
                velocityA, velocityB, motionA, motionB, useGravity, targetVelocity, maxImpulseForMotor);
        }

        // Called by: Orientation tests, Max Impulse Tests
        // Constants: the number steps, solver iterations, initial motion of BodyA and bodyB
        // This test checks:
        // 1) Verifies bodyA target velocity has been achieved within the given time
        // 2) Verifies that the orientation of bodyA has not changed while it moved
        // 3) Verifies that the maxImpulse is never exceeded
        // 4) Verifies that the variance in the average linear velocity is within a threshold
        // 5) Verifies that the variance in the average angular velocity is within a threshold
        void TestSimulateLinearVelocityMotor(string testName, Joint jointData, int numSubsteps, int numSolverIterations,
            MotionVelocity velocityA, MotionVelocity velocityB, MotionData motionA, MotionData motionB,
            bool useGravity, float3 targetVelocity, float maxImpulse)
        {
            int numFrames = 6;  // duration = 0.1s
            int numStabilizingSteps = 0; //takes some iterations to reach the target velocity

            var angularVelocity0 = velocityA.AngularVelocity;
            var rotation0 = motionA.WorldFromMotion.rot;
            var motorOrientation = math.normalizesafe(targetVelocity); //to only consider direction the motor is acting on

            MotorTestRunner.TestSimulateMotor(testName, ref jointData, MotorTestRunner.JointType.LinearVelocityMotor,
                ref velocityA, ref velocityB, ref motionA, ref motionB,
                useGravity, maxImpulse, motorOrientation, numSubsteps, numSolverIterations, numFrames, numStabilizingSteps,
                out float3 accumulateAngularVelocity, out float3 accumulateLinearVelocity);

            // Note: off-axis tests require the *10 to pass
            var testThreshold = 1e-6;

            var meanAngular = accumulateAngularVelocity / numFrames;
            var meanLinear = accumulateLinearVelocity / numFrames;

            // The motor should maintain a constant velocity over many iterations. Verify variance is within threshold:
            var compareToTarget = math.abs(meanLinear - targetVelocity);
            string failureMessage = $"{testName}: Averaged linear velocity failed test with mean {meanLinear}. Target: {targetVelocity}";
            Assert.LessOrEqual(compareToTarget.x, testThreshold, failureMessage);
            Assert.LessOrEqual(compareToTarget.y, testThreshold, failureMessage);
            Assert.LessOrEqual(compareToTarget.z, testThreshold, failureMessage);

            // Angular Velocity should not have changed during test
            compareToTarget = math.abs(meanAngular - angularVelocity0);
            failureMessage = $"{testName}: Averaged angular velocity failed test with mean {meanAngular}. Target: {angularVelocity0}";
            Assert.LessOrEqual(compareToTarget.x, testThreshold, failureMessage);
            Assert.LessOrEqual(compareToTarget.y, testThreshold, failureMessage);
            Assert.LessOrEqual(compareToTarget.z, testThreshold, failureMessage);

            // Linear velocity after simulation should be within testThreshold of the target
            compareToTarget = math.abs(velocityA.LinearVelocity - targetVelocity);
            failureMessage = $"{testName}: Final linear velocity failed test with linear velocity {velocityA.LinearVelocity}. Target: {targetVelocity}";
            Assert.LessOrEqual(compareToTarget.x, testThreshold, failureMessage);
            Assert.LessOrEqual(compareToTarget.y, testThreshold, failureMessage);
            Assert.LessOrEqual(compareToTarget.z, testThreshold, failureMessage);

            // Verify that the orientation of bodyA hasn't changed while it moved to the target
            var orientationDifference = math.mul(motionA.WorldFromMotion.rot, rotation0).ToEulerAngles();
            failureMessage = $"{testName}: Linear velocity motor orientation changed during simulation from {rotation0.ToEulerAngles()} to {motionA.WorldFromMotion.rot.ToEulerAngles()}";
            Assert.LessOrEqual(orientationDifference.x, testThreshold, failureMessage);
            Assert.LessOrEqual(orientationDifference.y, testThreshold, failureMessage);
            Assert.LessOrEqual(orientationDifference.z, testThreshold, failureMessage);
        }

        // Check for positive and negative targets that are axis-aligned and off-axis.
        private static readonly float3[] k_LVM_DirectionOfMovement =
        {
            new float3(0f, 0f, -1f),
            new float3(0f, 0f, 1f),

            new float3(0f, -1f, 0f),
            new float3(0f, 1f, 0f),

            new float3(-1f, 0f, 0f),
            new float3(1f, 0f, 0f),

            new float3(-1f, -1f, 0f),
            new float3(-1f, -1f, -1f),
            new float3(0f, -1f, 1f),
            new float3(1f, 0f, 1f)
        };

        // Test with gravity enabled and disabled.
        private static readonly bool[] k_LVM_UseGravity =
        {
            true,
            false
        };

        // Vary initial target velocity
        private static readonly bool[] k_LVM_InitAtTargetVelocity =
        {
            true, // warm start test with v = target velocity
            false // begin test with v = 0
        };

        // Since the value of spring frequency and damping ratio are so coupled, do not test the permutations separately
        // First index (x) = Spring Frequency, Second index (y) = Damping Ratio
        private static readonly float2[] k_LVM_SpringFrequencyAndDampingRatio =
        {
            new float2(Constraint.DefaultSpringFrequency, Constraint.DefaultDampingRatio),
            new float2(Constraint.DefaultSpringFrequency, 0.75f)
        };

        private static readonly int2[] k_LVM_Steps =
        {
            new int2(1, 4),
            new int2(4, 1)
        };

        // Test Case Input: direction of movement vector, if gravity is enabled/disabled, if we initialize the test
        // at 0 or at the target velocity, vary spring & damping.
        // Checking for direction of movements that are axis-aligned with permutations of the other input data
        private static TestCaseData[] k_LVM_Permutations = MakePermutations();
        private static TestCaseData[] MakePermutations()
        {
            int count = 0;
            int length = k_LVM_Steps.Length * k_LVM_UseGravity.Length * k_LVM_DirectionOfMovement.Length *
                k_LVM_InitAtTargetVelocity.Length * k_LVM_SpringFrequencyAndDampingRatio.Length;
            TestCaseData[] testList = new TestCaseData[length];
            foreach (int2 iStep in k_LVM_Steps)
            {
                foreach (bool iGravity in k_LVM_UseGravity)
                {
                    foreach (float3 iDirection in k_LVM_DirectionOfMovement)
                    {
                        foreach (bool iTarget in k_LVM_InitAtTargetVelocity)
                        {
                            foreach (float2 iSpringAndDamping in k_LVM_SpringFrequencyAndDampingRatio)
                            {
                                var name =
                                    $"[{iStep.x}/{iStep.y}] Test {count}: gravity:{iGravity}, direction:{iDirection}, initAtTarget:{iTarget}, springF:{iSpringAndDamping.x}, dampingR:{iSpringAndDamping.y}";
                                testList[count] = new TestCaseData(iStep.x, iStep.y, iDirection, iGravity, iTarget,
                                    iSpringAndDamping.x, iSpringAndDamping.y).SetName(name);
                                count++;
                            }
                        }
                    }
                }
            }

            return testList;
        }

        // For a given target speed and impulse, the test case vary the direction of movement, if gravity is used and if
        // the initial velocity is zero or not. The constant orientation of bodyA and bodyB is such that bodyB is static
        // and bodyA is located directly under bodyB. These tests use a maxImpulse of infinity.
        [TestCaseSource(nameof(k_LVM_Permutations))]
        public void OrientationTests_LVM(int numSubsteps, int numSolverIterations, float3 directionOfMovement,
            bool useGravity, bool initAtTargetVelocity, float springFrequency, float dampingRatio)
        {
            // Constants: BodyB is resting above BodyA
            RigidTransform worldFromA = new RigidTransform(quaternion.identity, new float3(-0.5f, 5f, -4f));
            RigidTransform worldFromB = new RigidTransform(quaternion.identity, new float3(-0.5f, 6f, -4f));
            var targetSpeed = 10.0f;  // in m/s
            var anchorPosition = float3.zero;
            var maxImpulse = math.INFINITY;

            // Calculated variables
            var axis = math.normalize(directionOfMovement);
            var targetVelocity = targetSpeed * axis;

            // Set up MotionVelocity and MotionData for each body
            MotorTestUtility.SetupMotionVelocity(out MotionVelocity velocityA, out MotionVelocity velocityB);
            if (initAtTargetVelocity) velocityA.LinearVelocity = targetVelocity;
            MotorTestUtility.SetupMotionData(worldFromA, worldFromB, out MotionData motionA, out MotionData motionB);

            var jointFrameA = JacobianUtilities.CalculateDefaultBodyFramesForConnectedBody(
                worldFromA, worldFromB, anchorPosition, axis, out BodyFrame jointFrameB, false);

            Joint joint = MotorTestRunner.CreateTestMotor(
                PhysicsJoint.CreateLinearVelocityMotor(jointFrameA, jointFrameB, targetSpeed, maxImpulse, springFrequency, dampingRatio));

            TestSimulateLinearVelocityMotor("Orientation Tests (LVM)", joint, numSubsteps, numSolverIterations,
                velocityA, velocityB, motionA, motionB, useGravity, targetVelocity, maxImpulse);
        }

        // Test Case Input: direction of movement vector, if gravity is enabled/disabled, if we initialize the test
        // at 0 or at the target velocity.
        // Checking for direction of movements that are axis-aligned or off-axis with permutations of the other input data
        private static readonly TestCaseData[] k_LVM_maxImpulseTestCases =
        {
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), false, false).SetName("1/4: On-Axis maxImpulse, gravity off, v0=0"),
            new TestCaseData(1, 4, new float3(1f, 1f, -1f), false, false).SetName("1/4: Off-Axis maxImpulse, gravity off, v0=0"),

            new TestCaseData(1, 4, new float3(0f, 0f, -1f), true, false).SetName("1/4: On-Axis maxImpulse, gravity on, v0=0"),
            new TestCaseData(1, 4, new float3(1f, 1f, -1f), true, false).SetName("1/4: Off-Axis maxImpulse, gravity on, v0=0"),

            new TestCaseData(4, 1, new float3(0f, 0f, -1f), false, false).SetName("4/1: On-Axis maxImpulse, gravity off, v0=0"),
            new TestCaseData(4, 1, new float3(1f, 1f, -1f), false, false).SetName("4/1: Off-Axis maxImpulse, gravity off, v0=0"),

            new TestCaseData(4, 1, new float3(0f, 0f, -1f), true, false).SetName("4/1: On-Axis maxImpulse, gravity on, v0=0"),
            new TestCaseData(4, 1, new float3(1f, 1f, -1f), true, false).SetName("4/1: Off-Axis maxImpulse, gravity on, v0=0"),
        };

        // For a given target speed and impulse, the test case vary the direction of movement, if gravity is used and if
        // the initial velocity is zero or not. The constant orientation of bodyA and bodyB is such that bodyB is static
        // and bodyA is located directly under bodyB.
        [TestCaseSource(nameof(k_LVM_maxImpulseTestCases))]
        public void MaxImpulseTests_LVM(int numSubsteps, int numSolverIterations, float3 directionOfMovement,
            bool useGravity, bool initAtTargetVelocity)
        {
            // Constants: BodyB is resting above BodyA
            RigidTransform worldFromA = new RigidTransform(quaternion.identity, new float3(-0.5f, 5f, -4f));
            RigidTransform worldFromB = new RigidTransform(quaternion.identity, new float3(-0.5f, 6f, -4f));
            var targetSpeed = 2.0f;  // in m/s
            var anchorPosition = float3.zero;
            var maxImpulse = 10.0f;

            // Calculated variables
            var axis = math.normalize(directionOfMovement);
            var targetVelocity = targetSpeed * axis;

            // Set up MotionVelocity and MotionData for each body
            MotorTestUtility.SetupMotionVelocity(out MotionVelocity velocityA, out MotionVelocity velocityB);
            if (initAtTargetVelocity) velocityA.LinearVelocity = targetVelocity;
            MotorTestUtility.SetupMotionData(worldFromA, worldFromB, out MotionData motionA, out MotionData motionB);

            var jointFrameA = JacobianUtilities.CalculateDefaultBodyFramesForConnectedBody(
                worldFromA, worldFromB, anchorPosition, axis, out BodyFrame jointFrameB, false);

            Joint joint = MotorTestRunner.CreateTestMotor(
                PhysicsJoint.CreateLinearVelocityMotor(jointFrameA, jointFrameB, targetSpeed, maxImpulse));

            TestSimulateLinearVelocityMotor("Max Impulse Tests (LVM)", joint, numSubsteps, numSolverIterations,
                velocityA, velocityB, motionA, motionB, useGravity, targetVelocity, maxImpulse);
        }
    }
}
