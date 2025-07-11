using NUnit.Framework;
using Unity.Mathematics;

namespace Unity.Physics.Tests.Dynamics.Motors
{
    [TestFixture]
    public class AngularVelocityMotorTests
    {
        // Called by Constant Velocity Tests. These tests require a larger test threshold. A smaller threshold is possible if
        // more frequency and numSteps are increased, don't want tests to take too long.
        // This test checks:
        // 1) Verifies bodyA target velocity has been achieved within the given time
        // 2) Verifies that the orientation of bodyA after one rotation matches the initial orientation
        // 3) Verifies that the position of bodyA after one rotation matches the initial position
        // 4) Verifies that the maxImpulse is never exceeded
        // 5) Verifies that the variance in the average angular velocity is within a threshold
        void TestSimulateAngularVelocityMotor_ConstantSpeed(string testName, Joint jointData,
            int numSubsteps, int numSolverIterations, MotionVelocity velocityA, MotionVelocity velocityB, MotionData motionA, MotionData motionB,
            bool useGravity, float3 targetVelocity, float maxImpulse)
        {
            int numFrames = 60; //duration 1.0s to get full rotation
            int numStabilizingSteps = 0; //needs to be zero to get initial position and rotation

            var motorOrientation = math.normalizesafe(targetVelocity); //to only consider direction the motor is acting on

            var position0 = motionA.WorldFromMotion.pos;
            var rotation0 = motionA.WorldFromMotion.rot;

            MotorTestRunner.TestSimulateMotor(testName, ref jointData, MotorTestRunner.JointType.AngularVelocityMotor,
                ref velocityA, ref velocityB, ref motionA, ref motionB,
                useGravity, maxImpulse, motorOrientation, numSubsteps, numSolverIterations, numFrames, numStabilizingSteps,
                out float3 accumulateAngularVelocity, out float3 accumulateLinearVelocity);

            // Testing thresholds:
            float thresholdAngular;
            float thresholdLinear;
            if (numSubsteps > 1) // can decrease the thresholds a bit if using substeps
            {
                thresholdAngular = 0.001f;
                thresholdLinear = 0.002f;
            }
            else
            {
                thresholdAngular = 0.01f;
                thresholdLinear = 0.01f;
            }

            // Angular speed after simulation should be within testThreshold of the target velocity:
            var compareToTarget = math.abs(velocityA.AngularVelocity - targetVelocity);
            string failureMessage = $"{testName}: Final angular velocity failed test with angular velocity {velocityA.AngularVelocity}. Target: {targetVelocity}";
            Assert.Less(compareToTarget.x, thresholdAngular, failureMessage);
            Assert.Less(compareToTarget.y, thresholdAngular, failureMessage);
            Assert.Less(compareToTarget.z, thresholdAngular, failureMessage);

            // After one full rotation, the position should match the initial position:
            var positionChange = math.abs(motionA.WorldFromMotion.pos - position0);
            failureMessage = $"{testName}: Position after one rotation {motionA.WorldFromMotion.pos} doesn't match initial position: {position0}";
            Assert.Less(positionChange.x, thresholdLinear, failureMessage);
            Assert.Less(positionChange.y, thresholdLinear, failureMessage);
            Assert.Less(positionChange.z, thresholdLinear, failureMessage);

            // After one full rotation, the rotation should match the initial rotation:
            var orientationDifference = math.mul(motionA.WorldFromMotion.rot, rotation0).ToEulerAngles();
            failureMessage = $"{testName}: Rotation after one rotation {motionA.WorldFromMotion.rot.ToEulerAngles()} doesn't match initial rotation: {rotation0.ToEulerAngles()}";
            Assert.Less(orientationDifference.x, thresholdAngular, failureMessage);
            Assert.Less(orientationDifference.y, thresholdAngular, failureMessage);
            Assert.Less(orientationDifference.z, thresholdAngular, failureMessage);

            // [Units rad/s] The motor should maintain a constant velocity around rotation axis over many iterations:
            var meanAngularVelocity = accumulateAngularVelocity / numFrames;
            compareToTarget = math.abs((meanAngularVelocity - targetVelocity) * motorOrientation);
            failureMessage = $"{testName}: Averaged angular velocity failed test with mean {meanAngularVelocity}. Target: {targetVelocity}";
            Assert.Less(compareToTarget.x, thresholdAngular, failureMessage);
            Assert.Less(compareToTarget.y, thresholdAngular, failureMessage);
            Assert.Less(compareToTarget.z, thresholdAngular, failureMessage);

            // After one full rotation, averaged linear velocity should be zero around rotation axis:
            var meanLinearVelocity = accumulateLinearVelocity / numFrames;
            compareToTarget = math.abs(meanLinearVelocity * motorOrientation);
            failureMessage = $"{testName}: Averaged linear velocity failed test with mean {meanLinearVelocity}. Target: 0";
            Assert.Less(compareToTarget.x, thresholdLinear, failureMessage);
            Assert.Less(compareToTarget.y, thresholdLinear, failureMessage);
            Assert.Less(compareToTarget.z, thresholdLinear, failureMessage);
        }

        // For a rotational axis, we chose pivot points that lie along each 4 (x,y) corners of a cube that rotates about
        // the z-axis and 1 at the center. All simulations start with the initial velocity at target velocity
        private static readonly TestCaseData[] k_AVM_ConstantVelocityTestCases =
        {
            // Arguments: axis of rotation, pivotPosition, using gravity
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(-0.5f, 0.5f, 0), false).SetName("1/4: Axis Aligned -z, -x/+y pivot, gravity off"), // v = -358.163, off by 1.836
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), false).SetName("1/4: Axis Aligned -z, +x/+y pivot, gravity off"),  // v = -358.163, off by 1.836
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(0f, 0f, 0),  false).SetName("1/4: Axis Aligned -z, 0/0 pivot, gravity off"),
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(-0.5f, -0.5f, 0), false).SetName("1/4: Axis Aligned -z, -x/-y pivot, gravity off"), // v = -358.163, off by 1.836
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(0.5f, -0.5f, 0), false).SetName("1/4: Axis Aligned -z, +x/-y pivot, gravity off"),  // v = -358.164, off by 1.835

            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(-0.5f, 0.5f, 0), true).SetName("1/4: Axis Aligned -z, -x/+y pivot, gravity on"), // v = -362.664, off by 2.665
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), true).SetName("1/4: Axis Aligned -z, +x/+y pivot, gravity on"),  // v = -353.918, off by 6.082
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(0f, 0f, 0), true).SetName("1/4: Axis Aligned -z, 0/0 pivot, gravity on"),
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(-0.5f, -0.5f, 0), true).SetName("1/4: Axis Aligned -z, -x/-y pivot, gravity on"), // v= = -362.394, off by 2.394
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(0.5f, -0.5f, 0), true).SetName("1/4: Axis Aligned -z, +x/-y pivot, gravity on"),  // v = -353.670, off by 6.329

            // Arguments: axis of rotation, pivotPosition, using gravity
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(-0.5f, 0.5f, 0), false).SetName("4/1: Axis Aligned -z, -x/+y pivot, gravity off"), // v = -358.163, off by 1.836
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), false).SetName("4/1: Axis Aligned -z, +x/+y pivot, gravity off"),  // v = -358.163, off by 1.836
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(0f, 0f, 0),  false).SetName("4/1: Axis Aligned -z, 0/0 pivot, gravity off"),
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(-0.5f, -0.5f, 0), false).SetName("4/1: Axis Aligned -z, -x/-y pivot, gravity off"), // v = -358.163, off by 1.836
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(0.5f, -0.5f, 0), false).SetName("4/1: Axis Aligned -z, +x/-y pivot, gravity off"),  // v = -358.164, off by 1.835

            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(-0.5f, 0.5f, 0), true).SetName("4/1: Axis Aligned -z, -x/+y pivot, gravity on"), // v = -362.664, off by 2.665
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), true).SetName("4/1: Axis Aligned -z, +x/+y pivot, gravity on"),  // v = -353.918, off by 6.082
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(0f, 0f, 0), true).SetName("4/1: Axis Aligned -z, 0/0 pivot, gravity on"),
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(-0.5f, -0.5f, 0), true).SetName("4/1: Axis Aligned -z, -x/-y pivot, gravity on"), // v= = -362.394, off by 2.394
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(0.5f, -0.5f, 0), true).SetName("4/1: Axis Aligned -z, +x/-y pivot, gravity on"),  // v = -353.670, off by 6.329
        };

        // Purpose of this test is for bodyA to make a full rotation in the expected amount of time and arrive back at its
        // starting point after one full rotation.
        [TestCaseSource(nameof(k_AVM_ConstantVelocityTestCases))]
        public void ConstantVelocityTest_AVM(int numSubsteps, int numSolverIterations, float3 axisOfRotation, float3 pivotPosition, bool useGravity)
        {
            // Constants: BodyB is resting above BodyA
            RigidTransform worldFromA = new RigidTransform(quaternion.identity, new float3(-0.5f, 5f, -4f));
            RigidTransform worldFromB = new RigidTransform(quaternion.identity, new float3(-0.5f, 6f, -4f));
            float targetSpeed = 360.0f;
            float maxImpulseForMotor = math.INFINITY;

            // Calculated variables
            var axis = math.normalize(axisOfRotation);
            var targetSpeedInRadians = math.radians(targetSpeed);
            var targetVelocityInRadians = targetSpeedInRadians * axis;

            MotorTestUtility.SetupMotionVelocity(out MotionVelocity velocityA, out MotionVelocity velocityB);
            velocityA.AngularVelocity = targetVelocityInRadians; //initialize at target velocity
            MotorTestUtility.SetupMotionData(worldFromA, worldFromB, out MotionData motionA, out MotionData motionB);

            var jointFrameA = JacobianUtilities.CalculateDefaultBodyFramesForConnectedBody(
                worldFromA, worldFromB, pivotPosition, axis, out BodyFrame jointFrameB, true);

            Joint joint = MotorTestRunner.CreateTestMotor(
                PhysicsJoint.CreateAngularVelocityMotor(jointFrameA, jointFrameB, targetSpeedInRadians, maxImpulseForMotor));

            TestSimulateAngularVelocityMotor_ConstantSpeed("Constant Velocity Tests (AVM)", joint,
                numSubsteps, numSolverIterations,
                velocityA, velocityB, motionA, motionB, useGravity, targetVelocityInRadians, maxImpulseForMotor);
        }

        // Purpose of this test is to verify that the motor reaches the target velocity. For more complex configurations,
        // (ie: any rotations not aligned with the center of mass) the linear components are not
        // Called by Orientation Tests, Max Impulse Tests
        // Constants: the number of steps, solver iterations, initial motion of bodyB
        // Variable parameters: gravity enabled/disabled, if initial velocity of bodyA starts at 0 or at
        // the target velocity
        // Since it can take some number of steps for the target to be reached, we do not accumulate the velocity
        // of bodyA for numStabilizingSteps, so that the average velocity is not skewed
        // checks: if the constant velocity is maintained after several steps, checks if the target
        // velocity is reached
        void TestSimulateAngularVelocityMotor(string testName, Joint jointData, int numSubsteps, int numSolverIterations,
            MotionVelocity velocityA, MotionVelocity velocityB, MotionData motionA, MotionData motionB,
            bool useGravity, float3 targetVelocity, float maxImpulse, bool isException = false)
        {
            int numFrames = 15; // duration = 0.25s
            int numStabilizingSteps = 5; //takes some iterations to reach the target velocity

            var rotation0 = motionA.WorldFromMotion.rot;
            var motorOrientation = math.normalizesafe(targetVelocity); //to only consider direction the motor is acting on

            MotorTestRunner.TestSimulateMotor(testName, ref jointData, MotorTestRunner.JointType.AngularVelocityMotor,
                ref velocityA, ref velocityB, ref motionA, ref motionB,
                useGravity, maxImpulse, motorOrientation, numSubsteps, numSolverIterations, numFrames, numStabilizingSteps,
                out float3 accumulateAngularVelocity, out float3 accumulateLinearVelocity, isException);

            // Testing thresholds:
            const float thresholdRotation = 0.02f;          // in rad
            const float thresholdAngularVelocity = 0.002f;  // in rad/s

            if (!isException)
            {
                // Angular speed after simulation should be within testThreshold of the target velocity:
                if (maxImpulse < math.EPSILON) targetVelocity = float3.zero; // with no gravity and no impulse, motor shouldn't move
                var compareToTarget = math.abs((velocityA.AngularVelocity - targetVelocity) * motorOrientation);
                string failureMessage = $"{testName}: Final angular velocity failed test with angular velocity {velocityA.AngularVelocity}. Target: {targetVelocity}";
                Assert.Less(compareToTarget.x, thresholdAngularVelocity, failureMessage);
                Assert.Less(compareToTarget.y, thresholdAngularVelocity, failureMessage);
                Assert.Less(compareToTarget.z, thresholdAngularVelocity, failureMessage);

                // The motor should maintain a constant velocity over many iterations, in rad/s:
                var meanAngularVelocity = accumulateAngularVelocity / numFrames;
                compareToTarget = math.abs((meanAngularVelocity - targetVelocity) * motorOrientation);
                failureMessage = $"{testName}: Averaged angular velocity failed test with mean {meanAngularVelocity}. Target: {targetVelocity}";
                Assert.Less(compareToTarget.x, thresholdAngularVelocity, failureMessage);
                Assert.Less(compareToTarget.y, thresholdAngularVelocity, failureMessage);
                Assert.Less(compareToTarget.z, thresholdAngularVelocity, failureMessage);

                // Verify that the rotation of bodyA arrived at expected rotation after simulation:
                var targetAngle = targetVelocity * MotorTestRunner.Timestep * (numFrames + numStabilizingSteps);
                var simulatedAngle = math.mul(motionA.WorldFromMotion.rot, rotation0).ToEulerAngles();
                compareToTarget = math.abs(simulatedAngle - targetAngle);
                failureMessage = $"{testName}: Rotation after simulation {simulatedAngle} doesn't match expected rotation: {targetAngle}";
                Assert.Less(compareToTarget.x, thresholdRotation, failureMessage);
                Assert.Less(compareToTarget.y, thresholdRotation, failureMessage);
                Assert.Less(compareToTarget.z, thresholdRotation, failureMessage);
            }
            else // deal with exception cases:
            {
                var velocityMagnitude = math.length(velocityA.AngularVelocity);
                string failureMessage = $"{testName}: Angular velocity {velocityA.AngularVelocity} should be non-zero.";
                Assert.IsTrue(velocityMagnitude > 0.0f, failureMessage);
            }
        }

        // Check for positive and negative targets that are axis-aligned.
        private static readonly float3[] k_AVM_RotationAxis =
        {
            new float3(0f, 0f, -1f),
            new float3(0f, 0f, 1f),

            new float3(0f, -1f, 0f),
            new float3(0f, 1f, 0f),

            new float3(-1f, 0f, 0f),
            new float3(1f, 0f, 0f),
        };

        // Check for these pivot positions
        private static readonly float3[] k_AVM_PivotPosition =
        {
            float3.zero,
            new float3(0.5f, 0.5f, 0f),
            new float3(0.5f, 0f, 0.5f),
            new float3(0f, 0.5f, 0.5f),
        };

        // Test with gravity enabled and disabled.
        private static readonly bool[] k_AVM_UseGravity =
        {
            true,
            false
        };

        // Vary initial target velocity
        private static readonly bool[] k_AVM_InitAtTargetVelocity =
        {
            true, // warm start test with v = target velocity
            false // begin test with v = 0
        };

        // Since the value of spring frequency and damping ratio are so coupled, do not test the permutations separately
        // First index (x) = Spring Frequency, Second index (y) = Damping Ratio
        private static readonly float2[] k_AVM_SpringFrequencyAndDampingRatio =
        {
            new float2(Constraint.DefaultSpringFrequency, Constraint.DefaultDampingRatio),
            new float2(Constraint.DefaultSpringFrequency, 0.75f)
        };

        private static readonly int2[] k_AVM_Steps =
        {
            new int2(1, 4),
            new int2(4, 1)
        };

        // Tests that for a given axis of rotation, pivot position, gravity setting, initial velocity, and spring/damping
        // values that the angular velocity of bodyA reaches the target and that the orientation of movement is correct
        // Constants: target speed, the max impulse of the motor
        private static TestCaseData[] k_AVM_Permutations = MakePermutations();
        private static TestCaseData[] MakePermutations()
        {
            int count = 0;
            int length = k_AVM_Steps.Length * k_AVM_UseGravity.Length * k_AVM_RotationAxis.Length * k_AVM_PivotPosition.Length *
                k_AVM_InitAtTargetVelocity.Length * k_AVM_SpringFrequencyAndDampingRatio.Length;
            TestCaseData[] testList = new TestCaseData[length];

            foreach (int2 iStep in k_AVM_Steps)
            {
                foreach (bool iGravity in k_AVM_UseGravity)
                {
                    foreach (float3 iAxis in k_AVM_RotationAxis)
                    {
                        foreach (float3 iPivot in k_AVM_PivotPosition)
                        {
                            foreach (bool iTarget in k_AVM_InitAtTargetVelocity)
                            {
                                foreach (float2 iSpringAndDamping in k_AVM_SpringFrequencyAndDampingRatio)
                                {
                                    var name =
                                        $"[{iStep.x}/{iStep.y}] Test {count}: gravity:{iGravity}, axis:{iAxis}, pivot:{iPivot}, initAtTarget:{iTarget}, springF:{iSpringAndDamping.x}, dampingR:{iSpringAndDamping.y}";
                                    testList[count] = new TestCaseData(iStep.x, iStep.y, iAxis, iPivot, iGravity, iTarget,
                                        iSpringAndDamping.x, iSpringAndDamping.y).SetName(name);
                                    count++;
                                }
                            }
                        }
                    }
                }
            }

            return testList;
        }

        [TestCaseSource(nameof(k_AVM_Permutations))]
        public void OrientationTests_AVM(int numSubsteps, int numSolverIterations, float3 axisOfRotation, float3 pivotPosition, bool useGravity,
            bool initAtTargetVelocity, float springFrequency, float dampingRatio)
        {
            // Input variables
            var targetSpeed = 45.0f;  //target is an angular velocity in deg/s
            var maxImpulse = math.INFINITY;

            // Calculated variables
            var axis = math.normalize(axisOfRotation);
            var targetSpeedInRadians = math.radians(targetSpeed);
            var targetVelocityInRadians = targetSpeedInRadians * axis;

            // BodyB is resting above BodyA
            RigidTransform worldFromA = new RigidTransform(quaternion.identity, new float3(-0.5f, 5f, -4f));
            RigidTransform worldFromB = new RigidTransform(quaternion.identity, new float3(-0.5f, 6f, -4f));

            MotorTestUtility.SetupMotionVelocity(out MotionVelocity velocityA, out MotionVelocity velocityB);
            if (initAtTargetVelocity) velocityA.AngularVelocity = targetVelocityInRadians;
            MotorTestUtility.SetupMotionData(worldFromA, worldFromB, out MotionData motionA, out MotionData motionB);

            var jointFrameA = JacobianUtilities.CalculateDefaultBodyFramesForConnectedBody(
                worldFromA, worldFromB, pivotPosition, axis, out BodyFrame jointFrameB, true);

            Joint joint = MotorTestRunner.CreateTestMotor(
                PhysicsJoint.CreateAngularVelocityMotor(jointFrameA, jointFrameB, targetSpeedInRadians,
                    maxImpulse, springFrequency, dampingRatio));

            TestSimulateAngularVelocityMotor("Orientation Tests (AVM)", joint, numSubsteps, numSolverIterations,
                velocityA, velocityB, motionA, motionB, useGravity, targetVelocityInRadians, maxImpulse, false);
        }

        private static readonly TestCaseData[] k_AVM_maxImpulseTestCases =
        {
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), 1.0f, false, false).SetName("1/4: On-Axis maxImpulse=1, gravity off"),
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), 1.0f, false, false).SetName("4/1: On-Axis maxImpulse=1, gravity off"),
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), 1.0f, true, false).SetName("1/4: On-Axis maxImpulse=1, gravity on"),
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), 1.0f, true, false).SetName("4/1: On-Axis maxImpulse=1, gravity on"),

            new TestCaseData(1, 4, new float3(1f, 1f, 1f), new float3(0.5f, 0.5f, 0.5f), 1.0f, false, false).SetName("1/4: Off-Axis maxImpulse=1, gravity off"),
            new TestCaseData(4, 1, new float3(1f, 1f, 1f), new float3(0.5f, 0.5f, 0.5f), 1.0f, false, false).SetName("4/1: Off-Axis maxImpulse=1, gravity off"),
            new TestCaseData(1, 4, new float3(1f, 1f, 1f), new float3(0.5f, 0.5f, 0.5f), 1.0f, true, false).SetName("1/4: Off-Axis maxImpulse=1, gravity on"),
            new TestCaseData(4, 1, new float3(1f, 1f, 1f), new float3(0.5f, 0.5f, 0.5f), 1.0f, true, false).SetName("4/1: Off-Axis maxImpulse=1, gravity on"),

            // mass balanced about pivot, so doesn't move with no impulse
            new TestCaseData(1, 4, new float3(1f, 1f, 1f), new float3(0.5f, 0.5f, 0.5f), 0.0f, false, false).SetName("1/4: Off-Axis maxImpulse=0, gravity off"),
            new TestCaseData(4, 1, new float3(1f, 1f, 1f), new float3(0.5f, 0.5f, 0.5f), 0.0f, false, false).SetName("4/1: Off-Axis maxImpulse=0, gravity off"),
            new TestCaseData(1, 4, new float3(1f, 1f, 1f), new float3(0.5f, 0.5f, 0.5f), 0.0f, true, false).SetName("1/4: Off-Axis maxImpulse=0, gravity on"),
            new TestCaseData(4, 1, new float3(1f, 1f, 1f), new float3(0.5f, 0.5f, 0.5f), 0.0f, true, false).SetName("4/1: Off-Axis maxImpulse=0, gravity on"),

            // with no gravity, the motor shouldn't move with no impulse
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), 0.0f, false, false).SetName("1/4: On-Axis maxImpulse=0, gravity off"),
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), 0.0f, false, false).SetName("4/1: On-Axis maxImpulse=0, gravity off"),

            // these configurations will dangle off pivot due to gravity
            new TestCaseData(1, 4, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), 0.0f, true, true).SetName("1/4: On-Axis maxImpulse=0, gravity on"),
            new TestCaseData(4, 1, new float3(0f, 0f, -1f), new float3(0.5f, 0.5f, 0), 0.0f, true, true).SetName("4/1: On-Axis maxImpulse=0, gravity on")
        };

        [TestCaseSource(nameof(k_AVM_maxImpulseTestCases))]
        public void MaxImpulseTest_AVM(int numSubsteps, int numSolverIterations, float3 axisOfRotation, float3 pivotPosition,
            float maxImpulse, bool useGravity, bool isException)
        {
            // Input variables
            var targetSpeed = 45.0f;  //target is an angular velocity in deg/s

            // Calculated variables
            var axis = math.normalize(axisOfRotation);
            var targetSpeedInRadians = math.radians(targetSpeed);
            var targetVelocityInRadians = targetSpeedInRadians * axis;

            // BodyB is resting above BodyA
            RigidTransform worldFromA = new RigidTransform(quaternion.identity, new float3(-0.5f, 5f, -4f));
            RigidTransform worldFromB = new RigidTransform(quaternion.identity, new float3(-0.5f, 6f, -4f));

            MotorTestUtility.SetupMotionVelocity(out MotionVelocity velocityA, out MotionVelocity velocityB);
            MotorTestUtility.SetupMotionData(worldFromA, worldFromB, out MotionData motionA, out MotionData motionB);

            var jointFrameA = JacobianUtilities.CalculateDefaultBodyFramesForConnectedBody(
                worldFromA, worldFromB, pivotPosition, axis, out BodyFrame jointFrameB, true);

            Joint joint = MotorTestRunner.CreateTestMotor(
                PhysicsJoint.CreateAngularVelocityMotor(jointFrameA, jointFrameB, targetSpeedInRadians, maxImpulse));

            TestSimulateAngularVelocityMotor("Max Impulse Tests (AVM)", joint, numSubsteps, numSolverIterations,
                velocityA, velocityB, motionA, motionB, useGravity, targetVelocityInRadians, maxImpulse, isException);
        }
    }
}
