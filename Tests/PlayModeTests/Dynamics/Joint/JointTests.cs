using Unity.Collections;
using NUnit.Framework;
using Unity.Mathematics;
using Random = Unity.Mathematics.Random;
using static Unity.Physics.Math;

namespace Unity.Physics.Tests.Joints
{
    /// <summary>
    /// These tests generate random motions and joints, simulate them for several steps, then verifies that the joint error is nearly zero.
    /// Passing test does not necessarily mean good-looking joint behavior or stability for systems of multiple constraints, but the test
    /// will catch a lot of basic mathematical errors in the solver.
    /// </summary>
    class JointTests
    {
        //
        // Tiny simulation for a single body pair and joint, used by all of the tests
        //

        void applyGravity(ref MotionVelocity velocity, ref MotionData motion, float3 gravity, float timestep)
        {
            if (velocity.InverseMass > 0.0f)
            {
                velocity.LinearVelocity += gravity * timestep;
            }
        }

        static void integrate(ref MotionVelocity velocity, ref MotionData motion, float timestep)
        {
            Integrator.Integrate(ref motion.WorldFromMotion, velocity, timestep);
        }

        //
        // Random test data generation
        //

        static float3 generateRandomCardinalAxis(ref Random rnd)
        {
            float3 axis = float3.zero;
            axis[rnd.NextInt(3)] = rnd.NextBool() ? 1 : -1;
            return axis;
        }

        static RigidTransform generateRandomTransform(ref Random rnd)
        {
            // Random rotation: 1 in 4 are identity, 3 in 16 are 90 or 180 degrees about i j or k, the rest are uniform random
            quaternion rot = quaternion.identity;
            if (rnd.NextInt(4) > 0)
            {
                if (rnd.NextInt(4) > 0)
                {
                    rot = rnd.NextQuaternionRotation();
                }
                else
                {
                    float angle = rnd.NextBool() ? 90 : 180;
                    rot = quaternion.AxisAngle(generateRandomCardinalAxis(ref rnd), angle);
                }
            }

            return new RigidTransform()
            {
                pos = rnd.NextInt(4) == 0 ? float3.zero : rnd.NextFloat3(-1.0f, 1.0f),
                rot = rot
            };
        }

        void generateRandomMotion(ref Random rnd, out MotionVelocity velocity, out MotionData motion, bool allowInfiniteMass)
        {
            motion = new MotionData
            {
                WorldFromMotion = generateRandomTransform(ref rnd),
                BodyFromMotion = generateRandomTransform(ref rnd)
            };

            float3 inertia = rnd.NextFloat3(1e-3f, 100.0f);
            switch (rnd.NextInt(3))
            {
                case 0: // all values random
                    break;
                case 1: // two values the same
                    int index = rnd.NextInt(3);
                    inertia[(index + 1) % 2] = inertia[index];
                    break;
                case 2: // all values the same
                    inertia = inertia.zzz;
                    break;
            }

            float3 nextLinVel;
            if (rnd.NextBool())
            {
                nextLinVel = float3.zero;
            }
            else
            {
                nextLinVel = rnd.NextFloat3(-50.0f, 50.0f);
            }
            float3 nextAngVel;
            if (rnd.NextBool())
            {
                nextAngVel = float3.zero;
            }
            else
            {
                nextAngVel = rnd.NextFloat3(-50.0f, 50.0f);
            }
            float3 nextInertia;
            float nextMass;
            if (allowInfiniteMass && rnd.NextBool())
            {
                nextInertia = float3.zero;
                nextMass = 0.0f;
            }
            else
            {
                nextMass = rnd.NextFloat(1e-3f, 100.0f);
                nextInertia = 1.0f / inertia;
            }
            velocity = new MotionVelocity
            {
                LinearVelocity = nextLinVel,
                AngularVelocity = nextAngVel,
                InverseInertia = nextInertia,
                InverseMass = nextMass
            };
        }

        //
        // Helpers
        //

        static RigidTransform getWorldFromBody(MotionData motion)
        {
            return math.mul(motion.WorldFromMotion, math.inverse(motion.BodyFromMotion));
        }

        static float3 getBodyPointVelocity(MotionVelocity velocity, MotionData motion, float3 positionInBodySpace, out float angularLength)
        {
            float3 positionInMotion = math.transform(math.inverse(motion.BodyFromMotion), positionInBodySpace);
            float3 angularInMotion = math.cross(velocity.AngularVelocity, positionInMotion);
            angularLength = math.length(angularInMotion);
            float3 angularInWorld = math.rotate(motion.WorldFromMotion, angularInMotion);
            return angularInWorld + velocity.LinearVelocity;
        }

        //
        // Test runner
        //

        delegate Joint GenerateJoint(ref Random rnd);

        unsafe static void SolveSingleJoint(Joint jointData, int numIterations, float timestep,
            ref MotionVelocity velocityA, ref MotionVelocity velocityB, ref MotionData motionA, ref MotionData motionB, out NativeStream jacobiansOut)
        {
            var stepInput = new Solver.StepInput
            {
                IsLastIteration = false,
                InvNumSolverIterations = 1.0f / numIterations,
                Timestep = timestep,
                InvTimestep = timestep > 0.0f ? 1.0f / timestep : 0.0f
            };

            // Build jacobians
            jacobiansOut = new NativeStream(1, Allocator.Temp);
            {
                NativeStream.Writer jacobianWriter = jacobiansOut.AsWriter();
                jacobianWriter.BeginForEachIndex(0);
                Solver.BuildJointJacobian(jointData, velocityA, velocityB, motionA, motionB, timestep, numIterations, ref jacobianWriter);
                jacobianWriter.EndForEachIndex();
            }

            var eventWriter = new NativeStream.Writer(); // no events expected

            // Solve the joint
            for (int iIteration = 0; iIteration < numIterations; iIteration++)
            {
                stepInput.IsLastIteration = (iIteration == numIterations - 1);
                NativeStream.Reader jacobianReader = jacobiansOut.AsReader();
                var jacIterator = new JacobianIterator(jacobianReader, 0);
                while (jacIterator.HasJacobiansLeft())
                {
                    ref JacobianHeader header = ref jacIterator.ReadJacobianHeader();
                    header.Solve(ref velocityA, ref velocityB, stepInput, ref eventWriter, ref eventWriter,
                        false, Solver.MotionStabilizationInput.Default, Solver.MotionStabilizationInput.Default);
                }
            }

            // After solving, integrate motions
            integrate(ref velocityA, ref motionA, timestep);
            integrate(ref velocityB, ref motionB, timestep);
        }

        unsafe void RunJointTest(string testName, GenerateJoint generateJoint)
        {
            uint numTests = 1000;
            uint dbgTest = 2472156941;
            if (dbgTest > 0)
            {
                numTests = 1;
            }

            Random rnd = new Random(58297436);
            for (int iTest = 0; iTest < numTests; iTest++)
            {
                if (dbgTest > 0)
                {
                    rnd.state = dbgTest;
                }
                uint state = rnd.state;

                // Generate a random ball and socket joint
                Joint jointData = generateJoint(ref rnd);

                // Generate random motions
                MotionVelocity velocityA, velocityB;
                MotionData motionA, motionB;
                generateRandomMotion(ref rnd, out velocityA, out motionA, true);
                generateRandomMotion(ref rnd, out velocityB, out motionB, !velocityA.IsKinematic);

                // Simulate the joint
                {
                    // Build input
                    const float timestep = 1.0f / 50.0f;
                    const int numIterations = 4;
                    const int numSteps = 15;
                    float3 gravity = new float3(0.0f, -9.81f, 0.0f);

                    // Simulate
                    for (int iStep = 0; iStep < numSteps; iStep++)
                    {
                        // Before solving, apply gravity
                        applyGravity(ref velocityA, ref motionA, gravity, timestep);
                        applyGravity(ref velocityB, ref motionB, gravity, timestep);

                        // Solve and integrate
                        SolveSingleJoint(jointData, numIterations, timestep, ref velocityA, ref velocityB, ref motionA, ref motionB, out NativeStream jacobians);

                        // Last step, check the joint error
                        if (iStep == numSteps - 1)
                        {
                            NativeStream.Reader jacobianReader = jacobians.AsReader();
                            var jacIterator = new JacobianIterator(jacobianReader, 0);
                            string failureMessage = testName + " failed " + iTest + " (" + state + ")";
                            while (jacIterator.HasJacobiansLeft())
                            {
                                ref JacobianHeader header = ref jacIterator.ReadJacobianHeader();
                                switch (header.Type)
                                {
                                    case JacobianType.LinearLimit:
                                        Assert.Less(header.AccessBaseJacobian<LinearLimitJacobian>().InitialError, 1e-3f, failureMessage + ": LinearLimitJacobian");
                                        break;
                                    case JacobianType.AngularLimit1D:
                                        Assert.Less(header.AccessBaseJacobian<AngularLimit1DJacobian>().InitialError, 1e-2f, failureMessage + ": AngularLimit1DJacobian");
                                        break;
                                    case JacobianType.AngularLimit2D:
                                        Assert.Less(header.AccessBaseJacobian<AngularLimit2DJacobian>().InitialError, 1e-2f, failureMessage + ": AngularLimit2DJacobian");
                                        break;
                                    case JacobianType.AngularLimit3D:
                                        Assert.Less(header.AccessBaseJacobian<AngularLimit3DJacobian>().InitialError, 1e-2f, failureMessage + ": AngularLimit3DJacobian");
                                        break;
                                    default:
                                        Assert.Fail(failureMessage + ": unexpected jacobian type");
                                        break;
                                }
                            }
                        }

                        // Cleanup
                        jacobians.Dispose();
                    }
                }
            }
        }

        //
        // Tests
        //

        static void generateRandomPivots(ref Random rnd, out float3 pivotA, out float3 pivotB)
        {
            pivotA = rnd.NextBool() ? float3.zero : rnd.NextFloat3(-1.0f, 1.0f);
            pivotB = rnd.NextBool() ? float3.zero : rnd.NextFloat3(-1.0f, 1.0f);
        }

        static void generateRandomAxes(ref Random rnd, out float3 axisA, out float3 axisB)
        {
            axisA = rnd.NextInt(4) == 0 ? generateRandomCardinalAxis(ref rnd) : rnd.NextFloat3Direction();
            axisB = rnd.NextInt(4) == 0 ? generateRandomCardinalAxis(ref rnd) : rnd.NextFloat3Direction();
        }

        static void generateRandomLimits(ref Random rnd, float minClosed, float maxClosed, out float min, out float max)
        {
            min = rnd.NextBool() ? float.MinValue : rnd.NextFloat(minClosed, maxClosed);
            max = rnd.NextBool() ? rnd.NextBool() ? float.MaxValue : min : rnd.NextFloat(min, maxClosed);
        }

        Joint CreateTestJoint(PhysicsJoint joint) => new Joint
        {
            AFromJoint = joint.BodyAFromJoint.AsMTransform(),
            BFromJoint = joint.BodyBFromJoint.AsMTransform(),
            Constraints = joint.GetConstraints()
        };

        [Test]
        public unsafe void BallAndSocketTest()
        {
            RunJointTest("BallAndSocketTest", (ref Random rnd) =>
            {
                generateRandomPivots(ref rnd, out float3 pivotA, out float3 pivotB);
                return CreateTestJoint(PhysicsJoint.CreateBallAndSocket(pivotA, pivotB));
            });
        }

        [Test]
        public unsafe void StiffSpringTest()
        {
            RunJointTest("StiffSpringTest", (ref Random rnd) =>
            {
                generateRandomPivots(ref rnd, out float3 pivotA, out float3 pivotB);
                generateRandomLimits(ref rnd, 0.0f, 0.5f, out float minDistance, out float maxDistance);
                return CreateTestJoint(PhysicsJoint.CreateLimitedDistance(pivotA, pivotB, new FloatRange(minDistance, maxDistance)));
            });
        }

        [Test]
        public unsafe void PrismaticTest()
        {
            RunJointTest("PrismaticTest", (ref Random rnd) =>
            {
                var jointFrameA = new BodyFrame();
                var jointFrameB = new BodyFrame();
                generateRandomPivots(ref rnd, out jointFrameA.Position, out jointFrameB.Position);
                generateRandomAxes(ref rnd, out jointFrameA.Axis, out jointFrameB.Axis);
                Math.CalculatePerpendicularNormalized(jointFrameA.Axis, out jointFrameA.PerpendicularAxis, out _);
                Math.CalculatePerpendicularNormalized(jointFrameB.Axis, out jointFrameB.PerpendicularAxis, out _);
                var distance = new FloatRange { Min = rnd.NextFloat(-0.5f, 0.5f) };
                distance.Max = rnd.NextBool() ? distance.Min : rnd.NextFloat(distance.Min, 0.5f); // note, can't use open limits because the accuracy can get too low as the pivots separate
                return CreateTestJoint(PhysicsJoint.CreatePrismatic(jointFrameA, jointFrameB, distance));
            });
        }

        [Test]
        public unsafe void HingeTest()
        {
            RunJointTest("HingeTest", (ref Random rnd) =>
            {
                var jointFrameA = new BodyFrame();
                var jointFrameB = new BodyFrame();
                generateRandomPivots(ref rnd, out jointFrameA.Position, out jointFrameB.Position);
                generateRandomAxes(ref rnd, out jointFrameA.Axis, out jointFrameB.Axis);
                Math.CalculatePerpendicularNormalized(jointFrameA.Axis, out jointFrameA.PerpendicularAxis, out _);
                Math.CalculatePerpendicularNormalized(jointFrameB.Axis, out jointFrameB.PerpendicularAxis, out _);
                return CreateTestJoint(PhysicsJoint.CreateHinge(jointFrameA, jointFrameB));
            });
        }

        [Test]
        public unsafe void LimitedHingeTest()
        {
            RunJointTest("LimitedHingeTest", (ref Random rnd) =>
            {
                var jointFrameA = new BodyFrame();
                var jointFrameB = new BodyFrame();
                generateRandomPivots(ref rnd, out jointFrameA.Position, out jointFrameB.Position);
                generateRandomAxes(ref rnd, out jointFrameA.Axis, out jointFrameB.Axis);
                Math.CalculatePerpendicularNormalized(jointFrameA.Axis, out jointFrameA.PerpendicularAxis, out _);
                Math.CalculatePerpendicularNormalized(jointFrameB.Axis, out jointFrameB.PerpendicularAxis, out _);
                FloatRange limits;
                generateRandomLimits(ref rnd, -(float)math.PI, (float)math.PI, out limits.Min, out limits.Max);
                return CreateTestJoint(PhysicsJoint.CreateLimitedHinge(jointFrameA, jointFrameB, limits));
            });
        }

        // TODO - test CreateRagdoll(), if it stays.  Doesn't fit nicely because it produces two JointDatas.

        [Test]
        public unsafe void FixedTest()
        {
            RunJointTest("FixedTest", (ref Random rnd) =>
            {
                var jointFrameA = new RigidTransform();
                var jointFrameB = new RigidTransform();
                generateRandomPivots(ref rnd, out jointFrameA.pos, out jointFrameB.pos);
                jointFrameA.rot = generateRandomTransform(ref rnd).rot;
                jointFrameB.rot = generateRandomTransform(ref rnd).rot;
                return CreateTestJoint(PhysicsJoint.CreateFixed(jointFrameA, jointFrameB));
            });
        }

        [Test]
        public unsafe void LimitedDOFTest()
        {
            RunJointTest("LimitedDOFTest", (ref Random rnd) =>
            {
                var linearAxes = new bool3(false);
                var angularAxes = new bool3(false);
                for (int i = 0; i < 3; i++) linearAxes[rnd.NextInt(0, 2)] = !linearAxes[rnd.NextInt(0, 2)];
                for (int i = 0; i < 3; i++) angularAxes[rnd.NextInt(0, 2)] = !angularAxes[rnd.NextInt(0, 2)];
                return CreateTestJoint(PhysicsJoint.CreateLimitedDOF(generateRandomTransform(ref rnd), linearAxes, angularAxes));
            });
        }

        [Test]
        public unsafe void TwistTest()
        {
            // Check that the twist constraint works in each axis.
            // Set up a constraint between a fixed and dynamic body, give the dynamic body
            // angular velocity about the limited axis, and verify that it stops at the limit
            for (int i = 0; i < 3; i++) // For each axis
            {
                for (int j = 0; j < 2; j++) // Negative / positive limit
                {
                    float3 axis = float3.zero;
                    axis[i] = 1.0f;

                    MotionVelocity velocityA = new MotionVelocity
                    {
                        LinearVelocity = float3.zero,
                        AngularVelocity = (j + j - 1) * axis,
                        InverseInertia = new float3(1),
                        InverseMass = 1
                    };

                    MotionVelocity velocityB = new MotionVelocity
                    {
                        LinearVelocity = float3.zero,
                        AngularVelocity = float3.zero,
                        InverseInertia = float3.zero,
                        InverseMass = 0.0f
                    };

                    MotionData motionA = new MotionData
                    {
                        WorldFromMotion = RigidTransform.identity,
                        BodyFromMotion = RigidTransform.identity
                    };

                    MotionData motionB = motionA;

                    const float angle = 0.5f;
                    float minLimit = (j - 1) * angle;
                    float maxLimit = j * angle;

                    var jointData = new Joint
                    {
                        AFromJoint = MTransform.Identity,
                        BFromJoint = MTransform.Identity
                    };
                    jointData.Constraints.Add(Constraint.Twist(i, new FloatRange(minLimit, maxLimit)));
                    SolveSingleJoint(jointData, 4, 1.0f, ref velocityA, ref velocityB, ref motionA, ref motionB, out NativeStream jacobians);

                    quaternion expectedOrientation = quaternion.AxisAngle(axis, minLimit + maxLimit);
                    Utils.TestUtils.AreEqual(expectedOrientation, motionA.WorldFromMotion.rot, 1e-3f);
                    jacobians.Dispose();
                }
            }
        }

        [Test]
        public unsafe void ZeroDimensionTest()
        {
            RunJointTest("LimitedDOFTestZeroDimension", (ref Random rnd) =>
            {
                // Create a joint with 2 constraints that have 0 dimensions
                var noAxes = new bool3(false);
                return CreateTestJoint(PhysicsJoint.CreateLimitedDOF(generateRandomTransform(ref rnd), noAxes, noAxes));
            });
        }
    }
}
