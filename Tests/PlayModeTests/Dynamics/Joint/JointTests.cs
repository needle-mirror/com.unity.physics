using NUnit.Framework;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Entities;
using Random = Unity.Mathematics.Random;
using static Unity.Physics.Math;
using Unity.Physics.Tests.Utils;

namespace Unity.Physics.Tests.Joints
{
    /// <summary>
    /// These tests generate random motions and joints, simulate them for several steps, then verifies that the joint error is nearly zero.
    /// Passing test does not necessarily mean good-looking joint behavior or stability for systems of multiple constraints, but the test
    /// will catch a lot of basic mathematical errors in the solver.
    /// </summary>
    public class JointTests
    {
        //
        // Tiny simulation for a single body pair and joint, used by all of the tests
        //

        void applyGravity(ref MotionVelocity velocity, ref MotionData motion, float3 gravity, float timestep)
        {
            if (velocity.InverseInertiaAndMass.w > 0.0f)
            {
                velocity.LinearVelocity += gravity * timestep;
            }
        }

        void integrate(ref MotionVelocity velocity, ref MotionData motion, float timestep)
        {
            motion.WorldFromMotion.pos += velocity.LinearVelocity * timestep;
            Integrator.IntegrateOrientation(ref motion.WorldFromMotion.rot, velocity.AngularVelocity, timestep);
        }

        //
        // Random test data generation
        //

        float3 generateRandomCardinalAxis(ref Random rnd)
        {
            float3 axis = float3.zero;
            axis[rnd.NextInt(3)] = rnd.NextBool() ? 1 : -1;
            return axis;
        }

        RigidTransform generateRandomTransform(ref Random rnd)
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

            velocity = new MotionVelocity
            {
                LinearVelocity = rnd.NextBool() ? float3.zero : rnd.NextFloat3(-50.0f, 50.0f),
                AngularVelocity = rnd.NextBool() ? float3.zero : rnd.NextFloat3(-50.0f, 50.0f),
                InverseInertiaAndMass = (allowInfiniteMass && rnd.NextBool()) ? float4.zero : new float4(1.0f / inertia, rnd.NextFloat(1e-3f, 100.0f))
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

        delegate BlobAssetReference<JointData> GenerateJoint(ref Random rnd);

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
                JointData* jointData = (JointData*)generateJoint(ref rnd).GetUnsafePtr();

                // Generate random motions
                MotionVelocity velocityA, velocityB;
                MotionData motionA, motionB;
                generateRandomMotion(ref rnd, out velocityA, out motionA, true);
                generateRandomMotion(ref rnd, out velocityB, out motionB, math.all(velocityA.InverseInertiaAndMass != 0.0f));
                
                // Simulate the joint
                {
                    // Build input
                    const float timestep = 1.0f / 50.0f;
                    const int numIterations = 4;
                    const int numSteps = 15;
                    float3 gravity = new float3(0.0f, -9.81f, 0.0f);
                    Solver.StepInput stepInput = new Solver.StepInput
                    {
                        IsLastIteration = false,
                        InvNumSolverIterations = 1.0f / numIterations,
                        Timestep = timestep
                    };
                    BlockStream.Writer eventWriter = new BlockStream.Writer(); // no events expected

                    // Simulate
                    for (int iStep = 0; iStep < numSteps; iStep++)
                    {
                        // Build jacobians
                        BlockStream jacobians = new BlockStream(1, 0);
                        {
                            BlockStream.Writer jacobianWriter = jacobians;
                            jacobianWriter.BeginForEachIndex(0);
                            Solver.BuildJointJacobian(jointData, new BodyIndexPair(), velocityA, velocityB, motionA, motionB, timestep, numIterations, ref jacobianWriter);
                            jacobianWriter.EndForEachIndex();
                        }

                        // Before solving, apply gravity
                        applyGravity(ref velocityA, ref motionA, gravity, timestep);
                        applyGravity(ref velocityB, ref motionB, gravity, timestep);

                        // Solve the joint
                        for (int iIteration = 0; iIteration < numIterations; iIteration++)
                        {
                            stepInput.IsLastIteration = (iIteration == numIterations - 1);
                            BlockStream.Reader jacobianReader = jacobians;
                            JacobianIterator jacIterator = new JacobianIterator(jacobianReader, 0);
                            while (jacIterator.HasJacobiansLeft())
                            {
                                ref JacobianHeader header = ref jacIterator.ReadJacobianHeader();
                                header.Solve(ref velocityA, ref velocityB, stepInput, ref eventWriter, ref eventWriter);
                            }
                        }

                        // After solving, integrate motions
                        integrate(ref velocityA, ref motionA, timestep);
                        integrate(ref velocityB, ref motionB, timestep);

                        // Last step, check the joint error
                        if (iStep == numSteps - 1)
                        {
                            BlockStream.Reader jacobianReader = jacobians;
                            JacobianIterator jacIterator = new JacobianIterator(jacobianReader, 0);
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

        void generateRandomPivots(ref Random rnd, out float3 pivotA, out float3 pivotB)
        {
            pivotA = rnd.NextBool() ? float3.zero : rnd.NextFloat3(-1.0f, 1.0f);
            pivotB = rnd.NextBool() ? float3.zero : rnd.NextFloat3(-1.0f, 1.0f);
        }

        void generateRandomAxes(ref Random rnd, out float3 axisA, out float3 axisB)
        {
            axisA = rnd.NextInt(4) == 0 ? generateRandomCardinalAxis(ref rnd) : rnd.NextFloat3Direction();
            axisB = rnd.NextInt(4) == 0 ? generateRandomCardinalAxis(ref rnd) : rnd.NextFloat3Direction();
        }

        void generateRandomLimits(ref Random rnd, float minClosed, float maxClosed, out float min, out float max)
        {
            min = rnd.NextBool() ? float.MinValue : rnd.NextFloat(minClosed, maxClosed);
            max = rnd.NextBool() ? rnd.NextBool() ? float.MaxValue : min : rnd.NextFloat(min, maxClosed);
        }

        [Test]
        public unsafe void BallAndSocketTest()
        {
            RunJointTest("BallAndSocketTest", (ref Random rnd) =>
            {
                generateRandomPivots(ref rnd, out float3 pivotA, out float3 pivotB);
                return JointData.CreateBallAndSocket(pivotA, pivotB);
            });
        }

        [Test]
        public unsafe void StiffSpringTest()
        {
            RunJointTest("StiffSpringTest", (ref Random rnd) =>
            {
                generateRandomPivots(ref rnd, out float3 pivotA, out float3 pivotB);
                generateRandomLimits(ref rnd, 0.0f, 0.5f, out float minDistance, out float maxDistance);
                return JointData.CreateStiffSpring(pivotA, pivotB, minDistance, maxDistance);
            });
        }

        [Test]
        public unsafe void PrismaticTest()
        {
            RunJointTest("PrismaticTest", (ref Random rnd) =>
            {
                generateRandomPivots(ref rnd, out float3 pivotA, out float3 pivotB);
                generateRandomAxes(ref rnd, out float3 axisA, out float3 axisB);
                Math.CalculatePerpendicularNormalized(axisA, out float3 perpendicularA, out float3 unusedA);
                Math.CalculatePerpendicularNormalized(axisB, out float3 perpendicularB, out float3 unusedB);
                float minDistance = rnd.NextFloat(-0.5f, 0.5f);
                float maxDistance = rnd.NextBool() ? minDistance : rnd.NextFloat(minDistance, 0.5f); // note, can't use open limits because the accuracy can get too low as the pivots separate
                return JointData.CreatePrismatic(pivotA, pivotB, axisA, axisB, perpendicularA, perpendicularB, minDistance, maxDistance, 0.0f, 0.0f);
            });
        }

        [Test]
        public unsafe void HingeTest()
        {
            RunJointTest("HingeTest", (ref Random rnd) =>
            {
                generateRandomPivots(ref rnd, out float3 pivotA, out float3 pivotB);
                generateRandomAxes(ref rnd, out float3 axisA, out float3 axisB);
                return JointData.CreateHinge(pivotA, pivotB, axisA, axisB);
            });
        }

        [Test]
        public unsafe void LimitedHingeTest()
        {
            RunJointTest("LimitedHingeTest", (ref Random rnd) =>
            {
                generateRandomPivots(ref rnd, out float3 pivotA, out float3 pivotB);
                generateRandomAxes(ref rnd, out float3 axisA, out float3 axisB);
                Math.CalculatePerpendicularNormalized(axisA, out float3 perpendicularA, out float3 unusedA);
                Math.CalculatePerpendicularNormalized(axisB, out float3 perpendicularB, out float3 unusedB);
                generateRandomLimits(ref rnd, -(float)math.PI, (float)math.PI, out float minAngle, out float maxAngle);
                return JointData.CreateLimitedHinge(pivotA, pivotB, axisA, axisB, perpendicularA, perpendicularB, minAngle, maxAngle);
            });
        }

        // TODO - test CreateRagdoll(), if it stays.  Doesn't fit nicely because it produces two JointDatas.

        [Test]
        public unsafe void FixedTest()
        {
            RunJointTest("FixedTest", (ref Random rnd) =>
            {
                generateRandomPivots(ref rnd, out float3 pivotA, out float3 pivotB);
                quaternion orientationA = generateRandomTransform(ref rnd).rot;
                quaternion orientationB = generateRandomTransform(ref rnd).rot;
                return JointData.CreateFixed(pivotA, pivotB, orientationA, orientationB);
            });
        }
    }
}
