using NUnit.Framework;
using Unity.Mathematics;
using static Unity.Physics.Math;
using Assert = UnityEngine.Assertions.Assert;

namespace Unity.Physics.Tests.Dynamics.Jacobians
{
    public class AngularLimit3DJacobianTests
    {
        [Test]
        public void BuildTest()
        {
            var jacobian = new AngularLimit3DJacobian();

            var aFromConstraint = MTransform.Identity;
            var bFromConstraint = MTransform.Identity;
            var velocityA = MotionVelocity.Zero;
            var velocityB = MotionVelocity.Zero;
            var motionA = MotionData.Zero;
            var motionB = MotionData.Zero;
            var constraint = new Constraint() { ConstrainedAxes = new bool3(true, true, true) };
            var tau = 1.0f;
            var damping = 1.0f;

            jacobian.Build(aFromConstraint, bFromConstraint, velocityA, velocityB, motionA, motionB, constraint, tau, damping);

            Assert.AreEqual(0.0f, jacobian.MinAngle);
            Assert.AreEqual(0.0f, jacobian.MaxAngle);
        }

        [Test]
        public void SolveTest()
        {
            var jacobian = new AngularLimit3DJacobian()
            {
                BFromA = quaternion.identity,
                MotionAFromJoint = quaternion.identity,
                MotionBFromJoint = quaternion.identity
            };

            var velocityA = MotionVelocity.Zero;
            var velocityB = MotionVelocity.Zero;
            var timestep = 1.0f;

            jacobian.Solve(ref velocityA, ref velocityB, timestep);

            Assert.AreEqual(MotionVelocity.Zero, velocityA);
            Assert.AreEqual(MotionVelocity.Zero, velocityB);
        }
    }
}
