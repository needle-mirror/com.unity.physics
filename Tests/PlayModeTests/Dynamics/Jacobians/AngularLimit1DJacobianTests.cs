using NUnit.Framework;
using Unity.Mathematics;
using static Unity.Physics.Math;
using Assert = UnityEngine.Assertions.Assert;

namespace Unity.Physics.Tests.Dynamics.Jacobians
{
    public class AngularLimit1DJacobianTests
    {
        [Test]
        public void BuildTest()
        {
            var jacobian = new AngularLimit1DJacobian();

            var aFromConstraint = MTransform.Identity;
            var bFromConstraint = MTransform.Identity;
            var velocityA = MotionVelocity.Zero;
            var velocityB = MotionVelocity.Zero;
            var motionA = MotionData.Zero;
            var motionB = MotionData.Zero;
            var constraint = new Constraint() { ConstrainedAxes = new bool3(true, false, false) };
            var tau = 1.0f;
            var damping = 1.0f;

            jacobian.Build(aFromConstraint, bFromConstraint, velocityA, velocityB, motionA, motionB, constraint, tau, damping);

            Assert.AreEqual(new float3(1.0f, 0.0f, 0.0f), jacobian.AxisInMotionA);
            Assert.AreEqual(0.0f, jacobian.MinAngle);
            Assert.AreEqual(0.0f, jacobian.MaxAngle);
            Assert.AreEqual(1.0f, jacobian.Tau);
            Assert.AreEqual(1.0f, jacobian.Damping);
            Assert.AreEqual(quaternion.identity, jacobian.MotionBFromA);
            Assert.AreEqual(quaternion.identity, jacobian.MotionAFromJoint);
            Assert.AreEqual(quaternion.identity, jacobian.MotionBFromJoint);

            Assert.AreEqual(0.0f, jacobian.InitialError);
        }
    }
}
