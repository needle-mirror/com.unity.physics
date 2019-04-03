using NUnit.Framework;
using Unity.Mathematics;
using static Unity.Physics.Math;
using Assert = UnityEngine.Assertions.Assert;

namespace Unity.Physics.Tests.Dynamics.Jacobians
{
    public class LinearLimitJacobianTests
    {
        [Test]
        public void BuildTest()
        {
            var jacobian = new LinearLimitJacobian();

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

            Assert.AreEqual(RigidTransform.identity, jacobian.WorldFromA);
            Assert.AreEqual(RigidTransform.identity, jacobian.WorldFromB);
            Assert.AreEqual(float3.zero, jacobian.PivotAinA);
            Assert.AreEqual(float3.zero, jacobian.PivotBinB);
            Assert.AreEqual(float3.zero, jacobian.AxisInB);
            Assert.IsFalse(jacobian.Is1D);
            Assert.AreEqual(0.0f, jacobian.MinDistance);
            Assert.AreEqual(0.0f, jacobian.MaxDistance);
            Assert.AreEqual(1.0f, jacobian.Tau);
            Assert.AreEqual(1.0f, jacobian.Damping);

            Assert.AreEqual(0.0f, jacobian.InitialError);
        }

        [Test]
        public void SolveTest()
        {
            var jacobian = new LinearLimitJacobian() { WorldFromA = RigidTransform.identity, WorldFromB = RigidTransform.identity };

            var velocityA = MotionVelocity.Zero;
            var velocityB = MotionVelocity.Zero;
            var timestep = 1.0f;

            jacobian.Solve(ref velocityA, ref velocityB, timestep);

            Assert.AreEqual(MotionVelocity.Zero, velocityA);
            Assert.AreEqual(MotionVelocity.Zero, velocityB);
        }
    }
}
