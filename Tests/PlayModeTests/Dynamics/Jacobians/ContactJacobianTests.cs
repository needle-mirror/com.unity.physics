using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;
using static Unity.Physics.Math;
using Assert = UnityEngine.Assertions.Assert;

namespace Unity.Physics.Tests.Dynamics.Jacobians
{
    public class ContactJacobiansTests
    {
        [Test]
        public void SolveTest()
        {
            var jacobian = new ContactJacobian();

            var jacHeader = new JacobianHeader();
            var velocityA = MotionVelocity.Zero;
            var velocityB = MotionVelocity.Zero;
            var stepInput = new Solver.StepInput();
            var collisionEventsWriter = new BlockStream.Writer();

            jacobian.Solve(ref jacHeader, ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter);

            Assert.AreEqual(new JacobianHeader(), jacHeader);
            Assert.AreEqual(MotionVelocity.Zero, velocityA);
            Assert.AreEqual(MotionVelocity.Zero, velocityB);
            Assert.AreEqual(new BlockStream.Writer(), collisionEventsWriter);
        }
    }
}
