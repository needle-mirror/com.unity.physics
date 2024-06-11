using NUnit.Framework;
using Unity.Mathematics;
using Unity.Physics.Authoring;
using UnityEngine;

namespace Unity.Physics.Tests.Authoring
{
    [TestFixture]
    internal class JointBaking : BaseJointBaker<ConfigurableJoint>
    {
        public override void Bake(UnityEngine.ConfigurableJoint authoring) {}

        private static readonly TestCaseData[] k_SpringAndDampingCases =
        {
            new TestCaseData(987f, 44f, 1f, 5f, 0.7f).SetName("Spring & damping set"),
            new TestCaseData(987f, 44f, 2f, 3.54f, 0.5f).SetName("Spring & damping set, m=2"),

            new TestCaseData(0f, 44f, 1f, 74341.31f, 44f).SetName("No Spring"),
            new TestCaseData(0f, 44f, 2f, 74341.31f, 44f).SetName("No Spring, m=2"),

            new TestCaseData(5000f, 0f, 1f, 11.25f, 0.0f).SetName("No Damping"),
            new TestCaseData(5000f, 0f, 2f, 7.95f, 0.0f).SetName("No Damping, m=2"),

            new TestCaseData(0f, 0f, 1f, 74341.31f, 2530.126f).SetName("No Spring or Damping"),
            new TestCaseData(0f, 0f, 2f, 74341.31f, 2530.126f).SetName("No Spring or Damping, m=2"),

            new TestCaseData(2.181826218e11f, 2.363644856e9f, 1f, 74341.31f, 2530.126f)
                .SetName("Input to get defaults"),
            new TestCaseData(4.363652436e11f, 4.727289713e9f, 2f, 74341.31f, 2530.126f).SetName(
                "Input to get defaults, m=2"),
        };

        [TestCaseSource(nameof(k_SpringAndDampingCases))]
        public void VerifySpringAndDampingConversion(float inSpring, float inDamping, float mass, float checkSpring,
            float checkDamping)
        {
            ConvertSpringDamperSettings(inSpring, inDamping, mass,
                out float outSpringFrequency, out float outDampingRatio);

            var testThreshold = 0.01f;

            var compareToTarget = math.abs(outSpringFrequency - checkSpring);
            string failureMessage =
                $"Spring Conversation fails by {compareToTarget}. Expected {checkSpring}, got {outSpringFrequency}";
            Assert.LessOrEqual(compareToTarget, testThreshold, failureMessage);

            compareToTarget = math.abs(outDampingRatio - checkDamping);
            failureMessage =
                $"Damping Conversation fails by {compareToTarget}. Expected {checkDamping}, got {outDampingRatio}";
            Assert.LessOrEqual(compareToTarget, testThreshold, failureMessage);
        }

        private static readonly TestCaseData[] k_ConfigurableJointMotorCases =
        {
            // Input: accumulated impulse, new impulse, step count, expected impulse afterwards
            // Cases a velocity motor should be found:
            new TestCaseData(0f, 0f, 0f, 0.75f, 100f, false, true).SetName("VM: zero velocity, no spring"),

            new TestCaseData(0f, 1f, 0f, 0.75f, 100f, false, true).SetName("VM: no spring"),
            new TestCaseData(0f, 1f, 300f, 0f, 100f, false, true).SetName("VM: no damping"),
            new TestCaseData(0f, 1f, 0f, 0f, 100f, false, true).SetName("VM: no spring, no damping"),
            new TestCaseData(0f, 1f, 300f, 0.75f, 100f, false, true).SetName("VM: spring & damping set"),
            new TestCaseData(0f, 1f, 300f, 0.75f, 0f, false, false)
                .SetName("not VM: no force"), //TODO: should ID as velocity motor (poorly constructed)

            new TestCaseData(0f, -1f, 0f, 0.75f, 100f, false, true).SetName("VM: -target, no spring"),
            new TestCaseData(0f, -1f, 300f, 0f, 100f, false, true).SetName("VM: -target, no damping"),
            new TestCaseData(0f, -1f, 0f, 0f, 100f, false, true).SetName("VM: -target, no spring, no damping"),
            new TestCaseData(0f, -1f, 300f, 0.75f, 100f, false, true).SetName("VM: -target, spring & damping set"),
            new TestCaseData(0f, -1f, 300f, 0.75f, 0f, false, false)
                .SetName("not VM: -target, no force"), //TODO: should ID as velocity motor (poorly constructed)

            // Cases a position motor should be found:
            new TestCaseData(0f, 0f, 300f, 0f, 100f, true, false).SetName("PM: zero position, no damping"),
            new TestCaseData(0f, 0f, 300f, 0.75f, 100f, true, false).SetName("PM: zero position, spring & damping set"),

            new TestCaseData(1f, 0f, 0f, 0.75f, 100f, true, false).SetName("PM: no spring"),
            new TestCaseData(1f, 0f, 300f, 0f, 100f, true, false).SetName("PM: no damping"),
            new TestCaseData(1f, 0f, 0f, 0f, 100f, true, false).SetName("PM: no spring, no damping"),
            new TestCaseData(1f, 0f, 300f, 0.75f, 100f, true, false).SetName("PM: spring & damping set"),
            new TestCaseData(1f, 0f, 300f, 0.75f, 0f, false, false)
                .SetName("not PM: no force"), //TODO: should ID as position motor (poorly constructed)

            new TestCaseData(-1f, 0f, 0f, 0.75f, 100f, true, false).SetName("PM: -target, no spring"),
            new TestCaseData(-1f, 0f, 300f, 0f, 100f, true, false).SetName("PM: -target, no damping"),
            new TestCaseData(-1f, 0f, 0f, 0f, 100f, true, false).SetName("PM: -target, no spring, no damping"),
            new TestCaseData(-1f, 0f, 300f, 0.75f, 100f, true, false).SetName("PM: -target, spring & damping set"),
            new TestCaseData(-1f, 0f, 300f, 0.75f, 0f, false, false)
                .SetName("not PM: -target, no force"), //TODO: should ID as position motor (poorly constructed)

            // Cases: not a motor
            new TestCaseData(0f, 0f, 0f, 0f, 3.402823e+38f, false, false).SetName("Not a motor: default values"),
            new TestCaseData(0f, 0f, 0f, 0f, 100f, false, false).SetName("Not a motor: default values w force changed"),
            new TestCaseData(0f, 0f, 0f, 0f, 0f, false, false).SetName("Not a motor: all zero"),
            new TestCaseData(0f, 0f, 300f, 0.75f, 0f, false, false).SetName(
                "Not a motor: spring & damping set, no force"),
            new TestCaseData(0f, 0f, 0f, 0.75f, 0f, false, false).SetName("Not a motor: no spring, no force"),
            new TestCaseData(0f, 0f, 300f, 0f, 0f, false, false).SetName("Not a motor: no damping, no force"),
            new TestCaseData(1f, 1f, 300f, 0.75f, 100f, false, false).SetName("Not a motor: both targets set"),
        };

        [TestCaseSource(nameof(k_ConfigurableJointMotorCases))]
        public void IsThisConfigurableJointAMotor(float targetPosition, float targetVelocity, float spring,
            float damper,
            float force, bool expectedPM, bool expectedVM)
        {
            ConfigurableBaker.CheckPerAxis(targetPosition, targetVelocity, spring, damper, force, "name",
                out bool isPositionMotor, out bool isVelocityMotor);

            string failureMessage = $"Error: This should be a position-type motor";
            Assert.IsFalse(isPositionMotor != expectedPM, failureMessage);

            failureMessage = $"Error: This should be a velocity-type motor";
            Assert.IsFalse(isVelocityMotor != expectedVM, failureMessage);
        }
    }
}
