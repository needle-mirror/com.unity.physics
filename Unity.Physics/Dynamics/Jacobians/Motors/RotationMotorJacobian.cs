using Unity.Burst;
using Unity.Mathematics;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // Solve data for a constraint that limits one degree of angular freedom
    [NoAlias]
    struct RotationMotorJacobian
    {
        // Limited axis in motion A space
        // TODO could calculate this from AxisIndex and MotionAFromJoint
        public float3 AxisInMotionA;
        public float3 Target;

        // Index of the limited axis
        public int AxisIndex;

        // Relative angle limits
        public float MinAngle;
        public float MaxAngle;

        // Relative orientation of the motions before solving
        public quaternion MotionBFromA;

        // Rotation to joint space from motion space
        public quaternion MotionAFromJoint;
        public quaternion MotionBFromJoint;

        // Error before solving
        public float InitialError;

        // Fraction of the position error to correct per step
        public float Tau;

        // Fraction of the velocity error to correct per step
        public float Damping;

        // Build the Jacobian
        public void Build(
            MTransform aFromConstraint, MTransform bFromConstraint,
            MotionData motionA, MotionData motionB,
            Constraint constraint, float tau, float damping)
        {
            AxisIndex = constraint.ConstrainedAxis1D;
            AxisInMotionA = math.normalize(aFromConstraint.Rotation[AxisIndex]);
            Target = constraint.Target; //is a float3 set of Euler angles in radians
            MinAngle = constraint.Min;
            MaxAngle = constraint.Max;
            Tau = tau;
            Damping = damping;
            MotionBFromA = math.mul(math.inverse(motionB.WorldFromMotion.rot), motionA.WorldFromMotion.rot);
            MotionAFromJoint = new quaternion(aFromConstraint.Rotation);
            MotionBFromJoint = new quaternion(bFromConstraint.Rotation);

            // Calculate the current error
            InitialError = CalculateError(MotionBFromA);
        }

        // Solve the Jacobian
        public void Solve(ref MotionVelocity velocityA, ref MotionVelocity velocityB, Solver.StepInput stepInput)
        {
            // Predict the relative orientation at the end of the step
            quaternion futureMotionBFromA = JacobianUtilities.IntegrateOrientationBFromA(MotionBFromA,
                velocityA.AngularVelocity, velocityB.AngularVelocity, stepInput.Timestep);

            // Calculate the effective mass
            float3 axisInMotionB = math.mul(futureMotionBFromA, -AxisInMotionA);
            float effectiveMass;
            {
                float invEffectiveMass = math.csum(AxisInMotionA * AxisInMotionA * velocityA.InverseInertia +
                    axisInMotionB * axisInMotionB * velocityB.InverseInertia);
                effectiveMass = math.select(1.0f / invEffectiveMass, 0.0f, invEffectiveMass == 0.0f);
            }

            // Calculate the error, adjust by tau and damping, and apply an impulse to correct it
            float futureError = CalculateError(futureMotionBFromA);
            float solveError = JacobianUtilities.CalculateCorrection(futureError, InitialError, Tau, Damping);

            float impulse = math.mul(effectiveMass, -solveError) * stepInput.InvTimestep;

            velocityA.ApplyAngularImpulse(impulse * AxisInMotionA);
            velocityB.ApplyAngularImpulse(impulse * axisInMotionB);
        }

        // Helper function
        private float CalculateError(quaternion motionBFromA)
        {
            float angle = 2.0f * math.acos(motionBFromA.value.w);
            quaternion targetAsQuaternion = quaternion.Euler(Target);
            float targetAngle = 2.0f * math.acos(targetAsQuaternion.value.w);
            return angle - targetAngle;
        }
    }
}
