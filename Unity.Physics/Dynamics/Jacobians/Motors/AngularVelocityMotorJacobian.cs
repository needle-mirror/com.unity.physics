using Unity.Burst;
using Unity.Mathematics;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // Solve data for a constraint that limits one degree of angular freedom
    [NoAlias]
    struct AngularVelocityMotorJacobian
    {
        // Limited axis in motion A space
        public float3 AxisInMotionA;
        public float3 Target;   //in rad/s

        // Index of the limited axis
        public int AxisIndex;

        // Relative orientation of the motions before solving
        public quaternion MotionBFromA;

        // Rotation to joint space from motion space
        public quaternion MotionAFromJoint;
        public quaternion MotionBFromJoint;

        // Maximum impulse that can be applied to the motor before it caps out (not a breaking impulse)
        public float MaxImpulseOfMotor;

        // Accumulated impulse applied over the number of solver iterations
        public float AccumulatedImpulse;

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
            AxisInMotionA = aFromConstraint.Rotation[AxisIndex];
            Target = AxisInMotionA * constraint.Target[AxisIndex]; //angular velocity as float3 in rad/s
            Tau = tau;
            Damping = damping;

            MaxImpulseOfMotor = math.abs(constraint.MaxImpulse.x); //using as magnitude, y&z components are unused
            AccumulatedImpulse = 0.0f;

            MotionBFromA = math.mul(math.inverse(motionB.WorldFromMotion.rot), motionA.WorldFromMotion.rot);
            MotionAFromJoint = new quaternion(aFromConstraint.Rotation);
            MotionBFromJoint = new quaternion(bFromConstraint.Rotation);

            // Calculate the current error: magnitude of target angular velocity, in rad/s
            InitialError = CalculateError();
        }

        // Solve the Jacobian
        public void Solve(ref MotionVelocity velocityA, ref MotionVelocity velocityB, Solver.StepInput stepInput)
        {
            // Predict the relative orientation at the end of the step
            quaternion futureMotionBFromA = JacobianUtilities.IntegrateOrientationBFromA(MotionBFromA,
                velocityA.AngularVelocity, velocityB.AngularVelocity, stepInput.Timestep);

            // Calculate the effective mass
            float3 axisInMotionB = math.mul(futureMotionBFromA, -AxisInMotionA); //projected onto axis in B
            float effectiveMass;
            {
                float invEffectiveMass = math.csum(AxisInMotionA * AxisInMotionA * velocityA.InverseInertia +
                    axisInMotionB * axisInMotionB * velocityB.InverseInertia);
                effectiveMass = math.select(1.0f / invEffectiveMass, 0.0f, invEffectiveMass == 0.0f);
            }

            var relativeVelocity = math.dot(velocityB.AngularVelocity, axisInMotionB) -
                math.dot(velocityA.AngularVelocity, AxisInMotionA);
            float rhs = -(math.dot(Target, axisInMotionB) - relativeVelocity) * Damping;

            float impulse = math.mul(effectiveMass, rhs);
            impulse = JacobianUtilities.CapImpulse(impulse, ref AccumulatedImpulse, MaxImpulseOfMotor);

            velocityA.ApplyAngularImpulse(impulse * AxisInMotionA);
            velocityB.ApplyAngularImpulse(impulse * axisInMotionB);
        }

        // Helper function
        private float CalculateError()
        {
            return math.length(Target); //magnitude of target velocity in rad/s
        }
    }
}
