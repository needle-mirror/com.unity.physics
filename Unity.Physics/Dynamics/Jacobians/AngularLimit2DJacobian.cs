using Unity.Mathematics;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // Solve data for a constraint that limits two degrees of angular freedom
    public struct AngularLimit2DJacobian
    {
        // Free axes in motion space
        public float3 AxisAinA;
        public float3 AxisBinB;

        // Relative angle limits
        public float MinAngle;
        public float MaxAngle;

        // Relative orientation before solving
        public quaternion BFromA;

        // Error before solving
        public float InitialError;

        // Fraction of the position error to correct per step
        public float Tau;

        // Fraction of the velocity error to correct per step
        public float Damping;

        // Build the Jacobian
        public void Build(
            MTransform aFromConstraint, MTransform bFromConstraint,
            MotionVelocity velocityA, MotionVelocity velocityB,
            MotionData motionA, MotionData motionB,
            Constraint constraint, float tau, float damping)
        {
            this = default(AngularLimit2DJacobian);
            
            // Copy the constraint data
            int freeIndex = constraint.FreeAxis2D;
            AxisAinA = aFromConstraint.Rotation[freeIndex];
            AxisBinB = bFromConstraint.Rotation[freeIndex];
            MinAngle = constraint.Min;
            MaxAngle = constraint.Max;
            Tau = tau;
            Damping = damping;
            BFromA = math.mul(math.inverse(motionB.WorldFromMotion.rot), motionA.WorldFromMotion.rot);

            // Calculate the initial error
            {
                float3 axisAinB = math.mul(BFromA, AxisAinA);
                float sinAngle = math.length(math.cross(axisAinB, AxisBinB));
                float cosAngle = math.dot(axisAinB, AxisBinB);
                float angle = math.atan2(sinAngle, cosAngle);
                InitialError = JacobianUtilities.CalculateError(angle, MinAngle, MaxAngle);
            }
        }

        // Solve the Jacobian
        public void Solve(ref MotionVelocity velocityA, ref MotionVelocity velocityB, float timestep)
        {
            // Predict the relative orientation at the end of the step
            quaternion futureBFromA = JacobianUtilities.IntegrateOrientationBFromA(BFromA, velocityA.AngularVelocity, velocityB.AngularVelocity, timestep);

            // Calculate the jacobian axis and angle
            float3 axisAinB = math.mul(futureBFromA, AxisAinA);
            float3 jacB0 = math.cross(axisAinB, AxisBinB);
            float3 jacA0 = math.mul(math.inverse(futureBFromA), -jacB0);
            float jacLengthSq = math.lengthsq(jacB0);
            float invJacLength = Math.RSqrtSafe(jacLengthSq);
            float futureAngle;
            {
                float sinAngle = jacLengthSq * invJacLength;
                float cosAngle = math.dot(axisAinB, AxisBinB);
                futureAngle = math.atan2(sinAngle, cosAngle);
            }

            // Choose a second jacobian axis perpendicular to A
            float3 jacB1 = math.cross(jacB0, axisAinB);
            float3 jacA1 = math.mul(math.inverse(futureBFromA), -jacB1);

            // Calculate effective mass
            float2 effectiveMass; // First column of the 2x2 matrix, we don't need the second column because the second component of error is zero
            {
                // Calculate the inverse effective mass matrix, then invert it
                float invEffMassDiag0 = math.csum(jacA0 * jacA0 * velocityA.InverseInertiaAndMass.xyz + jacB0 * jacB0 * velocityB.InverseInertiaAndMass.xyz);
                float invEffMassDiag1 = math.csum(jacA1 * jacA1 * velocityA.InverseInertiaAndMass.xyz + jacB1 * jacB1 * velocityB.InverseInertiaAndMass.xyz);
                float invEffMassOffDiag = math.csum(jacA0 * jacA1 * velocityA.InverseInertiaAndMass.xyz + jacB0 * jacB1 * velocityB.InverseInertiaAndMass.xyz);
                float det = invEffMassDiag0 * invEffMassDiag1 - invEffMassOffDiag * invEffMassOffDiag;
                float invDet = math.select(jacLengthSq / det, 0.0f, det == 0.0f); // scale by jacLengthSq because the jacs were not normalized
                effectiveMass = invDet * new float2(invEffMassDiag1, -invEffMassOffDiag);
            }

            // Normalize the jacobians
            jacA0 *= invJacLength;
            jacB0 *= invJacLength;
            jacA1 *= invJacLength;
            jacB1 *= invJacLength;

            // Calculate the error, adjust by tau and damping, and apply an impulse to correct it
            float futureError = JacobianUtilities.CalculateError(futureAngle, MinAngle, MaxAngle);
            float solveError = JacobianUtilities.CalculateCorrection(futureError, InitialError, Tau, Damping);
            float2 impulse = -effectiveMass * solveError * (1.0f / timestep);
            velocityA.ApplyAngularImpulse(impulse.x * jacA0 + impulse.y * jacA1);
            velocityB.ApplyAngularImpulse(impulse.x * jacB0 + impulse.y * jacB1);
        }
    }
}
