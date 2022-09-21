using Unity.Burst;
using Unity.Mathematics;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    // Solve data for a constraint that limits the linear distance between a pair of pivots in 1, 2, or 3 degrees of freedom
    [NoAlias]
    struct PositionMotorJacobian
    {
        // Pivot positions in motion space
        public float3 PivotAinA; //the anchor point of body with motor
        public float3 TargetInB; //the target position as seen by body B

        // Pivot distance limits
        public float MinDistance;
        public float MaxDistance;

        // Motion transforms before solving
        public RigidTransform WorldFromA;
        public RigidTransform WorldFromB;

        // If the constraint limits 1 DOF, this is the constrained axis.
        // If the constraint limits 2 DOF, this is the free axis.
        // If the constraint limits 3 DOF, this is unused and set to float3.zero
        public float3 AxisInB;

        // True if the jacobian limits one degree of freedom
        public bool Is1D;

        // Position error at the beginning of the step
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
            WorldFromA = motionA.WorldFromMotion;
            WorldFromB = motionB.WorldFromMotion;

            PivotAinA = aFromConstraint.Translation;
            TargetInB = constraint.Target + bFromConstraint.Translation;

            AxisInB = bFromConstraint.Rotation[constraint.ConstrainedAxis1D];
            Is1D = true;
            MinDistance = constraint.Min;
            MaxDistance = constraint.Max;
            Tau = tau;
            Damping = damping;

            // Calculate the current error
            InitialError = CalculateError(
                new MTransform(WorldFromA.rot, WorldFromA.pos),
                new MTransform(WorldFromB.rot, WorldFromB.pos),
                out float3 directionUnused);
        }

        private static void ApplyImpulse(float3 impulse, float3 ang0, float3 ang1, float3 ang2, ref MotionVelocity velocity)
        {
            velocity.ApplyLinearImpulse(impulse);
            velocity.ApplyAngularImpulse(impulse.x * ang0 + impulse.y * ang1 + impulse.z * ang2);
        }

        // Solve the Jacobian
        // Predict error at the end of the step and calculate the impulse to correct it
        public void Solve(ref MotionVelocity velocityA, ref MotionVelocity velocityB, Solver.StepInput stepInput)
        {
            // Predict the motions' transforms at the end of the step
            MTransform futureWorldFromA;
            MTransform futureWorldFromB;
            {
                quaternion dqA = Integrator.IntegrateAngularVelocity(velocityA.AngularVelocity, stepInput.Timestep);
                quaternion dqB = Integrator.IntegrateAngularVelocity(velocityB.AngularVelocity, stepInput.Timestep);
                quaternion futureOrientationA = math.normalize(math.mul(WorldFromA.rot, dqA));
                quaternion futureOrientationB = math.normalize(math.mul(WorldFromB.rot, dqB));
                futureWorldFromA = new MTransform(futureOrientationA, WorldFromA.pos + velocityA.LinearVelocity);
                futureWorldFromB = new MTransform(futureOrientationB, WorldFromB.pos + velocityB.LinearVelocity);
            }

            // Calculate the angulars
            CalculateAngulars(PivotAinA, futureWorldFromA.Rotation, out float3 angA0, out float3 angA1, out float3 angA2);
            CalculateAngulars(TargetInB, futureWorldFromB.Rotation, out float3 angB0, out float3 angB1, out float3 angB2);

            // Calculate effective mass
            float3 EffectiveMassDiag, EffectiveMassOffDiag;
            {
                // Calculate the inverse effective mass matrix
                float3 invEffectiveMassDiag = new float3(
                    JacobianUtilities.CalculateInvEffectiveMassDiag(angA0, velocityA.InverseInertia, velocityA.InverseMass,
                        angB0, velocityB.InverseInertia, velocityB.InverseMass),
                    JacobianUtilities.CalculateInvEffectiveMassDiag(angA1, velocityA.InverseInertia, velocityA.InverseMass,
                        angB1, velocityB.InverseInertia, velocityB.InverseMass),
                    JacobianUtilities.CalculateInvEffectiveMassDiag(angA2, velocityA.InverseInertia, velocityA.InverseMass,
                        angB2, velocityB.InverseInertia, velocityB.InverseMass));

                float3 invEffectiveMassOffDiag = new float3(
                    JacobianUtilities.CalculateInvEffectiveMassOffDiag(angA0, angA1, velocityA.InverseInertia, angB0, angB1, velocityB.InverseInertia),
                    JacobianUtilities.CalculateInvEffectiveMassOffDiag(angA0, angA2, velocityA.InverseInertia, angB0, angB2, velocityB.InverseInertia),
                    JacobianUtilities.CalculateInvEffectiveMassOffDiag(angA1, angA2, velocityA.InverseInertia, angB1, angB2, velocityB.InverseInertia));

                // Invert to get the effective mass matrix
                JacobianUtilities.InvertSymmetricMatrix(invEffectiveMassDiag, invEffectiveMassOffDiag, out EffectiveMassDiag, out EffectiveMassOffDiag);
            }

            // Find the difference between the future distance and the limit range, then apply tau and damping
            float futureDistanceError = CalculateError(futureWorldFromA, futureWorldFromB, out float3 futureDirection);

            // Calculate the impulse to correct the error
            float3 solveError = JacobianUtilities.CalculateCorrection(futureDistanceError, InitialError, Tau, Damping);
            float3x3 effectiveMass = JacobianUtilities.BuildSymmetricMatrix(EffectiveMassDiag, EffectiveMassOffDiag);

            float3 impulse = (futureDirection * math.mul(effectiveMass, solveError)) * stepInput.Timestep; //TODO: fix *timestep requirement

            // Apply the impulse
            ApplyImpulse(impulse, angA0, angA1, angA2, ref velocityA);
            ApplyImpulse(-impulse, angB0, angB1, angB2, ref velocityB);
        }

        #region Helpers

        private static void CalculateAngulars(float3 pivotInMotion, float3x3 worldFromMotionRotation, out float3 ang0, out float3 ang1, out float3 ang2)
        {
            // Jacobian directions are i, j, k
            // Angulars are pivotInMotion x (motionFromWorld * direction)
            float3x3 motionFromWorldRotation = math.transpose(worldFromMotionRotation);
            ang0 = math.cross(pivotInMotion, motionFromWorldRotation.c0);
            ang1 = math.cross(pivotInMotion, motionFromWorldRotation.c1);
            ang2 = math.cross(pivotInMotion, motionFromWorldRotation.c2);
        }

        private float CalculateError(MTransform worldFromA, MTransform worldFromB, out float3 direction)
        {
            // Find the direction from pivot A to B and the distance between them
            float3 pivotA = Mul(worldFromA, PivotAinA);
            float3 pivotB = Mul(worldFromB, TargetInB);
            float3 axis = math.mul(worldFromB.Rotation, AxisInB);
            direction = pivotB - pivotA;
            float dot = math.dot(direction, axis);

            // Project for lower-dimension joints
            float distance;
            if (Is1D)
            {
                // In 1D, distance is signed and measured along the axis
                distance = -dot;
                direction = -axis;
            }
            else
            {
                // In 2D / 3D, distance is nonnegative.  In 2D it is measured perpendicular to the axis.
                direction -= axis * dot;
                float futureDistanceSq = math.lengthsq(direction);
                float invFutureDistance = math.select(math.rsqrt(futureDistanceSq), 0.0f, futureDistanceSq == 0.0f);
                distance = futureDistanceSq * invFutureDistance;
                direction *= invFutureDistance;
            }

            // Find the difference between the future distance and the limit range
            return distance;
        }

        #endregion
    }
}
