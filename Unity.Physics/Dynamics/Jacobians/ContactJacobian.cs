using Unity.Collections;
using Unity.Mathematics;

namespace Unity.Physics
{
    public struct ContactJacobianAngular
    {
        public float3 AngularA;
        public float3 AngularB;
        public float EffectiveMass;
        public float Impulse; // Accumulated impulse
    }

    public struct ContactJacAngAndVelToReachCp
    {
        public ContactJacobianAngular Jac;

        // Velocity needed to reach the contact plane in one frame,
        // both if approaching (negative) and depenetrating (positive)
        public float VelToReachCp;
    }

    public struct SurfaceVelocity
    {
        // 0 and 1 for linear, 2 for angular
        public float3 ExtraFrictionDv;
    }

    public struct MassFactors
    {
        // TODO: mark these internal, add separate properties for InvInertiaFactor and InvMassFactor instead?
        public float4 InvInertiaAndMassFactorA;
        public float4 InvInertiaAndMassFactorB;

        public static MassFactors Default => new MassFactors
        {
            InvInertiaAndMassFactorA = new float4(1.0f),
            InvInertiaAndMassFactorB = new float4(1.0f)
        };
    }

    public struct BaseContactJacobian
    {
        public int NumContacts;
        public float3 Normal;

        internal static float GetJacVelocity(float3 linear, ContactJacobianAngular jacAngular, MotionVelocity velocityA, MotionVelocity velocityB)
        {
            float3 temp = (velocityA.LinearVelocity - velocityB.LinearVelocity) * linear;
            temp += velocityA.AngularVelocity * jacAngular.AngularA;
            temp += velocityB.AngularVelocity * jacAngular.AngularB;
            return math.csum(temp);
        }
    }

    // A Jacobian representing a set of contact points that apply impulses
    public struct ContactJacobian
    {
        public BaseContactJacobian BaseJacobian;
        public float CoefficientOfFriction;
        public float CoefficientOfRestitution;

        // Linear friction jacobians.  Only store the angular part, linear part can be recalculated from BaseJacobian.Normal 
        public ContactJacobianAngular Friction0; // EffectiveMass stores friction effective mass matrix element (0, 0)
        public ContactJacobianAngular Friction1; // EffectiveMass stores friction effective mass matrix element (1, 1)

        // Angular friction about the contact normal, no linear part
        public ContactJacobianAngular AngularFriction; // EffectiveMass stores friction effective mass matrix element (2, 2)
        public float3 FrictionEffectiveMassOffDiag; // Effective mass matrix (0, 1), (0, 2), (1, 2) == (1, 0), (2, 0), (2, 1)

        // Solve the Jacobian
        public void Solve(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            Solver.StepInput stepInput, ref BlockStream.Writer collisionEventsWriter)
        {
            // Copy velocity data
            MotionVelocity tempVelocityA = velocityA;
            MotionVelocity tempVelocityB = velocityB;
            if (jacHeader.HasMassFactors)
            {
                MassFactors jacMod = jacHeader.AccessMassFactors();
                tempVelocityA.InverseInertiaAndMass *= jacMod.InvInertiaAndMassFactorA;
                tempVelocityB.InverseInertiaAndMass *= jacMod.InvInertiaAndMassFactorB;
            }

            // Calculate maximum impulse per sub step
            float maxImpulseToApply;
            if (jacHeader.HasMaxImpulse)
                maxImpulseToApply = jacHeader.AccessMaxImpulse() * stepInput.Timestep * stepInput.InvNumSolverIterations;
            else
                maxImpulseToApply = float.MaxValue;

            // Solve normal impulses
            float sumImpulses = 0.0f;
            float4 totalAccumulatedImpulses = float4.zero;
            bool forceCollisionEvent = false;
            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                // Solve velocity so that predicted contact distance is greater than or equal to zero
                float relativeVelocity = BaseContactJacobian.GetJacVelocity(BaseJacobian.Normal, jacAngular.Jac, tempVelocityA, tempVelocityB);
                float dv = jacAngular.VelToReachCp - relativeVelocity;

                float impulse = dv * jacAngular.Jac.EffectiveMass;
                bool clipped = impulse > maxImpulseToApply;
                impulse = math.min(impulse, maxImpulseToApply);
                float accumulatedImpulse = math.max(jacAngular.Jac.Impulse + impulse, 0.0f);
                if (accumulatedImpulse != jacAngular.Jac.Impulse)
                {
                    float deltaImpulse = accumulatedImpulse - jacAngular.Jac.Impulse;
                    ApplyImpulse(deltaImpulse, BaseJacobian.Normal, jacAngular.Jac, ref tempVelocityA, ref tempVelocityB);
                }

                jacAngular.Jac.Impulse = accumulatedImpulse;
                sumImpulses += accumulatedImpulse;

                // If there are more than 4 contacts, accumulate their impulses to the last contact point
                totalAccumulatedImpulses[math.min(j, 3)] += jacAngular.Jac.Impulse;

                // Force contact event even when no impulse is applied, but there is penetration.
                // Also force when impulse was clipped, even if clipped to 0.
                forceCollisionEvent |= jacAngular.VelToReachCp > 0.0f || clipped;
            }

            // Export collision event
            if (stepInput.IsLastIteration && (math.any(totalAccumulatedImpulses > 0.0f) || forceCollisionEvent) && jacHeader.HasColliderKeys)
            {
                collisionEventsWriter.Write(new LowLevel.CollisionEvent
                {
                    BodyIndices = jacHeader.BodyPair,
                    ColliderKeys = jacHeader.AccessColliderKeys(),
                    Normal = BaseJacobian.Normal,
                    AccumulatedImpulses = totalAccumulatedImpulses
                });
            }

            // Solve friction
            if (sumImpulses > 0.0f)
            {
                // Choose friction axes
                Math.CalculatePerpendicularNormalized(BaseJacobian.Normal, out float3 frictionDir0, out float3 frictionDir1);

                // Calculate impulses for full stop
                float3 imp;
                {
                    // Calculate the jacobian dot velocity for each of the friction jacobians
                    float3 extraFrictionDv = jacHeader.HasSurfaceVelocity ? jacHeader.AccessSurfaceVelocity().ExtraFrictionDv : float3.zero;
                    float dv0 = extraFrictionDv.x - BaseContactJacobian.GetJacVelocity(frictionDir0, Friction0, tempVelocityA, tempVelocityB);
                    float dv1 = extraFrictionDv.y - BaseContactJacobian.GetJacVelocity(frictionDir1, Friction1, tempVelocityA, tempVelocityB);
                    float dva = extraFrictionDv.z - math.csum(AngularFriction.AngularA * tempVelocityA.AngularVelocity + AngularFriction.AngularB * tempVelocityB.AngularVelocity);

                    // Reassemble the effective mass matrix
                    float3 effectiveMassDiag = new float3(Friction0.EffectiveMass, Friction1.EffectiveMass, AngularFriction.EffectiveMass);
                    float3x3 effectiveMass = JacobianUtilities.BuildSymmetricMatrix(effectiveMassDiag, FrictionEffectiveMassOffDiag);

                    // Calculate the impulse
                    imp = math.mul(effectiveMass, new float3(dv0, dv1, dva));
                }

                // Clip TODO.ma calculate some contact radius and use it to influence balance between linear and angular friction
                float maxImpulse = sumImpulses * CoefficientOfFriction * stepInput.InvNumSolverIterations;
                float frictionImpulseSquared = math.lengthsq(imp);
                imp *= math.min(1.0f, maxImpulse * math.rsqrt(frictionImpulseSquared));

                // Apply impulses
                ApplyImpulse(imp.x, frictionDir0, Friction0, ref tempVelocityA, ref tempVelocityB);
                ApplyImpulse(imp.y, frictionDir1, Friction1, ref tempVelocityA, ref tempVelocityB);

                tempVelocityA.ApplyAngularImpulse(imp.z * AngularFriction.AngularA);
                tempVelocityB.ApplyAngularImpulse(imp.z * AngularFriction.AngularB);

                // Accumulate them
                Friction0.Impulse += imp.x;
                Friction1.Impulse += imp.y;
                AngularFriction.Impulse += imp.z;
            }

            // Write back
            velocityA = tempVelocityA;
            velocityB = tempVelocityB;
        }

        // Helper function
        private static void ApplyImpulse(
            float impulse, float3 linear, ContactJacobianAngular jacAngular,
            ref MotionVelocity velocityA, ref MotionVelocity velocityB)
        {
            velocityA.ApplyLinearImpulse(impulse * linear);
            velocityB.ApplyLinearImpulse(-impulse * linear);
            velocityA.ApplyAngularImpulse(impulse * jacAngular.AngularA);
            velocityB.ApplyAngularImpulse(impulse * jacAngular.AngularB);
        }
    }

    // A Jacobian representing a set of contact points that export trigger events
    public struct TriggerJacobian
    {
        public BaseContactJacobian BaseJacobian;
        public ColliderKeyPair ColliderKeys;

        // Solve the Jacobian
        public void Solve(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB, Solver.StepInput stepInput,
            ref BlockStream.Writer triggerEventsWriter)
        {
            // Export trigger events only in last iteration
            if (!stepInput.IsLastIteration)
            {
                return;
            }

            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                // Solve velocity so that predicted contact distance is greater than or equal to zero
                float relativeVelocity = BaseContactJacobian.GetJacVelocity(BaseJacobian.Normal, jacAngular.Jac, velocityA, velocityB);
                float dv = jacAngular.VelToReachCp - relativeVelocity;
                float impulse = dv * jacAngular.Jac.EffectiveMass;
                float accumulatedImpulse = math.max(jacAngular.Jac.Impulse + impulse, 0.0f);
                if (accumulatedImpulse != jacAngular.Jac.Impulse || jacAngular.VelToReachCp > 0.0f)
                {
                    // Export trigger event only if impulse is applied, or objects are penetrating
                    triggerEventsWriter.Write(new LowLevel.TriggerEvent
                    {
                        BodyIndices = jacHeader.BodyPair,
                        ColliderKeys = ColliderKeys,
                    });

                    return;
                }
            }
        }
    }
}
