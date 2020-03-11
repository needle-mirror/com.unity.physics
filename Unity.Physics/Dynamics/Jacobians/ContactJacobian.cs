using System;
using System.ComponentModel;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
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

    public struct ContactJacAngAndVelToReachCp  // TODO: better name
    {
        public ContactJacobianAngular Jac;

        // Velocity needed to reach the contact plane in one frame,
        // both if approaching (negative) and depenetrating (positive)
        public float VelToReachCp;
    }

    public struct SurfaceVelocity
    {
        // Velocities between the two contacting objects
        public float3 LinearVelocity;
        public float3 AngularVelocity;
    }

    public struct MassFactors
    {
        public float3 InverseInertiaFactorA;
        public float InverseMassFactorA;
        public float3 InverseInertiaFactorB;
        public float InverseMassFactorB;

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("InvInertiaAndMassFactorA has been deprecated. Use the individual InverseInertiaFactorA and InverseMassFactorA members instead. (RemovedAfter 2020-05-15)")]
        public float4 InvInertiaAndMassFactorA
        {
            get => new float4(InverseInertiaFactorA, InverseMassFactorA);
            set
            {
                InverseInertiaFactorA = value.xyz;
                InverseMassFactorA = value.w;
            }
        }

        [EditorBrowsable(EditorBrowsableState.Never)]
        [Obsolete("InvInertiaAndMassFactorB has been deprecated. Use the individual InverseInertiaFactorB and InverseMassFactorB members instead. (RemovedAfter 2020-05-15)")]
        public float4 InvInertiaAndMassFactorB
        {
            get => new float4(InverseInertiaFactorB, InverseMassFactorB);
            set
            {
                InverseInertiaFactorB = value.xyz;
                InverseMassFactorB = value.w;
            }
        }

        public static MassFactors Default => new MassFactors
        {
            InverseInertiaFactorA = new float3(1.0f),
            InverseMassFactorA = 1.0f,
            InverseInertiaFactorB = new float3(1.0f),
            InverseMassFactorB = 1.0f
        };
    }

    struct BaseContactJacobian
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
    [NoAlias]
    struct ContactJacobian
    {
        public BaseContactJacobian BaseJacobian;

        // Linear friction jacobians.  Only store the angular part, linear part can be recalculated from BaseJacobian.Normal 
        public ContactJacobianAngular Friction0; // EffectiveMass stores friction effective mass matrix element (0, 0)
        public ContactJacobianAngular Friction1; // EffectiveMass stores friction effective mass matrix element (1, 1)

        // Angular friction about the contact normal, no linear part
        public ContactJacobianAngular AngularFriction; // EffectiveMass stores friction effective mass matrix element (2, 2)
        public float3 FrictionEffectiveMassOffDiag; // Effective mass matrix (0, 1), (0, 2), (1, 2) == (1, 0), (2, 0), (2, 1)

        public float CoefficientOfFriction;

        // Generic solve method that dispatches to specific ones
        public void Solve(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter)
        {
            bool bothBodiesWithInfInertiaAndMass = velocityA.HasInfiniteInertiaAndMass && velocityB.HasInfiniteInertiaAndMass;
            if (bothBodiesWithInfInertiaAndMass)
            {
                SolveInfMassPair(ref jacHeader, velocityA, velocityB, stepInput, ref collisionEventsWriter);
            }
            else
            {
                SolveContact(ref jacHeader, ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter);
            }
        }

        // Solve the Jacobian
        public void SolveContact(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter)
        {
            // Copy velocity data
            MotionVelocity tempVelocityA = velocityA;
            MotionVelocity tempVelocityB = velocityB;
            if (jacHeader.HasMassFactors)
            {
                MassFactors jacMod = jacHeader.AccessMassFactors();
                tempVelocityA.InverseInertia *= jacMod.InverseInertiaFactorA;
                tempVelocityA.InverseMass *= jacMod.InverseMassFactorA;
                tempVelocityB.InverseInertia *= jacMod.InverseInertiaFactorB;
                tempVelocityB.InverseMass *= jacMod.InverseMassFactorB;
            }

            // Solve normal impulses
            float sumImpulses = 0.0f;
            float totalAccumulatedImpulse = 0.0f;
            bool forceCollisionEvent = false;
            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                // Solve velocity so that predicted contact distance is greater than or equal to zero
                float relativeVelocity = BaseContactJacobian.GetJacVelocity(BaseJacobian.Normal, jacAngular.Jac, tempVelocityA, tempVelocityB);
                float dv = jacAngular.VelToReachCp - relativeVelocity;

                float impulse = dv * jacAngular.Jac.EffectiveMass;
                float accumulatedImpulse = math.max(jacAngular.Jac.Impulse + impulse, 0.0f);
                if (accumulatedImpulse != jacAngular.Jac.Impulse)
                {
                    float deltaImpulse = accumulatedImpulse - jacAngular.Jac.Impulse;
                    ApplyImpulse(deltaImpulse, BaseJacobian.Normal, jacAngular.Jac, ref tempVelocityA, ref tempVelocityB);
                }

                jacAngular.Jac.Impulse = accumulatedImpulse;
                sumImpulses += accumulatedImpulse;
                totalAccumulatedImpulse += jacAngular.Jac.Impulse;

                // Force contact event even when no impulse is applied, but there is penetration.
                forceCollisionEvent |= jacAngular.VelToReachCp > 0.0f;
            }

            // Export collision event
            if (stepInput.IsLastIteration && (totalAccumulatedImpulse > 0.0f || forceCollisionEvent) && jacHeader.HasContactManifold)
            {
                ExportCollisionEvent(totalAccumulatedImpulse, ref jacHeader, ref collisionEventsWriter);
            }

            // Solve friction
            if (sumImpulses > 0.0f)
            {
                // Choose friction axes
                Math.CalculatePerpendicularNormalized(BaseJacobian.Normal, out float3 frictionDir0, out float3 frictionDir1);

                // Calculate impulses for full stop
                float3 imp;
                {
                    float3 extraFrictionDv = float3.zero;
                    if (jacHeader.HasSurfaceVelocity)
                    {
                        var surfVel = jacHeader.AccessSurfaceVelocity();

                        Math.CalculatePerpendicularNormalized(BaseJacobian.Normal, out float3 dir0, out float3 dir1);
                        float linVel0 = math.dot(surfVel.LinearVelocity, dir0);
                        float linVel1 = math.dot(surfVel.LinearVelocity, dir1);

                        float angVelProj = math.dot(surfVel.AngularVelocity, BaseJacobian.Normal);
                        extraFrictionDv = new float3(linVel0, linVel1, angVelProj);
                    }

                    // Calculate the jacobian dot velocity for each of the friction jacobians
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

        // Solve the infinite mass pair Jacobian
        public void SolveInfMassPair(
            ref JacobianHeader jacHeader, MotionVelocity velocityA, MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter)
        {
            // Infinite mass pairs are only interested in collision events,
            // so only last iteration is performed in that case
            if (!stepInput.IsLastIteration)
            {
                return;
            }

            // Calculate normal impulses and fire collision event
            // if at least one contact point would have an impulse applied
            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                float relativeVelocity = BaseContactJacobian.GetJacVelocity(BaseJacobian.Normal, jacAngular.Jac, velocityA, velocityB);
                float dv = jacAngular.VelToReachCp - relativeVelocity;
                if (jacAngular.VelToReachCp > 0 || dv > 0)
                {
                    // Export collision event only if impulse would be applied, or objects are penetrating
                    ExportCollisionEvent(0.0f, ref jacHeader, ref collisionEventsWriter);
 
                    return;
                }
            }
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

        private unsafe void ExportCollisionEvent(float totalAccumulatedImpulse, [NoAlias] ref JacobianHeader jacHeader,
            [NoAlias] ref NativeStream.Writer collisionEventsWriter)
        {
            // Write size before every event
            int collisionEventSize = CollisionEventData.CalculateSize(BaseJacobian.NumContacts);
            collisionEventsWriter.Write(collisionEventSize);

            // Allocate all necessary data for this event
            byte* eventPtr = collisionEventsWriter.Allocate(collisionEventSize);

            // Fill up event data
            ref CollisionEventData collisionEvent = ref UnsafeUtilityEx.AsRef<CollisionEventData>(eventPtr);
            collisionEvent.BodyIndices = jacHeader.BodyPair;
            collisionEvent.ColliderKeys = jacHeader.AccessColliderKeys();
            collisionEvent.Normal = BaseJacobian.Normal;
            collisionEvent.SolverImpulse = totalAccumulatedImpulse;
            collisionEvent.NumNarrowPhaseContactPoints = BaseJacobian.NumContacts;
            for (int i = 0; i < BaseJacobian.NumContacts; i++)
            {
                collisionEvent.AccessContactPoint(i) = jacHeader.AccessContactPoint(i);
            }
        }
    }

    // A Jacobian representing a set of contact points that export trigger events
    [NoAlias]
    struct TriggerJacobian
    {
        public BaseContactJacobian BaseJacobian;
        public ColliderKeyPair ColliderKeys;

        // Solve the Jacobian
        public void Solve(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB, Solver.StepInput stepInput,
            ref NativeStream.Writer triggerEventsWriter)
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
                if (jacAngular.VelToReachCp > 0 || dv > 0)
                {
                    // Export trigger event only if impulse would be applied, or objects are penetrating
                    triggerEventsWriter.Write(new TriggerEventData
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
