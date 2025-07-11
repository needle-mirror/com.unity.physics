using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using static Unity.Physics.Math;
using Assert = UnityEngine.Assertions.Assert;

namespace Unity.Physics
{
    /// <summary>   A container for use with contact jacobians that holds data specific to a single contact point in
    /// a ContactJacobian. </summary>
    public struct ContactJacobianAngular
    {
        /// <summary>   The angular component of Body A. </summary>
        public float3 AngularA;
        /// <summary>   The angular component of Body B. </summary>
        public float3 AngularB;
        /// <summary>   The effective mass. </summary>
        public float EffectiveMass;
        /// <summary>   Accumulated impulse for a single contact point over a single substep, (or numSolverIterations). </summary>
        public float Impulse;
    }

    /// <summary>   A contact jacobian angle and velocity to reach the contact plane. </summary>
    public struct ContactJacAngAndVelToReachCp  // TODO: better name
    {
        /// <summary>   The jacobian. </summary>
        public ContactJacobianAngular Jac;

        /// <summary>
        /// Velocity needed to reach the contact plane in one frame, both if approaching (negative) and
        /// depenetrating (positive)
        /// </summary>
        public float VelToReachCp;

        /// <summary>
        /// The distance to the contact point. This point is initially identified by the narrowphase and is initialized
        /// during the Jacobian Build stage. During the solve and integrate stage, this value is updated each substep.
        /// </summary>
        public float ContactDistance;

        /// <summary>
        /// A flag used to indicate if an impulse should be applied to the contact point. This flag is set to true by
        /// default and is only false if: the substepCount > 1, the coefficient of restitution is greater than 0 and
        /// CalculateRestitution returns false. Needed for brief contacts when substepping so that impulse isn't always
        /// applied during Solve.
        /// </summary>
        public bool ApplyImpulse;
    }

    /// <summary>   A velocity between the two contacting objects. </summary>
    public struct SurfaceVelocity
    {
        /// <summary>   Linear velocity between the two contacting objects. </summary>
        public float3 LinearVelocity;
        /// <summary>   Angular velocity between the two contacting objects. </summary>
        public float3 AngularVelocity;
    }

    /// <summary>   The mass factors. </summary>
    public struct MassFactors
    {
        /// <summary>   The inverse inertia factor a. </summary>
        public float3 InverseInertiaFactorA;
        /// <summary>   The inverse mass factor a. </summary>
        public float InverseMassFactorA;
        /// <summary>   The inverse inertia factor b. </summary>
        public float3 InverseInertiaFactorB;
        /// <summary>   The inverse mass factor b. </summary>
        public float InverseMassFactorB;

        /// <summary>   Gets the default. </summary>
        ///
        /// <value> The default. </value>
        public static MassFactors Default => new MassFactors
        {
            InverseInertiaFactorA = new float3(1.0f),
            InverseMassFactorA = 1.0f,
            InverseInertiaFactorB = new float3(1.0f),
            InverseMassFactorB = 1.0f
        };
    }

    /// <summary>   A base contact jacobian. </summary>
    struct BaseContactJacobian
    {
        /// <summary>   Number of contacts. </summary>
        public int NumContacts;
        /// <summary>   The normal. </summary>
        public float3 Normal;

        /// <summary>
        /// Returns the relative velocity between the dispatch pairs involved in the contact that are aligned with the
        /// contact normal.
        /// </summary>
        /// <param name="linear">  The contact normal. </param>
        /// <param name="jacAngular"></param>
        /// <param name="linVelA">  The linear velocity of BodyA. </param>
        /// <param name="angVelA">  The angular velocity of BodyA. </param>
        /// <param name="linVelB">  The linear velocity of BodyB. </param>
        /// <param name="angVelB">  The angular velocity of BodyB. </param>
        /// <returns></returns>
        internal static float GetJacVelocity(float3 linear, ContactJacobianAngular jacAngular,
            float3 linVelA, float3 angVelA, float3 linVelB, float3 angVelB)
        {
            float3 temp = (linVelA - linVelB) * linear;
            temp += angVelA * jacAngular.AngularA;
            temp += angVelB * jacAngular.AngularB;
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

        public float3 CenterA;
        public float3 CenterB;

        public float CoefficientOfFriction;
        public float CoefficientOfRestitution;
        public float SumImpulsesOverSubsteps;
        public float SumImpulsesOverSolverIterations;

        internal static void BuildIndividualContactJacobians(
            int contactPointIndex,
            float3 normal,
            MTransform worldFromA,
            MTransform worldFromB,
            float invTimestep,
            int numSubsteps,
            MotionVelocity velocityA,
            MotionVelocity velocityB,
            float sumInvMass,
            float maxDepenetrationVelocity,
            ref JacobianHeader jacobianHeader,
            ref float3 centerA,
            ref float3 centerB,
            ref NativeStream.Reader contactReader)
        {
            ref ContactJacAngAndVelToReachCp jacAngular = ref jacobianHeader.AccessAngularJacobian(contactPointIndex);
            ContactPoint contact = contactReader.Read<ContactPoint>();
            float3 pointOnB = contact.Position;
            float3 pointOnA = contact.Position + normal * contact.Distance;
            float3 armA = pointOnA - worldFromA.Translation;
            float3 armB = pointOnB - worldFromB.Translation;
            CalculateJacobian(worldFromA, worldFromB, normal, armA, armB,
                velocityA.InverseInertia, velocityB.InverseInertia, sumInvMass,
                out jacAngular.Jac.AngularA, out jacAngular.Jac.AngularB, out float invEffectiveMass);

            jacAngular.Jac.EffectiveMass = 1.0f / invEffectiveMass;
            jacAngular.Jac.Impulse = 0.0f;
            jacAngular.ApplyImpulse = true;

            // Collision detection is performed once per frame. Contact.Distance is estimated from narrowphase and
            // represents the distance between the body and a contact point. solveVelocity represents the velocity
            // the body needs to have to reach contact.Distance in frame timestep. If we take smaller timesteps with
            // substepping, we will travel less distance, but velocity should remain constant
            float solveDistance = contact.Distance; // Distance per frame timestep
            float solveVelocity = solveDistance * (invTimestep / numSubsteps); // velocity to reach CP by end of frame

            // how much velocity needs to be applied to depenetrate the body
            // if solveVelocity = -1 --> VelToReachCp = +1 (is depenetrating), solveDistance < 0, contact.Distance < 0
            // if solveVelocity = +1 --> VelToReachCp = -1   (is approaching), solveDistance > 0
            solveVelocity = math.max(-maxDepenetrationVelocity, solveVelocity);
            jacAngular.VelToReachCp = -solveVelocity;
            jacAngular.ContactDistance = contact.Distance;

            // Calculate average position for friction
            centerA += armA;
            centerB += armB;

            if (jacobianHeader.HasDetailedStaticMeshCollision)
            {
                ref JacobianPolygonData jacobianData = ref jacobianHeader.AccessJacobianContactData();
                jacobianData.ContactPoint = contact.Position;
                jacobianData.Normal = normal;
            }

            // Write the contact point to the jacobian stream if requested
            if (jacobianHeader.HasContactManifold)
            {
                jacobianHeader.AccessContactPoint(contactPointIndex) = contact;
            }
        }

        internal static void BuildFrictionJacobians(
            ref ContactJacobian contactJacobian,
            ref float3 centerA, ref float3 centerB,
            MTransform worldFromA, MTransform worldFromB,
            MotionVelocity velocityA, MotionVelocity velocityB,
            float sumInvMass)
        {
            // Clear accumulated impulse
            contactJacobian.Friction0.Impulse = 0.0f;
            contactJacobian.Friction1.Impulse = 0.0f;
            contactJacobian.AngularFriction.Impulse = 0.0f;

            // Calculate average position
            float invNumContacts = math.rcp(contactJacobian.BaseJacobian.NumContacts);
            centerA *= invNumContacts;
            centerB *= invNumContacts;

            // Choose friction axes
            CalculatePerpendicularNormalized(contactJacobian.BaseJacobian.Normal,
                out float3 frictionDir0, out float3 frictionDir1);

            // Build linear jacobian
            float invEffectiveMass0, invEffectiveMass1;
            {
                float3 armA = centerA;
                float3 armB = centerB;
                CalculateJacobian(worldFromA, worldFromB, frictionDir0, armA, armB,
                    velocityA.InverseInertia, velocityB.InverseInertia, sumInvMass,
                    out contactJacobian.Friction0.AngularA, out contactJacobian.Friction0.AngularB,
                    out invEffectiveMass0);
                CalculateJacobian(worldFromA, worldFromB, frictionDir1, armA, armB,
                    velocityA.InverseInertia, velocityB.InverseInertia, sumInvMass,
                    out contactJacobian.Friction1.AngularA, out contactJacobian.Friction1.AngularB,
                    out invEffectiveMass1);
            }

            // Build angular jacobian
            float invEffectiveMassAngular;
            {
                contactJacobian.AngularFriction.AngularA =
                    math.mul(worldFromA.InverseRotation, contactJacobian.BaseJacobian.Normal);
                contactJacobian.AngularFriction.AngularB =
                    math.mul(worldFromB.InverseRotation, -contactJacobian.BaseJacobian.Normal);

                float3 temp = contactJacobian.AngularFriction.AngularA * contactJacobian.AngularFriction.AngularA * velocityA.InverseInertia;
                temp += contactJacobian.AngularFriction.AngularB * contactJacobian.AngularFriction.AngularB * velocityB.InverseInertia;
                invEffectiveMassAngular = math.csum(temp);
            }

            // Build effective mass
            {
                // Build the inverse effective mass matrix
                var invEffectiveMassDiag = new float3(invEffectiveMass0, invEffectiveMass1, invEffectiveMassAngular);
                var invEffectiveMassOffDiag = new float3( // (0, 1), (0, 2), (1, 2)
                    JacobianUtilities.CalculateInvEffectiveMassOffDiag(
                        contactJacobian.Friction0.AngularA, contactJacobian.Friction1.AngularA, velocityA.InverseInertia,
                        contactJacobian.Friction0.AngularB, contactJacobian.Friction1.AngularB, velocityB.InverseInertia),
                    JacobianUtilities.CalculateInvEffectiveMassOffDiag(
                        contactJacobian.Friction0.AngularA, contactJacobian.AngularFriction.AngularA, velocityA.InverseInertia,
                        contactJacobian.Friction0.AngularB, contactJacobian.AngularFriction.AngularB, velocityB.InverseInertia),
                    JacobianUtilities.CalculateInvEffectiveMassOffDiag(
                        contactJacobian.Friction1.AngularA, contactJacobian.AngularFriction.AngularA, velocityA.InverseInertia,
                        contactJacobian.Friction1.AngularB, contactJacobian.AngularFriction.AngularB, velocityB.InverseInertia));

                // Invert the matrix and store it to the jacobians
                if (!JacobianUtilities.InvertSymmetricMatrix(
                    invEffectiveMassDiag, invEffectiveMassOffDiag,
                    out float3 effectiveMassDiag, out float3 effectiveMassOffDiag))
                {
                    // invEffectiveMass can be singular if the bodies have infinite inertia about the normal.
                    // In that case angular friction does nothing so we can regularize the matrix, set col2 = row2 = (0, 0, 1)
                    invEffectiveMassOffDiag.y = 0.0f;
                    invEffectiveMassOffDiag.z = 0.0f;
                    invEffectiveMassDiag.z = 1.0f;
                    bool success = JacobianUtilities.InvertSymmetricMatrix(
                        invEffectiveMassDiag, invEffectiveMassOffDiag,
                        out effectiveMassDiag, out effectiveMassOffDiag);
                    Assert.IsTrue(success); // it should never fail, if it does then friction will be disabled
                }
                contactJacobian.Friction0.EffectiveMass = effectiveMassDiag.x;
                contactJacobian.Friction1.EffectiveMass = effectiveMassDiag.y;
                contactJacobian.AngularFriction.EffectiveMass = effectiveMassDiag.z;
                contactJacobian.FrictionEffectiveMassOffDiag = effectiveMassOffDiag;
            }
        }

        // Updates the contact Jacobian position and mass data at the start of each substep
        public void Update([NoAlias] ref JacobianHeader jacobianHeader, in MotionVelocity velocityA, in MotionVelocity velocityB,
            in MTransform worldFromA, in MTransform worldFromB, Solver.StepInput stepInput)
        {
            ref ContactJacobian contactJacobian = ref jacobianHeader.AccessBaseJacobian<ContactJacobian>();

            float sumInvMass = velocityA.InverseMass + velocityB.InverseMass;

            SumImpulsesOverSolverIterations = 0.0f;
            int solveCount = 0; // tracks if a bounce has been solved for the current iteration. Used to update friction
            for (int j = 0; j < contactJacobian.BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacobianHeader.AccessAngularJacobian(j);
                jacAngular.Jac.Impulse = 0.0f;

                float dv = CalculateRelativeVelocityAlongNormal(velocityA, velocityB, ref jacAngular, BaseJacobian.Normal,
                    out float relativeVelocity);

                // Determine if the contact is reached during this substep: if the distance
                // travelled by the end of this substep results in a penetration, then we need
                // to bounce now.
                float distanceTraveled = stepInput.Timestep * relativeVelocity;
                float newDistanceToCp = jacAngular.ContactDistance + distanceTraveled;

                var applyRestitution = false;
                if (CoefficientOfRestitution > 0.0f)
                {
                    if (newDistanceToCp < 0.0f)
                    {
                        applyRestitution = CalculateRestitution(stepInput.Timestep, math.length(stepInput.Gravity),
                            CoefficientOfRestitution, ref jacAngular, relativeVelocity, jacAngular.ContactDistance, dv);
                    }
                }

                jacAngular.ContactDistance = newDistanceToCp;
                if (applyRestitution)
                {
                    solveCount++;
                    jacAngular.ContactDistance = 0.0f;  //Used as a flag during SolveContact to indicate impulses need to be calculated
                }
                else
                {
                    if (newDistanceToCp > 0.0f) jacAngular.ApplyImpulse = false;
                }
            }

            // TODO: if ApplyImpulse= false for all NumContacts, can skip friction building
            bool bothMotionsAreKinematic = velocityA.IsKinematic && velocityB.IsKinematic;
            if (!bothMotionsAreKinematic)
            {
                BuildFrictionJacobians(ref contactJacobian, ref CenterA, ref CenterB, worldFromA, worldFromB,
                    velocityA, velocityB, sumInvMass);

                // Reduce friction by 1/4 if there was restitution applied on any contact point
                if (solveCount > 0)
                {
                    Friction0.EffectiveMass *= 0.25f;
                    Friction1.EffectiveMass *= 0.25f;
                    AngularFriction.EffectiveMass *= 0.25f;
                    FrictionEffectiveMassOffDiag *= 0.25f;
                }
            }
        }

        // Generic solve method that dispatches to specific ones
        public void Solve(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter, bool enableFrictionVelocitiesHeuristic,
            Solver.MotionStabilizationInput motionStabilizationSolverInputA,
            Solver.MotionStabilizationInput motionStabilizationSolverInputB)
        {
            bool bothMotionsAreKinematic = velocityA.IsKinematic && velocityB.IsKinematic;
            if (bothMotionsAreKinematic)
            {
                // Note that static bodies are assigned with MotionVelocity.Zero.
                // So, at this point the bodies could be a kinematic vs kinematic pair, or a kinematic vs static pair.
                // Either way, both bodies have an infinite mass and applying contact impulses would have no effect.
                SolveInfMassPair(ref jacHeader, velocityA, velocityB, stepInput, ref collisionEventsWriter);
            }
            else
            {
                SolveContact(ref jacHeader, ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter,
                    enableFrictionVelocitiesHeuristic, motionStabilizationSolverInputA, motionStabilizationSolverInputB);
            }
        }

        // Solve the Contact Jacobian
        public void SolveContact(
            ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter, bool enableFrictionVelocitiesHeuristic,
            Solver.MotionStabilizationInput motionStabilizationSolverInputA,
            Solver.MotionStabilizationInput motionStabilizationSolverInputB)
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

            float sumImpulses = 0.0f;         // Impulse accumulated across NumContacts for current solve call (across multiple Jacobians)
            bool forceCollisionEvent = false; // Is raised when there is a penetration regardless of impulse conditions
            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                if (jacHeader.HasDetailedStaticMeshCollision)
                {
                    ref JacobianPolygonData jacContact = ref jacHeader.AccessJacobianContactData();
                    // Checking whether the contact needs processing.
                    if (!WillHitNextFrame(ref jacContact, ref jacAngular, tempVelocityA, tempVelocityB, stepInput.Timestep))
                        continue;
                }

                // Calculate the relative velocity in the NORMAL direction
                float dv = CalculateRelativeVelocityAlongNormal(tempVelocityA, tempVelocityB, ref jacAngular, BaseJacobian.Normal,
                    out float relativeVelocity);

                // Required check for substepping because having a contact is not guaranteed. Need to check if the
                // contact point has been reached this step (when ContactDistance<=0), otherwise, need to ensure that
                // no impulse will be applied
                if (!jacAngular.ApplyImpulse) dv = 0.0f;

                float impulse = dv * jacAngular.Jac.EffectiveMass;

                // Accumulation of impulses for each individual contact point over numSolverIterations. Cannot be negative
                float jacAccumulatedImpulse = math.max(jacAngular.Jac.Impulse + impulse, 0.0f);
                if (jacAccumulatedImpulse != jacAngular.Jac.Impulse)
                {
                    float deltaImpulse = jacAccumulatedImpulse - jacAngular.Jac.Impulse;
                    ApplyImpulse(deltaImpulse, BaseJacobian.Normal, jacAngular.Jac,
                        ref tempVelocityA, ref tempVelocityB,
                        motionStabilizationSolverInputA.InverseInertiaScale,
                        motionStabilizationSolverInputB.InverseInertiaScale);
                    SumImpulsesOverSolverIterations += deltaImpulse;
                }

                jacAngular.Jac.Impulse = jacAccumulatedImpulse;      // Update persistent contact point Jacobian
                sumImpulses += jacAccumulatedImpulse;

                // Force contact event even when no impulse is applied, but there is penetration.
                forceCollisionEvent |= jacAngular.VelToReachCp > 0.0f;
            }

            if (stepInput.IsLastSolverIteration)
            {
                SumImpulsesOverSubsteps += SumImpulsesOverSolverIterations;
            }

            // Export collision event
            if (stepInput.IsLastSubstepAndLastSolverIteration && jacHeader.HasContactManifold &&
                (SumImpulsesOverSubsteps > 0.0f || forceCollisionEvent))
            {
                ExportCollisionEvent(SumImpulsesOverSubsteps, ref jacHeader, ref collisionEventsWriter);
            }

            // Solve friction
            if (sumImpulses > 0.0f)
            {
                // Choose friction axes
                Math.CalculatePerpendicularNormalized(BaseJacobian.Normal, out float3 frictionDir0, out float3 frictionDir1);

                // Calculate impulses for full stop
                float3 imp;
                {
                    // Take velocities that produce minimum energy (between input and solver velocity) as friction input
                    float3 frictionLinVelA = tempVelocityA.LinearVelocity;
                    float3 frictionAngVelA = tempVelocityA.AngularVelocity;
                    float3 frictionLinVelB = tempVelocityB.LinearVelocity;
                    float3 frictionAngVelB = tempVelocityB.AngularVelocity;
                    if (enableFrictionVelocitiesHeuristic)
                    {
                        // if tempVelocityA.HasInfiniteMass || tempVelocityB.HasInfiniteMass, then GetFrictionVelocities calcs NaN and Inf with no warnings
                        GetFrictionVelocities(motionStabilizationSolverInputA.InputVelocity.Linear,
                            motionStabilizationSolverInputA.InputVelocity.Angular,
                            tempVelocityA.LinearVelocity, tempVelocityA.AngularVelocity,
                            math.rcp(tempVelocityA.InverseInertia), math.rcp(tempVelocityA.InverseMass),
                            out frictionLinVelA, out frictionAngVelA);
                        GetFrictionVelocities(motionStabilizationSolverInputB.InputVelocity.Linear,
                            motionStabilizationSolverInputB.InputVelocity.Angular,
                            tempVelocityB.LinearVelocity, tempVelocityB.AngularVelocity,
                            math.rcp(tempVelocityB.InverseInertia), math.rcp(tempVelocityB.InverseMass),
                            out frictionLinVelB, out frictionAngVelB);
                    }

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
                    float dv0 = extraFrictionDv.x - BaseContactJacobian.GetJacVelocity(frictionDir0, Friction0,
                        frictionLinVelA, frictionAngVelA,
                        frictionLinVelB, frictionAngVelB);
                    float dv1 = extraFrictionDv.y - BaseContactJacobian.GetJacVelocity(frictionDir1, Friction1,
                        frictionLinVelA, frictionAngVelA,
                        frictionLinVelB, frictionAngVelB);
                    float dva = extraFrictionDv.z - math.csum(AngularFriction.AngularA * frictionAngVelA +
                        AngularFriction.AngularB * frictionAngVelB);

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
                ApplyImpulse(imp.x, frictionDir0, Friction0, ref tempVelocityA, ref tempVelocityB,
                    motionStabilizationSolverInputA.InverseInertiaScale,
                    motionStabilizationSolverInputB.InverseInertiaScale);
                ApplyImpulse(imp.y, frictionDir1, Friction1, ref tempVelocityA, ref tempVelocityB,
                    motionStabilizationSolverInputA.InverseInertiaScale,
                    motionStabilizationSolverInputB.InverseInertiaScale);

                tempVelocityA.ApplyAngularImpulse(imp.z * AngularFriction.AngularA * motionStabilizationSolverInputA.InverseInertiaScale);
                tempVelocityB.ApplyAngularImpulse(imp.z * AngularFriction.AngularB * motionStabilizationSolverInputB.InverseInertiaScale);

                // Accumulate them
                Friction0.Impulse += imp.x;
                Friction1.Impulse += imp.y;
                AngularFriction.Impulse += imp.z;
            }

            // Write back linear and angular velocities. Changes to other properties, like InverseMass, should not be persisted.
            velocityA.LinearVelocity = tempVelocityA.LinearVelocity;
            velocityA.AngularVelocity = tempVelocityA.AngularVelocity;
            velocityB.LinearVelocity = tempVelocityB.LinearVelocity;
            velocityB.AngularVelocity = tempVelocityB.AngularVelocity;
        }

        // Solve the infinite mass pair Jacobian
        public void SolveInfMassPair(ref JacobianHeader jacHeader, MotionVelocity velocityA, MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer collisionEventsWriter)
        {
            // Infinite mass pairs are only interested in collision events,
            // so only last solver iteration is performed in that case
            if (!stepInput.IsLastSubstepAndLastSolverIteration)
            {
                return;
            }


            // Calculate normal impulses and fire collision event
            // if at least one contact point would have an impulse applied
            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);
                if (jacHeader.HasDetailedStaticMeshCollision)
                {
                    ref JacobianPolygonData jacContact = ref jacHeader.AccessJacobianContactData();
                    if (!WillHitNextFrame(ref jacContact, ref jacAngular, velocityA, velocityB, stepInput.Timestep))
                        continue;
                }

                float dv = CalculateRelativeVelocityAlongNormal(velocityA, velocityB, ref jacAngular, BaseJacobian.Normal,
                    out float relativeVelocity);

                if (jacAngular.VelToReachCp > 0 || dv > 0)
                {
                    // Export collision event only if impulse would be applied, or objects are penetrating
                    ExportCollisionEvent(0.0f, ref jacHeader, ref collisionEventsWriter);

                    return;
                }
            }
        }

        unsafe bool WillHitNextFrame(ref JacobianPolygonData jacContact,
            ref ContactJacAngAndVelToReachCp jacAngular,
            MotionVelocity velocityA,
            MotionVelocity velocityB,
            float timeStep)
        {
            bool IsNotIntersecting = jacAngular.ContactDistance > 0;
            var IsBodyAStatic = jacContact.IsBodyAStatic;
            if (IsNotIntersecting && jacContact.IsValid)
            {
                float3 contactPoint = IsBodyAStatic ? jacContact.ContactPoint : jacContact.ContactPoint + jacContact.Normal * jacAngular.ContactDistance;
                MotionVelocity motionVelocity = IsBodyAStatic ? velocityB : velocityA;

                // Create the predicted contact point which the contact will occur in the next timestep.
                float3 contactDirection = contactPoint - jacContact.CenterOfMass;
                float3 velocityTorsion = math.cross(motionVelocity.AngularVelocity, contactDirection);
                float3 linearVelocityAtContactPoint = velocityTorsion + motionVelocity.LinearVelocity;
                float3 predictedContactPoint = contactPoint + (timeStep * linearVelocityAtContactPoint);

                // Translate the vector to create the ray based on leaf local position to Evaluate the collision.
                AffineTransform inverseTransform = math.inverse(jacContact.Transform);
                AffineTransform bodyTransform = math.mul(inverseTransform, jacContact.LeafTransform);
                float3 origin = math.transform(bodyTransform, contactPoint);
                float3 displacement = math.transform(bodyTransform, predictedContactPoint) - origin;

                var maxFraction = 1.0f; // This is used to evaluate the fraction of the ray, considering its entire length.
                var hadHit = RaycastQueries.RayTriangle(origin,
                    displacement,
                    jacContact.Vertex0,
                    jacContact.Vertex1,
                    jacContact.Vertex2,
                    ref maxFraction,
                    out float3 unnormalizedNormal);

                if (!hadHit)
                {
#if DISPLAY_DETAILED_STATIC_MESH_DEBUG
                    // draw triangle
                    UnityEngine.Debug.DrawLine(jacContact.Vertex0, jacContact.Vertex2, UnityEngine.Color.magenta);
                    UnityEngine.Debug.DrawLine(jacContact.Vertex0, jacContact.Vertex1, UnityEngine.Color.magenta);
                    UnityEngine.Debug.DrawLine(jacContact.Vertex1, jacContact.Vertex2, UnityEngine.Color.magenta);
                    // draw ray
                    UnityEngine.Debug.DrawLine(contactPoint, predictedContactPoint, UnityEngine.Color.red);
                    UnityEngine.Debug.DrawLine(origin, origin + displacement, UnityEngine.Color.red);
#endif
                    return false;
                }
                else
                {
#if DISPLAY_DETAILED_STATIC_MESH_DEBUG
                    UnityEngine.Debug.DrawLine(contactPoint, predictedContactPoint, UnityEngine.Color.blue);
#endif
                }
            }

            return true;
        }

        // Helper functions
        /// <summary>
        /// The total kinetic energy is calculated for two time periods: At the end-of-frame and at the current point
        /// in time. If the current energy is more than the end of frame expected energy, then the velocities from
        /// the end of frame are returned. Otherwise, the current velocity is returned without modification.
        /// The purpose is to ensure that energy in the system doesn't grow.
        /// </summary>
        /// <param name="inputLinearVelocity"> motionStabilizationSolverInput.InputVelocity.Linear. Is the expected linear velocity at the end of the frame. </param>
        /// <param name="inputAngularVelocity"> motionStabilizationSolverInput.InputVelocity.Angular. Is the expected angular velocity at the end of the frame. </param>
        /// <param name="intermediateLinearVelocity"> tempVelocity.LinearVelocity. Is the current linear velocity of the body. </param>
        /// <param name="intermediateAngularVelocity"> tempVelocity.AngularVelocity. Is the current angular velocity of the body. </param>
        /// <param name="inertia"> math.rcp(tempVelocity.InverseInertia) </param>
        /// <param name="mass"> math.rcp(tempVelocity.InverseMass) </param>
        /// <param name="frictionLinearVelocityOut"> Output: frictionLinVel </param>
        /// <param name="frictionAngularVelocityOut"> Output: frictionAngVel</param>
        void GetFrictionVelocities(
            float3 inputLinearVelocity, float3 inputAngularVelocity,
            float3 intermediateLinearVelocity, float3 intermediateAngularVelocity,
            float3 inertia, float mass,
            out float3 frictionLinearVelocityOut, out float3 frictionAngularVelocityOut)
        {
            float inputEnergy; // Estimated energy at end of frame
            {
                // Energy = m * v^2 + I * w^2, units: kg m^2 / s^2
                float linearEnergySq = mass * math.lengthsq(inputLinearVelocity);
                float angularEnergySq = math.dot(inertia * inputAngularVelocity, inputAngularVelocity);
                inputEnergy = linearEnergySq + angularEnergySq;
            }

            float intermediateEnergy; // Current energy with calculated velocities during Solve
            {
                float linearEnergySq = mass * math.lengthsq(intermediateLinearVelocity);
                float angularEnergySq = math.dot(inertia * intermediateAngularVelocity, intermediateAngularVelocity);
                intermediateEnergy = linearEnergySq + angularEnergySq;
            }

            if (inputEnergy < intermediateEnergy) // More energy NOW than expected at end of frame
            {
                // Make sure we don't change the sign of intermediate velocity when using the input one.
                // If sign was to be changed, zero it out since it produces less energy.
                bool3 changedSignLin = inputLinearVelocity * intermediateLinearVelocity < float3.zero;
                bool3 changedSignAng = inputAngularVelocity * intermediateAngularVelocity < float3.zero;
                frictionLinearVelocityOut = math.select(inputLinearVelocity, float3.zero, changedSignLin);
                frictionAngularVelocityOut = math.select(inputAngularVelocity, float3.zero, changedSignAng);
            }
            else
            {
                frictionLinearVelocityOut = intermediateLinearVelocity;
                frictionAngularVelocityOut = intermediateAngularVelocity;
            }
        }

        private static void ApplyImpulse(
            float impulse, float3 linear, ContactJacobianAngular jacAngular,
            ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            float inverseInertiaScaleA = 1.0f, float inverseInertiaScaleB = 1.0f)
        {
            velocityA.ApplyLinearImpulse(impulse * linear);
            velocityB.ApplyLinearImpulse(-impulse * linear);

            // Scale the impulse with inverseInertiaScale
            velocityA.ApplyAngularImpulse(impulse * jacAngular.AngularA * inverseInertiaScaleA);
            velocityB.ApplyAngularImpulse(impulse * jacAngular.AngularB * inverseInertiaScaleB);
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
            ref CollisionEventData collisionEvent = ref UnsafeUtility.AsRef<CollisionEventData>(eventPtr);
            collisionEvent.BodyIndices = jacHeader.BodyPair;
            collisionEvent.ColliderKeys = jacHeader.AccessColliderKeys();
            collisionEvent.Entities = jacHeader.AccessEntities();
            collisionEvent.Normal = BaseJacobian.Normal;
            collisionEvent.SolverImpulse = totalAccumulatedImpulse;
            collisionEvent.NumNarrowPhaseContactPoints = BaseJacobian.NumContacts;
            for (int i = 0; i < BaseJacobian.NumContacts; i++)
            {
                collisionEvent.AccessContactPoint(i) = jacHeader.AccessContactPoint(i);
            }
        }

        /// Calculates the relative velocity between two bodies in the normal direction.
        /// Reminder:
        /// (is depenetrating) VelToReachCp &gt; 0, solveVelocity &lt; 0, solveDistance &lt; 0, contact.Distance &lt; 0.
        /// (is approaching)   VelToReachCp &lt; 0, solveVelocity &gt; 0, solveDistance &gt; 0, contact.Distance &gt; 0.
        /// Therefore, if VelToReachCp - relativeVelocity &gt; 0, then VelToReachCp > relativeVelocity
        /// if VelToReachCp &gt; 0 and VelToReachCp &gt; relativeVelocity,
        ///      there is depenetration and the bodies can't move fast enough to resolve the penetration
        /// if VelToReachCp &lt; 0 and VelToReachCp &gt; relativeVelocity, then there is nothing to worry about
        internal static float CalculateRelativeVelocityAlongNormal(MotionVelocity velocityA, MotionVelocity velocityB,
            ref ContactJacAngAndVelToReachCp jacAngular, float3 normal, out float relativeVelocity)
        {
            relativeVelocity = BaseContactJacobian.GetJacVelocity(normal, jacAngular.Jac,
                velocityA.LinearVelocity, velocityA.AngularVelocity,
                velocityB.LinearVelocity, velocityB.AngularVelocity);

            return jacAngular.VelToReachCp - relativeVelocity;
        }

        /// <summary>
        /// This method calculates the VelToReachCp correction so that the Solve will calculate an accurate impulse to
        /// redirect motion to apply a restitution effect. This is calculated using two fundamental physics equations:
        /// 1) v1 = v2 - at         [Remove gravity integration]
        ///     where v2 = relativeVelocity, at = -length(gravity) * timestep, and
        ///     v1 is the velocity without gravity applied (velocityBeforeBounce). This is scaled by the coefficient of restitution.
        ///     We are assuming that this is the maximum velocity of the body and that it is now at the contact point.
        ///     Gravity integration is removed because it can't fall anymore.
        /// 2) v2^2 = v1^2 + 2ad    [Calculate restitution velocity as body moves against gravity]
        ///     where a = acceleration due to gravity (and is negative), and d = delta distance (d2-d1).
        ///     This equation dictates that v2>v1 if "d<0", which is the case when a body is falling and d2=0. We are
        ///     however going to let d1=currentDistanceToCp and d2=0, which means that v1 = velocityBeforeBounce (v1 in eq'n1)
        ///     and v2 = velocityRebound (v1>v2). This rebound velocity is the velocity of the body after the restitution
        ///     and is used to correct the VelToReachCp.
        ///
        /// This method updates jacAngular.VelToReachCp and this updated value will always be positive.
        ///
        /// This is an approximation for 2 reasons:
        /// - The restitution impulse is applied as if contact point is on the contact plane. We are assuming the contact
        /// point will hit the contact plane with its current velocity, while actually it would have a portion of gravity
        /// applied before the actual hit. However, that velocity increase is less than gravity in one step.
        /// - gravityMagnitude is the actual value of gravity applied only when contact plane is directly opposite to
        /// gravity direction. Otherwise, this value will only be smaller. However, since this can only result in
        /// smaller bounce than the "correct" one, we can safely go with the default gravity value in all cases.
        /// </summary>
        /// <param name="timestep">  The substep timestep. </param>
        /// <param name="gravityMagnitude">  A float value of the length of the gravity vector.</param>
        /// <param name="coefficientOfRestitution">  The coefficient of Restitution. </param>
        /// <param name="jacAngular"></param>
        /// <param name="relativeVelocity">  The relative velocity between the two bodies at the current point in time. </param>
        /// <param name="currentDistanceToCp">  The contact point distance. This is equivalent to -jacAngular.VelToReachCp * timestep  </param>
        /// <param name="dv">  Delta Velocity: The difference between the velocity required to reach the contact point and the actual velocity </param>
        /// <returns></returns>
        internal static bool CalculateRestitution(float timestep, float gravityMagnitude, float coefficientOfRestitution,
            ref ContactJacAngAndVelToReachCp jacAngular, float relativeVelocity, float currentDistanceToCp, float dv)
        {
            bool applyRestitution = false;

            //TODO: need to account for velocity.GravityFactor (A? B? Both? How combine?)
            float negContactRestingVelocity = -gravityMagnitude * timestep;

            // Conditions to apply restitution:
            // 1) dv: The relative velocity is larger than the velocity threshold to reach the contact point
            // 2) Gravity threshold: The relative velocity must be larger than the 'at rest/gravity only' velocity
            if (dv > 0.0f && relativeVelocity < negContactRestingVelocity)
            {
                // Use equation 1 to calculate v1 (velocityBeforeBounce)
                float velocityBeforeBounce = (relativeVelocity - negContactRestingVelocity) * coefficientOfRestitution;
                float distanceToGround = math.max(currentDistanceToCp, 0.0f);

                // Use equation 2 to calculate rebound velocity (v1) based on v2 (the velocity at the bounce)
                float velocityRebound = math.sqrt(math.max(velocityBeforeBounce * velocityBeforeBounce - 2.0f * -gravityMagnitude * -distanceToGround, 0.0f));

                jacAngular.VelToReachCp = math.max(jacAngular.VelToReachCp - velocityRebound, 0.0f) + velocityRebound;
                applyRestitution = true;
            }

            return applyRestitution;
        }

        internal static void CalculateJacobian(MTransform worldFromA, MTransform worldFromB, float3 normal, float3 armA, float3 armB,
            float3 invInertiaA, float3 invInertiaB, float sumInvMass, out float3 angularA, out float3 angularB, out float invEffectiveMass)
        {
            float3 crossA = math.cross(armA, normal);
            angularA = math.mul(worldFromA.InverseRotation, crossA).xyz;

            float3 crossB = math.cross(normal, armB);
            angularB = math.mul(worldFromB.InverseRotation, crossB).xyz;

            float3 temp = angularA * angularA * invInertiaA + angularB * angularB * invInertiaB;
            invEffectiveMass = temp.x + temp.y + temp.z + sumInvMass;
        }
    }

    // A Jacobian representing a set of contact points that export trigger events
    [NoAlias]
    struct TriggerJacobian
    {
        public BaseContactJacobian BaseJacobian;
        public ColliderKeyPair ColliderKeys;
        public EntityPair Entities;

        // Solve the Jacobian
        public void Solve(ref JacobianHeader jacHeader, ref MotionVelocity velocityA, ref MotionVelocity velocityB,
            Solver.StepInput stepInput, ref NativeStream.Writer triggerEventsWriter)
        {
            // Export trigger events only in last solver iteration of last substep iteration
            if (!stepInput.IsLastSubstepAndLastSolverIteration)
            {
                return;
            }

            // A trigger event will be generated for only the first contact point with the required velocity
            for (int j = 0; j < BaseJacobian.NumContacts; j++)
            {
                ref ContactJacAngAndVelToReachCp jacAngular = ref jacHeader.AccessAngularJacobian(j);

                // Solve velocity so that predicted contact distance is greater than or equal to zero
                float dv = ContactJacobian.CalculateRelativeVelocityAlongNormal(velocityA, velocityB, ref jacAngular,
                    BaseJacobian.Normal, out float relativeVelocity);

                if (jacAngular.VelToReachCp > 0 || dv > 0)
                {
                    // Export trigger event only if impulse would be applied, or objects are penetrating
                    triggerEventsWriter.Write(new TriggerEventData
                    {
                        BodyIndices = jacHeader.BodyPair,
                        ColliderKeys = ColliderKeys,
                        Entities = Entities
                    });

                    return;
                }
            }
        }
    }
}
