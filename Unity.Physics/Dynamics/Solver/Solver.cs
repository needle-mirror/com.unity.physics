using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    /// <summary>   A static class that exposes Solver configuration structures. </summary>
    public static class Solver
    {
        /// <summary>
        /// Takes the timestep in ms and returns the inverse of the timestep (frequency) in Hz.
        /// </summary>
        /// <param name="timeStep"> The time step in ms. </param>
        /// <returns> The frequency in Hz. </returns>
        public static float CalculateInvTimeStep(float timeStep)
        {
            return timeStep > 0.0f ? 1.0f / timeStep : 0.0f;
        }

        /// <summary>   Settings for controlling the solver stabilization heuristic. </summary>
        public struct StabilizationHeuristicSettings
        {
            private byte m_EnableSolverStabilization;

            /// <summary>   Global switch to enable/disable the whole heuristic (false by default) </summary>
            ///
            /// <value> True if enable solver stabilization, false if not. </value>
            public bool EnableSolverStabilization
            {
                get => m_EnableSolverStabilization > 0;
                set => m_EnableSolverStabilization = (byte)(value ? 1 : 0);
            }

            // Individual features control (only valid when EnableSolverStabilizationHeuristic is true)

            private byte m_EnableFrictionVelocities;

            /// <summary>
            /// Switch to enable/disable heuristic when calculating friction velocities. Should be disabled
            /// only if it is causing behavior issues.
            /// </summary>
            ///
            /// <value> True if enable friction velocities, false if not. </value>
            public bool EnableFrictionVelocities
            {
                get => m_EnableFrictionVelocities > 0;
                set => m_EnableFrictionVelocities = (byte)(value ? 1 : 0);
            }

            /// <summary>
            /// Controls the intensity of the velocity clipping. Defaults to 1.0f, while other values will
            /// scale the intensity up/down. Shouldn't go higher than 5.0f, as it will result in bad behavior
            /// (too aggressive velocity clipping). Set it to 0.0f to disable the feature.
            /// </summary>
            public float VelocityClippingFactor;

            /// <summary>
            /// Controls the intensity of inertia scaling. Defaults to 1.0f, while other values will scale
            /// the intensity up/down. Shouldn't go higher than 5.0f, as it will result in bad behavior (too
            /// high inertia of bodies). Set it to 0.0f to disable the feature.
            /// </summary>
            public float InertiaScalingFactor;

            /// <summary>   The defualt stabilization options. </summary>
            public static readonly StabilizationHeuristicSettings Default = new StabilizationHeuristicSettings
            {
                m_EnableSolverStabilization = 0,
                m_EnableFrictionVelocities = 1,
                VelocityClippingFactor = 1.0f,
                InertiaScalingFactor = 1.0f
            };
        }

        /// <summary>   Data used for solver stabilization. </summary>
        public struct StabilizationData
        {
            /// <summary>   Constructor. </summary>
            ///
            /// <param name="stepInput">    The step input. </param>
            /// <param name="context">      The context. </param>
            public StabilizationData(SimulationStepInput stepInput, SimulationContext context)
            {
                StabilizationHeuristicSettings = stepInput.SolverStabilizationHeuristicSettings;
                Gravity = stepInput.Gravity;
                if (stepInput.SolverStabilizationHeuristicSettings.EnableSolverStabilization)
                {
                    InputVelocities = context.InputVelocities;
                    MotionData = context.SolverStabilizationMotionData;
                }
                else
                {
                    InputVelocities = default;
                    MotionData = default;
                }
            }

            // Settings for stabilization heuristics
            internal StabilizationHeuristicSettings StabilizationHeuristicSettings;

            // Disable container safety restriction because it will complain about aliasing
            // with SimulationContext buffers, and it is aliasing, but completely safe.
            // Also, we need the ability to have these not allocated when the feature is not used.

            // Data source: copy of gravity integrated MotionData from output of UpdateInputVelocitiesJob
            [NativeDisableParallelForRestriction]
            [NativeDisableContainerSafetyRestriction]
            internal NativeArray<Velocity> InputVelocities;

            [NativeDisableParallelForRestriction]
            [NativeDisableContainerSafetyRestriction]
            internal NativeArray<StabilizationMotionData> MotionData;

            // Gravity is used to define thresholds for stabilization,
            // and it's not needed in the solver unless stabilization is required.
            internal float3 Gravity;
        }

        // Per motion data for solver stabilization
        internal struct StabilizationMotionData
        {
            public float InverseInertiaScale;
            public byte NumPairs;
        }

        // Internal motion data input for the solver stabilization
        internal struct MotionStabilizationInput
        {
            public Velocity InputVelocity;
            public float InverseInertiaScale;

            public static readonly MotionStabilizationInput Default = new MotionStabilizationInput
            {
                InputVelocity = Velocity.Zero,
                InverseInertiaScale = 1.0f
            };
        }

        internal struct StepInput
        {
            /// <summary> Vector acceleration due to gravity in m/s^2 </summary>
            public float3 Gravity;
            /// <summary> Timestep in seconds. </summary>
            public float Timestep;
            /// <summary> InvTimestep (frequency) in Hertz. </summary>
            public float InvTimestep;
            /// <summary> The inverse of the number of solver iterations. </summary>
            public float InvNumSolverIterations;
            /// <summary> The total number of substeps per frame. </summary>
            public int NumSubsteps;
            /// <summary> The total number of solver iterations per substep. </summary>
            public int NumSolverIterations;
            /// <summary> The current substep. Is -1 when uninitialized. </summary>
            public int CurrentSubstep;
            /// <summary> The current solver iteration. Is -1 when uninitialized. </summary>
            public int CurrentSolverIteration;

            public bool IsFirstSubstep => CurrentSubstep == 0;
            public bool IsLastSubstep => CurrentSubstep == NumSubsteps - 1;
            public bool IsFirstSolverIteration => CurrentSolverIteration == 0;
            public bool IsLastSolverIteration => CurrentSolverIteration == NumSolverIterations - 1;
            public bool MultipleSubstepsAndFirstSolverIteration => NumSubsteps > 1 && IsFirstSolverIteration;
            public bool IsLastSubstepAndLastSolverIteration => IsLastSubstep && IsLastSolverIteration;
        }

        #region BuildJacobians

        // Schedule jobs to build Jacobians from the contacts stored in the simulation context for the first substep
        // where timeStep = frame timestep / numSubsteps
        internal static SimulationJobHandles ScheduleBuildJacobiansJobs(ref PhysicsWorld world,
            float timeStep, float gravityMagnitude, int numSubsteps, int numSolverIterations, JobHandle inputDeps,
            ref NativeList<DispatchPairSequencer.DispatchPair> dispatchPairs,
            ref DispatchPairSequencer.SolverSchedulerInfo solverSchedulerInfo,
            ref NativeStream contacts, ref NativeStream jacobians, bool multiThreaded = true)
        {
            SimulationJobHandles returnHandles = default;

            if (!multiThreaded)
            {
                returnHandles.FinalExecutionHandle = new BuildJacobiansJob
                {
                    ContactsReader = contacts.AsReader(),
                    JacobiansWriter = jacobians.AsWriter(),
                    TimeStep = timeStep,
                    GravityMagnitude = gravityMagnitude,
                    NumSubsteps = numSubsteps,
                    NumSolverIterations = numSolverIterations,
                    World = world,
                    DispatchPairs = dispatchPairs.AsDeferredJobArray()
                }.Schedule(inputDeps);
            }
            else
            {
                var buildJob = new ParallelBuildJacobiansJob
                {
                    ContactsReader = contacts.AsReader(),
                    JacobiansWriter = jacobians.AsWriter(),
                    TimeStep = timeStep,
                    GravityMagnitude = gravityMagnitude,
                    InvTimeStep = CalculateInvTimeStep(timeStep),
                    NumSubsteps = numSubsteps,
                    NumSolverIterations = numSolverIterations,
                    World = world,
                    DispatchPairs = dispatchPairs.AsDeferredJobArray(),
                    SolverSchedulerInfo = solverSchedulerInfo
                };
                JobHandle handle = buildJob.ScheduleUnsafeIndex0(solverSchedulerInfo.NumWorkItems, 1, inputDeps);

                returnHandles.FinalDisposeHandle = JobHandle.CombineDependencies(
                    dispatchPairs.Dispose(handle),
                    contacts.Dispose(handle));

                returnHandles.FinalExecutionHandle = handle;
            }

            return returnHandles;
        }

        [BurstCompile]
        struct BuildJacobiansJob : IJob
        {
            [ReadOnly] public PhysicsWorld World;

            public NativeStream.Reader ContactsReader;
            public NativeStream.Writer JacobiansWriter;
            public float TimeStep; // Substep timestep
            [ReadOnly] public float GravityMagnitude;
            [ReadOnly] public NativeArray<DispatchPairSequencer.DispatchPair> DispatchPairs;
            [ReadOnly] public int NumSubsteps;
            [ReadOnly] public int NumSolverIterations;

            public void Execute()
            {
                BuildJacobians(ref World, TimeStep, GravityMagnitude, NumSubsteps, NumSolverIterations,
                    DispatchPairs, ref ContactsReader, ref JacobiansWriter);
            }
        }

        [BurstCompile]
        struct ParallelBuildJacobiansJob : IJobParallelForDefer
        {
            [ReadOnly] public PhysicsWorld World;

            public NativeStream.Reader ContactsReader;
            public NativeStream.Writer JacobiansWriter;
            public float TimeStep;     // timestep_frame / NumSubsteps
            [ReadOnly] public float GravityMagnitude;
            [ReadOnly] public NativeArray<DispatchPairSequencer.DispatchPair> DispatchPairs;
            [ReadOnly] public int NumSubsteps;
            [ReadOnly] public int NumSolverIterations;
            public float InvTimeStep;  // Frequency
            [ReadOnly] public DispatchPairSequencer.SolverSchedulerInfo SolverSchedulerInfo;

            public void Execute(int workItemIndex)
            {
                int firstDispatchPairIndex = SolverSchedulerInfo.GetWorkItemReadOffset(workItemIndex, out int dispatchPairCount);

                ContactsReader.BeginForEachIndex(workItemIndex);
                JacobiansWriter.BeginForEachIndex(workItemIndex);
                BuildJacobians(ref World, TimeStep, InvTimeStep, GravityMagnitude, NumSubsteps, NumSolverIterations,
                    DispatchPairs, firstDispatchPairIndex, dispatchPairCount, ref ContactsReader, ref JacobiansWriter);
            }
        }

        /// <summary>
        /// Build Jacobians from the contacts and joints stored in the simulation context.
        /// </summary>
        internal static void BuildJacobians(ref PhysicsWorld world,
            float timeStep, float gravityMagnitude, int numSubsteps, int numSolverIterations,
            NativeArray<DispatchPairSequencer.DispatchPair> dispatchPairs,
            ref NativeStream.Reader contactsReader, ref NativeStream.Writer jacobiansWriter)
        {
            contactsReader.BeginForEachIndex(0);
            jacobiansWriter.BeginForEachIndex(0);
            float frequency = CalculateInvTimeStep(timeStep);

            BuildJacobians(ref world, timeStep, frequency, gravityMagnitude, numSubsteps, numSolverIterations, dispatchPairs,
                0, dispatchPairs.Length, ref contactsReader, ref jacobiansWriter);
        }

        private static unsafe void BuildJacobians(
            ref PhysicsWorld world,
            float timestep, // the substep time
            float frequency,
            float gravityMagnitude,
            int numSubsteps,
            int numSolverIterations,
            NativeArray<DispatchPairSequencer.DispatchPair> dispatchPairs,
            int firstDispatchPairIndex,
            int dispatchPairCount,
            ref NativeStream.Reader contactReader,
            ref NativeStream.Writer jacobianWriter)
        {
            // Source: Narrowphase uses predicted velocity to generate dispatchPairs and contactReader data
            for (int i = 0; i < dispatchPairCount; i++)
            {
                var pair = dispatchPairs[i + firstDispatchPairIndex];
                if (!pair.IsValid)
                {
                    continue;
                }

                var motionDatas = world.MotionDatas; // Motion data has gravity applied
                var motionVelocities = world.MotionVelocities;
                var bodies = world.Bodies;

                if (pair.IsContact)
                {
                    // At some point during this frame, there MAY be a contact between these bodies
                    while (contactReader.RemainingItemCount > 0)
                    {
                        // Check if this is the matching contact
                        {
                            var header = contactReader.Peek<ContactHeader>();
                            if (pair.BodyIndexA != header.BodyPair.BodyIndexA ||
                                pair.BodyIndexB != header.BodyPair.BodyIndexB)
                            {
                                break;
                            }
                        }

                        ref ContactHeader contactHeader = ref contactReader.Read<ContactHeader>();
                        GetMotions(contactHeader.BodyPair, ref motionDatas, ref motionVelocities,
                            out MotionVelocity velocityA, out MotionVelocity velocityB,
                            out MTransform worldFromA, out MTransform worldFromB);

                        float sumInvMass = velocityA.InverseMass + velocityB.InverseMass;
                        bool bothMotionsAreKinematic = velocityA.IsKinematic && velocityB.IsKinematic;

                        // Skip contact between infinite mass bodies which don't want to raise events. These cannot have any effect during solving.
                        // These should not normally appear, because the collision detector doesn't generate such contacts.
                        if (bothMotionsAreKinematic)
                        {
                            if ((contactHeader.JacobianFlags & (JacobianFlags.IsTrigger | JacobianFlags.EnableCollisionEvents)) == 0)
                            {
                                for (int j = 0; j < contactHeader.NumContacts; j++)
                                {
                                    contactReader.Read<ContactPoint>();
                                }
                                continue;
                            }
                        }

                        JacobianType jacType = ((int)(contactHeader.JacobianFlags) & (int)(JacobianFlags.IsTrigger)) != 0 ?
                            JacobianType.Trigger : JacobianType.Contact;

                        // Write size before every jacobian and allocate all necessary data for this jacobian
                        int jacobianSize = JacobianHeader.CalculateSize(jacType, contactHeader.JacobianFlags, contactHeader.NumContacts);
                        jacobianWriter.Write(jacobianSize);
                        byte* jacobianPtr = jacobianWriter.Allocate(jacobianSize);

#if DEVELOPMENT_BUILD
                        SafetyChecks.Check4ByteAlignmentAndThrow(jacobianPtr, nameof(jacobianPtr));
#endif
                        ref JacobianHeader jacobianHeader = ref UnsafeUtility.AsRef<JacobianHeader>(jacobianPtr);
                        jacobianHeader.BodyPair = contactHeader.BodyPair;
                        jacobianHeader.Type = jacType;
                        jacobianHeader.Flags = contactHeader.JacobianFlags;

                        var baseJac = new BaseContactJacobian
                        {
                            NumContacts = contactHeader.NumContacts,
                            Normal = contactHeader.Normal
                        };

                        // Body A must be dynamic
                        Assert.IsTrue(contactHeader.BodyPair.BodyIndexA < motionVelocities.Length);
                        bool isDynamicStaticPair = contactHeader.BodyPair.BodyIndexB >= motionVelocities.Length;

                        // If contact distance is negative, use an artificially reduced penetration depth to prevent the
                        // dynamic-dynamic contacts from depenetrating too quickly
                        float maxDepenetrationVelocity = isDynamicStaticPair ? float.MaxValue : 3.0f;

                        if (jacobianHeader.Type == JacobianType.Contact)
                        {
                            ref ContactJacobian contactJacobian = ref jacobianHeader.AccessBaseJacobian<ContactJacobian>();
                            contactJacobian.BaseJacobian = baseJac;
                            contactJacobian.CoefficientOfFriction = contactHeader.CoefficientOfFriction;
                            contactJacobian.CoefficientOfRestitution = contactHeader.CoefficientOfRestitution;
                            contactJacobian.SumImpulsesOverSubsteps = 0.0f;
                            contactJacobian.SumImpulsesOverSolverIterations = 0.0f;

                            // Write polygons from colliders if they are required for detailed static mesh collision detection.
                            WriteJacobianPolygonData(ref jacobianHeader, bodies, contactHeader, worldFromA, worldFromB, motionVelocities.Length);

                            // Initialize modifier data (in order from JacobianModifierFlags) before angular jacobians
                            InitModifierData(ref jacobianHeader, contactHeader.ColliderKeys, new EntityPair
                            {
                                EntityA = bodies[contactHeader.BodyPair.BodyIndexA].Entity,
                                EntityB = bodies[contactHeader.BodyPair.BodyIndexB].Entity
                            });

                            // Build ContactJacobian for each contact in the dispatch pair
                            var centerA = new float3(0.0f);
                            var centerB = new float3(0.0f);
                            int solveCount = 0; // tracks if a bounce has been solved for the current iteration. Used to update friction
                            for (int j = 0; j < contactHeader.NumContacts; j++)
                            {
                                ContactJacobian.BuildIndividualContactJacobians(j, contactJacobian.BaseJacobian.Normal,
                                    worldFromA, worldFromB, frequency, numSubsteps, velocityA, velocityB,
                                    sumInvMass, maxDepenetrationVelocity, ref jacobianHeader, ref centerA, ref centerB,
                                    ref contactReader);

                                if (contactJacobian.CoefficientOfRestitution > 0.0f)
                                {
                                    ref ContactJacAngAndVelToReachCp jacAngular = ref jacobianHeader.AccessAngularJacobian(j);

                                    float dv = ContactJacobian.CalculateRelativeVelocityAlongNormal(velocityA, velocityB, ref jacAngular,
                                        contactJacobian.BaseJacobian.Normal, out float relativeVelocity);

                                    bool applyRestitution = false;
                                    if (numSubsteps > 1)
                                    {
                                        // Determine if the contact is reached during this substep: if the distance
                                        // travelled by the end of this substep results in a penetration, then we need
                                        // to bounce now.
                                        float distanceTraveled = timestep * relativeVelocity;
                                        float newDistanceToCp = jacAngular.ContactDistance + distanceTraveled;
                                        if (newDistanceToCp < 0.0f)
                                        {
                                            applyRestitution = ContactJacobian.CalculateRestitution(timestep,
                                                gravityMagnitude, contactJacobian.CoefficientOfRestitution,
                                                ref jacAngular, relativeVelocity, jacAngular.ContactDistance, dv);
                                        }

                                        // If VelToReachCp was updated in CalculateRestution, update contact distance to 0.
                                        jacAngular.ContactDistance = applyRestitution ? 0.0f : newDistanceToCp;
                                        if (!applyRestitution) jacAngular.ApplyImpulse = false;
                                    }
                                    else
                                    {
                                        applyRestitution = ContactJacobian.CalculateRestitution(timestep,
                                            gravityMagnitude, contactJacobian.CoefficientOfRestitution,
                                            ref jacAngular, relativeVelocity, jacAngular.ContactDistance, dv);
                                    }

                                    if (applyRestitution) solveCount++;
                                }
                            }

                            contactJacobian.CenterA = centerA;
                            contactJacobian.CenterB = centerB;

                            // Build friction jacobians (skip friction between two infinite-mass objects)
                            if (!bothMotionsAreKinematic)
                            {
                                ContactJacobian.BuildFrictionJacobians(
                                    ref contactJacobian,
                                    ref centerA, ref centerB,
                                    worldFromA, worldFromB,
                                    velocityA, velocityB,
                                    sumInvMass);

                                // Reduce friction by 1/4 if there was restitution applied on any contact point
                                if (solveCount > 0)
                                {
                                    contactJacobian.Friction0.EffectiveMass *= 0.25f;
                                    contactJacobian.Friction1.EffectiveMass *= 0.25f;
                                    contactJacobian.AngularFriction.EffectiveMass *= 0.25f;
                                    contactJacobian.FrictionEffectiveMassOffDiag *= 0.25f;
                                }
                            }
                        }
                        // Much less data needed for triggers
                        else
                        {
                            ref TriggerJacobian triggerJacobian = ref jacobianHeader.AccessBaseJacobian<TriggerJacobian>();

                            triggerJacobian.BaseJacobian = baseJac;
                            triggerJacobian.ColliderKeys = contactHeader.ColliderKeys;
                            triggerJacobian.Entities = new EntityPair
                            {
                                EntityA = bodies[contactHeader.BodyPair.BodyIndexA].Entity,
                                EntityB = bodies[contactHeader.BodyPair.BodyIndexB].Entity
                            };

                            // Build normal jacobians
                            var centerA = new float3(0.0f);
                            var centerB = new float3(0.0f);
                            for (int j = 0; j < contactHeader.NumContacts; j++)
                            {
                                // Build the jacobian
                                ContactJacobian.BuildIndividualContactJacobians(
                                    j, triggerJacobian.BaseJacobian.Normal, worldFromA, worldFromB,
                                    frequency, numSubsteps, velocityA, velocityB, sumInvMass, maxDepenetrationVelocity,
                                    ref jacobianHeader, ref centerA, ref centerB, ref contactReader);
                            }
                        }
                    }
                }
                else
                {
                    Joint joint = world.Joints[pair.JointIndex];
                    // Need to fetch the real body indices from the joint, as the scheduler may have reordered them
                    int bodyIndexA = joint.BodyPair.BodyIndexA;
                    int bodyIndexB = joint.BodyPair.BodyIndexB;

                    GetMotion(ref world, bodyIndexA, out MotionVelocity velocityA, out MotionData motionA);
                    GetMotion(ref world, bodyIndexB, out MotionVelocity velocityB, out MotionData motionB);

                    BuildJointJacobian(joint, velocityA, velocityB, motionA, motionB, timestep,
                        numSolverIterations, ref jacobianWriter);
                }
            }

            contactReader.EndForEachIndex();
            jacobianWriter.EndForEachIndex();
        }

        private static unsafe void WriteJacobianPolygonData(ref JacobianHeader jacobianHeader, NativeArray<RigidBody> bodies, in ContactHeader contactHeader, in MTransform worldFromA, in MTransform worldFromB, int dynamicBodiesCount)
        {
            bool isBodyAStatic = contactHeader.BodyPair.BodyIndexA >= dynamicBodiesCount;
            bool isBodyBStatic = contactHeader.BodyPair.BodyIndexB >= dynamicBodiesCount;

            if (jacobianHeader.HasDetailedStaticMeshCollision && (isBodyAStatic || isBodyBStatic))
            {
                ref JacobianPolygonData jacobianPolygonContact = ref jacobianHeader.AccessJacobianContactData();

                // setup conditional variables.
                RigidBody body = default;
                float3 centerOfMass = default; // The center of mass is based on the opposite collider. A over B, and B over A.
                ColliderKey colliderKey = default;

                if (isBodyAStatic)
                {
                    centerOfMass = worldFromB.Translation;
                    body = bodies[contactHeader.BodyPair.BodyIndexA];
                    colliderKey = contactHeader.ColliderKeys.ColliderKeyA;
                }
                else
                {
                    centerOfMass = worldFromA.Translation;
                    body = bodies[contactHeader.BodyPair.BodyIndexB];
                    colliderKey = contactHeader.ColliderKeys.ColliderKeyB;
                }

                // assigning conditional variables
                jacobianPolygonContact.CenterOfMass = centerOfMass;
                jacobianPolygonContact.Transform = new AffineTransform(body.WorldFromBody.pos, body.WorldFromBody.rot, body.Scale);

                Collider* collider = (Collider*)body.Collider.GetUnsafePtr();
                SafetyChecks.CheckAreEqualAndThrow(true, collider != null);
                jacobianPolygonContact.IsValid = collider->GetLeaf(colliderKey, out ChildCollider leafCollider);
                if (jacobianPolygonContact.IsValid)
                {
                    PolygonCollider* polygon = (PolygonCollider*)leafCollider.Collider;
                    jacobianPolygonContact.Vertex0 = polygon->Vertices[0];
                    jacobianPolygonContact.Vertex1 = polygon->Vertices[1];
                    jacobianPolygonContact.Vertex2 = polygon->Vertices[2];
                    jacobianPolygonContact.LeafTransform = new AffineTransform(leafCollider.TransformFromChild);
                    jacobianPolygonContact.IsBodyAStatic = isBodyAStatic;
                    jacobianPolygonContact.IsBodyBStatic = isBodyBStatic;
                }
            }
        }

        #endregion //BuildJacobians

        #region BuildIndividualJacobians

        internal static unsafe void BuildJointJacobian(Joint joint,
            MotionVelocity velocityA, MotionVelocity velocityB, MotionData motionA, MotionData motionB,
            float timestep, int numSolverIterations, [NoAlias] ref NativeStream.Writer jacobianWriter)
        {
            var bodyAFromMotionA = new MTransform(motionA.BodyFromMotion);
            MTransform motionAFromJoint = Mul(Inverse(bodyAFromMotionA), joint.AFromJoint);

            var bodyBFromMotionB = new MTransform(motionB.BodyFromMotion);
            MTransform motionBFromJoint = Mul(Inverse(bodyBFromMotionB), joint.BFromJoint);

            ref var constraintBlock = ref joint.Constraints;
            fixed(void* ptr = &constraintBlock)
            {
                var constraintPtr = (Constraint*)ptr;
                for (var i = 0; i < constraintBlock.Length; i++)
                {
                    Constraint constraint = constraintPtr[i];
                    int constraintDimension = constraint.Dimension;
                    if (0 == constraintDimension)
                    {
                        // Unconstrained, so no need to create a header.
                        continue;
                    }

                    JacobianType jacType;
                    switch (constraint.Type)
                    {
                        case ConstraintType.Linear:
                            jacType = JacobianType.LinearLimit;
                            break;
                        case ConstraintType.Angular:
                            switch (constraintDimension)
                            {
                                case 1:
                                    jacType = JacobianType.AngularLimit1D;
                                    break;
                                case 2:
                                    jacType = JacobianType.AngularLimit2D;
                                    break;
                                case 3:
                                    jacType = JacobianType.AngularLimit3D;
                                    break;
                                default:
                                    SafetyChecks.ThrowNotImplementedException();
                                    return;
                            }

                            break;
                        case ConstraintType.RotationMotor:
                            jacType = JacobianType.RotationMotor;
                            break;
                        case ConstraintType.AngularVelocityMotor:
                            jacType = JacobianType.AngularVelocityMotor;
                            break;
                        case ConstraintType.PositionMotor:
                            jacType = JacobianType.PositionMotor;
                            break;
                        case ConstraintType.LinearVelocityMotor:
                            jacType = JacobianType.LinearVelocityMotor;
                            break;
                        default:
                            SafetyChecks.ThrowNotImplementedException();
                            return;
                    }

                    // Write size before every jacobian
                    JacobianFlags jacFlags = constraint.ShouldRaiseImpulseEvents ? JacobianFlags.EnableImpulseEvents : 0;
                    int jacobianSize = JacobianHeader.CalculateSize(jacType, jacFlags);
                    jacobianWriter.Write(jacobianSize);

                    // Allocate all necessary data for this jacobian
                    byte* jacobianPtr = jacobianWriter.Allocate(jacobianSize);
#if DEVELOPMENT_BUILD
                    SafetyChecks.Check4ByteAlignmentAndThrow(jacobianPtr, nameof(jacobianPtr));
#endif
                    ref JacobianHeader header = ref UnsafeUtility.AsRef<JacobianHeader>(jacobianPtr);
                    header.BodyPair = joint.BodyPair;
                    header.Type = jacType;
                    header.Flags = jacFlags;

                    JacobianUtilities.CalculateConstraintTauAndDamping(constraint.SpringFrequency, constraint.DampingRatio,
                        timestep, numSolverIterations, out float tau, out float damping);

                    // Build the Jacobian
                    switch (constraint.Type)
                    {
                        case ConstraintType.Linear:
                            header.AccessBaseJacobian<LinearLimitJacobian>().Build(
                                motionAFromJoint, motionBFromJoint, velocityA, velocityB,
                                motionA, motionB, constraint, tau, damping);
                            break;
                        case ConstraintType.Angular:
                            switch (constraintDimension)
                            {
                                case 1:
                                    header.AccessBaseJacobian<AngularLimit1DJacobian>().Build(
                                        motionAFromJoint, motionBFromJoint,
                                        velocityA, velocityB, motionA, motionB, constraint, tau, damping);
                                    break;
                                case 2:
                                    header.AccessBaseJacobian<AngularLimit2DJacobian>().Build(
                                        motionAFromJoint, motionBFromJoint,
                                        velocityA, velocityB, motionA, motionB,
                                        constraint, tau, damping);
                                    break;
                                case 3:
                                    header.AccessBaseJacobian<AngularLimit3DJacobian>().Build(
                                        motionAFromJoint, motionBFromJoint,
                                        velocityA, velocityB, motionA, motionB, constraint, tau, damping);
                                    break;
                                default:
                                    SafetyChecks.ThrowNotImplementedException();
                                    return;
                            }

                            break;
                        case ConstraintType.RotationMotor:
                            header.AccessBaseJacobian<RotationMotorJacobian>().Build(
                                motionAFromJoint, motionBFromJoint,
                                motionA, motionB, constraint, tau, damping);
                            break;
                        case ConstraintType.AngularVelocityMotor:
                            header.AccessBaseJacobian<AngularVelocityMotorJacobian>().Build(
                                motionAFromJoint, motionBFromJoint,
                                motionA, motionB, constraint, tau, damping);
                            break;
                        case ConstraintType.PositionMotor:
                            header.AccessBaseJacobian<PositionMotorJacobian>().Build(
                                motionAFromJoint, motionBFromJoint,
                                motionA, motionB, constraint, tau, damping);
                            break;
                        case ConstraintType.LinearVelocityMotor:
                            header.AccessBaseJacobian<LinearVelocityMotorJacobian>().Build(
                                motionAFromJoint, motionBFromJoint,
                                motionA, motionB, constraint, tau, damping);
                            break;
                        default:
                            SafetyChecks.ThrowNotImplementedException();
                            return;
                    }

                    if ((jacFlags & JacobianFlags.EnableImpulseEvents) != 0)
                    {
                        ref ImpulseEventSolverData impulseEventData = ref header.AccessImpulseEventSolverData();
                        impulseEventData.AccumulatedImpulse = float3.zero;
                        impulseEventData.JointEntity = joint.Entity;
                        impulseEventData.MaxImpulse = math.abs(constraint.MaxImpulse);
                    }
                }
            }
        }

        #endregion //BuildIndividualJacobians

        #region UpdateJacobians

        internal static JobHandle ScheduleUpdateJacobiansJobs(ref PhysicsWorld physicsWorld,
            ref NativeStream jacobians, ref DispatchPairSequencer.SolverSchedulerInfo solverSchedulerInfo,
            StepInput stepInput, JobHandle inputDeps, bool multiThreaded = true)
        {
            var updateHandle = inputDeps;

            if (!multiThreaded)
            {
                updateHandle = new UpdateJacobiansJob
                {
                    JacobiansReader = jacobians.AsReader(),
                    MotionDatas = physicsWorld.MotionDatas,
                    MotionVelocities = physicsWorld.MotionVelocities,
                    Bodies = physicsWorld.Bodies,
                    stepInput = stepInput,
                }.Schedule(inputDeps);
            }
            else
            {
                updateHandle = new ParallelUpdateJacobiansJob()
                {
                    JacobiansReader = jacobians.AsReader(),
                    MotionDatas = physicsWorld.MotionDatas,
                    MotionVelocities = physicsWorld.MotionVelocities,
                    Bodies = physicsWorld.Bodies,
                    stepInput = stepInput,
                }.ScheduleUnsafeIndex0(solverSchedulerInfo.NumWorkItems, 1, inputDeps);
            }

            return updateHandle;
        }

        [BurstCompile]
        [NoAlias]
        struct UpdateJacobiansJob : IJob
        {
            [NoAlias][ReadOnly] public NativeStream.Reader JacobiansReader;
            [NoAlias][ReadOnly] public NativeArray<MotionData> MotionDatas;
            [NoAlias][ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [NoAlias][ReadOnly] public NativeArray<RigidBody> Bodies;
            [ReadOnly] public StepInput stepInput;

            public void Execute()
            {
                Update(0, MotionDatas, MotionVelocities, Bodies, ref JacobiansReader, stepInput);
            }
        }

        [BurstCompile]
        [NoAlias]
        struct ParallelUpdateJacobiansJob : IJobParallelForDefer
        {
            [NoAlias][ReadOnly] public NativeStream.Reader JacobiansReader;
            [NoAlias][ReadOnly] public NativeArray<MotionData> MotionDatas;
            [NoAlias][ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [NoAlias][ReadOnly] public NativeArray<RigidBody> Bodies;
            [ReadOnly] public StepInput stepInput;

            public void Execute(int workItemIndex)
            {
                Update(workItemIndex, MotionDatas, MotionVelocities, Bodies,
                    ref JacobiansReader, stepInput);
            }
        }

        internal static void Update(int i,
            [NoAlias] NativeArray<MotionData> motionDatas,
            [NoAlias] NativeArray<MotionVelocity> motionVelocities,
            [NoAlias] NativeArray<RigidBody> bodies,
            [NoAlias] ref NativeStream.Reader jacobianReader,
            StepInput stepInput)
        {
            var jacIterator = new JacobianIterator(jacobianReader, i);
            while (jacIterator.HasJacobiansLeft())
            {
                ref JacobianHeader header = ref jacIterator.ReadJacobianHeader();

                // Static-static pairs should have been filtered during broadphase overlap test
                Assert.IsTrue(header.BodyPair.BodyIndexA < motionDatas.Length || header.BodyPair.BodyIndexB < motionDatas.Length);

                // Update the jacobians
                switch (header.Type)
                {
                    case JacobianType.Trigger:
                        break;

                    case JacobianType.Contact:
                        GetMotions(header.BodyPair, ref motionDatas, ref motionVelocities,
                            out MotionVelocity velocityA, out MotionVelocity velocityB,
                            out MTransform worldFromA, out MTransform worldFromB);

                        header.UpdateContact(in velocityA, in velocityB, in worldFromA, in worldFromB, stepInput);
                        break;

                    default: // for everything else
                        // Get the motion pair
                        MotionData motionDataA = header.BodyPair.BodyIndexA < motionDatas.Length ?
                            motionDatas[header.BodyPair.BodyIndexA] :
                            new MotionData
                        {
                            WorldFromMotion = bodies[header.BodyPair.BodyIndexA].WorldFromBody,
                            BodyFromMotion = RigidTransform.identity
                        };

                        MotionData motionDataB = header.BodyPair.BodyIndexB < motionDatas.Length ?
                            motionDatas[header.BodyPair.BodyIndexB] : new MotionData
                        {
                            WorldFromMotion = bodies[header.BodyPair.BodyIndexB].WorldFromBody,
                            BodyFromMotion = RigidTransform.identity
                        };

                        header.UpdateJoints(in motionDataA, in motionDataB);
                        break;
                }
            }
        }

        #endregion //UpdateBuildJacobians

        #region SolveJacobians

        /// <summary>   Schedule jobs to solve the Jacobians stored in the simulation context. </summary>
        internal static unsafe SimulationJobHandles ScheduleSolveJacobiansJobs(
            ref DynamicsWorld dynamicsWorld, StepInput stepInput,
            ref NativeStream jacobians, ref NativeStream collisionEvents, ref NativeStream triggerEvents,
            ref NativeStream impulseEvents, ref DispatchPairSequencer.SolverSchedulerInfo solverSchedulerInfo,
            StabilizationData solverStabilizationData, JobHandle inputDeps, bool multiThreaded = true)
        {
            SimulationJobHandles returnHandles = default;

            if (!multiThreaded)
            {
                returnHandles.FinalExecutionHandle = new SolverJob
                {
                    CollisionEventsWriter = collisionEvents.AsWriter(),
                    JacobiansReader = jacobians.AsReader(),
                    stepInput = stepInput,
                    TriggerEventsWriter = triggerEvents.AsWriter(),
                    ImpulseEventsWriter = impulseEvents.AsWriter(),
                    MotionVelocities = dynamicsWorld.MotionVelocities,
                    SolverStabilizationData = solverStabilizationData,
                }.Schedule(inputDeps);

                return returnHandles;
            }

            JobHandle handle = inputDeps;

            // early out if there are no work items to process
            int numPhases = solverSchedulerInfo.NumActivePhases[0];
            if (numPhases > 0)
            {
                {
                    var phaseInfoPtrs =
                        (DispatchPairSequencer.SolverSchedulerInfo.SolvePhaseInfo*)NativeArrayUnsafeUtility
                            .GetUnsafeBufferPointerWithoutChecks(solverSchedulerInfo.PhaseInfo);

                    float3 gravityNormalized = float3.zero;
                    if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization)
                    {
                        gravityNormalized = math.normalizesafe(solverStabilizationData.Gravity);
                    }

                    for (int iSolverIteration = 0; iSolverIteration < stepInput.NumSolverIterations; iSolverIteration++)
                    {
                        stepInput.CurrentSolverIteration = iSolverIteration;

                        for (int phaseId = 0; phaseId < numPhases; phaseId++)
                        {
                            var job = new ParallelSolverJob
                            {
                                JacobiansReader = jacobians.AsReader(),
                                PhaseIndex = phaseId,
                                Phases = solverSchedulerInfo.PhaseInfo,
                                MotionVelocities = dynamicsWorld.MotionVelocities,
                                SolverStabilizationData = solverStabilizationData,
                                stepInput = stepInput
                            };

                            // Only initialize event writers for last solver iteration jobs
                            if (stepInput.IsLastSolverIteration)
                            {
                                job.CollisionEventsWriter = collisionEvents.AsWriter();
                                job.TriggerEventsWriter = triggerEvents.AsWriter();
                                job.ImpulseEventsWriter = impulseEvents.AsWriter();
                            }

                            var info = phaseInfoPtrs[phaseId];
                            // Note: If we have duplicate body indices across batches in this phase we need to process the phase
                            // sequentially to prevent data races. In this case, we choose a large batch size (batch equal to number of work items)
                            // to prevent any parallelization of the work.
                            int batchSize = info.ContainsDuplicateIndices ? info.NumWorkItems : 1;
                            handle = job.Schedule(info.NumWorkItems, batchSize, handle);
                        }

                        // Stabilize velocities
                        if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization)
                        {
                            var stabilizeVelocitiesJob = new StabilizeVelocitiesJob
                            {
                                MotionVelocities = dynamicsWorld.MotionVelocities,
                                SolverStabilizationData = solverStabilizationData,
                                GravityPerStep = solverStabilizationData.Gravity * stepInput.Timestep, //job doesn't need gravity in stabilize struct
                                GravityNormalized = gravityNormalized,
                                IsFirstIteration = stepInput.IsFirstSolverIteration
                            };

                            handle = stabilizeVelocitiesJob.Schedule(dynamicsWorld.NumMotions, 64, handle);
                        }
                    }

                    returnHandles.FinalDisposeHandle = handle;
                }
            }

            // Dispose processed data
            if (stepInput.IsLastSubstep)
            {
                returnHandles.FinalDisposeHandle = JobHandle.CombineDependencies(
                    jacobians.Dispose(handle),
                    solverSchedulerInfo.ScheduleDisposeJob(handle),
                    returnHandles.FinalDisposeHandle);
            }
            returnHandles.FinalExecutionHandle = handle;

            return returnHandles;
        }

        [BurstCompile]
        [NoAlias]
        private struct SolverJob : IJob
        {
            public NativeArray<MotionVelocity> MotionVelocities;
            public StabilizationData SolverStabilizationData;

            [NoAlias]
            public NativeStream.Reader JacobiansReader;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer CollisionEventsWriter;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer TriggerEventsWriter;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer ImpulseEventsWriter;

            public StepInput stepInput;

            public void Execute()
            {
                SolveJacobians(ref JacobiansReader, MotionVelocities, stepInput,
                    ref CollisionEventsWriter, ref TriggerEventsWriter, ref ImpulseEventsWriter, SolverStabilizationData);
            }
        }

        /// <summary>   Solve the Jacobians stored in the simulation context. </summary>
        internal static void SolveJacobians(ref NativeStream.Reader jacobiansReader,
            NativeArray<MotionVelocity> motionVelocities, StepInput stepInput,
            ref NativeStream.Writer collisionEventsWriter, ref NativeStream.Writer triggerEventsWriter,
            ref NativeStream.Writer impulseEventsWriter, StabilizationData solverStabilizationData)
        {
            for (int solverIterationId = 0; solverIterationId < stepInput.NumSolverIterations; solverIterationId++)
            {
                stepInput.CurrentSolverIteration = solverIterationId;

                Solve(motionVelocities, ref jacobiansReader,
                    ref collisionEventsWriter, ref triggerEventsWriter, ref impulseEventsWriter,
                    0, stepInput, solverStabilizationData);

                if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization)
                {
                    StabilizeVelocities(motionVelocities, stepInput.IsFirstSolverIteration, stepInput.Timestep, solverStabilizationData);
                }
            }
        }

        [BurstCompile]
        [NoAlias]
        private struct ParallelSolverJob : IJobParallelFor
        {
            [NativeDisableParallelForRestriction]
            public NativeArray<MotionVelocity> MotionVelocities;

            [NativeDisableParallelForRestriction]
            public StabilizationData SolverStabilizationData;

            [NoAlias]
            public NativeStream.Reader JacobiansReader;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer CollisionEventsWriter;

            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer TriggerEventsWriter;

            [NativeDisableContainerSafetyRestriction]
            [NoAlias]
            public NativeStream.Writer ImpulseEventsWriter;

            [ReadOnly]
            public NativeArray<DispatchPairSequencer.SolverSchedulerInfo.SolvePhaseInfo> Phases;

            public int PhaseIndex;
            public StepInput stepInput;

            public void Execute(int workItemIndex)
            {
                int workItemStartIndexOffset = Phases[PhaseIndex].FirstWorkItemIndex;
                if (stepInput.IsLastSolverIteration)
                {
                    CollisionEventsWriter.PatchMinMaxRange(workItemIndex + workItemStartIndexOffset);
                    TriggerEventsWriter.PatchMinMaxRange(workItemIndex + workItemStartIndexOffset);
                    ImpulseEventsWriter.PatchMinMaxRange(workItemIndex + workItemStartIndexOffset);
                }

                Solve(MotionVelocities, ref JacobiansReader,
                    ref CollisionEventsWriter, ref TriggerEventsWriter, ref ImpulseEventsWriter,
                    workItemIndex + workItemStartIndexOffset, stepInput, SolverStabilizationData);
            }
        }

        private static void Solve(
            NativeArray<MotionVelocity> motionVelocities,
            [NoAlias] ref NativeStream.Reader jacobianReader,
            [NoAlias] ref NativeStream.Writer collisionEventsWriter,
            [NoAlias] ref NativeStream.Writer triggerEventsWriter,
            [NoAlias] ref NativeStream.Writer impulseEventsWriter,
            int workItemIndex,
            StepInput stepInput,
            StabilizationData solverStabilizationData)
        {
            if (stepInput.IsLastSubstepAndLastSolverIteration)
            {
                collisionEventsWriter.BeginForEachIndex(workItemIndex);
                triggerEventsWriter.BeginForEachIndex(workItemIndex);
                impulseEventsWriter.BeginForEachIndex(workItemIndex);
            }

            MotionStabilizationInput motionStabilizationSolverInputA = MotionStabilizationInput.Default;
            MotionStabilizationInput motionStabilizationSolverInputB = MotionStabilizationInput.Default;

            var jacIterator = new JacobianIterator(jacobianReader, workItemIndex);
            while (jacIterator.HasJacobiansLeft())
            {
                ref JacobianHeader header = ref jacIterator.ReadJacobianHeader();

                // Static-static pairs should have been filtered during broadphase overlap test
                Assert.IsTrue(header.BodyPair.BodyIndexA < motionVelocities.Length || header.BodyPair.BodyIndexB < motionVelocities.Length);

                // Get the motion pair
                MotionVelocity velocityA = header.BodyPair.BodyIndexA < motionVelocities.Length ?
                    motionVelocities[header.BodyPair.BodyIndexA] : MotionVelocity.Zero;
                MotionVelocity velocityB = header.BodyPair.BodyIndexB < motionVelocities.Length ?
                    motionVelocities[header.BodyPair.BodyIndexB] : MotionVelocity.Zero;

                // For Contacts Only: Populate the Solver stabilization data
                if (solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization
                    && header.Type == JacobianType.Contact)
                {
                    SolverStabilizationUpdate(ref header, stepInput.IsFirstSolverIteration, velocityA, velocityB,
                        solverStabilizationData, ref motionStabilizationSolverInputA, ref motionStabilizationSolverInputB);
                }

                // Solve the jacobian
                header.Solve(ref velocityA, ref velocityB, stepInput,
                    ref collisionEventsWriter, ref triggerEventsWriter, ref impulseEventsWriter,
                    solverStabilizationData.StabilizationHeuristicSettings.EnableSolverStabilization &&
                    solverStabilizationData.StabilizationHeuristicSettings.EnableFrictionVelocities,
                    motionStabilizationSolverInputA, motionStabilizationSolverInputB);

                // Write back velocity for dynamic bodies
                if (header.BodyPair.BodyIndexA < motionVelocities.Length)
                {
                    motionVelocities[header.BodyPair.BodyIndexA] = velocityA;
                }
                if (header.BodyPair.BodyIndexB < motionVelocities.Length)
                {
                    motionVelocities[header.BodyPair.BodyIndexB] = velocityB;
                }
            }

            if (stepInput.IsLastSubstepAndLastSolverIteration)
            {
                collisionEventsWriter.EndForEachIndex();
                triggerEventsWriter.EndForEachIndex();
                impulseEventsWriter.EndForEachIndex();
            }
        }

        #endregion //SolveJacobians

        #region UpdateInputVelocities

        /// <summary>
        /// Updates input velocities array with gravity integration for all dynamic bodies. The gravity integrated data
        /// is written to the linear velocity of inputVelocities. The AngularVelocity is copied without modification
        /// from the MotionVelocity data. The MotionVelocity data is not modified. MotionVelocities is not modified.
        /// </summary>
        internal static JobHandle ScheduleUpdateInputVelocitiesJob(NativeArray<MotionVelocity> motionVelocities,
            NativeArray<Velocity> inputVelocities,  float3 velocityFromGravity, JobHandle inputDeps, bool multiThreaded = true)
        {
            if (!multiThreaded)
            {
                var job = new UpdateInputVelocitiesJob
                {
                    MotionVelocities = motionVelocities,
                    InputVelocities = inputVelocities,
                    GravityVelocity = velocityFromGravity // Units: m/s. Input as: g * t
                };
                return job.Schedule(inputDeps);
            }
            else
            {
                var job = new ParallelUpdateInputVelocitiesJob
                {
                    MotionVelocities = motionVelocities,
                    InputVelocities = inputVelocities,
                    GravityAcceleration = velocityFromGravity
                };
                return job.Schedule(motionVelocities.Length, 64, inputDeps);
            }
        }

        [BurstCompile]
        private struct UpdateInputVelocitiesJob : IJob
        {
            public NativeArray<MotionVelocity> MotionVelocities;
            public NativeArray<Velocity> InputVelocities;
            public float3 GravityVelocity;

            public void Execute()
            {
                UpdateInputVelocities(MotionVelocities, InputVelocities, GravityVelocity);
            }
        }

        /// Predict gravity for all dynamic bodies. The predicted motion due to gravity is written to
        /// inputVelocities.
        internal static void UpdateInputVelocities(NativeArray<MotionVelocity> motionVelocities,
            NativeArray<Velocity> inputVelocities, float3 gravityVelocity)
        {
            for (int i = 0; i < motionVelocities.Length; i++)
            {
                ParallelUpdateInputVelocitiesJob.ExecuteImpl(i, gravityVelocity, motionVelocities,
                    inputVelocities);
            }
        }

        [BurstCompile]
        private struct ParallelUpdateInputVelocitiesJob : IJobParallelFor
        {
            public NativeArray<MotionVelocity> MotionVelocities;
            public NativeArray<Velocity> InputVelocities;
            public float3 GravityAcceleration;

            public void Execute(int i)
            {
                ExecuteImpl(i, GravityAcceleration, MotionVelocities, InputVelocities);
            }

            internal static void ExecuteImpl(int i, float3 velocityFromGravity,
                NativeArray<MotionVelocity> motionVelocities, NativeArray<Velocity> inputVelocities)
            {
                MotionVelocity motionVelocity = motionVelocities[i];

                // Predict what linear velocity will be due to gravity at end of frame
                inputVelocities[i] = new Velocity
                {
                    Linear = motionVelocity.LinearVelocity + velocityFromGravity * motionVelocity.GravityFactor,
                    Angular = motionVelocity.AngularVelocity
                };
            }
        }

        #endregion //UpdateInputVelocities

        #region ApplyGravityAndUpdateInputVelocities

        /// Schedules jobs that apply gravity to all dynamic bodies and update InputVelocities. This method:
        /// 1) updates MotionVelocities with the gravity integrated linear velocity.
        /// 2) updates InputVelocities with the new MotionVelocities Linear and Angular data.
        internal static JobHandle ScheduleApplyGravityAndUpdateInputVelocitiesJob(ref DynamicsWorld world, NativeArray<Velocity> inputVelocities,
            float3 gravityAcceleration, float timestep, JobHandle inputDeps, bool multiThreaded = true)
        {
            if (!multiThreaded)
            {
                var job = new ApplyGravityAndUpdateInputVelocitiesJob
                {
                    MotionVelocities = world.MotionVelocities,
                    InputVelocities = inputVelocities,
                    GravityAcceleration = gravityAcceleration,
                    Timestep = timestep
                };
                return job.Schedule(inputDeps);
            }
            else
            {
                var job = new ParallelApplyGravityAndUpdateInputVelocitiesJob
                {
                    MotionVelocities = world.MotionVelocities,
                    InputVelocities = inputVelocities,
                    GravityAcceleration = gravityAcceleration,
                    Timestep = timestep
                };
                return job.Schedule(world.MotionVelocities.Length, 64, inputDeps);
            }
        }

        [BurstCompile]
        private struct ApplyGravityAndUpdateInputVelocitiesJob : IJob
        {
            public NativeArray<MotionVelocity> MotionVelocities;
            public NativeArray<Velocity> InputVelocities;
            public float3 GravityAcceleration;
            public float Timestep;

            public void Execute()
            {
                ApplyGravityAndUpdateInputVelocities(MotionVelocities, InputVelocities, GravityAcceleration, Timestep);
            }
        }

        /// <summary>   Apply gravity to all dynamic bodies and update InputVelocities. This method:
        /// 1) updates MotionVelocities with the gravity integrated linear velocity.
        /// 2) updates InputVelocities with the new MotionVelocities Linear and Angular data.
        /// </summary>
        internal static void ApplyGravityAndUpdateInputVelocities(NativeArray<MotionVelocity> motionVelocities,
            NativeArray<Velocity> inputVelocities, float3 gravityAcceleration, float timestep)
        {
            for (int i = 0; i < motionVelocities.Length; i++)
            {
                ParallelApplyGravityAndUpdateInputVelocitiesJob.ExecuteImpl(i, motionVelocities, inputVelocities, gravityAcceleration, timestep);
            }
        }

        [BurstCompile]
        private struct ParallelApplyGravityAndUpdateInputVelocitiesJob : IJobParallelFor
        {
            public NativeArray<MotionVelocity> MotionVelocities;
            public NativeArray<Velocity> InputVelocities;
            public float3 GravityAcceleration;
            public float Timestep;

            public void Execute(int i)
            {
                ExecuteImpl(i, MotionVelocities, InputVelocities, GravityAcceleration, Timestep);
            }

            internal static void ExecuteImpl(int i, NativeArray<MotionVelocity> motionVelocities,
                NativeArray<Velocity> inputVelocities, float3 gravityAcceleration, float timestep)
            {
                MotionVelocity motionVelocity = motionVelocities[i];

                // Apply gravity
                motionVelocity.LinearVelocity += gravityAcceleration * motionVelocity.GravityFactor * timestep;

                // Write back
                motionVelocities[i] = motionVelocity;

                // Make a copy
                inputVelocities[i] = new Velocity
                {
                    Linear = motionVelocity.LinearVelocity,
                    Angular = motionVelocity.AngularVelocity
                };
            }
        }

        #endregion //ApplyGravityAndUpdateInputVelocities

        #region StabilizationHeuristics

        private static void StabilizeVelocities(NativeArray<MotionVelocity> motionVelocities,
            bool isFirstIteration, float timeStep, StabilizationData solverStabilizationData)
        {
            float3 gravityPerStep = solverStabilizationData.Gravity * timeStep;
            float3 gravityNormalized = math.normalizesafe(solverStabilizationData.Gravity);

            for (int i = 0; i < motionVelocities.Length; i++)
            {
                StabilizeVelocitiesJob.ExecuteImpl(i, motionVelocities, isFirstIteration,
                    gravityPerStep, gravityNormalized, solverStabilizationData);
            }
        }

        [BurstCompile]
        private struct StabilizeVelocitiesJob : IJobParallelFor
        {
            public NativeArray<MotionVelocity> MotionVelocities;
            public StabilizationData SolverStabilizationData;
            public float3 GravityPerStep;
            public float3 GravityNormalized;
            public bool IsFirstIteration;

            public void Execute(int i)
            {
                ExecuteImpl(i, MotionVelocities, IsFirstIteration, GravityPerStep, GravityNormalized, SolverStabilizationData);
            }

            internal static void ExecuteImpl(int i, NativeArray<MotionVelocity> motionVelocities,
                bool isFirstIteration, float3 gravityPerStep, float3 gravityNormalized,
                StabilizationData solverStabilizationData)
            {
                var motionData = solverStabilizationData.MotionData[i];
                int numPairs = motionData.NumPairs;
                if (numPairs == 0)
                {
                    return;
                }

                MotionVelocity motionVelocity = motionVelocities[i];

                // Skip kinematic bodies
                if (motionVelocity.InverseMass == 0.0f)
                {
                    return;
                }

                // Scale up inertia for other iterations
                if (isFirstIteration && numPairs > 1)
                {
                    float inertiaScale = 1.0f + 0.2f * (numPairs - 1) * solverStabilizationData.StabilizationHeuristicSettings.InertiaScalingFactor;
                    motionData.InverseInertiaScale = math.rcp(inertiaScale);
                    solverStabilizationData.MotionData[i] = motionData;
                }

                // Don't stabilize velocity component along the gravity vector
                float3 linVelVertical = math.dot(motionVelocity.LinearVelocity, gravityNormalized) * gravityNormalized;
                float3 linVelSideways = motionVelocity.LinearVelocity - linVelVertical;

                // Choose a very small gravity coefficient for clipping threshold
                float gravityCoefficient = (numPairs == 1 ? 0.1f : 0.25f) * solverStabilizationData.StabilizationHeuristicSettings.VelocityClippingFactor;

                // Linear velocity threshold
                float smallLinVelThresholdSq = math.lengthsq(gravityPerStep * motionVelocity.GravityFactor * gravityCoefficient);

                // Stabilize the velocities
                if (math.lengthsq(linVelSideways) < smallLinVelThresholdSq)
                {
                    motionVelocity.LinearVelocity = linVelVertical;

                    // Only clip angular if in contact with at least 2 bodies
                    if (numPairs > 1)
                    {
                        // Angular velocity threshold
                        if (motionVelocity.AngularExpansionFactor > 0.0f)
                        {
                            float angularFactorSq = math.rcp(motionVelocity.AngularExpansionFactor * motionVelocity.AngularExpansionFactor) * 0.01f;
                            float smallAngVelThresholdSq = smallLinVelThresholdSq * angularFactorSq;
                            if (math.lengthsq(motionVelocity.AngularVelocity) < smallAngVelThresholdSq)
                            {
                                motionVelocity.AngularVelocity = float3.zero;
                            }
                        }
                    }

                    // Write back
                    motionVelocities[i] = motionVelocity;
                }
            }
        }

        // Updates data for solver stabilization heuristic.
        // Updates number of pairs for dynamic bodies and resets inverse inertia scale in first iteration.
        // Also prepares motion stabilization solver data for current Jacobian to solve.
        private static void SolverStabilizationUpdate(
            ref JacobianHeader header, bool isFirstIteration,
            MotionVelocity velocityA, MotionVelocity velocityB,
            StabilizationData solverStabilizationData,
            ref MotionStabilizationInput motionStabilizationSolverInputA,
            ref MotionStabilizationInput motionStabilizationSolverInputB)
        {
            // Solver stabilization heuristic, count pairs and reset inverse inertia scale only in first iteration
            var inputVelocities = solverStabilizationData.InputVelocities;
            var motionData = solverStabilizationData.MotionData;
            if (isFirstIteration)
            {
                // Only count heavier (or up to 2 times lighter) bodies as pairs
                // Also reset inverse inertia scale
                if (header.BodyPair.BodyIndexA < motionData.Length)
                {
                    var data = motionData[header.BodyPair.BodyIndexA];
                    if (0.5f * velocityB.InverseMass <= velocityA.InverseMass)
                    {
                        data.NumPairs++;
                    }
                    data.InverseInertiaScale = 1.0f;
                    motionData[header.BodyPair.BodyIndexA] = data;
                }
                if (header.BodyPair.BodyIndexB < motionData.Length)
                {
                    var data = motionData[header.BodyPair.BodyIndexB];
                    if (0.5f * velocityA.InverseMass <= velocityB.InverseMass)
                    {
                        data.NumPairs++;
                    }
                    data.InverseInertiaScale = 1.0f;
                    motionData[header.BodyPair.BodyIndexB] = data;
                }
            }

            // Motion solver input stabilization data
            {
                if (solverStabilizationData.StabilizationHeuristicSettings.EnableFrictionVelocities)
                {
                    motionStabilizationSolverInputA.InputVelocity = header.BodyPair.BodyIndexA < inputVelocities.Length ?
                        inputVelocities[header.BodyPair.BodyIndexA] : Velocity.Zero;
                    motionStabilizationSolverInputB.InputVelocity = header.BodyPair.BodyIndexB < inputVelocities.Length ?
                        inputVelocities[header.BodyPair.BodyIndexB] : Velocity.Zero;
                }

                motionStabilizationSolverInputA.InverseInertiaScale = header.BodyPair.BodyIndexA < motionData.Length ?
                    motionData[header.BodyPair.BodyIndexA].InverseInertiaScale : 1.0f;
                motionStabilizationSolverInputB.InverseInertiaScale = header.BodyPair.BodyIndexB < motionData.Length ?
                    motionData[header.BodyPair.BodyIndexB].InverseInertiaScale : 1.0f;
            }
        }

        #endregion //StabilizationHeuristics

        #region Implementation

        private static void InitModifierData(ref JacobianHeader jacobianHeader, ColliderKeyPair colliderKeys, EntityPair entities)
        {
            if (jacobianHeader.HasContactManifold)
            {
                jacobianHeader.AccessColliderKeys() = colliderKeys;
                jacobianHeader.AccessEntities() = entities;
            }
            if (jacobianHeader.HasSurfaceVelocity)
            {
                jacobianHeader.AccessSurfaceVelocity() = new SurfaceVelocity();
            }
            if (jacobianHeader.HasMassFactors)
            {
                jacobianHeader.AccessMassFactors() = MassFactors.Default;
            }
        }

        private static void GetMotions(
            BodyIndexPair pair,
            ref NativeArray<MotionData> motionDatas,
            ref NativeArray<MotionVelocity> motionVelocities,
            out MotionVelocity velocityA,
            out MotionVelocity velocityB,
            out MTransform worldFromA,
            out MTransform worldFromB)
        {
            bool bodyAIsStatic = pair.BodyIndexA >= motionVelocities.Length;
            bool bodyBIsStatic = pair.BodyIndexB >= motionVelocities.Length;

            if (bodyAIsStatic)
            {
                if (bodyBIsStatic)
                {
                    Assert.IsTrue(false); // static-static pairs should have been filtered during broadphase overlap test
                    velocityA = MotionVelocity.Zero;
                    velocityB = MotionVelocity.Zero;
                    worldFromA = MTransform.Identity;
                    worldFromB = MTransform.Identity;
                    return;
                }

                velocityA = MotionVelocity.Zero;
                velocityB = motionVelocities[pair.BodyIndexB];

                worldFromA = MTransform.Identity;
                worldFromB = new MTransform(motionDatas[pair.BodyIndexB].WorldFromMotion);
            }
            else if (bodyBIsStatic)
            {
                velocityA = motionVelocities[pair.BodyIndexA];
                velocityB = MotionVelocity.Zero;

                worldFromA = new MTransform(motionDatas[pair.BodyIndexA].WorldFromMotion);
                worldFromB = MTransform.Identity;
            }
            else
            {
                velocityA = motionVelocities[pair.BodyIndexA];
                velocityB = motionVelocities[pair.BodyIndexB];

                worldFromA = new MTransform(motionDatas[pair.BodyIndexA].WorldFromMotion);
                worldFromB = new MTransform(motionDatas[pair.BodyIndexB].WorldFromMotion);
            }
        }

        // Gets a body's motion, even if the body is static
        // TODO - share code with GetMotions()?
        private static void GetMotion([NoAlias] ref PhysicsWorld world, int bodyIndex,
            [NoAlias] out MotionVelocity velocity, [NoAlias] out MotionData motion)
        {
            if (bodyIndex >= world.MotionVelocities.Length)
            {
                // Body is static
                velocity = MotionVelocity.Zero;
                motion = new MotionData
                {
                    WorldFromMotion = world.Bodies[bodyIndex].WorldFromBody,
                    BodyFromMotion = RigidTransform.identity
                        // remaining fields all zero
                };
            }
            else
            {
                // Body is dynamic
                velocity = world.MotionVelocities[bodyIndex];
                motion = world.MotionDatas[bodyIndex];
            }
        }

        #endregion //Implementation
    }
}
