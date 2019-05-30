using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Assertions;
using static Unity.Physics.Math;

namespace Unity.Physics
{
    public static class Solver
    {
        public struct StepInput
        {
            public bool IsLastIteration;
            public float InvNumSolverIterations;
            public float Timestep;
        }

        // Schedule some jobs to build Jacobians from the contacts stored in the simulation context
        public static JobHandle ScheduleBuildContactJacobiansJobs(ref DynamicsWorld world, float timeStep, float gravityAcceleration, ref Simulation.Context context, JobHandle inputDeps)
        {
            var buildJob = new BuildContactJacobiansJob
            {
                ContactReader = context.Contacts,
                JointJacobianReader = context.JointJacobians,
                JacobianWriter = context.Jacobians,
                TimeStep = timeStep,
                GravityAcceleration = gravityAcceleration,
                MotionDatas = world.MotionDatas,
                MotionVelocities = world.MotionVelocities
            };

            
            JobHandle handle = buildJob.ScheduleUnsafeIndex0(context.SolverSchedulerInfo.NumWorkItems, 1, inputDeps);

            context.DisposeContacts = context.Contacts.Dispose(handle);

            return handle;
        }

        // Schedule some jobs to solve the Jacobians stored in the simulation context
        public static unsafe JobHandle ScheduleSolveJacobiansJobs(ref DynamicsWorld dynamicsWorld, float timestep, int numIterations, ref Simulation.Context context, JobHandle inputDeps)
        {
            JobHandle handle;

            int numPhases = context.SolverSchedulerInfo.NumPhases;

            // Use persistent allocator to allow these to live until the start of next step
            {
                var workItemList = context.SolverSchedulerInfo.NumWorkItems;
                
                //TODO: Change this to Allocator.TempJob when https://github.com/Unity-Technologies/Unity.Physics/issues/7 is resolved
                var collisionEventStreamHandle = BlockStream.ScheduleConstruct(out context.CollisionEventStream, workItemList, 0xb17b474f, inputDeps, Allocator.Persistent);
                var triggerEventStreamHandle = BlockStream.ScheduleConstruct(out context.TriggerEventStream, workItemList, 0x43875d8f, inputDeps, Allocator.Persistent);

                handle = JobHandle.CombineDependencies(collisionEventStreamHandle, triggerEventStreamHandle);
                
                float invNumIterations = math.rcp(numIterations);

                var phaseInfoPtrs = (Scheduler.SolverSchedulerInfo.SolvePhaseInfo*)NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(context.SolverSchedulerInfo.PhaseInfo);

                for (int solverIterationId = 0; solverIterationId < numIterations; solverIterationId++)
                {
                    bool lastIteration = solverIterationId == numIterations - 1;
                    for (int phaseId = 0; phaseId < numPhases; phaseId++)
                    {
                        var job = new SolverJob
                        {
                            JacobianReader = context.Jacobians,
                            PhaseIndex = phaseId,
                            Phases = context.SolverSchedulerInfo.PhaseInfo,
                            MotionVelocities = dynamicsWorld.MotionVelocities,
                            StepInput = new StepInput
                            {
                                InvNumSolverIterations = invNumIterations,
                                IsLastIteration = lastIteration,
                                Timestep = timestep
                            }
                        };

                        // Only initialize event writers for last solver iteration jobs
                        if (lastIteration)
                        {
                            job.CollisionEventsWriter = context.CollisionEventStream;
                            job.TriggerEventsWriter = context.TriggerEventStream;
                        }

                        // NOTE: The last phase must be executed on a single job since it  
                        // int.MaxValue can't be used as batchSize since 19.1 overflows in that case... 
                        bool isLastPhase = phaseId == numPhases - 1;
                        int batchSize = isLastPhase ? (int.MaxValue / 2) : 1;
                        
                        int* numWorkItems = &(phaseInfoPtrs[phaseId].NumWorkItems);
                        handle = job.Schedule(numWorkItems, batchSize, handle);
                    }
                }
            }

            // Dispose processed data
            context.DisposeJacobians = context.Jacobians.Dispose(handle);
            context.DisposeJointJacobians = context.JointJacobians.Dispose(handle);
            context.DisposeSolverSchedulerData = context.SolverSchedulerInfo.ScheduleDisposeJob(handle);

            return handle;
        }

        #region Jobs

        [BurstCompile]
        private struct BuildContactJacobiansJob : IJobParallelForDefer
        {
            [ReadOnly] public NativeSlice<MotionData> MotionDatas;
            [ReadOnly] public NativeSlice<MotionVelocity> MotionVelocities;

            public BlockStream.Reader ContactReader;
            public BlockStream.Reader JointJacobianReader;
            public BlockStream.Writer JacobianWriter;
            public float TimeStep;
            public float GravityAcceleration;

            public void Execute(int workItemIndex)
            {
                BuildContactJacobians(ref MotionDatas, ref MotionVelocities, ref ContactReader, ref JointJacobianReader, ref JacobianWriter,
                    TimeStep, GravityAcceleration, workItemIndex);
            }
        }

        [BurstCompile]
        private struct SolverJob : IJobParallelForDefer
        {
            [NativeDisableParallelForRestriction]
            public NativeSlice<MotionVelocity> MotionVelocities;

            public BlockStream.Reader JacobianReader;
            
            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            public BlockStream.Writer CollisionEventsWriter;
            //@TODO: Unity should have a Allow null safety restriction
            [NativeDisableContainerSafetyRestriction]
            public BlockStream.Writer TriggerEventsWriter;
            
            [ReadOnly]
            public NativeArray<Scheduler.SolverSchedulerInfo.SolvePhaseInfo> Phases;
            public int PhaseIndex;
            public StepInput StepInput;

            public void Execute(int workItemIndex)
            {
                var workItemStartIndexOffset = Phases[PhaseIndex].FirstWorkItemIndex;

                CollisionEventsWriter.PatchMinMaxRange(workItemIndex + workItemStartIndexOffset);
                TriggerEventsWriter.PatchMinMaxRange(workItemIndex + workItemStartIndexOffset);
                
                Solve(MotionVelocities, ref JacobianReader, ref CollisionEventsWriter, ref TriggerEventsWriter, workItemIndex + workItemStartIndexOffset, StepInput);
            }
        }

        #endregion

        #region Implementation

        private static void BuildJacobian(MTransform worldFromA, MTransform worldFromB, float3 normal, float3 armA, float3 armB,
            float3 invInertiaA, float3 invInertiaB, float sumInvMass, out float3 angularA, out float3 angularB, out float invEffectiveMass)
        {
            float3 crossA = math.cross(armA, normal);
            angularA = math.mul(worldFromA.InverseRotation, crossA).xyz;

            float3 crossB = math.cross(normal, armB);
            angularB = math.mul(worldFromB.InverseRotation, crossB).xyz;

            float3 temp = angularA * angularA * invInertiaA + angularB * angularB * invInertiaB;
            invEffectiveMass = temp.x + temp.y + temp.z + sumInvMass;
        }

        private static void BuildContactJacobian(
            int contactPointIndex,
            float3 normal,
            MTransform worldFromA,
            MTransform worldFromB,
            float timestep,
            float invDt,
            MotionVelocity velocityA,
            MotionVelocity velocityB,
            float sumInvMass,
            ref JacobianHeader jacobianHeader,
            ref float3 centerA,
            ref float3 centerB,
            ref BlockStream.Reader contactReader)
        {
            ref ContactJacAngAndVelToReachCp jacAngular = ref jacobianHeader.AccessAngularJacobian(contactPointIndex);
            ContactPoint contact = contactReader.Read<ContactPoint>();
            float3 pointOnB = contact.Position;
            float3 pointOnA = contact.Position + normal * contact.Distance;
            float3 armA = pointOnA - worldFromA.Translation;
            float3 armB = pointOnB - worldFromB.Translation;
            float invEffectiveMass;
            BuildJacobian(worldFromA, worldFromB, normal, armA, armB, velocityA.InverseInertiaAndMass.xyz, velocityB.InverseInertiaAndMass.xyz, sumInvMass,
                out jacAngular.Jac.AngularA, out jacAngular.Jac.AngularB, out invEffectiveMass);
            jacAngular.Jac.EffectiveMass = 1.0f / invEffectiveMass;
            jacAngular.Jac.Impulse = 0.0f;

            float solveDistance = contact.Distance;
            float solveVelocity = solveDistance * invDt;

            // If contact distance is negative, use an artificially reduced penetration depth to prevent the contact from depenetrating too quickly
            const float maxDepenetrationVelocity = 3.0f; // meter/seconds time step independent
            solveVelocity = math.max(-maxDepenetrationVelocity, solveVelocity);

            jacAngular.VelToReachCp = -solveVelocity;

            // Calculate average position for friction
            centerA += armA;
            centerB += armB;
        }

        private static void InitModifierData(ref JacobianHeader jacobianHeader, ColliderKeyPair colliderKeys)
        {
            if (jacobianHeader.HasColliderKeys)
            {
                jacobianHeader.AccessColliderKeys() = colliderKeys;
            }
            if (jacobianHeader.HasSurfaceVelocity)
            {
                jacobianHeader.AccessSurfaceVelocity() = new SurfaceVelocity();
            }
            if (jacobianHeader.HasMaxImpulse)
            {
                jacobianHeader.AccessMaxImpulse() = float.MaxValue;
            }
            if (jacobianHeader.HasMassFactors)
            {
                jacobianHeader.AccessMassFactors() = MassFactors.Default;
            }
        }

        private static void GetMotions(
            BodyIndexPair pair,
            ref NativeSlice<MotionData> motionDatas,
            ref NativeSlice<MotionVelocity> motionVelocities,
            out MotionVelocity velocityA,
            out MotionVelocity velocityB,
            out MTransform worldFromA,
            out MTransform worldFromB)
        {
            bool bodyAIsStatic = pair.BodyAIndex >= motionVelocities.Length;
            bool bodyBIsStatic = pair.BodyBIndex >= motionVelocities.Length;

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
                velocityB = motionVelocities[pair.BodyBIndex];

                worldFromA = MTransform.Identity;
                worldFromB = new MTransform(motionDatas[pair.BodyBIndex].WorldFromMotion);
            }
            else if (bodyBIsStatic)
            {
                velocityA = motionVelocities[pair.BodyAIndex];
                velocityB = MotionVelocity.Zero;
                
                worldFromA = new MTransform(motionDatas[pair.BodyAIndex].WorldFromMotion);
                worldFromB = MTransform.Identity;
            }
            else
            {
                velocityA = motionVelocities[pair.BodyAIndex];
                velocityB = motionVelocities[pair.BodyBIndex];
                
                worldFromA = new MTransform(motionDatas[pair.BodyAIndex].WorldFromMotion);
                worldFromB = new MTransform(motionDatas[pair.BodyBIndex].WorldFromMotion);
            }
        }

        private static void GetMotionVelocities(
            BodyIndexPair pair,
            ref NativeSlice<MotionVelocity> motionVelocities,
            out MotionVelocity velocityA,
            out MotionVelocity velocityB)
        {
            bool bodyAIsStatic = pair.BodyAIndex >= motionVelocities.Length;
            bool bodyBIsStatic = pair.BodyBIndex >= motionVelocities.Length;

            if (bodyAIsStatic)
            {
                if (bodyBIsStatic)
                {
                    Assert.IsTrue(false); // static-static pairs should have been filtered during broadphase overlap test
                    velocityA = MotionVelocity.Zero;
                    velocityB = MotionVelocity.Zero;
                    return;
                }

                velocityA = MotionVelocity.Zero;
                velocityB = motionVelocities[pair.BodyBIndex];
            }
            else if (bodyBIsStatic)
            {
                velocityA = motionVelocities[pair.BodyAIndex];
                velocityB = MotionVelocity.Zero;
            }
            else
            {
                velocityA = motionVelocities[pair.BodyAIndex];
                velocityB = motionVelocities[pair.BodyBIndex];
            }
        }

        private static unsafe void AppendJointJacobiansToContactStream(
            int workItemIndex, ref BlockStream.Reader jointJacobianReader, ref BlockStream.Writer jacobianWriter)
        {
            JacobianIterator jacIterator = new JacobianIterator(jointJacobianReader, workItemIndex);
            while (jacIterator.HasJacobiansLeft())
            {
                short jacobianSize;
                ref JacobianHeader jacHeader = ref jacIterator.ReadJacobianHeader(out jacobianSize);
                if ((jacHeader.Flags & JacobianFlags.Disabled) == 0)
                {
                    // Allocate enough memory and copy the data
                    jacobianWriter.Write(jacobianSize);
                    byte* jacDataPtr = jacobianWriter.Allocate(jacobianSize);
                    ref JacobianHeader copiedJacobianHeader = ref UnsafeUtilityEx.AsRef<JacobianHeader>(jacDataPtr);
                    copiedJacobianHeader = jacHeader;

                    switch (jacHeader.Type)
                    {
                        case JacobianType.LinearLimit:
                            copiedJacobianHeader.AccessBaseJacobian<LinearLimitJacobian>() = jacHeader.AccessBaseJacobian<LinearLimitJacobian>();
                            break;
                        case JacobianType.AngularLimit1D:
                            copiedJacobianHeader.AccessBaseJacobian<AngularLimit1DJacobian>() = jacHeader.AccessBaseJacobian<AngularLimit1DJacobian>();
                            break;
                        case JacobianType.AngularLimit2D:
                            copiedJacobianHeader.AccessBaseJacobian<AngularLimit2DJacobian>() = jacHeader.AccessBaseJacobian<AngularLimit2DJacobian>();
                            break;
                        case JacobianType.AngularLimit3D:
                            copiedJacobianHeader.AccessBaseJacobian<AngularLimit3DJacobian>() = jacHeader.AccessBaseJacobian<AngularLimit3DJacobian>();
                            break;
                        default:
                            throw new NotImplementedException();
                    }
                }
            }
        }

        private static unsafe void BuildContactJacobians(
            ref NativeSlice<MotionData> motionDatas,
            ref NativeSlice<MotionVelocity> motionVelocities,
            ref BlockStream.Reader contactReader,
            ref BlockStream.Reader jointJacobianReader,
            ref BlockStream.Writer jacobianWriter,
            float timestep,
            float gravityAcceleration,
            int workItemIndex)
        {
            float invDt = 1.0f / timestep;

            // Contact resting velocity for restitution
            float negContactRestingVelocity = -gravityAcceleration * timestep;
            bool applyRestitution = false;

            contactReader.BeginForEachIndex(workItemIndex);
            jacobianWriter.BeginForEachIndex(workItemIndex);
            while (contactReader.RemainingItemCount > 0)
            {
                ref ContactHeader contactHeader = ref contactReader.Read<ContactHeader>();

                JacobianType jacType = ((int)(contactHeader.JacobianFlags) & (int)(JacobianFlags.IsTrigger)) != 0 ?
                    JacobianType.Trigger : JacobianType.Contact;
                JacobianFlags jacFlags = contactHeader.JacobianFlags;

                // Get the motion pair
                MotionVelocity velocityA, velocityB;
                MTransform worldFromA, worldFromB;
                GetMotions(contactHeader.BodyPair, ref motionDatas, ref motionVelocities, out velocityA, out velocityB, out worldFromA, out worldFromB);

                float sumInvMass = velocityA.InverseInertiaAndMass.w + velocityB.InverseInertiaAndMass.w;

                if (jacType == JacobianType.Contact && sumInvMass == 0)
                {
                    // Skip contacts between two infinite-mass objects
                    for (int j = 0; j < contactHeader.NumContacts; j++)
                    {
                        contactReader.Read<ContactPoint>();
                    }
                    continue;
                }

                // Write size before every jacobian
                short jacobianSize = (short)JacobianHeader.CalculateSize(jacType, jacFlags, contactHeader.NumContacts);
                jacobianWriter.Write(jacobianSize);

                // Allocate all necessary data for this jacobian
                byte* jacDataPtr = jacobianWriter.Allocate(jacobianSize);
                ref JacobianHeader jacobianHeader = ref UnsafeUtilityEx.AsRef<JacobianHeader>(jacDataPtr);
                jacobianHeader.BodyPair = contactHeader.BodyPair;
                jacobianHeader.Type = jacType;
                jacobianHeader.Flags = jacFlags;

                BaseContactJacobian baseJac = new BaseContactJacobian();
                baseJac.NumContacts = contactHeader.NumContacts;
                baseJac.Normal = contactHeader.Normal;

                if (jacobianHeader.Type == JacobianType.Contact)
                {
                    ref ContactJacobian contactJacobian = ref jacobianHeader.AccessBaseJacobian<ContactJacobian>();
                    contactJacobian.BaseJacobian = baseJac;
                    contactJacobian.CoefficientOfFriction = contactHeader.CoefficientOfFriction;
                    contactJacobian.CoefficientOfRestitution = contactHeader.CoefficientOfRestitution;

                    // Initialize modifier data (in order from JacobianModifierFlags) before angular jacobians
                    InitModifierData(ref jacobianHeader, contactHeader.ColliderKeys);

                    // Build normal jacobians
                    float3 centerA = new float3(0.0f);
                    float3 centerB = new float3(0.0f);
                    for (int j = 0; j < contactHeader.NumContacts; j++)
                    {
                        // Build the jacobian
                        BuildContactJacobian(
                            j, contactJacobian.BaseJacobian.Normal, worldFromA, worldFromB, timestep, invDt, velocityA, velocityB, sumInvMass,
                            ref jacobianHeader, ref centerA, ref centerB, ref contactReader);

                        // Restitution (optional)
                        if (contactJacobian.CoefficientOfRestitution > 0.0f)
                        {
                            ref ContactJacAngAndVelToReachCp jacAngular = ref jacobianHeader.AccessAngularJacobian(j);
                            float relativeVelocity = BaseContactJacobian.GetJacVelocity(baseJac.Normal, jacAngular.Jac, velocityA, velocityB);
                            float dv = jacAngular.VelToReachCp - relativeVelocity;
                            if (dv > 0.0f && relativeVelocity < negContactRestingVelocity)
                            {
                                float restitutionVelocity = (relativeVelocity - negContactRestingVelocity) * contactJacobian.CoefficientOfRestitution;
                                jacAngular.VelToReachCp =
                                    math.max(jacAngular.VelToReachCp + restitutionVelocity * timestep, 0.0f) -
                                    restitutionVelocity;

                                // Remember that restitution should be applied
                                applyRestitution = true;
                            }
                        }
                    }

                    // Build friction jacobians
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
                        CalculatePerpendicularNormalized(contactJacobian.BaseJacobian.Normal, out float3 frictionDir0, out float3 frictionDir1);

                        // Build linear jacobian
                        float invEffectiveMass0, invEffectiveMass1;
                        {
                            float3 armA = centerA;
                            float3 armB = centerB;
                            BuildJacobian(worldFromA, worldFromB, frictionDir0, armA, armB, velocityA.InverseInertiaAndMass.xyz, velocityB.InverseInertiaAndMass.xyz, sumInvMass,
                                out contactJacobian.Friction0.AngularA, out contactJacobian.Friction0.AngularB, out invEffectiveMass0);
                            BuildJacobian(worldFromA, worldFromB, frictionDir1, armA, armB, velocityA.InverseInertiaAndMass.xyz, velocityB.InverseInertiaAndMass.xyz, sumInvMass,
                                out contactJacobian.Friction1.AngularA, out contactJacobian.Friction1.AngularB, out invEffectiveMass1);
                        }

                        // Build angular jacobian
                        float invEffectiveMassAngular;
                        {
                            contactJacobian.AngularFriction.AngularA = math.mul(worldFromA.InverseRotation, contactJacobian.BaseJacobian.Normal);
                            contactJacobian.AngularFriction.AngularB = math.mul(worldFromB.InverseRotation, -contactJacobian.BaseJacobian.Normal);
                            float3 temp = contactJacobian.AngularFriction.AngularA * contactJacobian.AngularFriction.AngularA * velocityA.InverseInertiaAndMass.xyz;
                            temp += contactJacobian.AngularFriction.AngularB * contactJacobian.AngularFriction.AngularB * velocityB.InverseInertiaAndMass.xyz;
                            invEffectiveMassAngular = math.csum(temp);
                        }

                        // Build effective mass
                        {
                            // Build the inverse effective mass matrix
                            float3 invEffectiveMassDiag = new float3(invEffectiveMass0, invEffectiveMass1, invEffectiveMassAngular);
                            float3 invEffectiveMassOffDiag = new float3( // (0, 1), (0, 2), (1, 2)
                                JacobianUtilities.CalculateInvEffectiveMassOffDiag(contactJacobian.Friction0.AngularA, contactJacobian.Friction1.AngularA, velocityA.InverseInertiaAndMass.xyz,
                                contactJacobian.Friction0.AngularB, contactJacobian.Friction1.AngularB, velocityB.InverseInertiaAndMass.xyz),
                                JacobianUtilities.CalculateInvEffectiveMassOffDiag(contactJacobian.Friction0.AngularA, contactJacobian.AngularFriction.AngularA, velocityA.InverseInertiaAndMass.xyz,
                                contactJacobian.Friction0.AngularB, contactJacobian.AngularFriction.AngularB, velocityB.InverseInertiaAndMass.xyz),
                                JacobianUtilities.CalculateInvEffectiveMassOffDiag(contactJacobian.Friction1.AngularA, contactJacobian.AngularFriction.AngularA, velocityA.InverseInertiaAndMass.xyz,
                                contactJacobian.Friction1.AngularB, contactJacobian.AngularFriction.AngularB, velocityB.InverseInertiaAndMass.xyz));

                            // Invert the matrix and store it to the jacobians
                            float3 effectiveMassDiag, effectiveMassOffDiag;
                            if (!JacobianUtilities.InvertSymmetricMatrix(invEffectiveMassDiag, invEffectiveMassOffDiag, out effectiveMassDiag, out effectiveMassOffDiag))
                            {
                                // invEffectiveMass can be singular if the bodies have infinite inertia about the normal.
                                // In that case angular friction does nothing so we can regularize the matrix, set col2 = row2 = (0, 0, 1)
                                invEffectiveMassOffDiag.y = 0.0f;
                                invEffectiveMassOffDiag.z = 0.0f;
                                invEffectiveMassDiag.z = 1.0f;
                                bool success = JacobianUtilities.InvertSymmetricMatrix(invEffectiveMassDiag, invEffectiveMassOffDiag, out effectiveMassDiag, out effectiveMassOffDiag);
                                Assert.IsTrue(success); // it should never fail, if it does then friction will be disabled
                            }
                            contactJacobian.Friction0.EffectiveMass = effectiveMassDiag.x;
                            contactJacobian.Friction1.EffectiveMass = effectiveMassDiag.y;
                            contactJacobian.AngularFriction.EffectiveMass = effectiveMassDiag.z;
                            contactJacobian.FrictionEffectiveMassOffDiag = effectiveMassOffDiag;
                        }

                        // Reduce friction to 1/4 of the impulse if there will be restitution
                        if (applyRestitution)
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

                    // Build normal jacobians
                    float3 centerA = new float3(0.0f);
                    float3 centerB = new float3(0.0f);
                    for (int j = 0; j < contactHeader.NumContacts; j++)
                    {
                        // Build the jacobian
                        BuildContactJacobian(
                            j, triggerJacobian.BaseJacobian.Normal, worldFromA, worldFromB, timestep, invDt, velocityA, velocityB, sumInvMass,
                            ref jacobianHeader, ref centerA, ref centerB, ref contactReader);
                    }
                }
            }

            // Copy joint jacobians into the main jacobian stream
            AppendJointJacobiansToContactStream(workItemIndex, ref jointJacobianReader, ref jacobianWriter);

            contactReader.EndForEachIndex();
            jacobianWriter.EndForEachIndex();
        }

        public static unsafe void BuildJointJacobian(JointData* jointData, BodyIndexPair pair,
            MotionVelocity velocityA, MotionVelocity velocityB, MotionData motionA, MotionData motionB,
            float timestep, int numIterations, ref BlockStream.Writer jacobianWriter)
        {
            MTransform bodyAFromMotionA = new MTransform(motionA.BodyFromMotion);
            MTransform motionAFromJoint = Mul(Inverse(bodyAFromMotionA), jointData->AFromJoint);

            MTransform bodyBFromMotionB = new MTransform(motionB.BodyFromMotion);
            MTransform motionBFromJoint = Mul(Inverse(bodyBFromMotionB), jointData->BFromJoint);

            for (int i = 0; i < jointData->NumConstraints; i++)
            {
                Constraint constraint = jointData->Constraints[i];

                JacobianType jacType;
                switch (constraint.Type)
                {
                    case ConstraintType.Linear:
                        jacType = JacobianType.LinearLimit;
                        break;
                    case ConstraintType.Angular:
                        switch (constraint.Dimension)
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
                                throw new NotImplementedException();
                        }
                        break;
                    default:
                        throw new NotImplementedException();
                }

                // Write size before every jacobian
                JacobianFlags jacFlags = 0;
                short jacobianSize = (short)JacobianHeader.CalculateSize(jacType, jacFlags);
                jacobianWriter.Write(jacobianSize);

                // Allocate all necessary data for this jacobian
                byte* jacDataPtr = jacobianWriter.Allocate(jacobianSize);
                ref JacobianHeader header = ref UnsafeUtilityEx.AsRef<JacobianHeader>(jacDataPtr);
                header.BodyPair = pair;
                header.Type = jacType;
                header.Flags = jacFlags;

                JacobianUtilities.CalculateTauAndDamping(constraint, timestep, numIterations, out float tau, out float damping);

                // Build the Jacobian
                switch (constraint.Type)
                {
                    case ConstraintType.Linear:
                        header.AccessBaseJacobian<LinearLimitJacobian>().Build(
                            motionAFromJoint, motionBFromJoint,
                            velocityA, velocityB, motionA, motionB, constraint, tau, damping);
                        break;
                    case ConstraintType.Angular:
                        switch (constraint.Dimension)
                        {
                            case 1:
                                header.AccessBaseJacobian<AngularLimit1DJacobian>().Build(
                                    motionAFromJoint, motionBFromJoint,
                                    velocityA, velocityB, motionA, motionB, constraint, tau, damping);
                                break;
                            case 2:
                                header.AccessBaseJacobian<AngularLimit2DJacobian>().Build(
                                    motionAFromJoint, motionBFromJoint,
                                    velocityA, velocityB, motionA, motionB, constraint, tau, damping);
                                break;
                            case 3:
                                header.AccessBaseJacobian<AngularLimit3DJacobian>().Build(
                                    motionAFromJoint, motionBFromJoint,
                                    velocityA, velocityB, motionA, motionB, constraint, tau, damping);
                                break;
                            default:
                                throw new NotImplementedException();
                        }
                        break;
                    default:
                        throw new NotImplementedException();
                }
            }
        }

        private static void Solve(
            NativeSlice<MotionVelocity> motionVelocities,
            ref BlockStream.Reader jacobianReader,
            ref BlockStream.Writer collisionEventsWriter,
            ref BlockStream.Writer triggerEventsWriter,
            int workItemIndex,
            StepInput stepInput)
        {
            if (stepInput.IsLastIteration)
            {
                collisionEventsWriter.BeginForEachIndex(workItemIndex);
                triggerEventsWriter.BeginForEachIndex(workItemIndex);
            }

            JacobianIterator jacIterator = new JacobianIterator(jacobianReader, workItemIndex);
            while (jacIterator.HasJacobiansLeft())
            {
                ref JacobianHeader header = ref jacIterator.ReadJacobianHeader();

                // Get the motion pair
                GetMotionVelocities(header.BodyPair, ref motionVelocities, out MotionVelocity velocityA, out MotionVelocity velocityB);

                // Solve the jacobian
                header.Solve(ref velocityA, ref velocityB, stepInput, ref collisionEventsWriter, ref triggerEventsWriter);

                // Write back velocity for dynamic bodies
                if (header.BodyPair.BodyAIndex < motionVelocities.Length)
                {
                    motionVelocities[header.BodyPair.BodyAIndex] = velocityA;
                }
                if (header.BodyPair.BodyBIndex < motionVelocities.Length)
                {
                    motionVelocities[header.BodyPair.BodyBIndex] = velocityB;
                }
            }

            if (stepInput.IsLastIteration)
            {
                collisionEventsWriter.EndForEachIndex();
                triggerEventsWriter.EndForEachIndex();
            }
        }

        #endregion
    }
}
