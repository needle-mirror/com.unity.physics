using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.Physics
{
    // Interface for jobs that iterate through the list of Jacobians before they are solved
    public interface IJacobiansJob
    {
        // Note, multiple Jacobians can share the same header.
        void Execute(ref ModifiableJacobianHeader header, ref ModifiableContactJacobian jacobian);
        void Execute(ref ModifiableJacobianHeader header, ref ModifiableTriggerJacobian jacobian);
    }

    public unsafe struct ModifiableJacobianHeader
    {
        internal JacobianHeader* m_Header;
        public bool ModifiersChanged { get; private set; }
        public bool AngularChanged { get; private set; }

        public EntityPair Entities { get; internal set; }
        public BodyIndexPair BodyPair => m_Header->BodyPair;
        public JacobianType Type => m_Header->Type;
        public JacobianFlags Flags
        {
            get => m_Header->Flags;
            set
            {
                // Some flags change the size of the jacobian; don't allow these to be changed:
                byte notPermitted = (byte)(JacobianFlags.EnableSurfaceVelocity | JacobianFlags.EnableMassFactors | JacobianFlags.EnableMassFactors);
                byte userFlags = (byte)value;
                byte alreadySet = (byte)m_Header->Flags;

                if((notPermitted & (userFlags ^ alreadySet)) != 0)
                {
                    throw new NotSupportedException("Cannot change flags which alter jacobian size");
                }

                m_Header->Flags = value;
            }
        }
        public bool HasColliderKeys => m_Header->HasColliderKeys;
        public ColliderKeyPair ColliderKeys => m_Header->ColliderKeys;

        public bool HasMassFactors => m_Header->HasMassFactors;
        public MassFactors MassFactors
        {
            get => m_Header->MassFactors;
            set
            {
                m_Header->MassFactors = value;
                ModifiersChanged = true;
            }
        }

        public bool HasSurfaceVelocity => m_Header->HasSurfaceVelocity;
        public SurfaceVelocity SurfaceVelocity
        {
            get => m_Header->SurfaceVelocity;
            set
            {
                m_Header->SurfaceVelocity = value;
                ModifiersChanged = true;
            }
        }

        public bool HasMaxImpulse => m_Header->HasMaxImpulse;
        public float MaxImpulse
        {
            get => m_Header->MaxImpulse;
            set
            {
                m_Header->MaxImpulse = value;
                ModifiersChanged = true;
            }
        }

        public ContactJacAngAndVelToReachCp GetAngularJacobian(int i)
        {
            return m_Header->AccessAngularJacobian(i);
        }

        public void SetAngularJacobian(int i, ContactJacAngAndVelToReachCp j)
        {
            m_Header->AccessAngularJacobian(i) = j;
            AngularChanged = true;
        }
    }

    public unsafe struct ModifiableContactJacobian
    {
        internal ContactJacobian* m_ContactJacobian;
        public bool Modified { get; private set; }

        public int NumContacts => m_ContactJacobian->BaseJacobian.NumContacts;

        public float3 Normal
        {
            get => m_ContactJacobian->BaseJacobian.Normal;
            set
            {
                m_ContactJacobian->BaseJacobian.Normal = value;
                Modified = true;
            }
        }

        public float CoefficientOfFriction
        {
            get => m_ContactJacobian->CoefficientOfFriction;
            set
            {
                m_ContactJacobian->CoefficientOfFriction = value;
                Modified = true;
            }
        }

        public float CoefficientOfRestitution
        {
            get => m_ContactJacobian->CoefficientOfRestitution;
            set
            {
                m_ContactJacobian->CoefficientOfRestitution = value;
                Modified = true;
            }
        }

        // Angular friction about the contact normal, no linear part
        public float3 FrictionEffectiveMassOffDiag
        {
            get => m_ContactJacobian->FrictionEffectiveMassOffDiag;
            set
            {
                m_ContactJacobian->FrictionEffectiveMassOffDiag = value;
                Modified = true;
            }
        }

        public ContactJacobianAngular Friction0
        {
            get => m_ContactJacobian->Friction0;
            set
            {
                m_ContactJacobian->Friction0 = value;
                Modified = true;
            }
        }

        public ContactJacobianAngular Friction1
        {
            get => m_ContactJacobian->Friction1;
            set
            {
                m_ContactJacobian->Friction1 = value;
                Modified = true;
            }
        }

        public ContactJacobianAngular AngularFriction
        {
            get => m_ContactJacobian->AngularFriction;
            set
            {
                m_ContactJacobian->AngularFriction = value;
                Modified = true;
            }
        }
    }

    public struct ModifiableTriggerJacobian
    {
        internal unsafe TriggerJacobian* m_TriggerJacobian;
    }

    public static class IJacobiansJobExtensions
    {
#if !HAVOK_PHYSICS_EXISTS
        // Default IJacobiansJob.Schedule() implementation.
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IJacobiansJob
        {
            return ScheduleImpl(jobData, simulation, ref world, inputDeps);
        }
#else
        // In this case IJacobiansJob.Schedule() is provided by the Havok.Physics assembly.
        // This is a stub to catch when that assembly is missing.
        //<todo.eoin.modifier Put in a link to documentation for this:
        [Obsolete("This error occurs when HAVOK_PHYSICS_EXISTS is defined but Havok.Physics is missing from your package's asmdef references", true)]
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps,
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF _causeCompileError = HAVOK_PHYSICS_MISSING_FROM_ASMDEF.HAVOK_PHYSICS_MISSING_FROM_ASMDEF)
            where T : struct, IJacobiansJob
        {
            return new JobHandle();
        }

        public enum HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        {
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        }
#endif

        internal static unsafe JobHandle ScheduleImpl<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IJacobiansJob
        {
            if (simulation.Type == SimulationType.UnityPhysics)
            {
                var data = new JacobiansJobData<T>
                {
                    UserJobData = jobData,
                    StreamReader = ((Simulation)simulation).m_Context.Jacobians,
                    NumWorkItems = ((Simulation)simulation).m_Context.SolverSchedulerInfo.NumWorkItems,
                    Bodies = world.Bodies
                };
                var parameters = new JobsUtility.JobScheduleParameters(
                    UnsafeUtility.AddressOf(ref data),
                    JacobiansJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
                return JobsUtility.Schedule(ref parameters);
            }
            return inputDeps;
        }

        private unsafe struct JacobiansJobData<T> where T : struct
        {
            public T UserJobData;
            public BlockStream.Reader StreamReader;

            [ReadOnly]
            public NativeArray<int> NumWorkItems;
            
            //Need to disable aliasing restriction in case T has a NativeSlice of PhysicsWorld.Bodies:
            [ReadOnly] [NativeDisableContainerSafetyRestriction] public NativeSlice<RigidBody> Bodies;

            int m_CurrentWorkItem;

            public bool HasItemsLeft => StreamReader.RemainingItemCount > 0;

            public JacobianHeader* ReadJacobianHeader()
            {
                short readSize = Read<short>();
                return (JacobianHeader*)Read(readSize);
            }

            private byte* Read(int size)
            {
                byte* dataPtr = StreamReader.Read(size);
                MoveReaderToNextForEachIndex();
                return dataPtr;
            }

            private ref T2 Read<T2>() where T2 : struct
            {
                int size = UnsafeUtility.SizeOf<T2>();
                return ref UnsafeUtilityEx.AsRef<T2>(Read(size));
            }

            public void MoveReaderToNextForEachIndex()
            {
                int numWorkItems = NumWorkItems[0];
                while (StreamReader.RemainingItemCount == 0 && m_CurrentWorkItem < numWorkItems)
                {
                    StreamReader.BeginForEachIndex(m_CurrentWorkItem);
                    m_CurrentWorkItem++;
                }
            }
        }

        private struct JacobiansJobProcess<T> where T : struct, IJacobiansJob
        {
            static IntPtr jobReflectionData;

            public static IntPtr Initialize()
            {
                if (jobReflectionData == IntPtr.Zero)
                {
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(JacobiansJobData<T>),
                        typeof(T), JobType.Single, (ExecuteJobFunction)Execute);
                }
                return jobReflectionData;
            }

            public delegate void ExecuteJobFunction(ref JacobiansJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

            public unsafe static void Execute(ref JacobiansJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
            {
                jobData.MoveReaderToNextForEachIndex();
                while (jobData.HasItemsLeft)
                {
                    JacobianHeader* header = jobData.ReadJacobianHeader();

                    var h = new ModifiableJacobianHeader
                    {
                        m_Header = header,
                        Entities = new EntityPair
                        {
                            EntityA = jobData.Bodies[header->BodyPair.BodyAIndex].Entity,
                            EntityB = jobData.Bodies[header->BodyPair.BodyBIndex].Entity
                        }
                    };
                    if (header->Type == JacobianType.Contact)
                    {
                        var contact = new ModifiableContactJacobian
                        {
                            m_ContactJacobian = (ContactJacobian*)UnsafeUtility.AddressOf(ref header->AccessBaseJacobian<ContactJacobian>())
                        };
                        jobData.UserJobData.Execute(ref h, ref contact);
                    }
                    else if (header->Type == JacobianType.Trigger)
                    {
                        var trigger = new ModifiableTriggerJacobian
                        {
                            m_TriggerJacobian = (TriggerJacobian*)UnsafeUtility.AddressOf(ref header->AccessBaseJacobian<TriggerJacobian>())
                        };

                        jobData.UserJobData.Execute(ref h, ref trigger);
                    }
                }
            }
        }
    }
}
