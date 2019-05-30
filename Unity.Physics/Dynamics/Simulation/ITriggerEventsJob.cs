using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;

namespace Unity.Physics
{
    // An event raised when a pair of bodies involving a trigger material have overlapped during solving.
    public struct TriggerEvent
    {
        internal LowLevel.TriggerEvent EventData;
        public EntityPair Entities { get; internal set; }
        public BodyIndexPair BodyIndices => EventData.BodyIndices;
        public ColliderKeyPair ColliderKeys => EventData.ColliderKeys;
    }

    // Interface for jobs that iterate through the list of trigger events produced by the solver.
    public interface ITriggerEventsJob
    {
        void Execute(TriggerEvent triggerEvent);
    }

    public static class ITriggerEventJobExtensions
    {
#if !HAVOK_PHYSICS_EXISTS
        // Default IContactsJob.Schedule() implementation.
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, ITriggerEventsJob
        {
            return ScheduleImpl(jobData, simulation, ref world, inputDeps);
        }
#else
        // In this case IContactsJob.Schedule() is provided by the Havok.Physics assembly.
        // This is a stub to catch when that assembly is missing.
        //<todo.eoin.modifier Put in a link to documentation for this:
        [Obsolete("This error occurs when HAVOK_PHYSICS_EXISTS is defined but Havok.Physics is missing from your package's asmdef references", true)]
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps,
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF _causeCompileError = HAVOK_PHYSICS_MISSING_FROM_ASMDEF.HAVOK_PHYSICS_MISSING_FROM_ASMDEF)
            where T : struct, ITriggerEventsJob
        {
            return new JobHandle();
        }

        public enum HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        {
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        }
#endif

        internal static unsafe JobHandle ScheduleImpl<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, ITriggerEventsJob
        {
            if (simulation.Type == SimulationType.UnityPhysics)
            {
                var data = new TriggerEventJobData<T>
                {
                    UserJobData = jobData,
                    EventReader = ((Simulation)simulation).TriggerEvents,
                    Bodies = world.Bodies
                };
                var parameters = new JobsUtility.JobScheduleParameters(UnsafeUtility.AddressOf(ref data), TriggerEventJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
                return JobsUtility.Schedule(ref parameters);
            }
            return inputDeps;
        }

        private unsafe struct TriggerEventJobData<T> where T : struct
        {
            public T UserJobData;
            [NativeDisableContainerSafetyRestriction] public LowLevel.TriggerEvents EventReader;
            //Need to disable aliasing restriction in case T has a NativeSlice of PhysicsWorld.Bodies:
            [ReadOnly] [NativeDisableContainerSafetyRestriction] public NativeSlice<RigidBody> Bodies;
        }

        private struct TriggerEventJobProcess<T> where T : struct, ITriggerEventsJob
        {
            static IntPtr jobReflectionData;

            public static IntPtr Initialize()
            {
                if (jobReflectionData == IntPtr.Zero)
                {
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(TriggerEventJobData<T>),
                        typeof(T), JobType.Single, (ExecuteJobFunction)Execute);
                }
                return jobReflectionData;
            }

            public delegate void ExecuteJobFunction(ref TriggerEventJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

            public unsafe static void Execute(ref TriggerEventJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
            {
                foreach(LowLevel.TriggerEvent te in jobData.EventReader)
                {
                    TriggerEvent evt = new TriggerEvent
                    {
                        EventData = te,
                        Entities = new EntityPair
                        {
                            EntityA = jobData.Bodies[te.BodyIndices.BodyAIndex].Entity,
                            EntityB = jobData.Bodies[te.BodyIndices.BodyBIndex].Entity
                        }
                    };
                    jobData.UserJobData.Execute(evt);
                }
            }
        }
    }
}
