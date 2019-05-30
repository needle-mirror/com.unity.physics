using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;

namespace Unity.Physics
{
    // Interface for jobs that iterate through the list of potentially overlapping body pairs produced by the broad phase
    public interface IBodyPairsJob
    {
        void Execute(ref ModifiableBodyPair pair);
    }

    public struct ModifiableBodyPair
    {
        public EntityPair Entities { get; internal set; }
        public BodyIndexPair BodyIndices { get; internal set; }

        public void Disable()
        {
            BodyIndices = BodyIndexPair.Invalid;
        }
    }

    public static class IBodyPairsJobExtensions
    {
#if !HAVOK_PHYSICS_EXISTS
        // Default IBodyPairsJob.Schedule() implementation
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IBodyPairsJob
        {
            return ScheduleImpl(jobData, simulation, ref world, inputDeps);
        }
#else
        // In this case IBodyPairsJob.Schedule() is provided by the Havok.Physics assembly.
        // This is a stub to catch when that assembly is missing.
        //<todo.eoin.modifier Put in a link to documentation for this
        [Obsolete("This error occurs when HAVOK_PHYSICS_EXISTS is defined but Havok.Physics is missing from your package's asmdef references", true)]
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps,
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF _causeCompileError = HAVOK_PHYSICS_MISSING_FROM_ASMDEF.HAVOK_PHYSICS_MISSING_FROM_ASMDEF)
            where T : struct, IBodyPairsJob
        {
            return new JobHandle();
        }

        public enum HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        {
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        }
#endif

        internal static unsafe JobHandle ScheduleImpl<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IBodyPairsJob
        {
            if (simulation.Type == SimulationType.UnityPhysics)
            {
                var data = new BodyPairsJobData<T>
                {
                    UserJobData = jobData,
                    PhasedDispatchPairs = ((Simulation)simulation).m_Context.PhasedDispatchPairs.AsDeferredJobArray(),
                    Bodies = world.Bodies
                };
                var parameters = new JobsUtility.JobScheduleParameters(
                    UnsafeUtility.AddressOf(ref data),
                    BodyPairsJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
                return JobsUtility.Schedule(ref parameters);
            }
            return inputDeps;
        }

        private struct BodyPairsJobData<T> where T : struct
        {
            public T UserJobData;
            public NativeArray<Scheduler.DispatchPair> PhasedDispatchPairs;
            //Need to disable aliasing restriction in case T has a NativeSlice of PhysicsWorld.Bodies:
            [ReadOnly] [NativeDisableContainerSafetyRestriction] public NativeSlice<RigidBody> Bodies;
        }

        private struct BodyPairsJobProcess<T> where T : struct, IBodyPairsJob
        {
            static IntPtr jobReflectionData;

            public static IntPtr Initialize()
            {
                if (jobReflectionData == IntPtr.Zero)
                {
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(BodyPairsJobData<T>),
                        typeof(T), JobType.Single, (ExecuteJobFunction)Execute);
                }
                return jobReflectionData;
            }

            public delegate void ExecuteJobFunction(ref BodyPairsJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

            public unsafe static void Execute(ref BodyPairsJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
            {
                int currentIdx = 0;
                while (currentIdx < jobData.PhasedDispatchPairs.Length)
                {
                    var dispatchPair = jobData.PhasedDispatchPairs[currentIdx];
                    var pair = new ModifiableBodyPair
                    {
                        BodyIndices = new BodyIndexPair { BodyAIndex = dispatchPair.BodyAIndex, BodyBIndex = dispatchPair.BodyBIndex },
                        Entities = new EntityPair
                        {
                            EntityA = jobData.Bodies[dispatchPair.BodyAIndex].Entity,
                            EntityB = jobData.Bodies[dispatchPair.BodyBIndex].Entity
                        }
                    };

                    jobData.UserJobData.Execute(ref pair);

                    if (pair.BodyIndices.BodyAIndex == -1 || pair.BodyIndices.BodyBIndex == -1)
                    {
                        jobData.PhasedDispatchPairs[currentIdx] = Scheduler.DispatchPair.Invalid;
                    }

                    do
                    {
                        currentIdx++;
                    } while (currentIdx < jobData.PhasedDispatchPairs.Length && jobData.PhasedDispatchPairs[currentIdx].IsJoint);
                }
            }
        }
    }
}
