using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;

namespace Unity.Physics
{
    // INTERNAL UnityPhysics interface for jobs that iterate through the list of potentially overlapping body pairs produced by the broad phase
    // Important: Only use inside UnityPhysics code! Jobs in other projects should implement IBodyPairsJob.
    [JobProducerType(typeof(IBodyPairsJobExtensions.BodyPairsJobProcess<>))]
    public interface IBodyPairsJobBase
    {
        void Execute(ref ModifiableBodyPair pair);
    }

#if !HAVOK_PHYSICS_EXISTS

    // Interface for jobs that iterate through the list of potentially overlapping body pairs produced by the broad phase
    public interface IBodyPairsJob : IBodyPairsJobBase
    {
    }

#endif

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
        // Default Schedule() implementation for IBodyPairsJob
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IBodyPairsJobBase
        {
            // Should work only for UnityPhysics
            if (simulation.Type != SimulationType.UnityPhysics)
            {
                return inputDeps;
            }

            return ScheduleUnityPhysicsBodyPairsJob(jobData, simulation, ref world, inputDeps);
        }
#else
        // In this case Schedule() implementation for IBodyPairsJob is provided by the Havok.Physics assembly.
        // This is a stub to catch when that assembly is missing.
        //<todo.eoin.modifier Put in a link to documentation for this
        [Obsolete("This error occurs when HAVOK_PHYSICS_EXISTS is defined but Havok.Physics is missing from your package's asmdef references. (DoNotRemove)", true)]
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps,
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF _causeCompileError = HAVOK_PHYSICS_MISSING_FROM_ASMDEF.HAVOK_PHYSICS_MISSING_FROM_ASMDEF)
            where T : struct, IBodyPairsJobBase
        {
            return new JobHandle();
        }

        public enum HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        {
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        }
#endif

        internal static unsafe JobHandle ScheduleUnityPhysicsBodyPairsJob<T>(T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, IBodyPairsJobBase
        {
            if (simulation.Type != SimulationType.UnityPhysics)
            {
                throw new ArgumentException($"Simulation type {simulation.Type} is not supported! Should be called only for SimulationType.UnityPhysics.");
            }

            var data = new BodyPairsJobData<T>
            {
                UserJobData = jobData,
                PhasedDispatchPairs = ((Simulation)simulation).StepContext.PhasedDispatchPairs.AsDeferredJobArray(),
                Bodies = world.Bodies
            };
            var parameters = new JobsUtility.JobScheduleParameters(
                UnsafeUtility.AddressOf(ref data),
                BodyPairsJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
            return JobsUtility.Schedule(ref parameters);
        }

        internal struct BodyPairsJobData<T> where T : struct
        {
            public T UserJobData;
            public NativeArray<DispatchPairSequencer.DispatchPair> PhasedDispatchPairs;
            //Need to disable aliasing restriction in case T has a NativeSlice of PhysicsWorld.Bodies:
            [ReadOnly] [NativeDisableContainerSafetyRestriction] public NativeSlice<RigidBody> Bodies;
        }

        internal struct BodyPairsJobProcess<T> where T : struct, IBodyPairsJobBase
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
                    DispatchPairSequencer.DispatchPair dispatchPair = jobData.PhasedDispatchPairs[currentIdx];
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
                        jobData.PhasedDispatchPairs[currentIdx] = DispatchPairSequencer.DispatchPair.Invalid;
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
