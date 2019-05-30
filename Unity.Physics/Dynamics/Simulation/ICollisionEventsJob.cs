using System;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Unity.Physics
{
    // An event raised when a pair of bodies have collided during solving.
    public struct CollisionEvent
    {
        internal LowLevel.CollisionEvent EventData;
        public EntityPair Entities { get; internal set; }
        public BodyIndexPair BodyIndices => EventData.BodyIndices;
        public ColliderKeyPair ColliderKeys => EventData.ColliderKeys;
        public float3 Normal => EventData.Normal;
        public float4 AccumulatedImpulses => EventData.AccumulatedImpulses;
    }

    // Interface for jobs that iterate through the list of collision events produced by the solver.
    public interface ICollisionEventsJob
    {
        void Execute(CollisionEvent collisionEvent);
    }

    public static class ICollisionEventJobExtensions
    {
#if !HAVOK_PHYSICS_EXISTS
        // Default IContactsJob.Schedule() implementation.
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, ICollisionEventsJob
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
            where T : struct, ICollisionEventsJob
        {
            return new JobHandle();
        }

        public enum HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        {
            HAVOK_PHYSICS_MISSING_FROM_ASMDEF
        }
#endif

        internal static unsafe JobHandle ScheduleImpl<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, ICollisionEventsJob
        {
            if (simulation.Type == SimulationType.UnityPhysics)
            {
                var data = new CollisionEventJobData<T>
                {
                    UserJobData = jobData,
                    EventReader = ((Simulation)simulation).CollisionEvents,
                    Bodies = world.Bodies
                };
                var parameters = new JobsUtility.JobScheduleParameters(UnsafeUtility.AddressOf(ref data), CollisionEventJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
                return JobsUtility.Schedule(ref parameters);
            }
            return inputDeps;
        }

        private unsafe struct CollisionEventJobData<T> where T : struct
        {
            public T UserJobData;
            [NativeDisableContainerSafetyRestriction] public LowLevel.CollisionEvents EventReader;
            //Need to disable aliasing restriction in case T has a NativeSlice of PhysicsWorld.Bodies:
            [ReadOnly] [NativeDisableContainerSafetyRestriction] public NativeSlice<RigidBody> Bodies;
        }

        private struct CollisionEventJobProcess<T> where T : struct, ICollisionEventsJob
        {
            static IntPtr jobReflectionData;

            public static IntPtr Initialize()
            {
                if (jobReflectionData == IntPtr.Zero)
                {
                    jobReflectionData = JobsUtility.CreateJobReflectionData(typeof(CollisionEventJobData<T>),
                        typeof(T), JobType.Single, (ExecuteJobFunction)Execute);
                }
                return jobReflectionData;
            }

            public delegate void ExecuteJobFunction(ref CollisionEventJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex);

            public unsafe static void Execute(ref CollisionEventJobData<T> jobData, IntPtr additionalData,
                IntPtr bufferRangePatchData, ref JobRanges ranges, int jobIndex)
            {
                foreach(LowLevel.CollisionEvent ce in jobData.EventReader)
                {
                    CollisionEvent evt = new CollisionEvent
                    {
                        EventData = ce,
                        Entities = new EntityPair
                        {
                            EntityA = jobData.Bodies[ce.BodyIndices.BodyAIndex].Entity,
                            EntityB = jobData.Bodies[ce.BodyIndices.BodyBIndex].Entity
                        }
                    };
                    jobData.UserJobData.Execute(evt);
                }
            }
        }
    }
}
