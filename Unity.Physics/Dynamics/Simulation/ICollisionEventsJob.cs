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
        internal float TimeStep;
        internal Velocity InputVelocityA;
        internal Velocity InputVelocityB;
        internal NativeArray<ContactPoint> NarrowPhaseContactPoints;

        public EntityPair Entities { get; internal set; }

        public BodyIndexPair BodyIndices => EventData.BodyIndices;
        public ColliderKeyPair ColliderKeys => EventData.ColliderKeys;
        public float3 Normal => EventData.Normal;

        // Calculate extra details about the collision.
        // Note: Since the solver does not naturally produce this data, it requires some computation.
        public Details CalculateDetails(ref PhysicsWorld physicsWorld)
        {
            return EventData.CalculateDetails(ref physicsWorld, TimeStep, InputVelocityA, InputVelocityB, NarrowPhaseContactPoints);
        }
        
        // Extra details about a collision
        public struct Details
        {
            // Estimated contact point positions (in world space).
            // Use this information about individual contact point positions
            // to apply custom logic, for example looking at the Length
            // to differentiate between vertex (1 point), edge (2 point)
            // or face (3 or more points) collision.
            public NativeArray<float3> EstimatedContactPointPositions;

            // Estimated total impulse applied
            public float EstimatedImpulse;

            // Calculate the average contact point position
            public float3 AverageContactPointPosition
            {
                get
                {
                    var pos = float3.zero;
                    for (int i = 0; i < EstimatedContactPointPositions.Length; i++)
                    {
                        pos += EstimatedContactPointPositions[i];
                    }
                    return pos / EstimatedContactPointPositions.Length;
                }
            }
        }
    }

    // Interface for jobs that iterate through the list of collision events produced by the solver.
    [JobProducerType(typeof(ICollisionEventJobExtensions.CollisionEventJobProcess<>))]
    public interface ICollisionEventsJob
    {
        void Execute(CollisionEvent collisionEvent);
    }

    public static class ICollisionEventJobExtensions
    {
#if !HAVOK_PHYSICS_EXISTS
        // Default ICollisionEventsJob.Schedule() implementation.
        public static unsafe JobHandle Schedule<T>(this T jobData, ISimulation simulation, ref PhysicsWorld world, JobHandle inputDeps)
            where T : struct, ICollisionEventsJob
        {
            return ScheduleImpl(jobData, simulation, ref world, inputDeps);
        }
#else
        // In this case ICollisionEventsJob.Schedule() is provided by the Havok.Physics assembly.
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
                    Bodies = world.Bodies,
                    TimeStep = ((Simulation)simulation).m_Context.TimeStep,
                    InputVelocities = ((Simulation)simulation).m_Context.InputVelocities
                };

                // Ensure the input dependencies include the end-of-simulation job, so events will have been generated
                inputDeps = JobHandle.CombineDependencies(inputDeps, simulation.FinalSimulationJobHandle);

                var parameters = new JobsUtility.JobScheduleParameters(UnsafeUtility.AddressOf(ref data), CollisionEventJobProcess<T>.Initialize(), inputDeps, ScheduleMode.Batched);
                return JobsUtility.Schedule(ref parameters);
            }
            return inputDeps;
        }

        internal unsafe struct CollisionEventJobData<T> where T : struct
        {
            public T UserJobData;
            [NativeDisableContainerSafetyRestriction] public LowLevel.CollisionEvents EventReader;
            // Disable aliasing restriction in case T has a NativeSlice of PhysicsWorld.Bodies
            [ReadOnly, NativeDisableContainerSafetyRestriction] public NativeSlice<RigidBody> Bodies;
            [ReadOnly] public float TimeStep;
            [ReadOnly] public NativeSlice<Velocity> InputVelocities;
        }

        internal struct CollisionEventJobProcess<T> where T : struct, ICollisionEventsJob
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
                foreach (ref LowLevel.CollisionEvent eventData in jobData.EventReader)
                {
                    int numContactPoints = eventData.NumNarrowPhaseContactPoints;
                    var contactPoints = new NativeArray<ContactPoint>(numContactPoints, Allocator.Temp);
                    for (int i = 0; i < numContactPoints; i++)
                    {
                        contactPoints[i] = eventData.AccessContactPoint(i);
                    }

                    int bodyAIndex = eventData.BodyIndices.BodyAIndex;
                    int bodyBIndex = eventData.BodyIndices.BodyBIndex;
                    jobData.UserJobData.Execute(new CollisionEvent
                    {
                        EventData = eventData,
                        Entities = new EntityPair
                        {
                            EntityA = jobData.Bodies[bodyAIndex].Entity,
                            EntityB = jobData.Bodies[bodyBIndex].Entity
                        },
                        TimeStep = jobData.TimeStep,
                        InputVelocityA = bodyAIndex < jobData.InputVelocities.Length ? jobData.InputVelocities[bodyAIndex] : Velocity.Zero,
                        InputVelocityB = bodyBIndex < jobData.InputVelocities.Length ? jobData.InputVelocities[bodyBIndex] : Velocity.Zero,
                        NarrowPhaseContactPoints = contactPoints
                    });
                }
            }
        }
    }
}
