using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

namespace Unity.Physics
{
    // Processes body pairs and creates contacts from them
    public static class NarrowPhase
    {
        // Iterates the provided dispatch pairs and creates contacts and based on them.
        public static void CreateContacts(ref PhysicsWorld world, NativeArray<DispatchPairSequencer.DispatchPair> dispatchPairs, float timeStep,
            ref NativeStream.Writer contactsWriter)
        {
            contactsWriter.BeginForEachIndex(0);

            CreateContactsJob.ExecuteImpl(ref world, timeStep, dispatchPairs, 0, dispatchPairs.Length, ref contactsWriter);

            contactsWriter.EndForEachIndex();
        }

        // Schedules a set of jobs to iterate the provided dispatch pairs and create contacts based on them.
        internal static SimulationJobHandles ScheduleCreateContactsJobs(ref PhysicsWorld world, float timeStep,
            ref NativeStream contacts, ref NativeStream jacobians, ref NativeList<DispatchPairSequencer.DispatchPair> dispatchPairs,
            JobHandle inputDeps, ref DispatchPairSequencer.SolverSchedulerInfo solverSchedulerInfo, int threadCountHint = 0)
        {
            SimulationJobHandles returnHandles = default;

            if (threadCountHint <= 0)
            {
                contacts = new NativeStream(1, Allocator.TempJob);
                jacobians = new NativeStream(1, Allocator.TempJob);
                returnHandles.FinalExecutionHandle = new CreateContactsJob
                {
                    World = world,
                    TimeStep = timeStep,
                    DispatchPairs = dispatchPairs.AsDeferredJobArray(),
                    SolverSchedulerInfo = solverSchedulerInfo,
                    ContactsWriter = contacts.AsWriter()
                }.Schedule(inputDeps);
            }
            else
            {
                var numWorkItems = solverSchedulerInfo.NumWorkItems;
                var contactsHandle = NativeStream.ScheduleConstruct(out contacts, numWorkItems, inputDeps, Allocator.TempJob);
                var jacobiansHandle = NativeStream.ScheduleConstruct(out jacobians, numWorkItems, inputDeps, Allocator.TempJob);

                var processHandle = new CreateContactsJob
                {
                    World = world,
                    TimeStep = timeStep,
                    DispatchPairs = dispatchPairs.AsDeferredJobArray(),
                    SolverSchedulerInfo = solverSchedulerInfo,
                    ContactsWriter = contacts.AsWriter()
                }.ScheduleUnsafeIndex0(numWorkItems, 1, JobHandle.CombineDependencies(contactsHandle, jacobiansHandle));


                returnHandles.FinalExecutionHandle = processHandle;
            }

            return returnHandles;
        }

        [BurstCompile]
        [NoAlias]
        struct CreateContactsJob : IJobParallelForDefer, IJob
        {
            [NoAlias, ReadOnly] public PhysicsWorld World;
            [ReadOnly] public float TimeStep;
            [ReadOnly] public NativeArray<DispatchPairSequencer.DispatchPair> DispatchPairs;
            [NoAlias] public NativeStream.Writer ContactsWriter;

            // IJobParallelForDefer specific
            [NoAlias, ReadOnly] public DispatchPairSequencer.SolverSchedulerInfo SolverSchedulerInfo;

            public unsafe void Execute(int workItemIndex)
            {
                int dispatchPairReadOffset = SolverSchedulerInfo.GetWorkItemReadOffset(workItemIndex, out int numPairsToRead);

                ContactsWriter.BeginForEachIndex(workItemIndex);

                ExecuteImpl(ref World, TimeStep, DispatchPairs, dispatchPairReadOffset, numPairsToRead, ref ContactsWriter);

                ContactsWriter.EndForEachIndex();
            }

            public void Execute()
            {
                CreateContacts(ref World, DispatchPairs, TimeStep, ref ContactsWriter);
            }

            internal static unsafe void ExecuteImpl(ref PhysicsWorld world, float timeStep,
                NativeArray<DispatchPairSequencer.DispatchPair> dispatchPairs,
                int dispatchPairReadOffset, int numPairsToRead, ref NativeStream.Writer contactWriter)
            {
                for (int i = 0; i < numPairsToRead; i++)
                {
                    DispatchPairSequencer.DispatchPair dispatchPair = dispatchPairs[dispatchPairReadOffset + i];

                    // Invalid pairs can exist by being disabled by users
                    if (dispatchPair.IsValid)
                    {
                        if (dispatchPair.IsContact)
                        {
                            // Create contact manifolds for this pair of bodies
                            var pair = new BodyIndexPair
                            {
                                BodyAIndex = dispatchPair.BodyAIndex,
                                BodyBIndex = dispatchPair.BodyBIndex
                            };

                            RigidBody rigidBodyA = world.Bodies[pair.BodyAIndex];
                            RigidBody rigidBodyB = world.Bodies[pair.BodyBIndex];

                            MotionVelocity motionVelocityA = pair.BodyAIndex < world.MotionVelocities.Length ?
                                world.MotionVelocities[pair.BodyAIndex] : MotionVelocity.Zero;
                            MotionVelocity motionVelocityB = pair.BodyBIndex < world.MotionVelocities.Length ?
                                world.MotionVelocities[pair.BodyBIndex] : MotionVelocity.Zero;

                            ManifoldQueries.BodyBody(rigidBodyA, rigidBodyB, motionVelocityA, motionVelocityB,
                                world.CollisionWorld.CollisionTolerance, timeStep, pair, ref contactWriter);
                        }
                    }
                }
            }
        }
    }
}
