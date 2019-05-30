using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;


namespace Unity.Physics
{
    // Body pair processor / dispatcher
    public static class NarrowPhase // TODO: rename
    {
    
        unsafe public static JobHandle ScheduleProcessBodyPairsJobs(ref PhysicsWorld world, float timeStep, int numIterations, ref Simulation.Context context, JobHandle inputDeps)
        {
            var numWorkItems = context.SolverSchedulerInfo.NumWorkItems;
            var contactsHandle = BlockStream.ScheduleConstruct(out context.Contacts, numWorkItems, 0xcf97529c, inputDeps);
            var jointJacobiansHandle = BlockStream.ScheduleConstruct(out context.JointJacobians, numWorkItems, 0xd3185f82, inputDeps);
            var jacobiansHandle = BlockStream.ScheduleConstruct(out context.Jacobians, numWorkItems, 0x8d8f394d, inputDeps);

            var processHandle = new ProcessBodyPairsJob
            {
                World = world,
                TimeStep = timeStep,
                NumIterations = numIterations,
                PhasedDispatchPairs = context.PhasedDispatchPairs.AsDeferredJobArray(),
                SolverSchedulerInfo = context.SolverSchedulerInfo,
                ContactWriter = context.Contacts,
                JointJacobianWriter = context.JointJacobians,
            }.ScheduleUnsafeIndex0(numWorkItems, 1, JobHandle.CombineDependencies(contactsHandle, jointJacobiansHandle, jacobiansHandle));

            
            context.DisposeProcessBodyPairs = NativeListUtilityTemp.DisposeHotFix(ref context.PhasedDispatchPairs, processHandle);

            return processHandle;
        }
        
        [BurstCompile]
        private struct ProcessBodyPairsJob : IJobParallelForDefer
        {
            [ReadOnly] public PhysicsWorld World;
            [ReadOnly] public float TimeStep;
            [ReadOnly] public int NumIterations;
            [ReadOnly] public NativeArray<Scheduler.DispatchPair> PhasedDispatchPairs;
            [ReadOnly] public Scheduler.SolverSchedulerInfo SolverSchedulerInfo;
            public BlockStream.Writer ContactWriter;
            public BlockStream.Writer JointJacobianWriter;

            public unsafe void Execute(int workItemIndex)
            {
                int dispatchPairReadOffset = SolverSchedulerInfo.GetWorkItemReadOffset(workItemIndex, out int numPairsToRead);

                ContactWriter.BeginForEachIndex(workItemIndex);
                JointJacobianWriter.BeginForEachIndex(workItemIndex);

                for (int i = 0; i < numPairsToRead; i++)
                {
                    Scheduler.DispatchPair dispatchPair = PhasedDispatchPairs[dispatchPairReadOffset + i];

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

                            ManifoldQueries.BodyBody(ref World, pair, TimeStep, ref ContactWriter);
                        }
                        else
                        {
                            Joint joint = World.Joints[dispatchPair.JointIndex];
                            // Need to fetch the real body indices from the joint, as the scheduler may have reordered them
                            int bodyAIndex = joint.BodyPair.BodyAIndex;
                            int bodyBIndex = joint.BodyPair.BodyBIndex;

                            GetMotion(ref World, bodyAIndex, out MotionVelocity velocityA, out MotionData motionA);
                            GetMotion(ref World, bodyBIndex, out MotionVelocity velocityB, out MotionData motionB);
                            Solver.BuildJointJacobian(joint.JointData, joint.BodyPair, velocityA, velocityB, motionA, motionB, TimeStep, NumIterations, ref JointJacobianWriter);
                        }
                    }
                }

                JointJacobianWriter.EndForEachIndex();
                ContactWriter.EndForEachIndex();
            }

            // Gets a body's motion, even if the body is static
            // TODO - share code with Solver.GetMotions()?
            private static void GetMotion(ref PhysicsWorld world, int bodyIndex, out MotionVelocity velocity, out MotionData motion)
            {
                if (bodyIndex >= world.MotionVelocities.Length)
                {
                    // Body is static
                    RigidBody body = world.Bodies[bodyIndex];
                    velocity = MotionVelocity.Zero;
                    motion = new MotionData
                    {
                        WorldFromMotion = body.WorldFromBody,
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
        }
    }
}
