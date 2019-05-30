using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.Physics.Systems
{
    // A system which copies transforms and velocities from the physics world back to the original entity components.
    // CK: We make sure we update before CopyTransformToGameObjectSystem so that hybrid GameObjects can work with this OK, even if that path is slow.
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem)), UpdateBefore(typeof(TransformSystemGroup))]
    public class ExportPhysicsWorld : JobComponentSystem
    {
        public JobHandle FinalJobHandle { get; private set; }

        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        StepPhysicsWorld m_StepPhysicsWorldSystem;

        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_StepPhysicsWorldSystem = World.GetOrCreateSystem<StepPhysicsWorld>();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            JobHandle handle = JobHandle.CombineDependencies(inputDeps, m_StepPhysicsWorldSystem.FinalSimulationJobHandle);

            ref PhysicsWorld world = ref m_BuildPhysicsWorldSystem.PhysicsWorld;

            var positionType = GetArchetypeChunkComponentType<Translation>();
            var rotationType = GetArchetypeChunkComponentType<Rotation>();
            var velocityType = GetArchetypeChunkComponentType<PhysicsVelocity>();

            handle = new ExportDynamicBodiesJob
            {
                MotionVelocities = world.MotionVelocities,
                MotionDatas = world.MotionDatas,

                PositionType = positionType,
                RotationType = rotationType,
                VelocityType = velocityType
            }.Schedule(m_BuildPhysicsWorldSystem.DynamicEntityGroup, handle);

            FinalJobHandle = handle;
            return handle;
        }

        [BurstCompile]
        internal struct ExportDynamicBodiesJob : IJobChunk
        {
            [ReadOnly] public NativeSlice<MotionVelocity> MotionVelocities;
            [ReadOnly] public NativeSlice<MotionData> MotionDatas;

            public ArchetypeChunkComponentType<Translation> PositionType;
            public ArchetypeChunkComponentType<Rotation> RotationType;
            public ArchetypeChunkComponentType<PhysicsVelocity> VelocityType;

            public void Execute(ArchetypeChunk chunk, int chunkIndex, int entityStartIndex)
            {
                var chunkPositions = chunk.GetNativeArray(PositionType);
                var chunkRotations = chunk.GetNativeArray(RotationType);
                var chunkVelocities = chunk.GetNativeArray(VelocityType);

                int numItems = chunk.Count;
                for(int i = 0, motionIndex = entityStartIndex; i < numItems; i++, motionIndex++)
                {
                    MotionData md = MotionDatas[motionIndex];
                    RigidTransform worldFromBody = math.mul(md.WorldFromMotion, math.inverse(md.BodyFromMotion));
                    chunkPositions[i] = new Translation { Value = worldFromBody.pos };
                    chunkRotations[i] = new Rotation { Value = worldFromBody.rot };
                    chunkVelocities[i] = new PhysicsVelocity
                    {
                        Linear = MotionVelocities[motionIndex].LinearVelocity,
                        Angular = MotionVelocities[motionIndex].AngularVelocity
                    };
                }
            }
        }
    }
}
