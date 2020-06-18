using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.Physics.Systems
{
    // A system which copies transforms and velocities from the physics world back to the original entity components.
    // CK: We make sure we update before CopyTransformToGameObjectSystem so that hybrid GameObjects can work with this OK, even if that path is slow.
    [UpdateAfter(typeof(StepPhysicsWorld)), UpdateBefore(typeof(EndFramePhysicsSystem)), UpdateBefore(typeof(TransformSystemGroup))]
    public class ExportPhysicsWorld : SystemBase, IPhysicsSystem
    {
        private JobHandle m_InputDependency;
        private JobHandle m_OutputDependency;

        [Obsolete("FinalJobHandle has been deprecated. Use GetOutputDependency() instead. (RemovedAfter 2020-08-07) (UnityUpgradable) -> GetOutputDependency()")]
        public JobHandle FinalJobHandle => GetOutputDependency();

        BuildPhysicsWorld m_BuildPhysicsWorldSystem;
        StepPhysicsWorld m_StepPhysicsWorldSystem;
        EndFramePhysicsSystem m_EndFramePhysicsSystem;

        internal unsafe struct SharedData : System.IDisposable
        {
            [NativeDisableUnsafePtrRestriction]
            public AtomicSafetyManager* SafetyManager;
           
            public static SharedData Create()
            {
                var sharedData = new SharedData();
                sharedData.SafetyManager = (AtomicSafetyManager*)UnsafeUtility.Malloc(UnsafeUtility.SizeOf<AtomicSafetyManager>(), 16, Allocator.Persistent);
                *sharedData.SafetyManager = AtomicSafetyManager.Create();

                return sharedData;
            }
            
            public void Dispose()
            {
                SafetyManager->Dispose();
            }

            public void Sync()
            {
                SafetyManager->BumpTemporaryHandleVersions();
            }
        }

        private SharedData m_SharedData;
        
        protected override void OnCreate()
        {
            m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
            m_StepPhysicsWorldSystem = World.GetOrCreateSystem<StepPhysicsWorld>();
            m_EndFramePhysicsSystem = World.GetOrCreateSystem<EndFramePhysicsSystem>();
            
            m_SharedData = SharedData.Create();
        }

        protected override void OnDestroy()
        {
           m_SharedData.Dispose(); 
        }
        
        protected override void OnUpdate()
        {
            // Combine implicit input dependency with the user one
            JobHandle handle = JobHandle.CombineDependencies(Dependency, m_InputDependency);

            ref PhysicsWorld world = ref m_BuildPhysicsWorldSystem.PhysicsWorld;

            var positionType = GetComponentTypeHandle<Translation>();
            var rotationType = GetComponentTypeHandle<Rotation>();
            var velocityType = GetComponentTypeHandle<PhysicsVelocity>();

            handle = new ExportDynamicBodiesJob
            {
                MotionVelocities = world.MotionVelocities,
                MotionDatas = world.MotionDatas,

                PositionType = positionType,
                RotationType = rotationType,
                VelocityType = velocityType
            }.Schedule(m_BuildPhysicsWorldSystem.DynamicEntityGroup, handle);

            // Sync shared data. 
            m_SharedData.Sync();
            
            int numCollisionWorldProxies = m_BuildPhysicsWorldSystem.CollisionWorldProxyGroup.CalculateEntityCount();
            if (numCollisionWorldProxies > 0)
            {
                handle = new CopyCollisionWorld
                {
                    World = m_BuildPhysicsWorldSystem.PhysicsWorld,
                    SharedData = m_SharedData,
                    ProxyType = GetComponentTypeHandle<CollisionWorldProxy>()
                }.Schedule(m_BuildPhysicsWorldSystem.CollisionWorldProxyGroup, handle);
            }  

            m_OutputDependency = handle;

            // Combine implicit output dependency with user one
            Dependency = JobHandle.CombineDependencies(m_OutputDependency, Dependency);

            // Inform next system in the pipeline of its dependency
            m_EndFramePhysicsSystem.AddInputDependency(m_OutputDependency);

            // Invalidate input dependency since it's been used now
            m_InputDependency = default;
        }

        public void AddInputDependency(JobHandle inputDep)
        {
            m_InputDependency = JobHandle.CombineDependencies(m_InputDependency, inputDep);
        }

        public JobHandle GetOutputDependency()
        {
            return m_OutputDependency;
        }

        [BurstCompile]
        internal struct ExportDynamicBodiesJob : IJobChunk
        {
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [ReadOnly] public NativeArray<MotionData> MotionDatas;

            public ComponentTypeHandle<Translation> PositionType;
            public ComponentTypeHandle<Rotation> RotationType;
            public ComponentTypeHandle<PhysicsVelocity> VelocityType;

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
        
        [BurstCompile]
        internal unsafe struct CopyCollisionWorld : IJobChunk
        {
            [ReadOnly] public PhysicsWorld World;
            public SharedData SharedData;
            public ComponentTypeHandle<CollisionWorldProxy> ProxyType;

            public void Execute(ArchetypeChunk chunk, int chunkIndex, int firstEntityIndex)
            {
                NativeArray<CollisionWorldProxy> chunkProxies = chunk.GetNativeArray(ProxyType);

                var proxy = new CollisionWorldProxy(World.CollisionWorld, SharedData.SafetyManager);

                for (var i = 0; i < chunk.Count; ++i)
                    chunkProxies[i] = proxy;
            }
        }

#if !UNITY_ENTITIES_0_12_OR_NEWER
        ComponentTypeHandle<T> GetComponentTypeHandle<T>() where T : struct, IComponentData => new ComponentTypeHandle<T> { Value = GetArchetypeChunkComponentType<T>() };
#endif
    }
}
