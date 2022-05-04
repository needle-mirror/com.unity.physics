using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Transforms;

namespace Unity.Physics.Systems
{
    // A system which builds the physics world based on the entity world.
    // The world will contain a rigid body for every entity which has a rigid body component,
    // and a joint for every entity which has a joint component.
    [UpdateInGroup(typeof(FixedStepSimulationSystemGroup))]
    [UpdateBefore(typeof(StepPhysicsWorld))]
    [AlwaysUpdateSystem]
    public partial class BuildPhysicsWorld : SystemBase
    {
        private JobHandle m_InputDependencyToComplete;

        public PhysicsWorldData PhysicsData;
        public PhysicsWorldIndex WorldFilter;

        public ref PhysicsWorld PhysicsWorld => ref PhysicsData.PhysicsWorld;
        public EntityQuery DynamicEntityGroup => PhysicsData.DynamicEntityGroup;
        public EntityQuery StaticEntityGroup => PhysicsData.StaticEntityGroup;
        public EntityQuery JointEntityGroup => PhysicsData.JointEntityGroup;

        public EntityQuery CollisionWorldProxyGroup { get; private set; }

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
        internal NativeParallelHashMap<uint, long> IntegrityCheckMap = new NativeParallelHashMap<uint, long>(4, Allocator.Persistent);
#endif

        protected override void OnCreate()
        {
            base.OnCreate();

            WorldFilter = new PhysicsWorldIndex();
            PhysicsData = new PhysicsWorldData(EntityManager, WorldFilter);

            CollisionWorldProxyGroup = GetEntityQuery(ComponentType.ReadWrite<CollisionWorldProxy>());

            // Make sure PhysicsRuntimeData is registered as singleton
            {
                var runtimeDataEntity = EntityManager.CreateEntity();
                EntityManager.AddComponentData(runtimeDataEntity, new PhysicsSystemRuntimeData());
            }
        }

        protected override void OnDestroy()
        {
            PhysicsData.Dispose();

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
            IntegrityCheckMap.Dispose();
#endif

            base.OnDestroy();
        }

        protected override void OnStartRunning()
        {
            base.OnStartRunning();
            this.RegisterPhysicsRuntimeSystemReadWrite();
        }

        protected override void OnUpdate()
        {
            // Make sure last frame's physics jobs are complete before any new ones start
            m_InputDependencyToComplete.Complete();

            float timeStep = Time.DeltaTime;

            PhysicsStep stepComponent = PhysicsStep.Default;
            if (HasSingleton<PhysicsStep>())
            {
                stepComponent = GetSingleton<PhysicsStep>();
            }

            Dependency = PhysicsWorldBuilder.SchedulePhysicsWorldBuild(this, ref PhysicsData,
                Dependency, timeStep, stepComponent.MultiThreaded > 0, stepComponent.Gravity, LastSystemVersion);

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
            RecordIntegrity(IntegrityCheckMap);
#endif

            m_InputDependencyToComplete = default;
        }

        /// <summary>
        /// Adds the dependency that BuildPhysicsWorld will complete on the next OnUpdate() call.
        /// Multiple dependencies can be added this way (they are combined).
        /// BuildPhysicsWorld resets the PhysicsWorld immediately in the OnUpdate() method (not through jobs),
        /// so any jobs that rely on that data should use this to make sure their data is not ruined before they access it.
        /// </summary>
        public void AddInputDependencyToComplete(JobHandle dependencyToComplete)
        {
            m_InputDependencyToComplete = JobHandle.CombineDependencies(m_InputDependencyToComplete, dependencyToComplete);
        }

        [Obsolete("AddInputDependency() has been deprecated. Please call RegisterPhysicsRuntimeSystemReadWrite() or RegisterPhysicsRuntimeSystemReadOnly() in your system's OnStartRunning() to achieve the same effect. (RemovedAfter 2021-05-01)", true)]
        public void AddInputDependency(JobHandle inputDep) {}

        [Obsolete("GetOutputDependency() has been deprecated. Please call RegisterPhysicsRuntimeSystemReadWrite() or RegisterPhysicsRuntimeSystemReadOnly() in your system's OnStartRunning() to achieve the same effect. (RemovedAfter 2021-05-01)", true)]
        public JobHandle GetOutputDependency() => default;

        #region Integrity checks

        private static class Jobs
        {
            [BurstCompile]
            internal struct RecordDynamicBodyIntegrity : IJobEntityBatch
            {
                [ReadOnly] public ComponentTypeHandle<Translation> PositionType;
                [ReadOnly] public ComponentTypeHandle<Rotation> RotationType;
                [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;

                public NativeParallelHashMap<uint, long> IntegrityCheckMap;

                internal static void AddOrIncrement(NativeParallelHashMap<uint, long> integrityCheckMap, uint systemVersion)
                {
                    if (integrityCheckMap.TryGetValue(systemVersion, out long occurences))
                    {
                        integrityCheckMap.Remove(systemVersion);
                        integrityCheckMap.Add(systemVersion, occurences + 1);
                    }
                    else
                    {
                        integrityCheckMap.Add(systemVersion, 1);
                    }
                }

                public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
                {
                    AddOrIncrement(IntegrityCheckMap, batchInChunk.GetOrderVersion());
                    AddOrIncrement(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsVelocityType));
                    if (batchInChunk.Has(PositionType))
                    {
                        AddOrIncrement(IntegrityCheckMap, batchInChunk.GetChangeVersion(PositionType));
                    }
                    if (batchInChunk.Has(RotationType))
                    {
                        AddOrIncrement(IntegrityCheckMap, batchInChunk.GetChangeVersion(RotationType));
                    }
                    if (batchInChunk.Has(PhysicsColliderType))
                    {
                        AddOrIncrement(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsColliderType));
                    }
                }
            }

            [BurstCompile]
            internal struct RecordColliderIntegrity : IJobEntityBatch
            {
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
                public NativeParallelHashMap<uint, long> IntegrityCheckMap;

                public void Execute(ArchetypeChunk batchInChunk, int batchIndex)
                {
                    if (batchInChunk.Has(PhysicsColliderType))
                    {
                        RecordDynamicBodyIntegrity.AddOrIncrement(IntegrityCheckMap, batchInChunk.GetChangeVersion(PhysicsColliderType));
                    }
                }
            }
        }

        internal void RecordIntegrity(NativeParallelHashMap<uint, long> integrityCheckMap)
        {
            var positionType = GetComponentTypeHandle<Translation>(true);
            var rotationType = GetComponentTypeHandle<Rotation>(true);
            var physicsColliderType = GetComponentTypeHandle<PhysicsCollider>(true);
            var physicsVelocityType = GetComponentTypeHandle<PhysicsVelocity>(true);

            integrityCheckMap.Clear();

            var dynamicBodyIntegrity = new Jobs.RecordDynamicBodyIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PositionType = positionType,
                RotationType = rotationType,
                PhysicsVelocityType = physicsVelocityType,
                PhysicsColliderType = physicsColliderType
            };

            var handle = dynamicBodyIntegrity.Schedule(PhysicsData.DynamicEntityGroup, Dependency);

            var staticBodyColliderIntegrity = new Jobs.RecordColliderIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PhysicsColliderType = physicsColliderType
            };

            handle = staticBodyColliderIntegrity.Schedule(PhysicsData.StaticEntityGroup, handle);
            Dependency = JobHandle.CombineDependencies(Dependency, handle);
        }

        #endregion
    }
}
