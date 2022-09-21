using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Transforms;
using UnityEngine.Assertions;

namespace Unity.Physics.Systems
{
    // Make sure that:
    // 1. BuildPhysicsWorldDependencyResolver is always updated just before BuildPhysicsWorld
    // 2. BuildPhysicsWorld is the last system to be updated in [PhysicsInitializeGroup]
    // This is done to prevent race conditions if users put someting to UpdateIn[PhysicsInitializeGroup].
    // They shouldn't be doing so, but it is nice to prevent unresolvable race conditions.
    [UpdateInGroup(typeof(PhysicsInitializeGroup), OrderLast = true)]
    internal class PhysicsInitializeGroupInternal : ComponentSystemGroup
    {
    }

    [UpdateInGroup(typeof(PhysicsInitializeGroupInternal), OrderFirst = true)]
    [CreateAfter(typeof(BuildPhysicsWorld))]
    [BurstCompile]
    internal partial struct BuildPhysicsWorldDependencyResolver : ISystem
    {
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            SystemAPI.GetSingletonRW<PhysicsWorldSingleton>();
            SystemAPI.GetSingletonRW<SimulationSingleton>();
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state)
        {
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            ref var buildPhysicsData = ref state.EntityManager.GetComponentDataRW<BuildPhysicsWorldData>(state.WorldUnmanaged.GetExistingUnmanagedSystem<BuildPhysicsWorld>()).ValueRW;
            buildPhysicsData.AddInputDependencyToComplete(state.Dependency);
        }
    }

    /// <summary>
    /// Public system data for this world's instance of a <see cref="BuildPhysicsWorld"/> system.
    ///
    /// Contains physics world data based on the entity world. The physics world data will contain a
    /// rigid body for every entity which has a rigid body component, and a joint for every entity
    /// which has a joint component.
    /// </summary>
    public struct BuildPhysicsWorldData : IComponentData
    {
        internal JobHandle m_InputDependencyToComplete;

        /// <summary>
        /// Information describing the <see cref="PhysicsWorldData"/>. Important : avoid using
        /// BuildPhysicsWorldData.PhysicsWorldData.PhysicsWorld. Use <see cref="PhysicsWorldSingleton"/>
        /// instead.
        /// </summary>
        public PhysicsWorldData PhysicsData;

        /// <summary>   A filter specifying the world. </summary>
        public PhysicsWorldIndex WorldFilter;

        /// <summary>   Gets the group the dynamic bodies belongs to. </summary>
        ///
        /// <value> The dynamic entity group. </value>
        public EntityQuery DynamicEntityGroup => PhysicsData.DynamicEntityGroup;

        /// <summary>   Gets the group the static bodies belongs to. </summary>
        ///
        /// <value> The static entity group. </value>
        public EntityQuery StaticEntityGroup => PhysicsData.StaticEntityGroup;

        /// <summary>   Gets the group the joints belongs to. </summary>
        ///
        /// <value> The joint entity group. </value>
        public EntityQuery JointEntityGroup => PhysicsData.JointEntityGroup;

        /// <summary>   True if the static bodies have changed this frame. </summary>
        ///
        /// <value> The have static bodies changed flag. </value>
        public NativeReference<int> HaveStaticBodiesChanged => PhysicsData.HaveStaticBodiesChanged;

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
        internal NativeParallelHashMap<uint, long> IntegrityCheckMap;
#endif

        /// <summary>
        /// Adds the dependency that BuildPhysicsWorld will complete on the next OnUpdate() call.
        /// Multiple dependencies can be added this way (they are combined).
        /// BuildPhysicsWorld resets the PhysicsWorld immediately in the OnUpdate() method (not through jobs),
        /// so any jobs that rely on that data should use this to make sure their data is not ruined before they access it.
        /// </summary>
        internal void AddInputDependencyToComplete(JobHandle dependencyToComplete)
        {
            m_InputDependencyToComplete = JobHandle.CombineDependencies(m_InputDependencyToComplete, dependencyToComplete);
        }
    }

    /// <summary>
    /// A system which builds the physics world based on the entity world. The world will contain a
    /// rigid body for every entity which has a rigid body component, and a joint for every entity
    /// which has a joint component.
    /// </summary>
    [BurstCompile]
    [UpdateInGroup(typeof(PhysicsInitializeGroupInternal), OrderLast = true)]
    public partial struct BuildPhysicsWorld : ISystem
    {
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            var worldFilter = new PhysicsWorldIndex();
            var physicsData = new PhysicsWorldData(ref state, worldFilter);
            state.EntityManager.AddComponentData(state.SystemHandle, new BuildPhysicsWorldData
            {
                WorldFilter = worldFilter,
                PhysicsData = physicsData,
#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
                IntegrityCheckMap = new NativeParallelHashMap<uint, long>(4, Allocator.Persistent),
#endif
            });

            state.EntityManager.CreateSingleton(
                new PhysicsWorldSingleton
                {
                    PhysicsWorld = physicsData.PhysicsWorld,
                    PhysicsWorldIndex = worldFilter
                });
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state)
        {
            ref var buildPhysicsData = ref state.EntityManager.GetComponentDataRW<BuildPhysicsWorldData>(state.SystemHandle).ValueRW;
            buildPhysicsData.PhysicsData.Dispose();

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
            buildPhysicsData.IntegrityCheckMap.Dispose();
#endif
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            ref var buildPhysicsData = ref state.EntityManager.GetComponentDataRW<BuildPhysicsWorldData>(state.SystemHandle).ValueRW;
            buildPhysicsData.m_InputDependencyToComplete.Complete();

            float timeStep = SystemAPI.Time.DeltaTime;

            if (!SystemAPI.TryGetSingleton<PhysicsStep>(out PhysicsStep stepComponent))
            {
                stepComponent = PhysicsStep.Default;
            }

            state.Dependency = PhysicsWorldBuilder.SchedulePhysicsWorldBuild(ref state, ref buildPhysicsData.PhysicsData, state.Dependency,
                timeStep, stepComponent.MultiThreaded > 0, stepComponent.Gravity, state.LastSystemVersion);

            SystemAPI.SetSingleton(new PhysicsWorldSingleton
            {
                PhysicsWorld = buildPhysicsData.PhysicsData.PhysicsWorld,
                PhysicsWorldIndex = buildPhysicsData.WorldFilter
            });

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
            RecordIntegrity(buildPhysicsData.IntegrityCheckMap, ref state);
#endif

            buildPhysicsData.m_InputDependencyToComplete = default;
        }

        #region Integrity checks

        private static class Jobs
        {
            [BurstCompile]
            internal struct RecordDynamicBodyIntegrity : IJobChunk
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

                public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
                {
                    Assert.IsFalse(useEnabledMask);
                    AddOrIncrement(IntegrityCheckMap, chunk.GetOrderVersion());
                    AddOrIncrement(IntegrityCheckMap, chunk.GetChangeVersion(PhysicsVelocityType));
                    if (chunk.Has(PositionType))
                    {
                        AddOrIncrement(IntegrityCheckMap, chunk.GetChangeVersion(PositionType));
                    }
                    if (chunk.Has(RotationType))
                    {
                        AddOrIncrement(IntegrityCheckMap, chunk.GetChangeVersion(RotationType));
                    }
                    if (chunk.Has(PhysicsColliderType))
                    {
                        AddOrIncrement(IntegrityCheckMap, chunk.GetChangeVersion(PhysicsColliderType));
                    }
                }
            }

            [BurstCompile]
            internal struct RecordColliderIntegrity : IJobChunk
            {
                [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
                public NativeParallelHashMap<uint, long> IntegrityCheckMap;

                public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
                {
                    Assert.IsFalse(useEnabledMask);
                    if (chunk.Has(PhysicsColliderType))
                    {
                        RecordDynamicBodyIntegrity.AddOrIncrement(IntegrityCheckMap, chunk.GetChangeVersion(PhysicsColliderType));
                    }
                }
            }
        }

        internal void RecordIntegrity(NativeParallelHashMap<uint, long> integrityCheckMap, ref SystemState state)
        {
            integrityCheckMap.Clear();

            var buildPhysicsData = state.EntityManager.GetComponentData<BuildPhysicsWorldData>(state.SystemHandle);

            var dynamicBodyIntegrity = new Jobs.RecordDynamicBodyIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PositionType = buildPhysicsData.PhysicsData.ComponentHandles.PositionType,
                RotationType = buildPhysicsData.PhysicsData.ComponentHandles.RotationType,
                PhysicsVelocityType = buildPhysicsData.PhysicsData.ComponentHandles.PhysicsVelocityType,
                PhysicsColliderType = buildPhysicsData.PhysicsData.ComponentHandles.PhysicsColliderType
            };

            var handle = dynamicBodyIntegrity.Schedule(buildPhysicsData.PhysicsData.DynamicEntityGroup, state.Dependency);

            var staticBodyColliderIntegrity = new Jobs.RecordColliderIntegrity
            {
                IntegrityCheckMap = integrityCheckMap,
                PhysicsColliderType = buildPhysicsData.PhysicsData.ComponentHandles.PhysicsColliderType
            };

            handle = staticBodyColliderIntegrity.Schedule(buildPhysicsData.PhysicsData.StaticEntityGroup, handle);
            state.Dependency = JobHandle.CombineDependencies(state.Dependency, handle);
        }

        #endregion
    }

    struct DummySimulationData : IComponentData
    {
        byte dummyData;  // Without data, the component is zero-sized, and accessing it will result in error
        internal DummySimulation m_Simulation;

        internal unsafe void DisableSystemChain(ref SystemState systemStateRef)
        {
            systemStateRef.Enabled = false;
            m_Simulation.Dispose();
        }

        internal unsafe void EnableSystemChain(ref SystemState systemStateRef)
        {
            systemStateRef.Enabled = true;
            m_Simulation = new DummySimulation();
        }
    }

    [UpdateInGroup(typeof(PhysicsSimulationGroup))]
    [BurstCompile]
    internal struct DummySimulationSystem : ISystem
    {
        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            state.EntityManager.AddComponentData(state.SystemHandle, new DummySimulationData
            {
                // This one is enabled by default
                m_Simulation = new DummySimulation(),
            });
        }

        [BurstCompile]
        public void OnDestroy(ref SystemState state)
        {
            state.EntityManager.GetComponentDataRW<DummySimulationData>(state.SystemHandle).ValueRW.m_Simulation.Dispose();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            // do nothing;
        }
    }
}
