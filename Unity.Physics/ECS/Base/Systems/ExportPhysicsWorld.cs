using System;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Transforms;
using UnityEngine.Assertions;

namespace Unity.Physics.Systems
{
    /// <summary>
    /// A system which copies transforms and velocities from the physics world back to the original
    /// entity components. The last system to run in <see cref="PhysicsSystemGroup"/>.
    /// </summary>
    [UpdateInGroup(typeof(PhysicsSystemGroup))]
    [UpdateAfter(typeof(PhysicsSimulationGroup))]
    [CreateAfter(typeof(BuildPhysicsWorld))]
    [BurstCompile]
    public partial struct ExportPhysicsWorld : ISystem
    {
        PhysicsWorldExporter.ExportPhysicsWorldTypeHandles m_ComponentTypeHandles;

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
        IntegrityComponentHandles m_IntegrityCheckHandles;
#endif

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_ComponentTypeHandles = new PhysicsWorldExporter.ExportPhysicsWorldTypeHandles(ref state);
#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS

            m_IntegrityCheckHandles = new IntegrityComponentHandles(ref state);
#endif
            // Register a ReadOnly dependency on PhysicsWorldSingleton
            SystemAPI.GetSingleton<PhysicsWorldSingleton>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            JobHandle handle = state.Dependency;

            ref var buildPhysicsData = ref SystemAPI.GetSingletonRW<BuildPhysicsWorldData>().ValueRW;

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
            if (!SystemAPI.TryGetSingleton<PhysicsStep>(out var physicsStep))
            {
                physicsStep = PhysicsStep.Default;
            }
            handle = CheckIntegrity(ref state, handle, ref buildPhysicsData, physicsStep);
#endif
            handle = PhysicsWorldExporter.SchedulePhysicsWorldExport(ref state, ref m_ComponentTypeHandles, buildPhysicsData.PhysicsData.PhysicsWorld, handle, buildPhysicsData.PhysicsData.DynamicEntityGroup);

            // Combine implicit output dependency with user one
            state.Dependency = JobHandle.CombineDependencies(state.Dependency, handle);
        }

        #region Integrity checks

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS

        struct IntegrityComponentHandles
        {
            public ComponentTypeHandle<LocalTransform> LocalTransformType;
            public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
            public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;

            public ComponentLookup<PhysicsTemporalCoherenceInfo> PhysicsTemporalCoherenceInfoLookup;

            public IntegrityComponentHandles(ref SystemState state)
            {
                LocalTransformType = state.GetComponentTypeHandle<LocalTransform>(true);
                PhysicsColliderType = state.GetComponentTypeHandle<PhysicsCollider>(true);
                PhysicsVelocityType = state.GetComponentTypeHandle<PhysicsVelocity>(true);
                PhysicsTemporalCoherenceInfoLookup = state.GetComponentLookup<PhysicsTemporalCoherenceInfo>(true);
            }

            public void Update(ref SystemState state)
            {
                LocalTransformType.Update(ref state);
                PhysicsColliderType.Update(ref state);
                PhysicsVelocityType.Update(ref state);
                PhysicsTemporalCoherenceInfoLookup.Update(ref state);
            }
        }

        [BurstCompile]
        struct DetermineTreeNodeCountJob : IJob
        {
            [WriteOnly]
            public NativeReference<int> NodeCount;
            [ReadOnly]
            public Broadphase.Tree Tree;

            public void Execute()
            {
                NodeCount.Value = Tree.Nodes.IsCreated ? Tree.Nodes.Length : 0;
            }
        }

        JobHandle CheckBroadphaseIntegrity(bool staticTree, in BuildPhysicsWorldData buildPhysicsWorldData, JobHandle inputDeps)
        {
            var tree = staticTree ? buildPhysicsWorldData.PhysicsData.PhysicsWorld.CollisionWorld.Broadphase.StaticTree : buildPhysicsWorldData.PhysicsData.PhysicsWorld.CollisionWorld.Broadphase.DynamicTree;
            var nodeCount = new NativeReference<int>(1, Allocator.TempJob);
            inputDeps = new DetermineTreeNodeCountJob
            {
                NodeCount = nodeCount,
                Tree = tree
            }.Schedule(inputDeps);

            inputDeps = new CheckIncrementalBroadphaseIntegrity
            {
                RigidBodies = staticTree ? buildPhysicsWorldData.PhysicsData.PhysicsWorld.StaticBodies : buildPhysicsWorldData.PhysicsData.PhysicsWorld.DynamicBodies,
                PhysicsTemporalCoherenceInfoLookup = m_IntegrityCheckHandles.PhysicsTemporalCoherenceInfoLookup,
                Tree = tree,
                Static = staticTree,
            }.ScheduleUnsafe(nodeCount, 16, inputDeps);

            inputDeps = nodeCount.Dispose(inputDeps);
            return inputDeps;
        }

        JobHandle CheckIntegrity(ref SystemState state, JobHandle inputDeps, ref BuildPhysicsWorldData buildPhysicsData, in PhysicsStep physicsStep)
        {
            m_IntegrityCheckHandles.Update(ref state);

            // body and collider integrity checks:

            JobHandle bodyColliderIntegrityHandle = default;
            // Note: we skip these integrity checks if the required map hasn't been created. This is the case
            // if the IntegrityCheckSystem has not been created.
            if (buildPhysicsData.IntegrityCheckMap.IsCreated)
            {
                var localTransformType = m_IntegrityCheckHandles.LocalTransformType;
                var physicsColliderType = m_IntegrityCheckHandles.PhysicsColliderType;
                var physicsVelocityType = m_IntegrityCheckHandles.PhysicsVelocityType;

                bodyColliderIntegrityHandle = new CheckDynamicBodyIntegrity
                {
                    IntegrityCheckMap = buildPhysicsData.IntegrityCheckMap,
                    LocalTransformType = localTransformType,
                    PhysicsVelocityType = physicsVelocityType,
                    PhysicsColliderType = physicsColliderType
                }.Schedule(buildPhysicsData.DynamicEntityGroup, inputDeps);

                bodyColliderIntegrityHandle = new CheckColliderIntegrity
                {
                    IntegrityCheckMap = buildPhysicsData.IntegrityCheckMap,
                    PhysicsColliderType = physicsColliderType
                }.Schedule(buildPhysicsData.StaticEntityGroup, bodyColliderIntegrityHandle);

                bodyColliderIntegrityHandle = new CheckTotalBodyAndColliderIntegrity
                {
                    IntegrityCheckMap = buildPhysicsData.IntegrityCheckMap
                }.Schedule(bodyColliderIntegrityHandle);
            }

            // incremental broadphase integrity checks:

            JobHandle broadphaseIntegrityHandleDynamic = default;
            JobHandle broadphaseIntegrityHandleStatic = default;
            if (physicsStep.IncrementalDynamicBroadphase)
            {
                broadphaseIntegrityHandleDynamic = CheckBroadphaseIntegrity(false, buildPhysicsData, inputDeps);
            }
            if (physicsStep.IncrementalStaticBroadphase)
            {
                broadphaseIntegrityHandleStatic = CheckBroadphaseIntegrity(true, buildPhysicsData, inputDeps);
            }
            var broadphaseIntegrityHandle = JobHandle.CombineDependencies(broadphaseIntegrityHandleDynamic, broadphaseIntegrityHandleStatic);

            return JobHandle.CombineDependencies(broadphaseIntegrityHandle, bodyColliderIntegrityHandle);
        }

        [BurstCompile]
        struct CheckDynamicBodyIntegrity : IJobChunk
        {
            [ReadOnly] public ComponentTypeHandle<LocalTransform> LocalTransformType;
            [ReadOnly] public ComponentTypeHandle<PhysicsVelocity> PhysicsVelocityType;
            [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
            public NativeParallelHashMap<uint, long> IntegrityCheckMap;

            internal static void DecrementIfExists(NativeParallelHashMap<uint, long> integrityCheckMap, uint systemVersion)
            {
                if (integrityCheckMap.TryGetValue(systemVersion, out long occurrences))
                {
                    integrityCheckMap.Remove(systemVersion);
                    integrityCheckMap.Add(systemVersion, occurrences - 1);
                }
            }

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                Assert.IsFalse(useEnabledMask);
                DecrementIfExists(IntegrityCheckMap, chunk.GetOrderVersion());
                DecrementIfExists(IntegrityCheckMap, chunk.GetChangeVersion(ref PhysicsVelocityType));

                if (chunk.Has(ref LocalTransformType))
                {
                    DecrementIfExists(IntegrityCheckMap, chunk.GetChangeVersion(ref LocalTransformType));
                }

                if (chunk.Has(ref PhysicsColliderType))
                {
                    DecrementIfExists(IntegrityCheckMap, chunk.GetChangeVersion(ref PhysicsColliderType));

                    var colliders = chunk.GetNativeArray(ref PhysicsColliderType);
                    CheckColliderFilterIntegrity(colliders);
                }
            }
        }

        [BurstCompile]
        struct CheckColliderIntegrity : IJobChunk
        {
            [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;
            public NativeParallelHashMap<uint, long> IntegrityCheckMap;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                Assert.IsFalse(useEnabledMask);
                if (chunk.Has(ref PhysicsColliderType))
                {
                    CheckDynamicBodyIntegrity.DecrementIfExists(IntegrityCheckMap, chunk.GetChangeVersion(ref PhysicsColliderType));

                    var colliders = chunk.GetNativeArray(ref PhysicsColliderType);
                    CheckColliderFilterIntegrity(colliders);
                }
            }
        }

        [BurstCompile]
        struct CheckTotalBodyAndColliderIntegrity : IJob
        {
            public NativeParallelHashMap<uint, long> IntegrityCheckMap;
            public void Execute()
            {
                var values = IntegrityCheckMap.GetValueArray(Allocator.Temp);
                var validIntegrity = true;
                for (int i = 0; i < values.Length; i++)
                {
                    if (values[i] != 0)
                    {
                        validIntegrity = false;
                        break;
                    }
                }
                if (!validIntegrity)
                {
                    SafetyChecks.ThrowInvalidOperationException("Adding/removing components or changing position/rotation/velocity/collider ECS data" +
                        " on dynamic entities during physics step");
                }
            }
        }

        // Verifies combined collision filter of compound colliders
        // ToDo: add the same for mesh once per-triangle filters are supported
        static void CheckColliderFilterIntegrity(NativeArray<PhysicsCollider> colliders)
        {
            for (int i = 0; i < colliders.Length; i++)
            {
                var collider = colliders[i];
                if (collider.IsValid && collider.Value.Value.Type == ColliderType.Compound)
                {
                    unsafe
                    {
                        var compoundCollider = (CompoundCollider*)collider.ColliderPtr;

                        var rootFilter = compoundCollider->GetCollisionFilter();
                        var combinedFilter = CollisionFilter.Zero;

                        for (int childIndex = 0; childIndex < compoundCollider->NumChildren; childIndex++)
                        {
                            combinedFilter = CollisionFilter.CreateUnion(combinedFilter, compoundCollider->GetCollisionFilter(compoundCollider->ConvertChildIndexToColliderKey(childIndex)));
                        }

                        // GroupIndex has no concept of union. Creating one from children has no guarantees
                        // that it will be the same as the GroupIndex of the root, so we can't compare those two.
                        // Setting combinedFilter's GroupIndex to rootFilter's will exclude GroupIndex from comparing the two filters.
                        combinedFilter.GroupIndex = rootFilter.GroupIndex;

                        // Check that the combined filter (excluding GroupIndex) of all children is the same as root filter.
                        // If not, it means user has forgotten to call RefreshCollisionFilter() on the CompoundCollider.
                        if (!rootFilter.Equals(combinedFilter))
                        {
                            SafetyChecks.ThrowInvalidOperationException("CollisionFilter of a compound collider is not a union of its children. " +
                                "You must call CompoundCollider.RefreshCollisionFilter() to update the root filter after changing child filters.");
                        }
                    }
                }
            }
        }

        [BurstCompile]
        struct CheckIncrementalBroadphaseIntegrity : IJobParallelForDefer
        {
            [ReadOnly]
            public NativeArray<RigidBody> RigidBodies;
            [ReadOnly]
            public Broadphase.Tree Tree;
            [ReadOnly]
            public ComponentLookup<PhysicsTemporalCoherenceInfo> PhysicsTemporalCoherenceInfoLookup;
            public bool Static;

            public void Execute(int index)
            {
                // Go over all leaf nodes in the tree and for each slot in the leaf check if the present data corresponds to the
                // temporal coherence info of the rigid body entity the slot entry represents.

                var node = Tree.Nodes[index];
                if (node.IsLeaf)
                {
                    for (int i = 0; i < 4; ++i)
                    {
                        if (node.IsLeafValid(i))
                        {
                            // make sure the body index is valid
                            var bodyIndex = node.Data[i];
                            if (bodyIndex < 0 && bodyIndex > RigidBodies.Length)
                            {
                                SafetyChecks.ThrowInvalidOperationException($"Incremental broadphase integrity check failed for this tree (static={Static}, number of bodies={RigidBodies.Length}). Body index {bodyIndex} out of bounds. Did you modify or remove a PhysicsTemporalCoherenceInfo component?");
                            }

                            var entity = RigidBodies[bodyIndex].Entity;

                            // make sure we have an entity for each rigid body in the tree
                            if (entity == Entity.Null)
                            {
                                SafetyChecks.ThrowInvalidOperationException($"Incremental broadphase integrity check failed for this tree (static={Static}). Rigid body with index {bodyIndex} does not have a valid entity.");
                            }

                            // check if we have temporal coherence info
                            if (!PhysicsTemporalCoherenceInfoLookup.HasComponent(entity))
                            {
                                SafetyChecks.ThrowInvalidOperationException($"Incremental broadphase integrity check failed for this tree (static={Static}). Entity {entity.ToFixedString()} does not have PhysicsTemporalCoherenceInfo component. Did you remove a PhysicsTemporalCoherenceInfo component?");
                            }

                            // check if temporal coherence info is valid:

                            var temporalCoherenceInfo = PhysicsTemporalCoherenceInfoLookup[entity];

                            if (temporalCoherenceInfo.LastRigidBodyIndex != bodyIndex)
                            {
                                SafetyChecks.ThrowInvalidOperationException($"Incremental broadphase integrity check failed for this tree (static={Static}). Did you modify a PhysicsTemporalCoherenceInfo component? Temporal coherence info of Entity {entity.ToFixedString()} contains invalid rigid body index {temporalCoherenceInfo.LastRigidBodyIndex} (expected: {bodyIndex}).");
                            }

                            if (temporalCoherenceInfo.LastBvhNodeIndex != index)
                            {
                                SafetyChecks.ThrowInvalidOperationException($"Incremental broadphase integrity check failed for this tree (static={Static}). Did you modify a PhysicsTemporalCoherenceInfo component? PhysicsTemporalCoherenceInfo of Entity {entity.ToFixedString()} contains invalid node index {temporalCoherenceInfo.LastBvhNodeIndex} (expected: {index}).");
                            }

                            if (temporalCoherenceInfo.LastBvhLeafSlotIndex != i)
                            {
                                SafetyChecks.ThrowInvalidOperationException($"Incremental broadphase integrity check failed for this tree (static={Static}). Did you modify a PhysicsTemporalCoherenceInfo component? PhysicsTemporalCoherenceInfo of Entity {entity.ToFixedString()} contains invalid leaf slot index {temporalCoherenceInfo.LastBvhLeafSlotIndex} (expected: {i}).");
                            }

                            if (temporalCoherenceInfo.StaticBvh != Static)
                            {
                                SafetyChecks.ThrowInvalidOperationException($"Incremental broadphase integrity check failed for this tree (static={Static}). Did you modify a PhysicsTemporalCoherenceInfo component? PhysicsTemporalCoherenceInfo of Entity {entity.ToFixedString()} is for a different tree (static={temporalCoherenceInfo.StaticBvh}).");
                            }
                        }
                    }
                }
            }
        }
#endif

        #endregion
    }
}
