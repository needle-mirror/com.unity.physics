using System;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;

using UnityEngine.Profiling;
using Hash128 = Unity.Entities.Hash128;

namespace Unity.Physics.Authoring
{
    /// <summary>
    ///     Compound collider baking system.
    /// </summary>
    [UpdateBefore(typeof(EndColliderBakingSystem))]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    public partial class BuildCompoundCollidersBakingSystem : SystemBase
    {
        BeginColliderBakingSystem m_BeginColliderBakingSystem;

        BlobAssetComputationContext<int, Collider> BlobComputationContext =>
            m_BeginColliderBakingSystem.BlobComputationContext;

        EntityQuery m_RebakedRootQuery;
        EntityQuery m_RootQuery;
        EntityQuery m_ColliderSourceQuery;
        ComponentTypeSet m_StaticCleanUpTypes;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_BeginColliderBakingSystem = World.GetOrCreateSystemManaged<BeginColliderBakingSystem>();
            m_RebakedRootQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new[] { ComponentType.ReadOnly<PhysicsRootBaked>() },
                Options = EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities
            });
            m_RootQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new[] { ComponentType.ReadOnly<PhysicsCompoundData>() },
                Options = EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities
            });
            m_ColliderSourceQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new[] { ComponentType.ReadOnly<PhysicsColliderBakedData>() },
                Options = EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities
            });

            m_StaticCleanUpTypes = new ComponentTypeSet(
                typeof(PhysicsWorldIndex),
                typeof(PhysicsCollider),
                typeof(PhysicsColliderKeyEntityPair),
                typeof(PhysicsPostProcessData)
            );
        }

        struct ChildInstance : IComparable<ChildInstance>
        {
            public Hash128 Hash;
            public bool IsLeaf;
            public CompoundCollider.ColliderBlobInstance Child;

            public int CompareTo(ChildInstance other) => Hash.CompareTo(other.Hash);
        }

        struct DeferredCompoundResult
        {
            public Hash128 Hash;
            public BlobAssetReference<Collider> Result;
        }

        [WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities)]
        partial struct ChildrenGatheringJobHandleJob : IJobEntity
        {
            [ReadOnly] public NativeParallelHashMap<Entity, int> rootEntitiesLookUp;
            [ReadOnly] public BlobAssetComputationContext<int, Collider> blobComputationContext;
            public NativeParallelMultiHashMap<Entity, ChildInstance>.ParallelWriter childrenPerRootWriter;
            private void Execute(in PhysicsColliderBakedData blobBakingData)
            {
                // Check if we care about this entity
                if (rootEntitiesLookUp.ContainsKey(blobBakingData.BodyEntity))
                {
                    blobComputationContext.GetBlobAsset(blobBakingData.Hash, out var blobAsset);
                    childrenPerRootWriter.Add(blobBakingData.BodyEntity, new ChildInstance()
                    {
                        Hash = blobBakingData.Hash,
                        IsLeaf = blobBakingData.IsLeafEntityBody,
                        Child = new CompoundCollider.ColliderBlobInstance
                        {
                            Collider = blobAsset,
                            CompoundFromChild = blobBakingData.BodyFromShape,
                            Entity = blobBakingData.ChildEntity,
                        }
                    });
                }
            }
        }

        [WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities)]
        partial struct BlobCalculationJobHandleJob : IJobEntity
        {
            [ReadOnly] public NativeParallelMultiHashMap<Entity, ChildInstance> childrenPerRoot;
            [ReadOnly] public BlobAssetComputationContext<int, Collider> blobComputationContext;
            [ReadOnly] public NativeParallelHashMap<Entity, int> rootEntitiesLookUp;
            public NativeParallelHashMap<Hash128, int>.ParallelWriter deduplicationHashMapWriter;
            [NativeDisableParallelForRestriction] public NativeArray<DeferredCompoundResult> deferredCompoundResults;

            private void Execute(Entity rootEntity, ref PhysicsCompoundData rootBaking, ref PhysicsCollider rootCollider)
            {
                // Reset values in case of incremental
                rootCollider.Value = default;
                rootBaking.Hash = default;
                rootBaking.AssociateBlobToBody = false;
                rootBaking.DeferredCompoundBlob = false;
                rootBaking.RegisterBlob = false;

                if (childrenPerRoot.TryGetFirstValue(rootEntity, out var instance, out var it))
                {
                    // Look ahead to check if there is only one child and if it is a leaf, as this is a fast path
                    var itLookAhead = it;
                    if (instance.IsLeaf && !childrenPerRoot.TryGetNextValue(out var other, ref itLookAhead))
                    {
                        // Fast Path, we can reuse the blob data straightaway
                        rootCollider.Value = instance.Child.Collider;
                        rootBaking.Hash = instance.Hash;
                        rootBaking.AssociateBlobToBody = false;
                        rootBaking.DeferredCompoundBlob = false;
                    }
                    else
                    {
                        // We need to store the data in a list, so it can be sorted
                        var colliders = new NativeList<ChildInstance>(1, Allocator.Temp);
                        colliders.Add(instance);
                        while (childrenPerRoot.TryGetNextValue(out instance, ref it))
                        {
                            colliders.Add(instance);
                        }

                        // sort children by hash to ensure deterministic results
                        // required because instance ID on hash map key is non-deterministic between runs,
                        // but it affects the order of values returned by NativeParallelMultiHashMap
                        colliders.Sort();

                        // Calculate compound hash
                        var hashGenerator = new xxHash3.StreamingState(false);
                        foreach (var collider in colliders)
                        {
                            hashGenerator.Update(collider.Hash);
                            hashGenerator.Update(collider.Child.CompoundFromChild);
                        }

                        Hash128 compoundHash = new Hash128(hashGenerator.DigestHash128());
                        rootBaking.Hash = compoundHash;
                        rootBaking.AssociateBlobToBody = true;

                        if (blobComputationContext.NeedToComputeBlobAsset(compoundHash))
                        {
                            // We need to deduplicate before creating new compound colliders
                            int rootIndex = rootEntitiesLookUp[rootEntity];

                            if (!deduplicationHashMapWriter.TryAdd(rootBaking.Hash, rootIndex))
                            {
                                // We are not the first one to try adding this hash, so we are not responsible for calculating the compound blob and we will need to collect it later
                                rootBaking.DeferredCompoundBlob = true;
                                rootBaking.RegisterBlob = false;
                            }
                            else
                            {
                                // Calculate the Compound Collider
                                rootBaking.DeferredCompoundBlob = false;
                                rootBaking.RegisterBlob = true;
                                var colliderBlobs = new NativeArray<CompoundCollider.ColliderBlobInstance>(colliders.Length, Allocator.Temp);
                                for (int index = 0; index < colliders.Length; ++index)
                                {
                                    colliderBlobs[index] = colliders[index].Child;
                                }
                                rootCollider.Value = CompoundCollider.Create(colliderBlobs);
                                deferredCompoundResults[rootIndex] = new DeferredCompoundResult()
                                {
                                    Hash = compoundHash,
                                    Result = rootCollider.Value
                                };
                                colliderBlobs.Dispose();
                            }
                        }
                        else
                        {
                            // Look up the blob in the BlobComputationContext
                            blobComputationContext.GetBlobAsset(compoundHash, out var blob);
                            rootCollider.Value = blob;
                            rootBaking.DeferredCompoundBlob = false;
                            rootBaking.RegisterBlob = false;
                        }
                        colliders.Dispose();
                    }
                }
            }
        }

        [BurstCompile]
        struct AddColliderKeyEntityPairBufferJob : IJobChunk
        {
            [ReadOnly] public ComponentTypeHandle<PhysicsCompoundData> PhysicsCompoundDataHandle;
            public EntityTypeHandle Entities;
            public EntityCommandBuffer.ParallelWriter ECB;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask,
                in v128 chunkEnabledMask)
            {
                var entities = chunk.GetNativeArray(Entities);
                var compoundDataArray = chunk.GetNativeArray(ref PhysicsCompoundDataHandle);
                for (int i = 0; i < entities.Length; ++i)
                {
                    if (compoundDataArray[i].AssociateBlobToBody)
                    {
                        ECB.AddBuffer<PhysicsColliderKeyEntityPair>(unfilteredChunkIndex, entities[i]);
                    }
                }
            }
        };

        [WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities)]
        partial struct DeferredResolutionJobHandleJob : IJobEntity
        {
            [ReadOnly] public NativeParallelHashMap<Hash128, int> deduplicationHashMap;
            [ReadOnly] public NativeArray<DeferredCompoundResult> deferredCompoundResults;

            private void Execute(ref DynamicBuffer<PhysicsColliderKeyEntityPair> colliderKeyEntityBuffer, ref PhysicsCollider rootCollider, in PhysicsCompoundData rootBaking)
            {
                colliderKeyEntityBuffer.Clear();

                if (rootBaking.AssociateBlobToBody)
                {
                    if (rootBaking.DeferredCompoundBlob)
                    {
                        // We need to collect the compound blob
                        int resultIndex = deduplicationHashMap[rootBaking.Hash];
                        rootCollider.Value = deferredCompoundResults[resultIndex].Result;
                    }

                    // Fill in the children colliders
                    unsafe
                    {
                        var compoundCollider = (CompoundCollider*)rootCollider.ColliderPtr;
                        for (int i = 0; i < compoundCollider->NumChildren; i++)
                        {
                            colliderKeyEntityBuffer.Add(new PhysicsColliderKeyEntityPair()
                            {
                                Entity = compoundCollider->Children[i].Entity,
                                Key = compoundCollider->ConvertChildIndexToColliderKey(i)
                            });
                        }
                    }
                }
            }
        }

        [WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities)]
        partial struct BlobContextUpdateJobHandleJob : IJobEntity
        {
            public BlobAssetComputationContext<int, Collider> blobComputationContext;
            [ReadOnly] public NativeParallelHashMap<Hash128, int> deduplicationHashMap;
            [ReadOnly] public NativeArray<DeferredCompoundResult> deferredCompoundResults;

            private void Execute(in PhysicsCompoundData rootBaking)
            {
                if (rootBaking.AssociateBlobToBody && rootBaking.RegisterBlob)
                {
                    // Register the blob asset if needed
                    int resultIndex = deduplicationHashMap[rootBaking.Hash];
                    blobComputationContext.AddComputedBlobAsset(rootBaking.Hash, deferredCompoundResults[resultIndex].Result);
                }
            }
        }
        protected override void OnUpdate()
        {
            Profiler.BeginSample("Build Compound Colliders");

            int maxRootCount = m_RootQuery.CalculateEntityCount();

            // Collect the root entities that needs regeneration because they rebaked
            using var rootEntities = m_RebakedRootQuery.ToEntityArray(Allocator.TempJob);
            NativeParallelHashMap<Entity, int> rootEntitiesLookUp = new NativeParallelHashMap<Entity, int>(maxRootCount, Allocator.TempJob);
            int count = 0;
            foreach (var rootEntity in rootEntities)
            {
                rootEntitiesLookUp.Add(rootEntity, count);
                ++count;
            }

            // Collect all the root entities that didn't rebake but any of their colliders did
            if (m_ColliderSourceQuery.CalculateChunkCount() > 0)
            {
                using var rebakedColliders = m_ColliderSourceQuery.ToComponentDataArray<PhysicsColliderBakedData>(Allocator.TempJob);
                foreach (var collider in rebakedColliders)
                {
                    if (!rootEntitiesLookUp.ContainsKey(collider.BodyEntity) && m_RootQuery.Matches(collider.BodyEntity))
                    {
                        rootEntitiesLookUp.Add(collider.BodyEntity, count);
                        ++count;
                    }
                }
            }

            // Collect the relevant collider info for each root
            var maxColliderCount = m_ColliderSourceQuery.CalculateEntityCount();

            var deferredCompoundResults = new NativeArray<DeferredCompoundResult>(count, Allocator.TempJob);
            var deduplicationHashMap = new NativeParallelHashMap<Hash128, int>(maxRootCount, Allocator.TempJob);
            var deduplicationHashMapWriter = deduplicationHashMap.AsParallelWriter();

            NativeParallelMultiHashMap<Entity, ChildInstance> childrenPerRoot = new NativeParallelMultiHashMap<Entity, ChildInstance>(maxColliderCount, Allocator.TempJob);
            var childrenPerRootWriter = childrenPerRoot.AsParallelWriter();
            var blobComputationContext = BlobComputationContext;

            JobHandle childrenGatheringJobHandle = new ChildrenGatheringJobHandleJob
            {
                rootEntitiesLookUp = rootEntitiesLookUp,
                blobComputationContext = blobComputationContext,
                childrenPerRootWriter = childrenPerRootWriter
            }.ScheduleParallel(default(JobHandle));

            // We need to compose the blobs
            JobHandle blobCalculationJobHandle = new BlobCalculationJobHandleJob
            {
                childrenPerRoot = childrenPerRoot,
                blobComputationContext = blobComputationContext,
                rootEntitiesLookUp = rootEntitiesLookUp,
                deduplicationHashMapWriter = deduplicationHashMapWriter,
                deferredCompoundResults = deferredCompoundResults
            }.ScheduleParallel(childrenGatheringJobHandle);

            Profiler.EndSample();

            // Add the PhysicsColliderKeyEntityPair buffer to the root entities
            var ecb = new EntityCommandBuffer(Allocator.TempJob);
            JobHandle addBufferJobHandle = new AddColliderKeyEntityPairBufferJob
            {
                PhysicsCompoundDataHandle = GetComponentTypeHandle<PhysicsCompoundData>(true),
                Entities = GetEntityTypeHandle(),
                ECB = ecb.AsParallelWriter()
            }.Schedule(m_RootQuery, blobCalculationJobHandle);

            // Update the blob assets relation to the authoring component
            JobHandle blobContextUpdateJobHandle = new BlobContextUpdateJobHandleJob
            {
                blobComputationContext = blobComputationContext,
                deduplicationHashMap = deduplicationHashMap,
                deferredCompoundResults = deferredCompoundResults
            }.Schedule(blobCalculationJobHandle);

            var combinedJobHandle = JobHandle.CombineDependencies(blobContextUpdateJobHandle, addBufferJobHandle);

            combinedJobHandle.Complete();

            ecb.Playback(EntityManager);
            ecb.Dispose();

            // Update the PhysicsColliderKeyEntityPair buffer
            new DeferredResolutionJobHandleJob
            {
                deduplicationHashMap = deduplicationHashMap,
                deferredCompoundResults = deferredCompoundResults
            }.ScheduleParallel(addBufferJobHandle).Complete();

            // Check for unused StaticOptimizeEntity roots
            ecb = new EntityCommandBuffer(WorldUpdateAllocator);
            foreach (var(_, entity) in SystemAPI.Query<RefRO<StaticOptimizePhysicsBaking>>()
                     .WithEntityAccess()
                     .WithOptions(EntityQueryOptions.IncludePrefab | EntityQueryOptions.IncludeDisabledEntities))
            {
                if (!childrenPerRoot.ContainsKey(entity))
                {
                    // There was a StaticOptimizeEntity root that was not used, so we need to clean up the added components
                    ecb.RemoveComponent(entity, m_StaticCleanUpTypes);
                }
            }
            ecb.Playback(EntityManager);
            ecb.Dispose();

            deferredCompoundResults.Dispose(combinedJobHandle);
            deduplicationHashMap.Dispose(combinedJobHandle);
            childrenPerRoot.Dispose(combinedJobHandle);
            rootEntitiesLookUp.Dispose(combinedJobHandle);
        }
    }
}
