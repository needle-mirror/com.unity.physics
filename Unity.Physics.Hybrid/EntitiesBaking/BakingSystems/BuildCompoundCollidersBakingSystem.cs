using System;
using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;

using UnityEngine.Profiling;
using Hash128 = Unity.Entities.Hash128;

namespace Unity.Physics.Authoring
{
    [UpdateBefore(typeof(EndColliderBakingSystem))]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    public partial class BuildCompoundCollidersBakingSystem : SystemBase
    {
        BeginColliderBakingSystem m_BeginColliderBakingSystem;

        BlobAssetComputationContext<int, Collider> BlobComputationContext =>
            m_BeginColliderBakingSystem.BlobComputationContext;

        EntityQuery m_RebakedRootQuery;
        EntityQuery m_RebakedCollidersQuery;
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
                Options = EntityQueryOptions.IncludePrefab
            });
            m_RebakedCollidersQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new[] { ComponentType.ReadOnly<PhysicsColliderAuthoringData>(), ComponentType.ReadOnly<PhysicsColliderBakedData>() },
                Options = EntityQueryOptions.IncludePrefab
            });
            m_RootQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new[] { ComponentType.ReadOnly<PhysicsCompoundData>() },
                Options = EntityQueryOptions.IncludePrefab
            });
            m_ColliderSourceQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new[] { ComponentType.ReadOnly<PhysicsColliderBakedData>() },
                Options = EntityQueryOptions.IncludePrefab
            });

            GetEntityQuery(new EntityQueryDesc
            {
                All = new[] { ComponentType.ReadOnly<PhysicsCompoundData>() },
                Options = EntityQueryOptions.IncludePrefab
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
            if (m_RebakedCollidersQuery.CalculateChunkCount() > 0)
            {
                using var rebakedColliders = m_RebakedCollidersQuery.ToComponentDataArray<PhysicsColliderBakedData>(Allocator.TempJob);
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

            NativeMultiHashMap<Entity, ChildInstance> childrenPerRoot = new NativeMultiHashMap<Entity, ChildInstance>(maxColliderCount, Allocator.TempJob);
            var childrenPerRootWriter = childrenPerRoot.AsParallelWriter();
            var blobComputationContext = BlobComputationContext;
            var childrenGatheringJobHandle =
                Entities
                    .ForEach((in PhysicsColliderBakedData blobBakingData) =>
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
                }).WithEntityQueryOptions(EntityQueryOptions.IncludePrefab)
                    .WithReadOnly(blobComputationContext)
                    .WithReadOnly(rootEntitiesLookUp)
                    .ScheduleParallel(default);

            // We need to compose the blobs
            var blobCalculationJobHandle =
                Entities
                    .ForEach((Entity rootEntity, ref PhysicsCompoundData rootBaking, ref PhysicsCollider rootCollider) =>
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
                            // but it affects the order of values returned by NativeMultiHashMap
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
                }).WithNativeDisableParallelForRestriction(deferredCompoundResults)
                    .WithReadOnly(blobComputationContext)
                    .WithReadOnly(rootEntitiesLookUp)
                    .WithReadOnly(childrenPerRoot)
                    .WithEntityQueryOptions(EntityQueryOptions.IncludePrefab)
                    .ScheduleParallel(childrenGatheringJobHandle);

            Profiler.EndSample();

            // Update the PhysicsColliderKeyEntityPair buffer
            var deferredResolutionJobHandle =
                Entities
                    .ForEach((ref DynamicBuffer<PhysicsColliderKeyEntityPair> colliderKeyEntityBuffer, ref PhysicsCollider rootCollider, in PhysicsCompoundData rootBaking) =>
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
                            for (uint i = 0; i < compoundCollider->NumChildren; i++)
                            {
                                var childEntity = compoundCollider->Children[(int)i].Entity;
                                var childKey = new ColliderKey(compoundCollider->NumColliderKeyBits, i);

                                colliderKeyEntityBuffer.Add(new PhysicsColliderKeyEntityPair()
                                {
                                    Entity = childEntity,
                                    Key = childKey
                                });
                            }
                        }
                    }
                }).WithReadOnly(deduplicationHashMap)
                    .WithReadOnly(deferredCompoundResults)
                    .WithEntityQueryOptions(EntityQueryOptions.IncludePrefab)
                    .ScheduleParallel(blobCalculationJobHandle);

            // Update the blob assets relation to the authoring component
            var blobContextUpdateJobHandle =
                Entities
                    .ForEach((in PhysicsCompoundData rootBaking) =>
                {
                    if (rootBaking.AssociateBlobToBody)
                    {
                        if (rootBaking.RegisterBlob)
                        {
                            // Register the blob asset if needed
                            int resultIndex = deduplicationHashMap[rootBaking.Hash];
                            blobComputationContext.AddComputedBlobAsset(rootBaking.Hash, deferredCompoundResults[resultIndex].Result);
                        }

                        // Register the hash to the body authoring object
                        blobComputationContext.AssociateBlobAssetWithUnityObject(rootBaking.Hash, rootBaking.ConvertedBodyInstanceID);
                    }
                }).WithReadOnly(deduplicationHashMap)
                    .WithReadOnly(deferredCompoundResults)
                    .WithEntityQueryOptions(EntityQueryOptions.IncludePrefab)
                    .Schedule(blobCalculationJobHandle);

            var combinedJobHandle = JobHandle.CombineDependencies(blobContextUpdateJobHandle, deferredResolutionJobHandle);

            combinedJobHandle.Complete();

            // Check for unused StaticOptimizeEntity roots
            var manager = EntityManager;
            Entities
                .WithAll<StaticOptimizePhysicsBaking>()
                .ForEach((Entity entity) =>
                {
                    if (!childrenPerRoot.ContainsKey(entity))
                    {
                        // There was a StaticOptimizeEntity root that was not used, so we need to clean up the added components
                        manager.RemoveComponent(entity, m_StaticCleanUpTypes);
                    }
                }).WithStructuralChanges().WithReadOnly(childrenPerRoot).Run();

            deferredCompoundResults.Dispose(combinedJobHandle);
            deduplicationHashMap.Dispose(combinedJobHandle);
            childrenPerRoot.Dispose(combinedJobHandle);
            rootEntitiesLookUp.Dispose(combinedJobHandle);
        }
    }
}
