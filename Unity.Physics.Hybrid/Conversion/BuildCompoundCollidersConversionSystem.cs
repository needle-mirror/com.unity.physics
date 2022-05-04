using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections.NotBurstCompatible;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;
using Hash128 = Unity.Entities.Hash128;

namespace Unity.Physics.Authoring
{
    [UpdateBefore(typeof(EndColliderConversionSystem))]
    [AlwaysUpdateSystem]
    public sealed class BuildCompoundCollidersConversionSystem : GameObjectConversionSystem
    {
        // lifetime tied to system instance (i.e. LiveLink session) for incremental conversion
        NativeParallelMultiHashMap<Entity, ColliderInstance> m_AllLeafCollidersByBody;

        internal void SetLeafDirty(ColliderInstance leaf) => m_ChangedLeavesByBody.Add(leaf.BodyEntity, leaf);

        NativeParallelMultiHashMap<Entity, ColliderInstance> m_ChangedLeavesByBody;

        BeginColliderConversionSystem m_BeginColliderConversionSystem;
        EndColliderConversionSystem m_EndColliderConversionSystem;

        BlobAssetComputationContext<int, Collider> BlobComputationContext =>
            m_BeginColliderConversionSystem.BlobComputationContext;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_AllLeafCollidersByBody = new NativeParallelMultiHashMap<Entity, ColliderInstance>(16, Allocator.Persistent);
            m_ChangedLeavesByBody = new NativeParallelMultiHashMap<Entity, ColliderInstance>(16, Allocator.Persistent);
            m_BeginColliderConversionSystem = World.GetOrCreateSystem<BeginColliderConversionSystem>();
            m_EndColliderConversionSystem = World.GetOrCreateSystem<EndColliderConversionSystem>();
        }

        protected override void OnDestroy()
        {
            base.OnDestroy();
            m_AllLeafCollidersByBody.Dispose();
            m_ChangedLeavesByBody.Dispose();
        }

        struct ChildInstance : IComparable<ChildInstance>
        {
            public Hash128 Hash;
            public CompoundCollider.ColliderBlobInstance Child;

            public int CompareTo(ChildInstance other) => Hash.CompareTo(other.Hash);
        }

        protected override void OnUpdate()
        {
            Profiler.BeginSample("Build Compound Colliders");

            // Assign PhysicsCollider components to rigid body entities, merging multiples into compounds as needed
            var changedBodies = m_ChangedLeavesByBody.GetUniqueKeyArray(Allocator.Temp);
            using (changedBodies.Item1)
            {
                // Loop through all the bodies that changed
                for (var k = 0; k < changedBodies.Item2; ++k)
                {
                    var body = changedBodies.Item1[k];
                    var collider = DstEntityManager.HasComponent<PhysicsCollider>(body)
                        ? DstEntityManager.GetComponentData<PhysicsCollider>(body)
                        : new PhysicsCollider();
                    var children =
                        new NativeParallelHashMap<ColliderInstanceId, ChildInstance>(16, Allocator.Temp);

                    var isLeafEntityBody = true;

                    // The current body that changed may have one or more shape to process, loop through all of them
                    if (m_ChangedLeavesByBody.TryGetFirstValue(body, out var shape, out var iterator))
                    {
                        do
                        {
                            var replaced = false;

                            // Look for existing known shape. For this there is no magic, O(n) scan on the body's shapes
                            if (m_AllLeafCollidersByBody.TryGetFirstValue(body, out var existingShape, out var existingIterator))
                            {
                                do
                                {
                                    // If the current child is the one we care about then replace its associated data
                                    if (existingShape.ShapeEntity.Equals(shape.ShapeEntity))
                                    {
                                        m_AllLeafCollidersByBody.SetValue(shape, existingIterator);
                                        replaced = true;
                                        break;
                                    }
                                }
                                while (m_AllLeafCollidersByBody.TryGetNextValue(out existingShape, ref existingIterator));
                            }

                            // Add the shape if it did not exist already
                            if (!replaced)
                                m_AllLeafCollidersByBody.Add(body, shape);

                            // Add the shape to the list of children to process later
                            if (BlobComputationContext.GetBlobAsset(shape.Hash, out var blobAsset))
                            {
                                var child = new ChildInstance
                                {
                                    Hash = shape.Hash,
                                    Child = new CompoundCollider.ColliderBlobInstance
                                    {
                                        Collider = blobAsset,
                                        CompoundFromChild = shape.BodyFromShape,
                                        Entity = shape.ChildEntity
                                    }
                                };
                                children.Add(shape.ToColliderInstanceId(), child);

                                isLeafEntityBody &= shape.ShapeEntity.Equals(body);
                            }
                            else
                            {
                                var gameObject = m_EndColliderConversionSystem.GetConvertedAuthoringComponent(shape.ConvertedAuthoringComponentIndex).gameObject;
                                Debug.LogWarning($"Couldn't convert Collider for GameObject '{gameObject.name}'.");
                            }
                        }
                        while (m_ChangedLeavesByBody.TryGetNextValue(out shape, ref iterator));
                    }

                    // Add all children that did not change
                    if (m_AllLeafCollidersByBody.TryGetFirstValue(body, out shape, out var it))
                    {
                        do
                        {
                            isLeafEntityBody &= shape.ShapeEntity.Equals(body);

                            if (BlobComputationContext.GetBlobAsset(shape.Hash, out var blobAsset))
                            {
                                var child = new ChildInstance
                                {
                                    Hash = shape.Hash,
                                    Child = new CompoundCollider.ColliderBlobInstance
                                    {
                                        Collider = blobAsset,
                                        CompoundFromChild = shape.BodyFromShape,
                                        Entity = shape.ChildEntity
                                    }
                                };
                                children.TryAdd(shape.ToColliderInstanceId(), child);
                            }
                        }
                        while (m_AllLeafCollidersByBody.TryGetNextValue(out shape, ref it));
                    }

                    // Get the list of colliders to (re)build
                    var colliders = children.GetValueArray(Allocator.TempJob);

                    // If the leaf is the same entity as the body, and there is a single shape, use it as-is; otherwise create a compound
                    // (assume a single leaf should still be a compound so that local offset values in authoring representation are retained)
                    if (colliders.Length > 0)
                    {
                        if (isLeafEntityBody && colliders.Length == 1)
                        {
                            collider.Value = colliders[0].Child.Collider;
                        }
                        else
                        {
                            // sort children by hash to ensure deterministic results
                            // required because instance ID on hash map key is non-deterministic between runs,
                            // but it affects the order of values returned by NativeHashMap<T>.GetValueArray()
                            colliders.Sort();

                            // otherwise it is a compound
                            var childHashes = new NativeArray<Hash128>(colliders.Length, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                            var childOffsets = new NativeArray<RigidTransform>(colliders.Length, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                            var childBlobs = new NativeArray<CompoundCollider.ColliderBlobInstance>(colliders.Length, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
                            for (var i = 0; i < children.Count(); ++i)
                            {
                                childHashes[i] = colliders[i].Hash;
                                childOffsets[i] = colliders[i].Child.CompoundFromChild;
                                childBlobs[i] = colliders[i].Child;
                            }

                            Profiler.BeginSample("Generate Hash for Compound");
                            var compoundHash = new NativeArray<Hash128>(1, Allocator.TempJob);
                            new HashChildrenJob
                            {
                                ChildHashes = childHashes,
                                ChildOffsets = childOffsets,
                                Output = compoundHash
                            }.Run();
                            Profiler.EndSample();

                            var gameObject = m_EndColliderConversionSystem.GetConvertedAuthoringComponent(shape.ConvertedBodyTransformIndex).gameObject;
                            BlobComputationContext.AssociateBlobAssetWithUnityObject(compoundHash[0], gameObject);

                            if (!BlobComputationContext.NeedToComputeBlobAsset(compoundHash[0]))
                                BlobComputationContext.GetBlobAsset(compoundHash[0], out collider.Value);
                            else
                            {
                                BlobComputationContext.AddBlobAssetToCompute(compoundHash[0], 0);

                                using (var compound = new NativeArray<BlobAssetReference<Collider>>(1, Allocator.TempJob))
                                {
                                    new CreateCompoundJob
                                    {
                                        Children = childBlobs,
                                        Output = compound
                                    }.Run();
                                    collider.Value = compound[0];
                                    BlobComputationContext.AddComputedBlobAsset(compoundHash[0], collider.Value);
                                }
                            }

                            compoundHash.Dispose();
                            childBlobs.Dispose();

                            // If we created a CompoundCollider, add the mapping between each Child and original Entity.
                            var colliderKeyEntityBuffer = DstEntityManager.AddBuffer<PhysicsColliderKeyEntityPair>(body);
                            colliderKeyEntityBuffer.Clear();
                            unsafe
                            {
                                var compoundCollider = (CompoundCollider*)collider.ColliderPtr;
                                for (uint i = 0; i < compoundCollider->NumChildren; i++)
                                {
                                    var childEntity = compoundCollider->Children[(int)i].Entity;
                                    var childKey = new ColliderKey(compoundCollider->NumColliderKeyBits, i);

                                    colliderKeyEntityBuffer.Add(new PhysicsColliderKeyEntityPair() { Entity = childEntity, Key = childKey });
                                }
                            }
                        }
                    }

                    colliders.Dispose();
                    children.Dispose();

                    DstEntityManager.AddOrSetComponent(body, collider);
                }
            }

            m_ChangedLeavesByBody.Clear();

            Profiler.EndSample();
        }

#if !(UNITY_ANDROID && !UNITY_64) // !Android32
        // Getting memory alignment errors from HashUtility.Hash128 on Android32
        [BurstCompile]
#endif
        unsafe struct HashChildrenJob : IJob
        {
            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<Hash128> ChildHashes;
            [DeallocateOnJobCompletion]
            [ReadOnly] public NativeArray<RigidTransform> ChildOffsets;

            public NativeArray<Hash128> Output;

            public void Execute()
            {
                var bytes = new NativeList<byte>(Allocator.Temp);
                bytes.Append(ChildHashes);
                bytes.Append(ChildOffsets);
                Output[0] = HashUtility.Hash128(bytes.GetUnsafeReadOnlyPtr(), bytes.Length);
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        struct CreateCompoundJob : IJob
        {
            [ReadOnly] public NativeArray<CompoundCollider.ColliderBlobInstance> Children;

            public NativeArray<BlobAssetReference<Collider>> Output;

            public void Execute() => Output[0] = CompoundCollider.Create(Children);
        }
    }
}
