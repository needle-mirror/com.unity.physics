using System;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Physics.Aspects;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Physics.Systems;
using Unity.Transforms;

namespace Unity.Physics
{
    /// <summary>
    /// A collection of rigid bodies wrapped by a bounding volume hierarchy. This allows to do
    /// collision queries such as raycasting, overlap testing, etc.
    /// </summary>
    [NoAlias]
    public struct CollisionWorld : ICollidable, IAspectQueryable, IDisposable
    {
        [NoAlias] private NativeArray<RigidBody> m_Bodies;    // storage for all the rigid bodies
        [NoAlias] internal Broadphase Broadphase;             // bounding volume hierarchies around subsets of the rigid bodies
        [NoAlias] internal NativeParallelHashMap<Entity, int> EntityBodyIndexMap;
        [NativeDisableContainerSafetyRestriction]
        [NoAlias] private NativeList<BlobAssetReference<Collider>> m_ColliderDeepCopies;

        /// <summary>   Gets the number of bodies. </summary>
        ///
        /// <value> The total number of bodies. </value>
        public int NumBodies => Broadphase.NumStaticBodies + Broadphase.NumDynamicBodies;

        /// <summary>   Gets the number of static bodies. </summary>
        ///
        /// <value> The total number of static bodies. </value>
        public int NumStaticBodies => Broadphase.NumStaticBodies;

        /// <summary>   Gets the number of dynamic bodies. </summary>
        ///
        /// <value> The total number of dynamic bodies. </value>
        public int NumDynamicBodies => Broadphase.NumDynamicBodies;

        /// <summary>   Gets the bodies. </summary>
        ///
        /// <value> The bodies. </value>
        public NativeArray<RigidBody> Bodies => m_Bodies.GetSubArray(0, NumBodies);

        /// <summary>   Gets the static bodies. </summary>
        ///
        /// <value> The static bodies. </value>
        public NativeArray<RigidBody> StaticBodies => m_Bodies.GetSubArray(NumDynamicBodies, NumStaticBodies);

        /// <summary>   Gets the dynamic bodies. </summary>
        ///
        /// <value> The dynamic bodies. </value>
        public NativeArray<RigidBody> DynamicBodies => m_Bodies.GetSubArray(0, NumDynamicBodies);

        /// <summary>
        /// <para>Sets the collision tolerance.</para>
        /// <para>Contacts are always created between rigid bodies if they are closer than this distance threshold. Default
        /// value is <see cref="DefaultCollisionTolerance"/>.</para>
        /// </summary>
        ///
        /// <value> The collision tolerance. </value>
        public float CollisionTolerance;

        /// <summary>
        /// Default <see cref="CollisionTolerance"/> value.
        /// </summary>
        public const float DefaultCollisionTolerance = 0.01f;

        /// <summary>
        /// Construct a collision world with the given number of uninitialized rigid bodies.
        /// </summary>
        ///
        /// <param name="numStaticBodies">  Number of static bodies. </param>
        /// <param name="numDynamicBodies"> Number of dynamic bodies. </param>
        public CollisionWorld(int numStaticBodies, int numDynamicBodies)
        {
            m_Bodies = new NativeArray<RigidBody>(numStaticBodies + numDynamicBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            Broadphase = new Broadphase(numStaticBodies, numDynamicBodies);
            EntityBodyIndexMap = new NativeParallelHashMap<Entity, int>(m_Bodies.Length, Allocator.Persistent);
            m_ColliderDeepCopies = new NativeList<BlobAssetReference<Collider>>();
            CollisionTolerance = DefaultCollisionTolerance;
        }

        /// <summary>   Resets this object. </summary>
        ///
        /// <param name="numStaticBodies">  Number of static bodies. </param>
        /// <param name="numDynamicBodies"> Number of dynamic bodies. </param>
        public void Reset(int numStaticBodies, int numDynamicBodies)
        {
            SetCapacity(numStaticBodies + numDynamicBodies);
            Broadphase.Reset(numStaticBodies, numDynamicBodies);
            EntityBodyIndexMap.Clear();
        }

        /// <summary>   Sets the capacity. </summary>
        ///
        /// <param name="numBodies">    Number of bodies. </param>
        private void SetCapacity(int numBodies)
        {
            // Increase body storage if necessary
            if (m_Bodies.Length < numBodies)
            {
                m_Bodies.Dispose();
                m_Bodies = new NativeArray<RigidBody>(numBodies, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                EntityBodyIndexMap.Capacity = m_Bodies.Length;
            }
        }

        /// <summary>   Free internal memory. </summary>
        public void Dispose()
        {
            if (m_Bodies.IsCreated)
            {
                m_Bodies.Dispose();
            }
            Broadphase.Dispose();
            if (EntityBodyIndexMap.IsCreated)
            {
                EntityBodyIndexMap.Dispose();
            }

            if (m_ColliderDeepCopies.IsCreated)
            {
                foreach (var blob in m_ColliderDeepCopies)
                {
                    blob.Dispose();
                }
                m_ColliderDeepCopies.Dispose();
            }
        }

        /// <summary>
        /// <para>Clone the world.</para>
        /// <para>
        /// The Broadphase is deep copied.
        /// All data in rigid bodies is deep copied, except for their colliders, which are shallow copied.
        /// </para>
        /// </summary>
        ///
        /// <returns>   A copy of this object. </returns>
        public CollisionWorld Clone()
        {
            var clone = new CollisionWorld
            {
                m_Bodies = new NativeArray<RigidBody>(m_Bodies, Allocator.Persistent),
                Broadphase = Broadphase.Clone(),
                EntityBodyIndexMap = new NativeParallelHashMap<Entity, int>(m_Bodies.Length, Allocator.Persistent),
                m_ColliderDeepCopies = default
            };
            clone.UpdateBodyIndexMap();
            return clone;
        }

        /// <summary>
        /// <para>Clone the world.</para>
        /// <para>
        /// The Broadphase is deep copied.
        /// All data in rigid bodies is deep copied, except for their colliders,
        /// which are optionally deep copied or shallow copied.
        /// </para>
        /// </summary>
        ///
        /// <param name="deepCopyDynamicColliders">     Deep copy all dynamic colliders.</param>
        /// <param name="deepCopyStaticColliders">      Deep copy all static colliders.</param>
        /// <param name="deepCopyRigidBodyList">        Deep copy colliders of the rigid bodies in the list, each specified
        ///                                             by its <see cref="GetRigidBodyIndex">index</see>.
        ///                                             Must not contain duplicate entries, and must not contain indices of rigid bodies whose
        ///                                             colliders will already be deep copied due to the values chosen for the
        ///                                             <paramref name="deepCopyDynamicColliders"/> and
        ///                                             <paramref name="deepCopyStaticColliders"/> parameters.</param>
        /// <returns>   A copy of this object. </returns>
        public CollisionWorld Clone(bool deepCopyDynamicColliders, bool deepCopyStaticColliders = false, NativeList<int> deepCopyRigidBodyList = default)
        {
            SafetyChecks.CheckValidCloneRequestAndThrow(this, deepCopyDynamicColliders, deepCopyStaticColliders, deepCopyRigidBodyList);

            var clone = new CollisionWorld
            {
                m_Bodies = new NativeArray<RigidBody>(m_Bodies, Allocator.Persistent),
                Broadphase = Broadphase.Clone(),
                EntityBodyIndexMap = new NativeParallelHashMap<Entity, int>(m_Bodies.Length, Allocator.Persistent),
                m_ColliderDeepCopies = default
            };
            clone.UpdateBodyIndexMap();

            if (!deepCopyDynamicColliders && !deepCopyStaticColliders && deepCopyRigidBodyList.IsEmpty)
            {
                // no deep copies requested.
                return clone;
            }
            // else:

            // Deep copy colliders:

            // Compute required number of deep copies, assuming there are no overlaps between the requested static / dynamic collider deep copies
            // and the provided rigid body indices.
            var numDeepCopies = (deepCopyDynamicColliders ? NumDynamicBodies : 0)
                + (deepCopyStaticColliders ? NumStaticBodies : 0)
                + (deepCopyRigidBodyList.IsEmpty ? 0 : deepCopyRigidBodyList.Length);

            if (numDeepCopies == 0)
            {
                return clone;
            }
            // else:

            clone.m_ColliderDeepCopies = new NativeList<BlobAssetReference<Collider>>(numDeepCopies, Allocator.Persistent);

            // Deep copy dynamic colliders
            if (deepCopyDynamicColliders)
            {
                CloneColliders(clone.DynamicBodies, clone.m_ColliderDeepCopies);
            }

            // Deep copy static colliders
            if (deepCopyStaticColliders)
            {
                CloneColliders(clone.StaticBodies, clone.m_ColliderDeepCopies);
            }

            // Deep copy colliders of rigid bodies specified by index
            if (!deepCopyRigidBodyList.IsEmpty)
            {
                foreach (var i in deepCopyRigidBodyList)
                {
                    var body = clone.m_Bodies[i];
                    if (body.Collider.IsCreated)
                    {
                        var colliderClone = body.Collider.Value.Clone();
                        body.Collider = colliderClone;
                        clone.m_Bodies[i] = body;
                        clone.m_ColliderDeepCopies.Add(colliderClone);
                    }
                }
            }

            return clone;
        }

        static void CloneColliders(NativeArray<RigidBody> bodies, NativeList<BlobAssetReference<Collider>> colliderClones)
        {
            var numBodies = bodies.Length;
            for (int i = 0; i < numBodies; ++i)
            {
                var body = bodies[i];
                if (body.Collider.IsCreated)
                {
                    // clone and replace collider. Then add it to the collider clone list.
                    var colliderClone = body.Collider.Value.Clone();
                    body.Collider = colliderClone;
                    bodies[i] = body;
                    colliderClones.Add(colliderClone);
                }
            }
        }

        /// <summary>   Updates the body index map. </summary>
        public void UpdateBodyIndexMap()
        {
            EntityBodyIndexMap.Clear();
            for (int i = 0; i < m_Bodies.Length; i++)
            {
                EntityBodyIndexMap[m_Bodies[i].Entity] = i;
            }
        }

        /// <summary>   Gets the zero-based index of the rigid body. </summary>
        ///
        /// <param name="entity">   The entity. </param>
        ///
        /// <returns>   The rigid body index. </returns>
        public int GetRigidBodyIndex(Entity entity)
        {
            return EntityBodyIndexMap.TryGetValue(entity, out var index) ? index : -1;
        }

        /// <summary>   Build the broadphase based on the given world. </summary>
        ///
        /// <param name="world">            [in,out] The world. </param>
        /// <param name="timeStep">         The time step. </param>
        /// <param name="gravity">          The gravity. </param>
        /// <param name="buildStaticTree">  (Optional) True to build static tree. </param>
        public void BuildBroadphase(ref PhysicsWorld world, float timeStep, float3 gravity, bool buildStaticTree = true)
        {
            Broadphase.Build(world.StaticBodies, world.DynamicBodies, world.MotionVelocities,
                world.CollisionWorld.CollisionTolerance, timeStep, gravity, buildStaticTree, reset: true);
        }

        /// <summary>   Schedule a set of jobs to build the broadphase based on the given world. </summary>
        ///
        /// <param name="world">            [in,out] The world. </param>
        /// <param name="timeStep">         The time step. </param>
        /// <param name="gravity">          The gravity. </param>
        /// <param name="buildStaticTree">  The build static tree. </param>
        /// <param name="inputDeps">        The input deps. </param>
        /// <param name="multiThreaded">    (Optional) True if multi threaded. </param>
        ///
        /// <returns>   A JobHandle. </returns>
        public JobHandle ScheduleBuildBroadphaseJobs(ref PhysicsWorld world, float timeStep, float3 gravity, NativeReference<int>.ReadOnly buildStaticTree, JobHandle inputDeps, bool multiThreaded = true)
        {
            return Broadphase.ScheduleBuildJobs(ref world, timeStep, gravity, buildStaticTree, inputDeps, multiThreaded, reset: true);
        }

        static NativeArray<T> CreateArrayCopy<T>(NativeArray<T> array, Allocator allocator)
            where T : unmanaged
        {
            var arrayCopy = CollectionHelper.CreateNativeArray<T>(array.Length, allocator, NativeArrayOptions.UninitializedMemory);
            arrayCopy.CopyFrom(array);
            return arrayCopy;
        }

        internal JobHandle ScheduleBuildBroadphaseJobs(ref PhysicsWorld world, float timeStep, float3 gravity, int numDynamicBodies, int numStaticBodies, NativeReference<int>.ReadOnly buildStaticTree,
            EntityQuery dynamicEntityQuery, EntityQuery staticEntityQuery, EntityQuery invalidatedTemporalCoherenceInfoQuery, NativeArray<int> dynamicBodyChunkBaseEntityIndices, NativeArray<int> staticBodyChunkBaseEntityIndices,
            JobHandle inputDeps, Allocator worldUpdateAllocator, in PhysicsWorldData.PhysicsWorldComponentHandles componentHandles, uint lastSystemVersion,
            bool multiThreaded, bool incrementalDynamicBroadphase, bool incrementalStaticBroadphase)
        {
            using var jobHandles = new NativeList<JobHandle>(16, Allocator.Temp);
            jobHandles.Add(inputDeps);

            NativeArray<int> numDynamicBodyRemovalChunksArray = default;
            NativeArray<int> numStaticBodyRemovalChunksArray = default;
            int numInvalidatedTemporalCoherenceInfoChunks = 0;
            int numDynamicEntityChunks = 0;
            int numStaticEntityChunks = 0;
            int numDynamicOrStaticEntityChunks = 0;
            NativeStream removeDynamicBodyDataStream = default;
            NativeStream removeStaticBodyDataStream = default;
            NativeStream updateDynamicBodyDataStream = default;
            NativeStream updateStaticBodyDataStream = default;
            NativeStream insertDynamicBodyDataStream = default;
            NativeStream insertStaticBodyDataStream = default;
            JobHandle removeDynamicBodyStreamHandle = default;
            JobHandle removeStaticBodyStreamHandle = default;

            if (incrementalDynamicBroadphase || incrementalStaticBroadphase)
            {
                numDynamicBodyRemovalChunksArray = CollectionHelper.CreateNativeArray<int>(1, worldUpdateAllocator, NativeArrayOptions.UninitializedMemory);
                numStaticBodyRemovalChunksArray = CollectionHelper.CreateNativeArray<int>(1, worldUpdateAllocator, NativeArrayOptions.UninitializedMemory);
                numInvalidatedTemporalCoherenceInfoChunks = invalidatedTemporalCoherenceInfoQuery.CalculateChunkCount();
                numDynamicBodyRemovalChunksArray[0] = numInvalidatedTemporalCoherenceInfoChunks;
                numStaticBodyRemovalChunksArray[0] = numInvalidatedTemporalCoherenceInfoChunks;

                if (numStaticBodies > 0)
                {
                    numStaticEntityChunks = staticEntityQuery.CalculateChunkCount();
                    numDynamicOrStaticEntityChunks += numStaticEntityChunks;
                }

                if (numDynamicBodies > 0)
                {
                    numDynamicEntityChunks = dynamicEntityQuery.CalculateChunkCount();
                    numDynamicOrStaticEntityChunks += numDynamicEntityChunks;
                }

                numDynamicBodyRemovalChunksArray[0] += numDynamicOrStaticEntityChunks;
                numStaticBodyRemovalChunksArray[0] += numDynamicOrStaticEntityChunks;

                // Two reasons we need a remove body stream handle when either the incremental dynamic or static broadphase is used:
                // 1. Reinsertion required for bodies that have changed and were previously in the same tree.
                // 2. Removal of bodies that have been moved to the other tree. This is why we don't check here if there are
                //    any dynamic or static bodies currently available (numDynamicBodies > 0, numStaticBodies > 0).
                if (numDynamicBodyRemovalChunksArray[0] > 0)
                {
                    removeDynamicBodyStreamHandle = NativeStream.ScheduleConstruct(out removeDynamicBodyDataStream,
                        numDynamicBodyRemovalChunksArray, inputDeps, worldUpdateAllocator);
                    jobHandles.Add(removeDynamicBodyStreamHandle);
                }

                if (numStaticBodyRemovalChunksArray[0] > 0)
                {
                    removeStaticBodyStreamHandle = NativeStream.ScheduleConstruct(out removeStaticBodyDataStream,
                        numStaticBodyRemovalChunksArray, inputDeps, worldUpdateAllocator);
                    jobHandles.Add(removeStaticBodyStreamHandle);
                }
            }

            if (numDynamicBodies > 0 && incrementalDynamicBroadphase)
            {
                var numChunksArray = CollectionHelper.CreateNativeArray<int>(1, worldUpdateAllocator,
                    NativeArrayOptions.UninitializedMemory);
                numChunksArray[0] = numDynamicEntityChunks;

                var insertDynamicBodyStreamHandle = NativeStream.ScheduleConstruct(
                    out insertDynamicBodyDataStream,
                    numChunksArray, inputDeps, worldUpdateAllocator);

                var updateDynamicBodyStreamHandle = NativeStream.ScheduleConstruct(
                    out updateDynamicBodyDataStream,
                    numChunksArray, inputDeps, worldUpdateAllocator);

                jobHandles.Add(JobHandle.CombineDependencies(insertDynamicBodyStreamHandle, updateDynamicBodyStreamHandle));
            }

            if (numStaticBodies > 0 && incrementalStaticBroadphase)
            {
                var numChunksArray = CollectionHelper.CreateNativeArray<int>(1, worldUpdateAllocator, NativeArrayOptions.UninitializedMemory);
                numChunksArray[0] = numStaticEntityChunks;

                var insertStaticBodyStreamHandle = NativeStream.ScheduleConstruct(
                    out insertStaticBodyDataStream,
                    numChunksArray, inputDeps, worldUpdateAllocator);

                var updateStaticBodyStreamHandle = NativeStream.ScheduleConstruct(
                    out updateStaticBodyDataStream,
                    numChunksArray, inputDeps, worldUpdateAllocator);

                jobHandles.Add(JobHandle.CombineDependencies(insertStaticBodyStreamHandle, updateStaticBodyStreamHandle));
            }

            if (numInvalidatedTemporalCoherenceInfoChunks > 0)
            {
                // fallback creation in case the removal streams have not yet been created
                if (!removeDynamicBodyDataStream.IsCreated)
                {
                    removeDynamicBodyStreamHandle = NativeStream.ScheduleConstruct(out removeDynamicBodyDataStream, numDynamicBodyRemovalChunksArray, inputDeps, worldUpdateAllocator);
                }
                if (!removeStaticBodyDataStream.IsCreated)
                {
                    removeStaticBodyStreamHandle = NativeStream.ScheduleConstruct(out removeStaticBodyDataStream, numStaticBodyRemovalChunksArray, inputDeps, worldUpdateAllocator);
                }

                var removeBodyStreamDependency = JobHandle.CombineDependencies(removeDynamicBodyStreamHandle, removeStaticBodyStreamHandle);
                var collectInvalidatedCoherenceInfoJob = new CollectInvalidatedTemporalCoherenceInfoJob
                {
                    PhysicsTemporalCoherenceInfoTypeRW = componentHandles.PhysicsTemporalCoherenceInfoTypeRW,
                    RemoveDynamicBodyDataWriter = removeDynamicBodyDataStream.AsWriter(),
                    RemoveStaticBodyDataWriter = removeStaticBodyDataStream.AsWriter(),
                    DynamicStreamBufferOffset = numDynamicOrStaticEntityChunks,
                    StaticStreamBufferOffset = numDynamicOrStaticEntityChunks
                }.ScheduleParallel(invalidatedTemporalCoherenceInfoQuery, removeBodyStreamDependency);

                jobHandles.Add(collectInvalidatedCoherenceInfoJob);
            }

            if (incrementalStaticBroadphase)
            {
                world.CollisionWorld.Broadphase.StaticTree.RemoveBodyDataStream = removeStaticBodyDataStream;
            }

            if (incrementalDynamicBroadphase)
            {
                world.CollisionWorld.Broadphase.DynamicTree.RemoveBodyDataStream = removeDynamicBodyDataStream;
            }

            inputDeps = JobHandle.CombineDependencies(jobHandles.AsArray());
            jobHandles.Clear();

            if (incrementalDynamicBroadphase || incrementalStaticBroadphase)
            {
                JobHandle collectDynamicCoherenceInfoHandle = inputDeps;
                JobHandle collectStaticCoherenceInfoHandle = inputDeps;

                if (incrementalDynamicBroadphase)
                {
                    if (numDynamicBodies > 0)
                    {
                        var bodyFilters = world.CollisionWorld.Broadphase.DynamicTree.BodyFilters.AsArray();
                        var respondsToCollision = world.CollisionWorld.Broadphase.DynamicTree.RespondsToCollision.AsArray();

                        collectDynamicCoherenceInfoHandle = new CollectTemporalCoherenceInfoJob
                        {
                            PhysicsTemporalCoherenceInfoTypeRW = componentHandles.PhysicsTemporalCoherenceInfoTypeRW,
                            LocalToWorldType = componentHandles.LocalToWorldType,
                            LocalTransformType = componentHandles.LocalTransformType,
                            PhysicsColliderType = componentHandles.PhysicsColliderType,
#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
                            PhysicsWorldIndexType = componentHandles.PhysicsWorldIndexType,
#endif
                            ChunkBaseEntityIndices = dynamicBodyChunkBaseEntityIndices,
                            InsertBodyDataWriter = insertDynamicBodyDataStream.AsWriter(),
                            RemoveBodyDataWriter = removeDynamicBodyDataStream.AsWriter(),
                            RemoveBodyDataWriterOtherTree = removeStaticBodyDataStream.AsWriter(),
                            UpdateBodyDataWriter = updateDynamicBodyDataStream.AsWriter(),
                            OtherTreeBufferOffset = numStaticEntityChunks,

                            Nodes = world.CollisionWorld.Broadphase.DynamicTree.Nodes,
                            BodyFilters = bodyFilters,
                            RespondsToCollision = respondsToCollision,
                            BodyFiltersLastFrame = CreateArrayCopy(bodyFilters, worldUpdateAllocator),
                            RespondsToCollisionLastFrame = CreateArrayCopy(respondsToCollision, worldUpdateAllocator),
                            RigidBodies = world.CollisionWorld.DynamicBodies,
                            MotionVelocities = world.DynamicsWorld.MotionVelocities,
                            CollisionTolerance = world.CollisionWorld.CollisionTolerance,
                            TimeStep = timeStep,
                            Gravity = gravity,

                            LastSystemVersion = lastSystemVersion,
                        }.ScheduleParallel(dynamicEntityQuery, inputDeps);

                        // @todo: this could go into the tree itself, which could have a reset function with an "is incremental" flag, or just always allocate a small amount
                        // and clear each time we reset.
                        var updatedElementLocationDataList =
                            new NativeList<BoundingVolumeHierarchy.ElementLocationData>(numDynamicBodies,
                                worldUpdateAllocator);
                        world.CollisionWorld.Broadphase.DynamicTree.UpdatedElementLocationDataList = updatedElementLocationDataList;

                        world.CollisionWorld.Broadphase.DynamicTree.InsertBodyDataStream = insertDynamicBodyDataStream;
                        world.CollisionWorld.Broadphase.DynamicTree.UpdateBodyDataStream = updateDynamicBodyDataStream;
                    }

                    // Special case: deal with bodies that switched type from dynamic to static when the static broadphase is not incremental.
                    // In this case we need to remove the corresponding bodies from the dynamic tree ourselves here since we can't
                    // rely on the static processing to do it, as it isn't run.
                    if (!incrementalStaticBroadphase && numStaticBodies > 0)
                    {
                        var collectSwappedCoherenceInfoHandle = new CollectSwappedTemporalCoherenceInfoJob
                        {
                            PhysicsTemporalCoherenceInfoTypeRW = componentHandles.PhysicsTemporalCoherenceInfoTypeRW,
                            RemoveBodyDataWriter = removeDynamicBodyDataStream.AsWriter(),
                            BufferOffset = numDynamicEntityChunks,
                            Static = true
                        }.ScheduleParallel(staticEntityQuery, inputDeps);

                        collectDynamicCoherenceInfoHandle = JobHandle.CombineDependencies(collectDynamicCoherenceInfoHandle, collectSwappedCoherenceInfoHandle);
                    }
                }

                if (incrementalStaticBroadphase)
                {
                    if (numStaticBodies > 0)
                    {
                        var bodyFilters = world.CollisionWorld.Broadphase.StaticTree.BodyFilters.AsArray();
                        var respondsToCollision = world.CollisionWorld.Broadphase.StaticTree.RespondsToCollision.AsArray();

                        collectStaticCoherenceInfoHandle = new CollectTemporalCoherenceInfoJob
                        {
                            PhysicsTemporalCoherenceInfoTypeRW = componentHandles.PhysicsTemporalCoherenceInfoTypeRW,
                            LocalToWorldType = componentHandles.LocalToWorldType,
                            LocalTransformType = componentHandles.LocalTransformType,
                            PhysicsColliderType = componentHandles.PhysicsColliderType,
#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
                            PhysicsWorldIndexType = componentHandles.PhysicsWorldIndexType,
#endif
                            ChunkBaseEntityIndices = staticBodyChunkBaseEntityIndices,
                            InsertBodyDataWriter = insertStaticBodyDataStream.AsWriter(),
                            RemoveBodyDataWriter = removeStaticBodyDataStream.AsWriter(),
                            RemoveBodyDataWriterOtherTree = removeDynamicBodyDataStream.AsWriter(),
                            UpdateBodyDataWriter = updateStaticBodyDataStream.AsWriter(),
                            OtherTreeBufferOffset = numDynamicEntityChunks,

                            Nodes = world.CollisionWorld.Broadphase.StaticTree.Nodes,
                            BodyFilters = bodyFilters,
                            RespondsToCollision = respondsToCollision,
                            BodyFiltersLastFrame = CreateArrayCopy(bodyFilters, worldUpdateAllocator),
                            RespondsToCollisionLastFrame = CreateArrayCopy(respondsToCollision, worldUpdateAllocator),
                            RigidBodies = world.CollisionWorld.StaticBodies,
                            Static = true,

                            LastSystemVersion = lastSystemVersion
                        }.ScheduleParallel(staticEntityQuery, inputDeps);

                        // @todo: this could go into the tree itself, which could have a reset function with an "is incremental" flag, or just always allocate a small amount
                        // and clear each time we reset.
                        var updatedElementLocationDataList =
                            new NativeList<BoundingVolumeHierarchy.ElementLocationData>(numStaticBodies,
                                worldUpdateAllocator);
                        world.CollisionWorld.Broadphase.StaticTree.UpdatedElementLocationDataList = updatedElementLocationDataList;

                        world.CollisionWorld.Broadphase.StaticTree.InsertBodyDataStream = insertStaticBodyDataStream;
                        world.CollisionWorld.Broadphase.StaticTree.UpdateBodyDataStream = updateStaticBodyDataStream;
                    }

                    // Special case: deal with bodies that switched type from static to dynamic when the dynamic broadphase is not incremental.
                    // In this case we need to remove the corresponding bodies from the static tree ourselves here since we can't
                    // rely on the dynamic processing to do it, as it isn't run.
                    if (!incrementalDynamicBroadphase && numDynamicBodies > 0)
                    {
                        var collectSwappedCoherenceInfoHandle = new CollectSwappedTemporalCoherenceInfoJob
                        {
                            PhysicsTemporalCoherenceInfoTypeRW = componentHandles.PhysicsTemporalCoherenceInfoTypeRW,
                            RemoveBodyDataWriter = removeStaticBodyDataStream.AsWriter(),
                            BufferOffset = numStaticEntityChunks,
                            Static = false
                        }.ScheduleParallel(dynamicEntityQuery, inputDeps);

                        collectDynamicCoherenceInfoHandle = JobHandle.CombineDependencies(collectDynamicCoherenceInfoHandle, collectSwappedCoherenceInfoHandle);
                    }
                }

                inputDeps =
                    JobHandle.CombineDependencies(collectDynamicCoherenceInfoHandle, collectStaticCoherenceInfoHandle);
            }

            // Note: we do not reset the broadphase here since this is done from within the system that calls this method. This is needed to support incremental broadphase updates.
            var buildBroadphaseHandle = Broadphase.ScheduleBuildJobs(ref world, timeStep, gravity, buildStaticTree, inputDeps,
                multiThreaded, reset: false, incrementalDynamicBroadphase, incrementalStaticBroadphase);

            if (incrementalDynamicBroadphase || incrementalStaticBroadphase)
            {
                JobHandle updateDynamicCoherenceInfoHandle = buildBroadphaseHandle;
                JobHandle updateStaticCoherenceInfoHandle = buildBroadphaseHandle;

                if (numDynamicBodies > 0 && incrementalDynamicBroadphase)
                {
                    var updateCoherenceInfoJob = new UpdateTemporalCoherenceInfoJob
                    {
                        PhysicsTemporalCoherenceInfoLookupRW = componentHandles.PhysicsTemporalCoherenceInfoLookupRW,
                        TemporalCoherenceDataList =
                            world.CollisionWorld.Broadphase.DynamicTree.UpdatedElementLocationDataList,
                        RigidBodies = world.CollisionWorld.DynamicBodies,
                        Static = false
                    };

                    updateDynamicCoherenceInfoHandle = updateCoherenceInfoJob.ScheduleByRef(
                        world.CollisionWorld.Broadphase.DynamicTree.UpdatedElementLocationDataList, 16,
                        buildBroadphaseHandle);
                }

                if (numStaticBodies > 0 && incrementalStaticBroadphase)
                {
                    var updateCoherenceInfoJob = new UpdateTemporalCoherenceInfoJob
                    {
                        PhysicsTemporalCoherenceInfoLookupRW = componentHandles.PhysicsTemporalCoherenceInfoLookupRW,
                        TemporalCoherenceDataList =
                            world.CollisionWorld.Broadphase.StaticTree.UpdatedElementLocationDataList,
                        RigidBodies = world.CollisionWorld.StaticBodies,
                        Static = true
                    };
                    updateStaticCoherenceInfoHandle = updateCoherenceInfoJob.ScheduleByRef(
                        world.CollisionWorld.Broadphase.StaticTree.UpdatedElementLocationDataList, 16,
                        buildBroadphaseHandle);
                }

                buildBroadphaseHandle = JobHandle.CombineDependencies(updateDynamicCoherenceInfoHandle,
                    updateStaticCoherenceInfoHandle);
            }

            return buildBroadphaseHandle;
        }

        /// <summary>
        /// Write all overlapping body pairs to the given streams, where at least one of the bodies is
        /// dynamic. The results are unsorted.
        /// </summary>
        ///
        /// <param name="dynamicVsDynamicPairsWriter"> [in,out] The dynamic vs dynamic pairs writer. </param>
        /// <param name="staticVsDynamicPairsWriter">  [in,out] The static vs dynamic pairs writer. </param>
        public void FindOverlaps(ref NativeStream.Writer dynamicVsDynamicPairsWriter, ref NativeStream.Writer staticVsDynamicPairsWriter)
        {
            Broadphase.FindOverlaps(ref dynamicVsDynamicPairsWriter, ref staticVsDynamicPairsWriter);
        }

        /// <summary>
        /// Schedule a set of jobs which will write all overlapping body pairs to the given steam, where
        /// at least one of the bodies is dynamic. The results are unsorted.
        /// </summary>
        ///
        /// <param name="dynamicVsDynamicPairsStream">  [out] The dynamic vs dynamic pairs stream. </param>
        /// <param name="staticVsDynamicPairsStream">   [out] The static vs dynamic pairs stream. </param>
        /// <param name="inputDeps">                    The input deps. </param>
        /// <param name="multiThreaded">                (Optional) True if multi threaded. </param>
        ///
        /// <returns>   The SimulationJobHandles. </returns>
        public SimulationJobHandles ScheduleFindOverlapsJobs(out NativeStream dynamicVsDynamicPairsStream, out NativeStream staticVsDynamicPairsStream,
            JobHandle inputDeps, bool multiThreaded = true)
        {
            return Broadphase.ScheduleFindOverlapsJobs(out dynamicVsDynamicPairsStream, out staticVsDynamicPairsStream, inputDeps, multiThreaded);
        }

        internal SimulationJobHandles ScheduleFindOverlapsJobsInternal(out NativeStream dynamicVsDynamicPairsStream, out NativeStream staticVsDynamicPairsStream,
            JobHandle inputDeps, bool multiThreaded, bool incrementalDynamicBroadphase, bool incrementalStaticBroadphase)
        {
            return Broadphase.ScheduleFindOverlapsJobs(out dynamicVsDynamicPairsStream, out staticVsDynamicPairsStream, inputDeps, multiThreaded, incrementalDynamicBroadphase, incrementalStaticBroadphase);
        }

        /// <summary>   Synchronize the collision world with the dynamics world. </summary>
        ///
        /// <param name="world">    [in,out] The world. </param>
        /// <param name="timeStep"> The time step. </param>
        /// <param name="gravity">  The gravity. </param>
        public void UpdateDynamicTree(ref PhysicsWorld world, float timeStep, float3 gravity)
        {
            // Synchronize transforms
            for (int i = 0; i < world.DynamicsWorld.NumMotions; i++)
            {
                UpdateRigidBodyTransformsJob.ExecuteImpl(i, world.MotionDatas, m_Bodies);
            }

            // Update broadphase
            float aabbMargin = world.CollisionWorld.CollisionTolerance * 0.5f;
            Broadphase.BuildDynamicTree(world.DynamicBodies, world.MotionVelocities, gravity, timeStep, aabbMargin);
        }

        /// <summary> Rebuild the static collision world. </summary>
        ///
        /// <param name="world">    [in,out] The world. </param>
        public void UpdateStaticTree(ref PhysicsWorld world)
        {
            float aabbMargin = world.CollisionWorld.CollisionTolerance * 0.5f;
            Broadphase.BuildStaticTree(world.StaticBodies, aabbMargin);
        }

        /// <summary>
        /// Schedule a set of jobs to update the static collision world.
        /// </summary>
        ///
        /// <param name="world">            [in,out] The world. </param>
        /// <param name="buildStaticTree">  The build static tree. </param>
        /// <param name="inputDeps">        The input deps. </param>
        /// <param name="multiThreaded">    (Optional) True if multi threaded. </param>
        ///
        /// <returns>   A JobHandle. </returns>
        public JobHandle ScheduleUpdateStaticTree(ref PhysicsWorld world,
            NativeReference<int>.ReadOnly buildStaticTree, JobHandle inputDeps, bool multiThreaded = true)
        {
            if (!multiThreaded)
            {
                return new UpdateStaticTreeJob
                {
                    World = world,
                    BuildStaticTree = buildStaticTree
                }.Schedule(inputDeps);
            }
            else
            {
                // Thread count is +1 for main thread
                return Broadphase.ScheduleStaticTreeBuildJobs(ref world, JobsUtility.JobWorkerCount + 1, buildStaticTree, inputDeps);
            }
        }

        /// <summary>
        /// Schedule a set of jobs to synchronize the collision world with the dynamics world.
        /// </summary>
        ///
        /// <param name="world">            [in,out] The world. </param>
        /// <param name="timeStep">         The time step. </param>
        /// <param name="gravity">          The gravity. </param>
        /// <param name="inputDeps">        The input deps. </param>
        /// <param name="multiThreaded">    (Optional) True if multi threaded. </param>
        ///
        /// <returns>   A JobHandle. </returns>
        public JobHandle ScheduleUpdateDynamicTree(ref PhysicsWorld world, float timeStep, float3 gravity, JobHandle inputDeps, bool multiThreaded = true)
        {
            if (!multiThreaded)
            {
                return new UpdateDynamicTreeJob
                {
                    World = world,
                    TimeStep = timeStep,
                    Gravity = gravity
                }.Schedule(inputDeps);
            }
            else
            {
                // Synchronize transforms
                JobHandle handle = new UpdateRigidBodyTransformsJob
                {
                    MotionDatas = world.MotionDatas,
                    RigidBodies = m_Bodies
                }.Schedule(world.MotionDatas.Length, 32, inputDeps);

                // Update broadphase
                // Thread count is +1 for main thread
                return Broadphase.ScheduleDynamicTreeBuildJobs(ref world, timeStep, gravity, JobsUtility.JobWorkerCount + 1, handle);
            }
        }

        #region Jobs

        [BurstCompile]
        private struct UpdateRigidBodyTransformsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<MotionData> MotionDatas;
            public NativeArray<RigidBody> RigidBodies;

            public void Execute(int i)
            {
                ExecuteImpl(i, MotionDatas, RigidBodies);
            }

            internal static void ExecuteImpl(int i, NativeArray<MotionData> motionDatas, NativeArray<RigidBody> rigidBodies)
            {
                RigidBody rb = rigidBodies[i];
                rb.WorldFromBody = math.mul(motionDatas[i].WorldFromMotion, math.inverse(motionDatas[i].BodyFromMotion));
                rigidBodies[i] = rb;
            }
        }

        [BurstCompile]
        struct UpdateDynamicTreeJob : IJob
        {
            public PhysicsWorld World;
            public float TimeStep;
            public float3 Gravity;

            public void Execute()
            {
                World.CollisionWorld.UpdateDynamicTree(ref World, TimeStep, Gravity);
            }
        }
        [BurstCompile]
        struct UpdateStaticTreeJob : IJob
        {
            public PhysicsWorld World;
            public NativeReference<int>.ReadOnly BuildStaticTree;
            public void Execute()
            {
                if (BuildStaticTree.Value != 0)
                    World.CollisionWorld.UpdateStaticTree(ref World);
            }
        }

        /// <summary>
        /// <para>
        /// Job responsible for collecting information about bodies that have switched between the dynamic and static trees
        /// since the last frame as part of the incremental broadphase feature.
        /// </para>
        /// <para>
        /// This happens when a dynamic body becomes static or vice versa.
        /// </para>
        /// </summary>
        [BurstCompile]
        struct CollectSwappedTemporalCoherenceInfoJob : IJobChunk
        {
            [NativeDisableContainerSafetyRestriction]
            public ComponentTypeHandle<PhysicsTemporalCoherenceInfo> PhysicsTemporalCoherenceInfoTypeRW;

            [NativeDisableContainerSafetyRestriction]
            public NativeStream.Writer RemoveBodyDataWriter;
            public int BufferOffset;

            [ReadOnly] public bool Static;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask,
                in v128 chunkEnabledMask)
            {
                if (!chunk.Has(ref PhysicsTemporalCoherenceInfoTypeRW))
                {
                    SafetyChecks.ThrowArgumentException(
                        "PhysicsTemporalCoherenceInfo component not found on rigid body. This component is required on " +
                        "all rigid bodies when incremental broadphase is enabled.");
                    return;
                }

                RemoveBodyDataWriter.BeginForEachIndex(unfilteredChunkIndex + BufferOffset);

                NativeArray<PhysicsTemporalCoherenceInfo> chunkCoherenceInfos =
                    chunk.GetNativeArray(ref PhysicsTemporalCoherenceInfoTypeRW);

                var entityEnumerator =
                    new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);

                while (entityEnumerator.NextEntityIndex(out int i))
                {
                    var coherenceInfo = chunkCoherenceInfos[i];
                    // The body has switched trees. We need to remove it from the previous tree.
                    if (coherenceInfo.Valid && coherenceInfo.StaticBvh != Static)
                    {
                        SafetyChecks.CheckAreEqualAndThrow(true, coherenceInfo.StaticBvh != Static);
                        // remove the body from the other tree
                        RemoveBodyDataWriter.Write(new BoundingVolumeHierarchy.RemovalData
                        {
                            NodeIndex = coherenceInfo.LastBvhNodeIndex,
                            LeafSlotIndex = coherenceInfo.LastBvhLeafSlotIndex
                        });

                        // invalidate temporal coherence info for clean re-insert next time
                        chunkCoherenceInfos[i] = PhysicsTemporalCoherenceInfo.Default;
                    }
                }

                RemoveBodyDataWriter.EndForEachIndex();
            }
        }

        /// <summary>
        /// <para>
        /// Job responsible for collecting temporal coherence info for bodies in the simulation as part of
        /// the incremental broadphase. The job performs updates of internal broadphase data based on the information found.
        /// </para>
        /// <para>
        /// As such, bodies that have not been in the broadphase previously will be detected and added to an insertion list.
        /// Analogously, bodies that have already been part of broadphase but have changed in ways relevant to the broadphase's
        /// spatial hashing (transformation change, collider change etc.), will have to be removed and re-inserted. These
        /// bodies will be added to the removal and insertion lists, and their relevant geometric information in the broadphase
        /// will be updated, that is, their AABB, and their collision filter and responds to collision flag copies.
        /// </para>
        /// </summary>
        [BurstCompile]
        struct CollectTemporalCoherenceInfoJob : IJobChunk
        {
            [NativeDisableContainerSafetyRestriction]
            public ComponentTypeHandle<PhysicsTemporalCoherenceInfo> PhysicsTemporalCoherenceInfoTypeRW;
            [ReadOnly] public ComponentTypeHandle<LocalToWorld> LocalToWorldType;
            [ReadOnly] public ComponentTypeHandle<LocalTransform> LocalTransformType;
            [ReadOnly] public ComponentTypeHandle<PhysicsCollider> PhysicsColliderType;

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
            [ReadOnly] public SharedComponentTypeHandle<PhysicsWorldIndex> PhysicsWorldIndexType;
#endif

            [ReadOnly] public NativeArray<int> ChunkBaseEntityIndices;

            [NativeDisableContainerSafetyRestriction]
            public NativeStream.Writer RemoveBodyDataWriter;
            [NativeDisableContainerSafetyRestriction]
            public NativeStream.Writer RemoveBodyDataWriterOtherTree;
            public NativeStream.Writer UpdateBodyDataWriter;
            public NativeStream.Writer InsertBodyDataWriter;
            public int OtherTreeBufferOffset;

            [NativeDisableContainerSafetyRestriction]
            public NativeList<BoundingVolumeHierarchy.Node> Nodes;
            [NativeDisableContainerSafetyRestriction]
            public NativeArray<CollisionFilter> BodyFilters;
            [NativeDisableContainerSafetyRestriction]
            public NativeArray<bool> RespondsToCollision;

            [ReadOnly]
            [NativeDisableContainerSafetyRestriction]
            public NativeArray<CollisionFilter> BodyFiltersLastFrame;
            [ReadOnly]
            [NativeDisableContainerSafetyRestriction]
            public NativeArray<bool> RespondsToCollisionLastFrame;

            [ReadOnly] public NativeArray<RigidBody> RigidBodies;
            [NativeDisableContainerSafetyRestriction]
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [ReadOnly] public float CollisionTolerance;
            [ReadOnly] public float TimeStep;
            [ReadOnly] public float3 Gravity;
            [ReadOnly] public bool Static;

            [ReadOnly] public uint LastSystemVersion;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask,
                in v128 chunkEnabledMask)
            {
                // Note: for fast chunk data change checks and fast data migrations,
                // enabled masks are not supported, analogous to the assumption in the CheckStaticBodyChangesJob.
                SafetyChecks.CheckAreEqualAndThrow(false, useEnabledMask);

#if ENABLE_UNITY_COLLECTIONS_CHECKS || UNITY_DOTS_DEBUG
                if (!chunk.Has(ref PhysicsTemporalCoherenceInfoTypeRW))
                {
                    SafetyChecks.ThrowArgumentException("PhysicsTemporalCoherenceInfo component not found on rigid body. This component is required on " +
                        "all rigid bodies when incremental broadphase is enabled.");
                    return;
                }
#endif

#if (UNITY_EDITOR || DEVELOPMENT_BUILD) && !UNITY_PHYSICS_DISABLE_INTEGRITY_CHECKS
                // The incremental broadphase feature is only supported in the default world
                var defaultWorldIndex = new PhysicsWorldIndex();
                var physicsWorldIndex = chunk.GetSharedComponent(PhysicsWorldIndexType);

                SafetyChecks.CheckAreEqualAndThrow(defaultWorldIndex.Value, physicsWorldIndex.Value);
#endif

                var transformChangedInChunk =
                    chunk.DidChange(ref LocalToWorldType, LastSystemVersion) ||
                    chunk.DidChange(ref LocalTransformType, LastSystemVersion);
                var colliderChangedInChunk = chunk.DidChange(ref PhysicsColliderType, LastSystemVersion);
                var temporalCoherenceDataChanged = chunk.DidChange(ref PhysicsTemporalCoherenceInfoTypeRW, LastSystemVersion);
                var chunkOrderChanged = chunk.DidOrderChange(LastSystemVersion);


                var firstEntityIndexInQuery = ChunkBaseEntityIndices[unfilteredChunkIndex];

                unsafe
                {
                    // Note: we absolutely need to use pointer access here, so that we can safely write to the
                    // data array in the leaf node entries in parallel. If we were to use the array's API and read the node
                    // out, modify the data and then assign it back (see below) we will cause a race condition.
                    var nodes = Nodes.GetUnsafePtr();
                    var chunkCoherenceInfos =
                        chunk.GetComponentDataPtrRW(ref PhysicsTemporalCoherenceInfoTypeRW);

                    // early out in case nothing of relevance for incremental broadphase changed in this chunk
                    if (!(transformChangedInChunk || colliderChangedInChunk || temporalCoherenceDataChanged || chunkOrderChanged))
                    {
                        var entityIter = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);

                        // Check if the starting index of this chunk changed.
                        // If not, we can bail out right here.
                        if (entityIter.NextEntityIndex(out int i))
                        {
                            var firstBodyIndex = firstEntityIndexInQuery + i;
                            var coherenceInfo = chunkCoherenceInfos[i];
                            var firstBodyIndexLastFrame = coherenceInfo.LastRigidBodyIndex;
                            SafetyChecks.CheckAreEqualAndThrow(true, coherenceInfo.Valid);
                            if (firstBodyIndex == firstBodyIndexLastFrame)
                            {
                                // starting index did not change, so we can safely bail out here.
                                return;
                            }
                            // else:

                            // Fast path here for only a fixed entity index shift:
                            // Nothing changed in this chunk, but the starting entity index changed.
                            // So we have to update internal broadphase data structures:
                            // 1. update data in BVH node and coherence info
                            // 2. shift data in RespondsToCollision and CollisionFilter arrays using memcpy from the last frame's data.

                            do
                            {
                                // 1:

                                var bodyIndex = firstEntityIndexInQuery + i;
                                coherenceInfo = chunkCoherenceInfos[i];
                                var lastBodyIndex = coherenceInfo.LastRigidBodyIndex;
                                SafetyChecks.CheckAreEqualAndThrow(true, coherenceInfo.Valid);
                                SafetyChecks.CheckAreEqualAndThrow(true, bodyIndex != lastBodyIndex);

                                // last known node index of the body
                                var nodeIndex = coherenceInfo.LastBvhNodeIndex;
                                SafetyChecks.CheckAreEqualAndThrow(true, nodeIndex > 0);

                                var node = nodes + nodeIndex;
                                SafetyChecks.CheckAreEqualAndThrow(true, node->IsLeaf);
                                // index of the leaf slot the body is located in within its node
                                var leafSlotIndex = coherenceInfo.LastBvhLeafSlotIndex;
                                // confirm that we are looking at the leaf slot of the right body
                                SafetyChecks.CheckAreEqualAndThrow(coherenceInfo.LastRigidBodyIndex,
                                    node->Data[leafSlotIndex]);

                                // update body index in node
                                node->Data[leafSlotIndex] = bodyIndex;
                                // confirm that there is no race condition
                                SafetyChecks.CheckAreEqualAndThrow(bodyIndex, node->Data[leafSlotIndex]);
                                // update changed rigid body index also in coherence info
                                coherenceInfo.LastRigidBodyIndex = bodyIndex;

                                // write out updated coherence info
                                chunkCoherenceInfos[i] = coherenceInfo;
                            }
                            while (entityIter.NextEntityIndex(out i));

                            // 2:

                            // Note: this shift operation is safe to do since we know that the chunk order has not changed since last frame,
                            // and we can use the chunk.Count as indication of the number of rigid body entities, as enabled masks are not supported for rigid bodies.
                            // See assertion that useEnabledMask is false above.
                            UnsafeUtility.MemCpy(
                                (CollisionFilter*)BodyFilters.GetUnsafePtr() + firstBodyIndex,
                                (CollisionFilter*)BodyFiltersLastFrame.GetUnsafePtr() + firstBodyIndexLastFrame,
                                sizeof(CollisionFilter) * chunk.Count);

                            UnsafeUtility.MemCpy(
                                (bool*)RespondsToCollision.GetUnsafePtr() + firstBodyIndex,
                                (bool*)RespondsToCollisionLastFrame.GetUnsafePtr() + firstBodyIndexLastFrame,
                                sizeof(bool) * chunk.Count);
                        }

                        return;
                    }

                    RemoveBodyDataWriter.BeginForEachIndex(unfilteredChunkIndex);
                    InsertBodyDataWriter.BeginForEachIndex(unfilteredChunkIndex);
                    UpdateBodyDataWriter.BeginForEachIndex(unfilteredChunkIndex);

                    if (OtherTreeBufferOffset != -1)
                    {
                        // only needed if the other tree (either static or dynamic) exists and we might need to
                        // remove bodies from it
                        RemoveBodyDataWriterOtherTree.BeginForEachIndex(unfilteredChunkIndex + OtherTreeBufferOffset);
                    }

                    var entityEnumerator =
                        new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);

                    var aabbMargin = CollisionTolerance * 0.5f;

                    while (entityEnumerator.NextEntityIndex(out int i))
                    {
                        var bodyIndex = firstEntityIndexInQuery + i;
                        var coherenceInfo = chunkCoherenceInfos[i];
                        var bodyIndexChanged = !coherenceInfo.Valid || bodyIndex != coherenceInfo.LastRigidBodyIndex;
                        var anythingChanged = bodyIndexChanged || transformChangedInChunk || colliderChangedInChunk || chunkOrderChanged;

                        if (anythingChanged)
                        {
                            bool previouslyInTree = coherenceInfo.Valid &&              // body was previously in some tree
                                coherenceInfo.StaticBvh == Static;                      // body was previously in this tree

                            // Check if the body was previously present in this tree. Update its data if required.
                            // If not, we need to insert it for the first time.
                            if (previouslyInTree)
                            {
                                // last known node index of the body
                                // If > 0, the body has already been in the tree. Otherwise, it needs to be newly inserted.
                                var nodeIndex = coherenceInfo.LastBvhNodeIndex;
                                SafetyChecks.CheckAreEqualAndThrow(true, nodeIndex > 0);

                                var node = nodes + nodeIndex;
                                SafetyChecks.CheckAreEqualAndThrow(true, node->IsLeaf);
                                // index of the leaf slot the body is located in within its node
                                var leafSlotIndex = coherenceInfo.LastBvhLeafSlotIndex;
                                // confirm that we are looking at the leaf slot of the right body
                                SafetyChecks.CheckAreEqualAndThrow(coherenceInfo.LastRigidBodyIndex, node->Data[leafSlotIndex]);

                                // update body index if required
                                if (bodyIndexChanged)
                                {
                                    node->Data[leafSlotIndex] = bodyIndex;
                                    // confirm that there is no race condition
                                    SafetyChecks.CheckAreEqualAndThrow(bodyIndex, node->Data[leafSlotIndex]);
                                    // update changed rigid body index also in coherence info
                                    coherenceInfo.LastRigidBodyIndex = bodyIndex;
                                }

                                var body = RigidBodies[bodyIndex];

                                // check if collider changed
                                bool colliderChanged = false;
                                if (colliderChangedInChunk)
                                {
                                    // check if the collider version changed for this entity
                                    var colliderVersion = body.Collider.Value.Version;
                                    colliderChanged = colliderVersion != coherenceInfo.LastColliderVersion;
                                    // update collider version in coherence info
                                    coherenceInfo.LastColliderVersion = colliderVersion;
                                }

                                // update coherence info if required
                                if (bodyIndexChanged || colliderChanged)
                                {
                                    chunkCoherenceInfos[i] = coherenceInfo;
                                }

                                var collisionFilter = CollisionFilter.Zero;
                                var needFilterOrResponseUpdate = bodyIndexChanged || colliderChanged;
                                var collisionFilterChanged = false;
                                // deal with collision filter and responds to collision flag
                                if (needFilterOrResponseUpdate)
                                {
                                    // If the body index changed or the collider was modified, we need to update the
                                    // body's entry in the BodyFilters and RespondsToCollision arrays.
                                    // If the body index changed, we just need to copy the data to the right location.
                                    // If the collider was modified, the collision filter or RespondsToCollision flag could have been changed.
                                    var respondsToCollision = false;
                                    if (body.Collider.IsCreated)
                                    {
                                        collisionFilter = body.Collider.Value.GetCollisionFilter();
                                        respondsToCollision = body.Collider.Value.RespondsToCollision;
                                    }

                                    var previousFilter = BodyFilters[bodyIndex];
                                    collisionFilterChanged = !previousFilter.Equals(collisionFilter);
                                    BodyFilters[bodyIndex] = collisionFilter;
                                    RespondsToCollision[bodyIndex] = respondsToCollision;
                                }

                                // check if we need to update the AABB in the broadphase
                                if (transformChangedInChunk || colliderChanged)
                                {
                                    // Recompute AABB:
                                    var aabb = Static ?
                                        Broadphase.PrepareStaticBodyDataJob.CalculateAabb(ref body, aabbMargin) :
                                        Broadphase.PrepareDynamicBodyDataJob.CalculateAabb(ref body, aabbMargin, Gravity,
                                        TimeStep, MotionVelocities[bodyIndex]);

                                    var previousAabb = node->Bounds.GetAabb(leafSlotIndex);
                                    var previousExtents = previousAabb.Extents;
                                    var previousCenter = previousAabb.Center;
                                    var newExtents = aabb.Extents;
                                    var newCenter = aabb.Center;
                                    var deltaExtentsSq = math.distancesq(newExtents, previousExtents);
                                    var deltaCenterSq = math.distancesq(newCenter, previousCenter);

                                    const float kEpsilonSq = math.EPSILON;
                                    var aabbChanged =
                                        deltaExtentsSq > kEpsilonSq ||
                                        deltaCenterSq > kEpsilonSq;

                                    var reinsertRequired = aabbChanged;

                                    // check if we can prevent reinsertion
                                    if (aabbChanged)
                                    {
                                        // if we are relatively close to the previous values, we can just issue an aabb update
                                        // and a refit without removal and re-insertion.

                                        var sweptAabb = Aabb.Union(previousAabb, aabb);
                                        var previousSurfaceArea = previousAabb.SurfaceArea;
                                        var sweptSurfaceGrowthRatio = sweptAabb.SurfaceArea / previousSurfaceArea;
                                        var surfaceChangeRatio = aabb.SurfaceArea / previousSurfaceArea;

                                        const float kMaxSweptSurfaceAreaGrowthFactor = 1.2f;
                                        const float kMinSurfaceAreaGrowthFactor = 0.5f;
                                        var refit = sweptSurfaceGrowthRatio<kMaxSweptSurfaceAreaGrowthFactor && surfaceChangeRatio> kMinSurfaceAreaGrowthFactor;

                                        if (refit)
                                        {
                                            // initiate refit instead of re-insertion
                                            reinsertRequired = false;

                                            var updateData = new BoundingVolumeHierarchy.UpdateData
                                            {
                                                Aabb = aabb,
                                                NodeIndex = nodeIndex,
                                                LeafSlotIndex = leafSlotIndex,
                                                UpdateCommandFlags = (byte)BoundingVolumeHierarchy.UpdateData.CommandFlags.UpdateAabb
                                            };

                                            // also update collision filter if required
                                            if (collisionFilterChanged)
                                            {
                                                updateData.UpdateCommandFlags |= (byte)BoundingVolumeHierarchy.UpdateData.CommandFlags.UpdateFilter;
                                            }

                                            UpdateBodyDataWriter.Write(updateData);
                                        }
                                    }
                                    else if (collisionFilterChanged)
                                    {
                                        // if the AABB didn't change but the collision filter did, we can just update the filter.
                                        // @todo: We could decide not to update the filter when the new filter is less permissive than before.

                                        UpdateBodyDataWriter.Write(new BoundingVolumeHierarchy.UpdateData
                                        {
                                            NodeIndex = nodeIndex,
                                            LeafSlotIndex = leafSlotIndex,
                                            UpdateCommandFlags = (byte)BoundingVolumeHierarchy.UpdateData.CommandFlags.UpdateFilter
                                        });
                                    }

                                    if (reinsertRequired)
                                    {
                                        RemoveBodyDataWriter.Write(new BoundingVolumeHierarchy.RemovalData
                                        {
                                            NodeIndex = nodeIndex,
                                            LeafSlotIndex = leafSlotIndex
                                        });

                                        var point = new BoundingVolumeHierarchy.PointAndIndex
                                        {
                                            Index = bodyIndex,
                                            Position = aabb.Center
                                        };

                                        if (!needFilterOrResponseUpdate && body.Collider.IsCreated)
                                        {
                                            collisionFilter = body.Collider.Value.GetCollisionFilter();
                                        }

                                        InsertBodyDataWriter.Write(new Broadphase.InsertionData
                                        {
                                            Aabb = aabb, PointAndIndex = point, Filter = collisionFilter
                                        });
                                    }
                                }
                            }
                            else // body was not yet in this tree and needs to be newly inserted.
                            {
                                // check if the body was previously in the other tree and needs to be removed from there
                                if (coherenceInfo.Valid)
                                {
                                    SafetyChecks.CheckAreEqualAndThrow(true, coherenceInfo.StaticBvh != Static);
                                    // remove the body from the other tree
                                    RemoveBodyDataWriterOtherTree.Write(new BoundingVolumeHierarchy.RemovalData
                                    {
                                        NodeIndex = coherenceInfo.LastBvhNodeIndex,
                                        LeafSlotIndex = coherenceInfo.LastBvhLeafSlotIndex
                                    });
                                }

                                // Compute AABB:
                                var body = RigidBodies[bodyIndex];
                                var aabb = Static ?
                                    Broadphase.PrepareStaticBodyDataJob.CalculateAabb(ref body, aabbMargin) :
                                    Broadphase.PrepareDynamicBodyDataJob.CalculateAabb(ref body, aabbMargin, Gravity,
                                    TimeStep, MotionVelocities[bodyIndex]);
                                var point = new BoundingVolumeHierarchy.PointAndIndex
                                {
                                    Index = bodyIndex,
                                    Position = aabb.Center
                                };

                                var collisionFilter = CollisionFilter.Zero;
                                var respondsToCollision = false;
                                if (body.Collider.IsCreated)
                                {
                                    collisionFilter = body.Collider.Value.GetCollisionFilter();
                                    respondsToCollision = body.Collider.Value.RespondsToCollision;
                                }

                                BodyFilters[bodyIndex] = collisionFilter;
                                RespondsToCollision[bodyIndex] = respondsToCollision;

                                InsertBodyDataWriter.Write(new Broadphase.InsertionData
                                {
                                    Aabb = aabb, PointAndIndex = point, Filter = collisionFilter
                                });
                            }
                        }
                    }

                    RemoveBodyDataWriter.EndForEachIndex();
                    InsertBodyDataWriter.EndForEachIndex();
                    UpdateBodyDataWriter.EndForEachIndex();
                    if (OtherTreeBufferOffset != -1)
                    {
                        RemoveBodyDataWriterOtherTree.EndForEachIndex();
                    }
                }
            }
        }

        /// <summary>
        /// Job responsible for updating the temporal coherence info for bodies after they have been inserted or moved
        /// in the broadphase. This job is part of the incremental broadphase.
        /// </summary>
        [BurstCompile]
        struct UpdateTemporalCoherenceInfoJob : IJobParallelForDefer
        {
            [NativeDisableContainerSafetyRestriction]
            public ComponentLookup<PhysicsTemporalCoherenceInfo> PhysicsTemporalCoherenceInfoLookupRW;
            [ReadOnly] public NativeList<BoundingVolumeHierarchy.ElementLocationData> TemporalCoherenceDataList;
            [ReadOnly] public NativeArray<RigidBody> RigidBodies;
            [ReadOnly] public bool Static;

            public void Execute(int index)
            {
                var data = TemporalCoherenceDataList[index];
                var rigidBody = RigidBodies[data.ElementIndex];
                var colliderVersion = rigidBody.Collider.IsCreated ? rigidBody.Collider.Value.Version : (byte)0;
                PhysicsTemporalCoherenceInfoLookupRW[rigidBody.Entity] = new PhysicsTemporalCoherenceInfo
                {
                    LastRigidBodyIndex = data.ElementIndex,
                    LastBvhNodeIndex = data.NodeIndex,
                    LastBvhLeafSlotIndex = data.LeafSlotIndex,
                    LastColliderVersion = colliderVersion,
                    StaticBvh = Static
                };
            }
        }

        /// <summary>
        /// <para>
        /// Job responsible for collecting information about rigid body entities that that have ceased to be rigid bodies
        /// since the last frame when the incremental broadphase is ensabled.
        /// </para>
        /// <para>
        /// This happens when a rigid body entity is destroyed or when a component required for a rigid body is removed.
        /// </para>
        /// </summary>
        [BurstCompile]
        struct CollectInvalidatedTemporalCoherenceInfoJob : IJobChunk
        {
            public ComponentTypeHandle<PhysicsTemporalCoherenceInfo> PhysicsTemporalCoherenceInfoTypeRW;

            [NativeDisableContainerSafetyRestriction]
            public NativeStream.Writer RemoveDynamicBodyDataWriter;
            [NativeDisableContainerSafetyRestriction]
            public NativeStream.Writer RemoveStaticBodyDataWriter;

            [ReadOnly] public int DynamicStreamBufferOffset;
            [ReadOnly] public int StaticStreamBufferOffset;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                SafetyChecks.CheckAreEqualAndThrow(true, chunk.Has(ref PhysicsTemporalCoherenceInfoTypeRW));

                RemoveDynamicBodyDataWriter.BeginForEachIndex(unfilteredChunkIndex + DynamicStreamBufferOffset);
                RemoveStaticBodyDataWriter.BeginForEachIndex(unfilteredChunkIndex + StaticStreamBufferOffset);

                NativeArray<PhysicsTemporalCoherenceInfo> chunkCoherenceInfos =
                    chunk.GetNativeArray(ref PhysicsTemporalCoherenceInfoTypeRW);

                var entityEnumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
                while (entityEnumerator.NextEntityIndex(out int i))
                {
                    var coherenceInfo = chunkCoherenceInfos[i];
                    if (coherenceInfo.Valid)
                    {
                        ref var writer = ref coherenceInfo.StaticBvh ?
                            ref RemoveStaticBodyDataWriter : ref RemoveDynamicBodyDataWriter;

                        writer.Write(new BoundingVolumeHierarchy.RemovalData
                        {
                            NodeIndex = coherenceInfo.LastBvhNodeIndex,
                            LeafSlotIndex = coherenceInfo.LastBvhLeafSlotIndex
                        });
                    }

                    // overwrite coherence info with default values which invalidates the data
                    chunkCoherenceInfos[i] = PhysicsTemporalCoherenceInfo.Default;
                }

                RemoveDynamicBodyDataWriter.EndForEachIndex();
                RemoveStaticBodyDataWriter.EndForEachIndex();
            }
        }

        #endregion

        #region ICollidable implementation

        /// <summary>   Calculates the aabb. </summary>
        ///
        /// <returns>   The calculated aabb. </returns>
        public Aabb CalculateAabb()
        {
            return Broadphase.Domain;
        }

        /// <summary>   Cast ray. </summary>
        ///
        /// <param name="input">    The input. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastRay(RaycastInput input) => QueryWrappers.RayCast(in this, input);

        /// <summary>   Cast ray. </summary>
        ///
        /// <param name="input">        The input. </param>
        /// <param name="closestHit">   [out] The closest hit. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastRay(RaycastInput input, out RaycastHit closestHit) => QueryWrappers.RayCast(in this, input, out closestHit);

        /// <summary>   Cast ray. </summary>
        ///
        /// <param name="input">    The input. </param>
        /// <param name="allHits">  [in,out] all hits. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits) => QueryWrappers.RayCast(in this, input, ref allHits);

        /// <summary>   Cast ray. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="input">        The input. </param>
        /// <param name="collector">    [in,out] The collector. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            input.QueryContext.InitScale();
            return Broadphase.CastRay(input, m_Bodies, ref collector);
        }

        /// <summary>   Cast collider. </summary>
        ///
        /// <param name="input">    The input. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider(ColliderCastInput input) => QueryWrappers.ColliderCast(in this, input);

        /// <summary>   Cast collider. </summary>
        ///
        /// <param name="input">        The input. </param>
        /// <param name="closestHit">   [out] The closest hit. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit) => QueryWrappers.ColliderCast(in this, input, out closestHit);

        /// <summary>   Cast collider. </summary>
        ///
        /// <param name="input">    The input. </param>
        /// <param name="allHits">  [in,out] all hits. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) => QueryWrappers.ColliderCast(in this, input, ref allHits);

        /// <summary>   Cast collider. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="input">        The input. </param>
        /// <param name="collector">    [in,out] The collector. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            input.InitScale();
            return Broadphase.CastCollider(input, m_Bodies, ref collector);
        }

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">    The input. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(PointDistanceInput input) => QueryWrappers.CalculateDistance(in this, input);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">        The input. </param>
        /// <param name="closestHit">   [out] The closest hit. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(in this, input, out closestHit);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">    The input. </param>
        /// <param name="allHits">  [in,out] all hits. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(in this, input, ref allHits);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="input">        The input. </param>
        /// <param name="collector">    [in,out] The collector. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            input.QueryContext.InitScale();
            return Broadphase.CalculateDistance(input, m_Bodies, ref collector);
        }

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">    The input. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(ColliderDistanceInput input) => QueryWrappers.CalculateDistance(in this, input);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">        The input. </param>
        /// <param name="closestHit">   [out] The closest hit. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit) => QueryWrappers.CalculateDistance(in this, input, out closestHit);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <param name="input">    The input. </param>
        /// <param name="allHits">  [in,out] all hits. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) => QueryWrappers.CalculateDistance(in this, input, ref allHits);

        /// <summary>   Calculates the distance. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="input">        The input. </param>
        /// <param name="collector">    [in,out] The collector. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>
        {
            input.InitScale();
            return Broadphase.CalculateDistance(input, m_Bodies, ref collector);
        }

        #region Aspect query impl
        #pragma warning disable CS0618 // Disable Aspects obsolete warnings

        /// <summary>   Cast a collider aspect against this <see cref="CollisionWorld"/>. </summary>
        ///
        /// <param name="colliderAspect">   The collider aspect. </param>
        /// <param name="direction">        The direction of the aspect. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider(in ColliderAspect colliderAspect, float3 direction, float maxDistance, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CastCollider(in this, colliderAspect, direction, maxDistance, queryInteraction);

        /// <summary>   Cast a collider aspect against this <see cref="CollisionWorld"/>. </summary>
        ///
        /// <param name="colliderAspect">   The collider aspect. </param>
        /// <param name="direction">        The direction of the aspect. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="closestHit">       [out] The closest hit. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider(in ColliderAspect colliderAspect, float3 direction, float maxDistance, out ColliderCastHit closestHit, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CastCollider(in this, colliderAspect, direction, maxDistance, out closestHit, queryInteraction);

        /// <summary>   Cast a collider aspect against this <see cref="CollisionWorld"/>. </summary>
        ///
        /// <param name="colliderAspect">   The collider aspect. </param>
        /// <param name="direction">        The direction of the aspect. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="allHits">          [in,out] all hits. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider(in ColliderAspect colliderAspect, float3 direction, float maxDistance, ref NativeList<ColliderCastHit> allHits, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CastCollider(in this, colliderAspect, direction, maxDistance, ref allHits, queryInteraction);

        /// <summary>   Cast a collider aspect against this <see cref="CollisionWorld"/>. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="colliderAspect">   The collider aspect. </param>
        /// <param name="direction">        The direction of the aspect. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CastCollider<T>(in ColliderAspect colliderAspect, float3 direction, float maxDistance, ref T collector, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
        {
            QueryInteractionCollector<ColliderCastHit, T> interactionCollector = new QueryInteractionCollector<ColliderCastHit, T>(ref collector, queryInteraction == QueryInteraction.Default, colliderAspect.Entity);

            ColliderCastInput input = new ColliderCastInput(colliderAspect.m_Collider.ValueRO.Value, colliderAspect.Position,
                colliderAspect.Position + direction * maxDistance, colliderAspect.Rotation, colliderAspect.Scale);
            return CastCollider(input, ref interactionCollector);
        }

        /// <summary>   Calculates the distance from the collider aspect. </summary>
        ///
        /// <param name="colliderAspect">   The collider aspect. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(in ColliderAspect colliderAspect, float maxDistance, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CalculateDistance(in this, colliderAspect, maxDistance, queryInteraction);

        /// <summary>   Calculates the distance from the collider aspect. </summary>
        ///
        /// <param name="colliderAspect">   The collider aspect. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="closestHit">       [out] The closest hit. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(in ColliderAspect colliderAspect, float maxDistance, out DistanceHit closestHit, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CalculateDistance(in this, colliderAspect, maxDistance, out closestHit, queryInteraction);

        /// <summary>   Calculates the distance from the collider aspect. </summary>
        ///
        /// <param name="colliderAspect">   The collider aspect. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="allHits">          [in,out] all hits. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance(in ColliderAspect colliderAspect, float maxDistance, ref NativeList<DistanceHit> allHits, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CalculateDistance(in this, colliderAspect, maxDistance, ref allHits, queryInteraction);

        /// <summary>   Calculates the distance from the collider aspect. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="colliderAspect">   The collider aspect. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CalculateDistance<T>(in ColliderAspect colliderAspect, float maxDistance, ref T collector, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
        {
            QueryInteractionCollector<DistanceHit, T> interactionCollector = new QueryInteractionCollector<DistanceHit, T>(ref collector, queryInteraction == QueryInteraction.IgnoreTriggers, colliderAspect.Entity);

            ColliderDistanceInput input = new ColliderDistanceInput(colliderAspect.m_Collider.ValueRO.Value, maxDistance,
                new RigidTransform(colliderAspect.Rotation, colliderAspect.Position), colliderAspect.Scale);
            return CalculateDistance(input, ref interactionCollector);
        }

        #pragma warning restore CS0618
        #endregion

        #region GO API Queries

        /// <summary>   Interfaces that represent queries that exist in the GameObjects world. </summary>
        ///
        /// <param name="position">         The position. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CheckSphere(float3 position, float radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckSphere(in this, position, radius, filter, queryInteraction);

        /// <summary>   Overlap sphere. </summary>
        ///
        /// <param name="position">         The position. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapSphere(float3 position, float radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapSphere(in this, position, radius, ref outHits, filter, queryInteraction);

        /// <summary>   Overlap sphere custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="position">         The position. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapSphereCustom<T>(float3 position, float radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapSphereCustom(in this, position, radius, ref collector, filter, queryInteraction);

        /// <summary>   Check capsule. </summary>
        ///
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CheckCapsule(float3 point1, float3 point2, float radius, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckCapsule(in this, point1, point2, radius, filter, queryInteraction);

        /// <summary>   Overlap capsule. </summary>
        ///
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapCapsule(float3 point1, float3 point2, float radius, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapCapsule(in this, point1, point2, radius, ref outHits, filter, queryInteraction);

        /// <summary>   Overlap capsule custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapCapsuleCustom<T>(float3 point1, float3 point2, float radius, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapCapsuleCustom(in this, point1, point2, radius, ref collector, filter, queryInteraction);

        /// <summary>   Check box. </summary>
        ///
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CheckBox(float3 center, quaternion orientation, float3 halfExtents, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CheckBox(in this, center, orientation, halfExtents, filter, queryInteraction);

        /// <summary>   Overlap box. </summary>
        ///
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapBox(float3 center, quaternion orientation, float3 halfExtents, ref NativeList<DistanceHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.OverlapBox(in this, center, orientation, halfExtents, ref outHits, filter, queryInteraction);

        /// <summary>   Overlap box custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapBoxCustom<T>(float3 center, quaternion orientation, float3 halfExtents, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<DistanceHit>
            => QueryWrappers.OverlapBoxCustom(in this, center, orientation, halfExtents, ref collector, filter, queryInteraction);

        /// <summary>   Sphere cast. </summary>
        ///
        /// <param name="origin">           The origin. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool SphereCast(float3 origin, float radius, float3 direction, float maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCast(in this, origin, radius, direction, maxDistance, filter, queryInteraction);

        /// <summary>   Sphere cast. </summary>
        ///
        /// <param name="origin">           The origin. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="hitInfo">          [out] Information describing the hit. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool SphereCast(float3 origin, float radius, float3 direction, float maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCast(in this, origin, radius, direction, maxDistance, out hitInfo, filter, queryInteraction);

        /// <summary>   Sphere cast all. </summary>
        ///
        /// <param name="origin">           The origin. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool SphereCastAll(float3 origin, float radius, float3 direction, float maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.SphereCastAll(in this, origin, radius, direction, maxDistance, ref outHits, filter, queryInteraction);

        /// <summary>   Sphere cast custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="origin">           The origin. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool SphereCastCustom<T>(float3 origin, float radius, float3 direction, float maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.SphereCastCustom(in this, origin, radius, direction, maxDistance, ref collector, filter, queryInteraction);

        /// <summary>   Box cast. </summary>
        ///
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool BoxCast(float3 center, quaternion orientation, float3 halfExtents, float3 direction, float maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCast(in this, center, orientation, halfExtents, direction, maxDistance, filter, queryInteraction);

        /// <summary>   Box cast. </summary>
        ///
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="hitInfo">          [out] Information describing the hit. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool BoxCast(float3 center, quaternion orientation, float3 halfExtents, float3 direction, float maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCast(in this, center, orientation, halfExtents, direction, maxDistance, out hitInfo, filter, queryInteraction);

        /// <summary>   Box cast all. </summary>
        ///
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool BoxCastAll(float3 center, quaternion orientation, float3 halfExtents, float3 direction, float maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.BoxCastAll(in this, center, orientation, halfExtents, direction, maxDistance, ref outHits, filter, queryInteraction);

        /// <summary>   Box cast custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="center">           The center. </param>
        /// <param name="orientation">      The orientation. </param>
        /// <param name="halfExtents">      Half extents of the box. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool BoxCastCustom<T>(float3 center, quaternion orientation, float3 halfExtents, float3 direction, float maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.BoxCastCustom(in this, center, orientation, halfExtents, direction, maxDistance, ref collector, filter, queryInteraction);

        /// <summary>   Capsule cast. </summary>
        ///
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CapsuleCast(float3 point1, float3 point2, float radius, float3 direction, float maxDistance, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCast(in this, point1, point2, radius, direction, maxDistance, filter, queryInteraction);

        /// <summary>   Capsule cast. </summary>
        ///
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="hitInfo">          [out] Information describing the hit. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CapsuleCast(float3 point1, float3 point2, float radius, float3 direction, float maxDistance, out ColliderCastHit hitInfo, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCast(in this, point1, point2, radius, direction, maxDistance, out hitInfo, filter, queryInteraction);

        /// <summary>   Capsule cast all. </summary>
        ///
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="outHits">          [in,out] The out hits. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CapsuleCastAll(float3 point1, float3 point2, float radius, float3 direction, float maxDistance, ref NativeList<ColliderCastHit> outHits, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default)
            => QueryWrappers.CapsuleCastAll(in this, point1, point2, radius, direction, maxDistance, ref outHits, filter, queryInteraction);

        /// <summary>   Capsule cast custom. </summary>
        ///
        /// <typeparam name="T">    Generic type parameter. </typeparam>
        /// <param name="point1">           The first point in capsule definition. </param>
        /// <param name="point2">           The second point in capsule definition. </param>
        /// <param name="radius">           The radius. </param>
        /// <param name="direction">        The direction. </param>
        /// <param name="maxDistance">      The maximum distance. </param>
        /// <param name="collector">        [in,out] The collector. </param>
        /// <param name="filter">           Specifies the filter. </param>
        /// <param name="queryInteraction"> (Optional) The query interaction. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool CapsuleCastCustom<T>(float3 point1, float3 point2, float radius, float3 direction, float maxDistance, ref T collector, CollisionFilter filter, QueryInteraction queryInteraction = QueryInteraction.Default) where T : struct, ICollector<ColliderCastHit>
            => QueryWrappers.CapsuleCastCustom(in this, point1, point2, radius, direction, maxDistance, ref collector, filter, queryInteraction);

        #endregion

        #endregion

        /// <summary>
        /// Test input against the broadphase tree, filling allHits with the body indices of every
        /// overlap. Returns true if there was at least overlap.
        /// </summary>
        ///
        /// <param name="input">    The input. </param>
        /// <param name="allHits">  [in,out] all hits. </param>
        ///
        /// <returns>   True if there is a hit, false otherwise. </returns>
        public bool OverlapAabb(OverlapAabbInput input, ref NativeList<int> allHits)
        {
            int hitsBefore = allHits.Length;
            Broadphase.OverlapAabb(input, m_Bodies, ref allHits);
            return allHits.Length > hitsBefore;
        }
    }
}
