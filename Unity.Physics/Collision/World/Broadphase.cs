//#define BVH_CHECK_INTEGRITY

using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Profiling;
using UnityEngine.Assertions;
using static Unity.Physics.BoundingVolumeHierarchy;

namespace Unity.Physics
{
    // A bounding volume around a collection of rigid bodies
    [NoAlias]
    internal struct Broadphase : IDisposable
    {
        internal struct InsertionData
        {
            public Aabb Aabb;
            public PointAndIndex PointAndIndex;
            public CollisionFilter Filter;
        }

        [NoAlias]
        public Tree StaticTree;  // The tree of static rigid bodies
        [NoAlias]
        public Tree DynamicTree; // The tree of dynamic rigid bodies

        public Aabb Domain =>
            Aabb.Union(StaticTree.BoundingVolumeHierarchy.Domain, DynamicTree.BoundingVolumeHierarchy.Domain);

        public int NumStaticBodies => StaticTree.NumBodies;

        public int NumDynamicBodies => DynamicTree.NumBodies;

        public Broadphase(int numStaticBodies, int numDynamicBodies)
        {
            StaticTree = new Tree(numStaticBodies);
            DynamicTree = new Tree(numDynamicBodies);
        }

        internal Broadphase(Tree staticTree, Tree dynamicTree)
        {
            StaticTree = staticTree;
            DynamicTree = dynamicTree;
        }

        public void Reset(int numStaticBodies, int numDynamicBodies)
        {
            StaticTree.Reset(numStaticBodies);
            DynamicTree.Reset(numDynamicBodies);
        }

        public void Dispose()
        {
            StaticTree.Dispose();
            DynamicTree.Dispose();
        }

        public Broadphase Clone()
        {
            return new Broadphase
            {
                StaticTree = StaticTree.Clone(),
                DynamicTree = DynamicTree.Clone(),
            };
        }

        #region Build

        /// <summary>
        /// Build the broadphase based on the given world.
        /// </summary>
        public void Build(NativeArray<RigidBody> staticBodies, NativeArray<RigidBody> dynamicBodies,
            NativeArray<MotionVelocity> motionVelocities, float collisionTolerance, float timeStep, float3 gravity, bool buildStaticTree = true, bool reset = true, bool incrementalDynamicTree = false, bool incrementalStaticTree = false)
        {
            float aabbMargin = collisionTolerance * 0.5f; // each body contributes half

            if (buildStaticTree)
            {
                if (reset)
                {
                    StaticTree.Reset(staticBodies.Length);
                }
                BuildStaticTree(staticBodies, aabbMargin, incrementalStaticTree);
            }

            if (reset)
            {
                DynamicTree.Reset(dynamicBodies.Length);
            }
            BuildDynamicTree(dynamicBodies, motionVelocities, gravity, timeStep, aabbMargin, incrementalDynamicTree);
        }

        /// <summary>
        /// Build the static tree of the broadphase based on the given array of rigid bodies.
        /// </summary>
        public void BuildStaticTree(NativeArray<RigidBody> staticBodies, float aabbMargin, bool incremental = false)
        {
            Assert.AreEqual(staticBodies.Length, StaticTree.NumBodies);

            if (staticBodies.Length == 0)
            {
                StaticTree.BoundingVolumeHierarchy.Clear();
                return;
            }
            // else:

            if (incremental)
            {
                StaticTree.BuildIncremental();
#if BVH_CHECK_INTEGRITY
                unsafe
                {
                    // Note: Remove one body since the static tree does not contain the default static body. It does not have a collider and is not included in the tree creation.
                    var expectedNumBodies = staticBodies.Length - 1;
                    StaticTree.BoundingVolumeHierarchy.CheckIntegrity(expectedNumBodies, StaticTree.BodyFilters.GetUnsafePtr());
                }
#endif
                return;
            }
            // else:

            // flag tree as not built incrementally, since we rebuild it from scratch
            StaticTree.Incremental = false;

            // Read bodies
            var aabbs = new NativeArray<Aabb>(staticBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(staticBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < staticBodies.Length; i++)
            {
                PrepareStaticBodyDataJob.ExecuteImpl(i, aabbMargin, staticBodies, aabbs, points, StaticTree.BodyFilters.AsArray(), StaticTree.RespondsToCollision.AsArray());
            }

            // Build tree
            var bvh = StaticTree.BoundingVolumeHierarchy;
            bvh.Build(points, aabbs, out int nodeCount);

            // Build node filters
            bvh.BuildCombinedCollisionFilter(StaticTree.BodyFilters.AsArray(), 1, nodeCount - 1);
        }

#if ENABLE_PROFILER
        static readonly ProfilerMarker s_RemovalsMarker = new(ProfilerCategory.Physics, "BVH.RemovalsAndUpdates");
        static readonly ProfilerMarker s_InsertionsMarker = new(ProfilerCategory.Physics, "BVH.Insertions");
#endif

        /// <summary>
        /// Build the dynamic tree of the broadphase based on the given array of rigid bodies and motions.
        /// </summary>
        public void BuildDynamicTree(NativeArray<RigidBody> dynamicBodies,
            NativeArray<MotionVelocity> motionVelocities, float3 gravity, float timeStep, float aabbMargin, bool incremental = false)
        {
            Assert.AreEqual(dynamicBodies.Length, DynamicTree.NumBodies);

            if (dynamicBodies.Length == 0)
            {
                DynamicTree.BoundingVolumeHierarchy.Clear();
                return;
            }
            // else:

            if (incremental)
            {
                DynamicTree.BuildIncremental();
#if BVH_CHECK_INTEGRITY
                unsafe
                {
                    DynamicTree.BoundingVolumeHierarchy.CheckIntegrity(dynamicBodies.Length, DynamicTree.BodyFilters.GetUnsafePtr());
                }
#endif
                return;
            }
            // else:

            // flag tree as not built incrementally, since we rebuild it from scratch
            DynamicTree.Incremental = false;

            var aabbs = new NativeArray<Aabb>(dynamicBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(dynamicBodies.Length, Allocator.Temp, NativeArrayOptions.UninitializedMemory);

            for (int i = 0; i < dynamicBodies.Length; i++)
            {
                PrepareDynamicBodyDataJob.ExecuteImpl(i, aabbMargin, gravity, timeStep, dynamicBodies, motionVelocities, aabbs, points,
                    DynamicTree.BodyFilters.AsArray(), DynamicTree.RespondsToCollision.AsArray());
            }

            var bvh = DynamicTree.BoundingVolumeHierarchy;

            bvh.Build(points, aabbs, out var nodeCount);
            bvh.BuildCombinedCollisionFilter(DynamicTree.BodyFilters.AsArray(), 1, nodeCount - 1);
        }

        /// <summary>
        /// Schedule a set of jobs to build the broadphase based on the given world.
        /// </summary>
        public JobHandle ScheduleBuildJobs(ref PhysicsWorld world, float timeStep, float3 gravity, NativeReference<int>.ReadOnly buildStaticTree, JobHandle inputDeps,
            bool multiThreaded = true, bool reset = true, bool incrementalDynamicTree = false, bool incrementalStaticTree = false)
        {
            if (!multiThreaded)
            {
                return new BuildBroadphaseJob
                {
                    StaticBodies = world.StaticBodies,
                    DynamicBodies = world.DynamicBodies,
                    MotionVelocities = world.MotionVelocities,
                    CollisionTolerance = world.CollisionWorld.CollisionTolerance,
                    TimeStep = timeStep,
                    Gravity = gravity,
                    BuildStaticTree = buildStaticTree,
                    Reset = reset,
                    Broadphase = this,
                    IncrementalDynamicTree = incrementalDynamicTree,
                    IncrementalStaticTree = incrementalStaticTree
                }.Schedule(inputDeps);
            }
            // else:

            // +1 for main thread
            int threadCount = JobsUtility.JobWorkerCount + 1;
            return JobHandle.CombineDependencies(
                ScheduleStaticTreeBuildJobs(ref world, threadCount, buildStaticTree, inputDeps, incrementalStaticTree),
                ScheduleDynamicTreeBuildJobs(ref world, timeStep, gravity, threadCount, inputDeps, incrementalDynamicTree));
        }

        /// <summary>
        /// Schedule a set of jobs to build the static tree of the broadphase based on the given world.
        /// </summary>
        public JobHandle ScheduleStaticTreeBuildJobs(
            ref PhysicsWorld world, int numThreadsHint, NativeReference<int>.ReadOnly shouldDoWork, JobHandle inputDeps, bool incremental = false)
        {
            Assert.AreEqual(world.NumStaticBodies, StaticTree.NumBodies);

            if (world.NumStaticBodies == 0)
            {
                StaticTree.BoundingVolumeHierarchy.Clear();
                return inputDeps;
            }
            // else:

            if (incremental)
            {
                return new BuildIncrementalTreeJob { Tree = StaticTree }.Schedule(inputDeps);
            }
            // else:

            var aabbs = new NativeArray<Aabb>(world.NumStaticBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(world.NumStaticBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            var numStaticBodiesArray = new NativeReference<int>(Allocator.TempJob);
            JobHandle handle = new PrepareNumStaticBodiesJob
            {
                NumStaticBodies = world.NumStaticBodies,
                BuildStaticTree = shouldDoWork,
                NumStaticBodiesArray = numStaticBodiesArray
            }.Schedule(inputDeps);

            handle = new PrepareStaticBodyDataJob
            {
                RigidBodies = world.StaticBodies,
                Aabbs = aabbs,
                Points = points,
                FiltersOut = StaticTree.BodyFilters.AsArray(),
                RespondsToCollisionOut = StaticTree.RespondsToCollision.AsArray(),
                AabbMargin = world.CollisionWorld.CollisionTolerance * 0.5f, // each body contributes half
            }.ScheduleUnsafe(numStaticBodiesArray, 32, handle);

            var buildHandle = StaticTree.BoundingVolumeHierarchy.ScheduleBuildJobs(
                points, aabbs, StaticTree.BodyFilters.AsArray(), shouldDoWork, numThreadsHint, handle, StaticTree.Ranges, StaticTree.BranchCount);

            return JobHandle.CombineDependencies(buildHandle, numStaticBodiesArray.Dispose(handle));
        }

        /// <summary>
        /// Schedule a set of jobs to build the dynamic tree of the broadphase based on the given world.
        /// </summary>
        public JobHandle ScheduleDynamicTreeBuildJobs(
            ref PhysicsWorld world, float timeStep, float3 gravity, int numThreadsHint, JobHandle inputDeps, bool incremental = false)
        {
            Assert.AreEqual(world.NumDynamicBodies, DynamicTree.NumBodies);

            if (world.NumDynamicBodies == 0)
            {
                DynamicTree.BoundingVolumeHierarchy.Clear();
                return inputDeps;
            }
            // else:

            if (incremental)
            {
                return new BuildIncrementalTreeJob { Tree = DynamicTree }.Schedule(inputDeps);
            }
            // else:

            var aabbs = new NativeArray<Aabb>(world.NumDynamicBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            var points = new NativeArray<PointAndIndex>(world.NumDynamicBodies, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            JobHandle handle = new PrepareDynamicBodyDataJob
            {
                RigidBodies = world.DynamicBodies,
                MotionVelocities = world.MotionVelocities,
                Aabbs = aabbs,
                Points = points,
                FiltersOut = DynamicTree.BodyFilters.AsArray(),
                RespondsToCollisionOut = DynamicTree.RespondsToCollision.AsArray(),
                AabbMargin = world.CollisionWorld.CollisionTolerance * 0.5f, // each body contributes half
                TimeStep = timeStep,
                Gravity = gravity
            }.Schedule(world.NumDynamicBodies, 32, inputDeps);

            var shouldDoWork = new NativeReference<int>(1, Allocator.TempJob);

            // Note: points and aabbs deallocated via ScheduleBuildJobs
            handle = DynamicTree.BoundingVolumeHierarchy.ScheduleBuildJobs(
                points, aabbs, DynamicTree.BodyFilters.AsArray(), shouldDoWork, numThreadsHint, handle,
                DynamicTree.Ranges, DynamicTree.BranchCount);

            return shouldDoWork.Dispose(handle);
        }

        #endregion

        #region Find overlaps

        // Write all overlapping body pairs to the given streams,
        // where at least one of the bodies is dynamic. The results are unsorted.
        public void FindOverlaps(ref NativeStream.Writer dynamicVsDynamicPairsWriter, ref NativeStream.Writer staticVsDynamicPairsWriter)
        {
            // Dynamic-dynamic
            {
                dynamicVsDynamicPairsWriter.BeginForEachIndex(0);
                DynamicVsDynamicFindOverlappingPairsJob.ExecuteImpl(
                    new int2(1, 1), DynamicTree, ref dynamicVsDynamicPairsWriter);
                dynamicVsDynamicPairsWriter.EndForEachIndex();
            }

            // Static-dynamic
            {
                staticVsDynamicPairsWriter.BeginForEachIndex(0);
                StaticVsDynamicFindOverlappingPairsJob.ExecuteImpl(
                    new int2(1, 1), StaticTree, DynamicTree, ref staticVsDynamicPairsWriter);
                staticVsDynamicPairsWriter.EndForEachIndex();
            }
        }

        // Schedule a set of jobs which will write all overlapping body pairs to the given stream,
        // where at least one of the bodies is dynamic. The results are unsorted.
        public SimulationJobHandles ScheduleFindOverlapsJobs(out NativeStream dynamicVsDynamicPairsStream, out NativeStream staticVsDynamicPairsStream,
            JobHandle inputDeps, bool multiThreaded = true, bool incrementalDynamicTree = false, bool incrementalStaticTree = false)
        {
            SimulationJobHandles returnHandles = default;

            if (!multiThreaded)
            {
                dynamicVsDynamicPairsStream = new NativeStream(1, Allocator.TempJob);
                staticVsDynamicPairsStream = new NativeStream(1, Allocator.TempJob);
                returnHandles.FinalExecutionHandle = new FindOverlapsJob
                {
                    Broadphase = this,
                    DynamicVsDynamicPairsWriter = dynamicVsDynamicPairsStream.AsWriter(),
                    StaticVsDynamicPairsWriter = staticVsDynamicPairsStream.AsWriter()
                }.Schedule(inputDeps);

                return returnHandles;
            }

            var dynamicVsDynamicNodePairIndices = new NativeList<int2>(Allocator.TempJob);
            var staticVsDynamicNodePairIndices = new NativeList<int2>(Allocator.TempJob);

            if (incrementalDynamicTree || incrementalStaticTree)
            {
                JobHandle dynamicTreeSplitJobHandle = default;
                JobHandle staticTreeSplitJobHandle = default;

                var threadCount = JobsUtility.JobWorkerCount + 1;
                if (incrementalDynamicTree)
                {
                    dynamicTreeSplitJobHandle = new CalculateTreeSplitDataJob
                    {
                        Tree = DynamicTree,
                        ThreadCount = threadCount
                    }.Schedule(inputDeps);
                }

                if (incrementalStaticTree)
                {
                    staticTreeSplitJobHandle = new CalculateTreeSplitDataJob
                    {
                        Tree = StaticTree,
                        ThreadCount = threadCount
                    }.Schedule(inputDeps);
                }

                inputDeps = JobHandle.CombineDependencies(dynamicTreeSplitJobHandle, staticTreeSplitJobHandle);
            }

            JobHandle allocateDeps = new AllocateDynamicVsStaticNodePairs
            {
                dynamicVsDynamicNodePairIndices = dynamicVsDynamicNodePairIndices,
                staticVsDynamicNodePairIndices = staticVsDynamicNodePairIndices,
                dynamicBranchCount = DynamicTree.BranchCount,
                staticBranchCount = StaticTree.BranchCount
            }.Schedule(inputDeps);

            // Build pairs of branch node indices
            JobHandle dynamicVsDynamicPairs = new DynamicVsDynamicBuildBranchNodePairsJob
            {
                Ranges = DynamicTree.Ranges,
                NumBranches = DynamicTree.BranchCount,
                NodePairIndices = dynamicVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(allocateDeps);

            JobHandle staticVsDynamicPairs = new StaticVsDynamicBuildBranchNodePairsJob
            {
                DynamicRanges = DynamicTree.Ranges,
                StaticRanges = StaticTree.Ranges,
                NumStaticBranches = StaticTree.BranchCount,
                NumDynamicBranches = DynamicTree.BranchCount,
                NodePairIndices = staticVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(allocateDeps);

            JobHandle dynamicConstruct = NativeStream.ScheduleConstruct(out dynamicVsDynamicPairsStream, dynamicVsDynamicNodePairIndices, allocateDeps, Allocator.TempJob);
            JobHandle staticConstruct = NativeStream.ScheduleConstruct(out staticVsDynamicPairsStream, staticVsDynamicNodePairIndices, allocateDeps, Allocator.TempJob);

            // Write all overlaps to the stream (also deallocates nodePairIndices)
            JobHandle dynamicVsDynamicHandle = new DynamicVsDynamicFindOverlappingPairsJob
            {
                DynamicTree = DynamicTree,
                PairWriter = dynamicVsDynamicPairsStream.AsWriter(),
                NodePairIndices = dynamicVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(dynamicVsDynamicNodePairIndices, 1, JobHandle.CombineDependencies(dynamicVsDynamicPairs, dynamicConstruct));

            // Write all overlaps to the stream (also deallocates nodePairIndices)
            JobHandle staticVsDynamicHandle = new StaticVsDynamicFindOverlappingPairsJob
            {
                StaticTree = StaticTree,
                DynamicTree = DynamicTree,
                PairWriter = staticVsDynamicPairsStream.AsWriter(),
                NodePairIndices = staticVsDynamicNodePairIndices.AsDeferredJobArray()
            }.Schedule(staticVsDynamicNodePairIndices, 1, JobHandle.CombineDependencies(staticVsDynamicPairs, staticConstruct));

            // Dispose node pair lists
            var disposeOverlapPairs0 = dynamicVsDynamicNodePairIndices.Dispose(dynamicVsDynamicHandle);
            var disposeOverlapPairs1 = staticVsDynamicNodePairIndices.Dispose(staticVsDynamicHandle);

            returnHandles.FinalDisposeHandle = JobHandle.CombineDependencies(disposeOverlapPairs0, disposeOverlapPairs1);
            returnHandles.FinalExecutionHandle = JobHandle.CombineDependencies(dynamicVsDynamicHandle, staticVsDynamicHandle);

            return returnHandles;
        }

        #endregion

        #region Tree

        // A tree of rigid bodies
        [NoAlias]
        public struct Tree : IDisposable
        {
            [NoAlias] public NativeList<Node> Nodes;    // The nodes of the bounding volume; used in bvh
            [NoAlias] public NativeList<CollisionFilter> NodeFilters;   // The collision filter for each node (a union of all its children); used in bvh
            [NativeDisableContainerSafetyRestriction]
            [NoAlias] public NativeList<CollisionFilter> BodyFilters;  // A copy of the collision filter of each body; used when finding overlap pairs.
            [NativeDisableContainerSafetyRestriction]
            [NoAlias] public NativeList<bool> RespondsToCollision; // A copy of the RespondsToCollision flag of each body
            [NoAlias] internal NativeArray<Builder.Range> Ranges;   // Element ranges used during building; Root node index of the ranges is used as input
                                                                    // to parallel implementation of finding the overlap pairs. See Broadphase.ScheduleFindOverlapsJobs.
            [NoAlias] internal NativeArray<int> BranchCount; // Number of branches built processed in parallel in multi-threaded implementation of bvh building.
                                                             // Used as input to parallel implementation of finding the overlap pairs. See Broadphase.ScheduleFindOverlapsJobs.

            [NoAlias]
            [NativeDisableContainerSafetyRestriction]
            NativeStream m_RemoveBodyDataStream;
            [NoAlias]
            [NativeDisableContainerSafetyRestriction]
            NativeStream m_UpdateBodyDataStream;
            [NoAlias]
            [NativeDisableContainerSafetyRestriction]
            NativeStream m_InsertBodyDataStream;
            [NoAlias]
            [NativeDisableContainerSafetyRestriction]
            NativeList<ElementLocationData> m_UpdatedElementLocationDataList;
            [NoAlias]
            IncrementalInsertionContext m_IncrementalInsertionContext;
            [NoAlias]
            NativeReference<bool> m_Incremental;

            // Data stream representing rigid bodies that need to be removed from the tree, e.g., due to their deletion.
            // Used as part of the incremental broadphase.
            public NativeStream RemoveBodyDataStream
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => m_RemoveBodyDataStream;
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set => m_RemoveBodyDataStream = value;
            }

            // Data stream representing rigid bodies for which data needs to be updated in the tree, i.e., their
            // Aabb or collision filter.
            // Used as part of the incremental broadphase.
            public NativeStream UpdateBodyDataStream
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => m_UpdateBodyDataStream;
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set => m_UpdateBodyDataStream = value;
            }

            // Data stream representing rigid bodies that need to be inserted into the tree, e.g., due to their creation.
            // Used as part of the incremental broadphase.
            public NativeStream InsertBodyDataStream
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => m_InsertBodyDataStream;
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set => m_InsertBodyDataStream = value;
            }

            // Latest incremental location data for rigid bodies which were inserted into the tree or moved within the tree.
            // Used for updating temporal coherence data for rigid bodies as part of the incremental broadphase.
            public NativeList<ElementLocationData> UpdatedElementLocationDataList
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get => m_UpdatedElementLocationDataList;
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                set => m_UpdatedElementLocationDataList = value;
            }

            internal Allocator Allocator;

            internal bool Incremental
            {
                get => m_Incremental.Value;
                set => m_Incremental.Value = value;
            }

            public BoundingVolumeHierarchy BoundingVolumeHierarchy
            {
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                get
                {
                    unsafe
                    {
                        return new BoundingVolumeHierarchy(Nodes.GetUnsafeList(), NodeFilters.GetUnsafeList(), clear: false);
                    }
                }
            }

            public int NumBodies => BodyFilters.IsCreated ? BodyFilters.Length : 0;

            public Tree(int numBodies, Allocator allocator = Allocator.Persistent)
            {
                this = default;
                Allocator = allocator;
                SetCapacity(numBodies);
                Ranges = new NativeArray<BoundingVolumeHierarchy.Builder.Range>(
                    BoundingVolumeHierarchy.Constants.MaxNumTreeBranches, allocator, NativeArrayOptions.UninitializedMemory);
                BranchCount = new NativeArray<int>(1, allocator, NativeArrayOptions.ClearMemory);

                m_RemoveBodyDataStream = default;
                m_UpdateBodyDataStream = default;
                m_InsertBodyDataStream = default;
                m_UpdatedElementLocationDataList = default;
                m_IncrementalInsertionContext = new IncrementalInsertionContext(128, allocator);

                m_Incremental = new NativeReference<bool>(allocator);
                m_Incremental.Value = false;
            }

            public void Reset(int numBodies)
            {
                if (numBodies != BodyFilters.Length)
                {
                    SetCapacity(numBodies);
                }

                m_RemoveBodyDataStream = default;
                m_UpdateBodyDataStream = default;
                m_InsertBodyDataStream = default;
                m_UpdatedElementLocationDataList = default;

                m_IncrementalInsertionContext.Clear();
            }

            private void SetCapacity(int numBodies)
            {
                int numNodes = numBodies + BoundingVolumeHierarchy.Constants.MaxNumTreeBranches;

                if (!Nodes.IsCreated)
                {
                    Nodes = new NativeList<BoundingVolumeHierarchy.Node>(numNodes, Allocator);
                    // Always initialize first 2 nodes as empty, to gracefully return from queries on an empty tree
                    Nodes.ResizeUninitialized(2);
                    Nodes[0] = Nodes[1] = BoundingVolumeHierarchy.Node.Empty;
                }
                else if (numNodes > Nodes.Capacity)
                {
                    Nodes.SetCapacity(numNodes);
                }

                if (!NodeFilters.IsCreated)
                {
                    NodeFilters = new NativeList<CollisionFilter>(numNodes, Allocator);
                    NodeFilters.ResizeUninitialized(2);
                    NodeFilters[0] = NodeFilters[1] = CollisionFilter.Zero;
                }
                else if (numNodes > NodeFilters.Capacity)
                {
                    NodeFilters.SetCapacity(numNodes);
                }

                if (!BodyFilters.IsCreated)
                {
                    BodyFilters = new NativeList<CollisionFilter>(numBodies, Allocator);
                }
                BodyFilters.ResizeUninitialized(numBodies);

                if (!RespondsToCollision.IsCreated)
                {
                    RespondsToCollision = new NativeList<bool>(numBodies, Allocator);
                }
                RespondsToCollision.ResizeUninitialized(numBodies);
            }

            public Tree Clone()
            {
                var clone = new Tree
                {
                    Allocator = Allocator,
                    Nodes = new NativeList<Node>(Nodes.Length, Allocator),
                    NodeFilters = new NativeList<CollisionFilter>(NodeFilters.Length, Allocator),
                    BodyFilters = new NativeList<CollisionFilter>(BodyFilters.Length, Allocator),
                    RespondsToCollision = new NativeList<bool>(RespondsToCollision.Length, Allocator),
                    Ranges = new NativeArray<BoundingVolumeHierarchy.Builder.Range>(Ranges, Allocator),
                    BranchCount = new NativeArray<int>(BranchCount, Allocator),
                    m_RemoveBodyDataStream = default,
                    m_UpdateBodyDataStream = default,
                    m_InsertBodyDataStream = default,
                    m_UpdatedElementLocationDataList = default,
                    m_IncrementalInsertionContext = new IncrementalInsertionContext(128, Allocator.Persistent),
                    m_Incremental = new NativeReference<bool>(Allocator.Persistent)
                };
                clone.Nodes.CopyFrom(Nodes);
                clone.NodeFilters.CopyFrom(NodeFilters);
                clone.BodyFilters.CopyFrom(BodyFilters);
                clone.RespondsToCollision.CopyFrom(RespondsToCollision);
                clone.Incremental = Incremental;

                return clone;
            }

            public void Dispose()
            {
                if (Nodes.IsCreated)
                    Nodes.Dispose();

                if (NodeFilters.IsCreated)
                    NodeFilters.Dispose();

                if (BodyFilters.IsCreated)
                    BodyFilters.Dispose();

                if (RespondsToCollision.IsCreated)
                    RespondsToCollision.Dispose();

                if (Ranges.IsCreated)
                    Ranges.Dispose();

                if (BranchCount.IsCreated)
                    BranchCount.Dispose();

                if (m_IncrementalInsertionContext.IsCreated)
                    m_IncrementalInsertionContext.Dispose();

                if (m_Incremental.IsCreated)
                    m_Incremental.Dispose();
            }

            [GenerateTestsForBurstCompatibility]
            public void BuildIncremental()
            {
                var bvh = BoundingVolumeHierarchy;
                if (bvh.NodeCount == 0 || // Tree has not yet been initialized. First two nodes in an empty tree are 0: invalid node, 1: root node.
                    !Incremental) // Tree was previously not built incrementally. We need to clear out any previous content to make sure we don't retain any leftover, untracked bodies.
                {
                    bvh.Clear();
                }

                Incremental = true;

                var removalsRequested = RemoveBodyDataStream.IsCreated && !RemoveBodyDataStream.IsEmpty();
                var updatesRequested = UpdateBodyDataStream.IsCreated && !UpdateBodyDataStream.IsEmpty();
                if (removalsRequested || updatesRequested)
                {
                    var removals = removalsRequested ? RemoveBodyDataStream.ToNativeArray<RemovalData>(Allocator.Temp) : new NativeArray<RemovalData>();
                    var updates = updatesRequested ? UpdateBodyDataStream.ToNativeArray<UpdateData>(Allocator.Temp) : new NativeArray<UpdateData>();
                    // remove and update bodies
                    if (removals.Length + updates.Length != 0)
                    {
#if ENABLE_PROFILER
                        s_RemovalsMarker.Begin();
#endif
                        bvh.RemoveOrUpdate(removals, updates, BodyFilters.AsArray());
#if ENABLE_PROFILER
                        s_RemovalsMarker.End();
#endif
                    }
                }

                var insertionsRequested = InsertBodyDataStream.IsCreated && !InsertBodyDataStream.IsEmpty();
                if (insertionsRequested)
                {
                    // insert bodies
#if ENABLE_PROFILER
                    s_InsertionsMarker.Begin();
#endif
                    m_IncrementalInsertionContext.Clear();

                    var insertionReader = InsertBodyDataStream.AsReader();
                    for (int i = 0; i < insertionReader.ForEachCount; ++i)
                    {
                        insertionReader.BeginForEachIndex(i);

                        while (insertionReader.RemainingItemCount > 0)
                        {
                            var data = insertionReader.Read<InsertionData>();
                            m_IncrementalInsertionContext.Insert(ref bvh, data.Aabb, data.PointAndIndex, data.Filter);
                        }

                        insertionReader.EndForEachIndex();
                    }

                    // copy updated element location data from insertion context to output list
                    SafetyChecks.CheckAreEqualAndThrow(true, UpdatedElementLocationDataList.IsCreated);
                    UpdatedElementLocationDataList.Clear();
                    m_IncrementalInsertionContext.CopyTo(UpdatedElementLocationDataList);

#if ENABLE_PROFILER
                    s_InsertionsMarker.End();
#endif
                }

                // todo: Going forward, rebuild everything from scratch if first time (no previous tree available) or if too many changes have been detected.
                // This should  be done in parallel via different jobs scheduled when scheduling the "build broadphase" jobs.
                // The check for previously existing tree can easily be done at that point.
                // Note though that if we are in incremental mode, we must also set the node count in the underlying BVH UnsafeList to the correct value.
                // Otherwise we can not incrementally operate on the "from scratch" created BVH. This is also required for the BVH optimization phase where
                // we would create a completely new, from scratch built BVH and then overwrite the existing one with it.
                // The node count of the UnsafeList is currently not yet set when building from scratch. See BoundingVolumeHierarchy.Build() (easy, build function knows node count)
                // and BuildBranch().
                // For BuildBranch() we will need to do obtain the final node count that is used in a post process, such as FinalizeTreeJob.
                // FinalizeTreeJob performs the remaining refit for the first portion of the tree (see
                // BoundingVolumeHierarchy.Refit(startNodeIndex, endNodeIndex <=== here it sets the minimum branch offset (starting node of the branch) from all BuildBranchJobs).
                // And at this point in time, we should be able to get the max node count as well, if the BuildBranchJobs communicate that information.
                // This could be done by providing a NativeArray<int> to the BuildBranchJobs, which is filled with the max node count per job, which we know within the
                // BuildBranch() function. In the FinalizeTreeJob we can then get the total node count by finding the max in the array.
            }
        }

        #endregion

        #region Queries

        internal struct RigidBodyOverlapsCollector : IOverlapCollector
        {
            public NativeList<int> RigidBodyIndices;

            public unsafe void AddRigidBodyIndices(int* indices, int count)
            {
                RigidBodyIndices.AddRange(indices, count);
            }

            public unsafe void AddColliderKeys(ColliderKey* keys, int count) => SafetyChecks.ThrowNotSupportedException();

            public void PushCompositeCollider(ColliderKeyPath compositeKey) => SafetyChecks.ThrowNotSupportedException();

            public void PopCompositeCollider(uint numCompositeKeyBits) => SafetyChecks.ThrowNotSupportedException();
        }

        // Test broadphase nodes against the aabb in input. For any overlapping
        // tree leaves, put the body indices into the output leafIndices.
        public void OverlapAabb(OverlapAabbInput input, NativeArray<RigidBody> rigidBodies, ref NativeList<int> rigidBodyIndices)
        {
            if (input.Filter.IsEmpty)
                return;
            var leafProcessor = new BvhLeafProcessor(rigidBodies);
            var leafCollector = new RigidBodyOverlapsCollector { RigidBodyIndices = rigidBodyIndices };

            leafProcessor.BaseRigidBodyIndex = DynamicTree.NumBodies;
            StaticTree.BoundingVolumeHierarchy.AabbOverlap(input, ref leafProcessor, ref leafCollector);

            leafProcessor.BaseRigidBodyIndex = 0;
            DynamicTree.BoundingVolumeHierarchy.AabbOverlap(input, ref leafProcessor, ref leafCollector);
        }

        public bool CastRay<T>(RaycastInput input, NativeArray<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<RaycastHit>
        {
            if (input.Filter.IsEmpty)
                return false;

            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            leafProcessor.BaseRigidBodyIndex = DynamicTree.NumBodies;
            bool hasHit = StaticTree.BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);

            leafProcessor.BaseRigidBodyIndex = 0;
            hasHit |= DynamicTree.BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);

            return hasHit;
        }

        public unsafe bool CastCollider<T>(ColliderCastInput input, NativeArray<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<ColliderCastHit>
        {
            Assert.IsTrue(input.Collider != null);
            if (input.Collider->GetCollisionFilter().IsEmpty)
                return false;

            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            leafProcessor.BaseRigidBodyIndex = DynamicTree.NumBodies;
            bool hasHit = StaticTree.BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);

            leafProcessor.BaseRigidBodyIndex = 0;
            hasHit |= DynamicTree.BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);

            return hasHit;
        }

        public bool CalculateDistance<T>(PointDistanceInput input, NativeArray<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            if (input.Filter.IsEmpty)
                return false;
            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            leafProcessor.BaseRigidBodyIndex = DynamicTree.NumBodies;
            bool hasHit = StaticTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);

            leafProcessor.BaseRigidBodyIndex = 0;
            hasHit |= DynamicTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);

            return hasHit;
        }

        public unsafe bool CalculateDistance<T>(ColliderDistanceInput input, NativeArray<RigidBody> rigidBodies, ref T collector)
            where T : struct, ICollector<DistanceHit>
        {
            Assert.IsTrue(input.Collider != null);
            if (input.Collider->GetCollisionFilter().IsEmpty)
                return false;
            var leafProcessor = new BvhLeafProcessor(rigidBodies);

            leafProcessor.BaseRigidBodyIndex = DynamicTree.NumBodies;
            bool hasHit = StaticTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);

            leafProcessor.BaseRigidBodyIndex = 0;
            hasHit |= DynamicTree.BoundingVolumeHierarchy.Distance(input, ref leafProcessor, ref collector);

            return hasHit;
        }

        internal struct BvhLeafProcessor :
            BoundingVolumeHierarchy.IRaycastLeafProcessor,
                BoundingVolumeHierarchy.IColliderCastLeafProcessor,
                BoundingVolumeHierarchy.IPointDistanceLeafProcessor,
                BoundingVolumeHierarchy.IColliderDistanceLeafProcessor,
                BoundingVolumeHierarchy.IAabbOverlapLeafProcessor
        {
            private readonly NativeArray<RigidBody> m_Bodies;
            public int BaseRigidBodyIndex;

            public BvhLeafProcessor(NativeArray<RigidBody> bodies)
            {
                m_Bodies = bodies;
                BaseRigidBodyIndex = 0;
            }

            public bool AabbOverlap(int rigidBodyIndex, ref NativeList<int> allHits)
            {
                allHits.Add(BaseRigidBodyIndex + rigidBodyIndex);
                return true;
            }

            public bool RayLeaf<T>(RaycastInput input, int rigidBodyIndex, ref T collector) where T : struct, ICollector<RaycastHit>
            {
                rigidBodyIndex += BaseRigidBodyIndex;

                input.QueryContext.IsInitialized = true;
                input.QueryContext.RigidBodyIndex = rigidBodyIndex;

                RigidBody body = m_Bodies[rigidBodyIndex];

                return body.CastRay(input, ref collector);
            }

            public unsafe bool ColliderCastLeaf<T>(ColliderCastInput input, int rigidBodyIndex, ref T collector)
                where T : struct, ICollector<ColliderCastHit>
            {
                rigidBodyIndex += BaseRigidBodyIndex;

                input.QueryContext.IsInitialized = true;
                input.QueryContext.RigidBodyIndex = rigidBodyIndex;

                RigidBody body = m_Bodies[rigidBodyIndex];

                return body.CastCollider(input, ref collector);
            }

            public bool DistanceLeaf<T>(PointDistanceInput input, int rigidBodyIndex, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                rigidBodyIndex += BaseRigidBodyIndex;

                input.QueryContext.IsInitialized = true;
                input.QueryContext.RigidBodyIndex = rigidBodyIndex;

                RigidBody body = m_Bodies[rigidBodyIndex];

                return body.CalculateDistance(input, ref collector);
            }

            public unsafe bool DistanceLeaf<T>(ColliderDistanceInput input, int rigidBodyIndex, ref T collector)
                where T : struct, ICollector<DistanceHit>
            {
                rigidBodyIndex += BaseRigidBodyIndex;

                input.QueryContext.IsInitialized = true;
                input.QueryContext.RigidBodyIndex = rigidBodyIndex;

                RigidBody body = m_Bodies[rigidBodyIndex];

                return body.CalculateDistance(input, ref collector);
            }

            public unsafe void AabbLeaf<T>(OverlapAabbInput input, int rigidBodyIndex, ref T collector)
                where T : struct, IOverlapCollector
            {
                rigidBodyIndex += BaseRigidBodyIndex;
                RigidBody body = m_Bodies[rigidBodyIndex];
                if (body.Collider.IsCreated && CollisionFilter.IsCollisionEnabled(input.Filter, body.Collider.Value.GetCollisionFilter()))
                {
                    collector.AddRigidBodyIndices(&rigidBodyIndex, 1);
                }
            }
        }

        #endregion

        #region Jobs

        // Builds the broadphase in a single job.
        [BurstCompile]
        struct BuildBroadphaseJob : IJob
        {
            [ReadOnly] public NativeArray<RigidBody> StaticBodies;
            [ReadOnly] public NativeArray<RigidBody> DynamicBodies;
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [ReadOnly] public float CollisionTolerance;
            [ReadOnly] public float TimeStep;
            [ReadOnly] public float3 Gravity;
            public NativeReference<int>.ReadOnly BuildStaticTree;

            public Broadphase Broadphase;

            [ReadOnly] public bool Reset;
            [ReadOnly] public bool IncrementalDynamicTree;
            [ReadOnly] public bool IncrementalStaticTree;

            public void Execute()
            {
                Broadphase.Build(StaticBodies, DynamicBodies, MotionVelocities, CollisionTolerance, TimeStep, Gravity, BuildStaticTree.Value == 1, Reset, IncrementalDynamicTree, IncrementalStaticTree);
            }
        }

        [BurstCompile]
        struct BuildIncrementalTreeJob : IJob
        {
            public Tree Tree;

            public void Execute()
            {
                Tree.BuildIncremental();
            }
        }

        // Writes pairs of overlapping broadphase AABBs to a stream, in a single job.
        [BurstCompile]
        struct FindOverlapsJob : IJob
        {
            [ReadOnly] public Broadphase Broadphase;
            public NativeStream.Writer DynamicVsDynamicPairsWriter;
            public NativeStream.Writer StaticVsDynamicPairsWriter;

            public void Execute()
            {
                Broadphase.FindOverlaps(ref DynamicVsDynamicPairsWriter, ref StaticVsDynamicPairsWriter);
            }
        }

        [BurstCompile]
        internal struct CalculateTreeSplitDataJob : IJob
        {
            public Tree Tree;
            [ReadOnly] public int ThreadCount;

            public void Execute()
            {
                Tree.BoundingVolumeHierarchy.CalculateTreeSplitData(Tree.Ranges, ThreadCount, out var branchCount);
                Tree.BranchCount[0] = branchCount;
            }
        }

        // Allocate memory for pair indices
        [BurstCompile]
        struct AllocateDynamicVsStaticNodePairs : IJob
        {
            [ReadOnly] public NativeArray<int> dynamicBranchCount;
            [ReadOnly] public NativeArray<int> staticBranchCount;

            public NativeList<int2> dynamicVsDynamicNodePairIndices;
            public NativeList<int2> staticVsDynamicNodePairIndices;

            public void Execute()
            {
                // number of dynamic/dynamic branch overlap pairs; identical to number of elements in a symmetric table of size (dynamicBranchCount, dynamicBranchCount)
                // without the diagonal, that is, the number of elements in the lower triangular part of the table.
                int numDynamicVsDynamicBranchOverlapPairs = dynamicBranchCount[0] * (dynamicBranchCount[0] + 1) / 2;
                dynamicVsDynamicNodePairIndices.ResizeUninitialized(numDynamicVsDynamicBranchOverlapPairs);

                // number of static/dynamic branch overlap pairs; identical to number of elements in a table of size (staticBranchCount, dynamicBranchCount)
                int numStaticVsDynamicBranchOverlapPairs = staticBranchCount[0] * dynamicBranchCount[0];
                staticVsDynamicNodePairIndices.ResizeUninitialized(numStaticVsDynamicBranchOverlapPairs);
            }
        }

        // Reads broadphase data from dynamic rigid bodies
        [BurstCompile]
        internal struct PrepareDynamicBodyDataJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<RigidBody> RigidBodies;
            [ReadOnly] public NativeArray<MotionVelocity> MotionVelocities;
            [ReadOnly] public float TimeStep;
            [ReadOnly] public float3 Gravity;
            [ReadOnly] public float AabbMargin;

            public NativeArray<PointAndIndex> Points;
            public NativeArray<Aabb> Aabbs;
            [NativeDisableContainerSafetyRestriction]
            public NativeArray<CollisionFilter> FiltersOut;
            [NativeDisableContainerSafetyRestriction]
            public NativeArray<bool> RespondsToCollisionOut;

            public void Execute(int index)
            {
                ExecuteImpl(index, AabbMargin, Gravity, TimeStep, RigidBodies, MotionVelocities, Aabbs, Points, FiltersOut, RespondsToCollisionOut);
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            internal static Aabb CalculateAabb(ref RigidBody body, float aabbMargin, float3 gravity, float timeStep, in MotionVelocity motionVelocity)
            {
                // Apply gravity only on a copy to get proper expansion for the AABB,
                // actual applying of gravity will be done later in the physics step
                var mv = motionVelocity;
                mv.LinearVelocity += gravity * timeStep * mv.GravityFactor;

                MotionExpansion expansion = mv.CalculateExpansion(timeStep);
                var aabb = expansion.ExpandAabb(body.CalculateAabb());
                aabb.Expand(aabbMargin);

                return aabb;
            }

            internal static void ExecuteImpl(int index, float aabbMargin, float3 gravity, float timeStep,
                NativeArray<RigidBody> rigidBodies, NativeArray<MotionVelocity> motionVelocities,
                NativeArray<Aabb> aabbs, NativeArray<PointAndIndex> points,
                NativeArray<CollisionFilter> filtersOut, NativeArray<bool> respondsToCollisionOut)
            {
                RigidBody body = rigidBodies[index];

                var aabb = CalculateAabb(ref body, aabbMargin, gravity, timeStep, motionVelocities[index]);

                if (body.Collider.IsCreated)
                {
                    filtersOut[index] = body.Collider.Value.GetCollisionFilter();
                    respondsToCollisionOut[index] = body.Collider.Value.RespondsToCollision;
                }
                else
                {
                    filtersOut[index] = CollisionFilter.Zero;
                    respondsToCollisionOut[index] = false;
                }

                aabbs[index] = aabb;
                points[index] = new PointAndIndex
                {
                    Position = aabb.Center,
                    Index = index
                };
            }
        }

        // Prepares the NumStaticBodies value for PrepareStaticBodyDataJob
        [BurstCompile]
        struct PrepareNumStaticBodiesJob : IJob
        {
            public int NumStaticBodies;
            public NativeReference<int>.ReadOnly BuildStaticTree;
            public NativeReference<int> NumStaticBodiesArray;

            public void Execute()
            {
                if (BuildStaticTree.Value == 1)
                {
                    NumStaticBodiesArray.Value = NumStaticBodies;
                }
                else
                {
                    NumStaticBodiesArray.Value = 0;
                }
            }
        }

        // Reads broadphase data from static rigid bodies
        [BurstCompile]
        internal struct PrepareStaticBodyDataJob : IJobParallelForDefer
        {
            [ReadOnly] public NativeArray<RigidBody> RigidBodies;
            [ReadOnly] public float AabbMargin;

            public NativeArray<Aabb> Aabbs;
            public NativeArray<PointAndIndex> Points;
            [NativeDisableContainerSafetyRestriction]
            public NativeArray<CollisionFilter> FiltersOut;
            [NativeDisableContainerSafetyRestriction]
            public NativeArray<bool> RespondsToCollisionOut;

            public void Execute(int index)
            {
                ExecuteImpl(index, AabbMargin, RigidBodies, Aabbs, Points, FiltersOut, RespondsToCollisionOut);
            }

            internal static Aabb CalculateAabb(ref RigidBody body, float aabbMargin)
            {
                var aabb = body.CalculateAabb();
                aabb.Expand(aabbMargin);

                return aabb;
            }

            internal static void ExecuteImpl(int index, float aabbMargin,
                NativeArray<RigidBody> rigidBodies, NativeArray<Aabb> aabbs, NativeArray<PointAndIndex> points,
                NativeArray<CollisionFilter> filtersOut, NativeArray<bool> respondsToCollisionOut)
            {
                RigidBody body = rigidBodies[index];

                Aabb aabb = CalculateAabb(ref body, aabbMargin);

                if (body.Collider.IsCreated)
                {
                    filtersOut[index] = body.Collider.Value.GetCollisionFilter();
                    respondsToCollisionOut[index] = body.Collider.Value.RespondsToCollision;
                }
                else
                {
                    filtersOut[index] = CollisionFilter.Zero;
                    respondsToCollisionOut[index] = false;
                }

                aabbs[index] = aabb;
                points[index] = new PointAndIndex
                {
                    Position = aabb.Center,
                    Index = index
                };
            }
        }

        // Builds a list of branch node index pairs (an input to FindOverlappingPairsJob)
        [BurstCompile]
        internal struct DynamicVsDynamicBuildBranchNodePairsJob : IJob
        {
            [ReadOnly] public NativeArray<Builder.Range> Ranges;
            [ReadOnly] public NativeArray<int> NumBranches;
            public NativeArray<int2> NodePairIndices;

            public void Execute()
            {
                int numBranches = NumBranches[0];

                int arrayIndex = 0;

                // First add all branch self overlaps.
                // Start with largest branch
                for (int i = 0; i < numBranches; i++)
                {
                    NodePairIndices[arrayIndex++] = new int2(Ranges[i].Root, Ranges[i].Root);
                }

                for (int i = 0; i < numBranches; i++)
                {
                    for (int j = i + 1; j < numBranches; j++)
                    {
                        var pair = new int2 { x = Ranges[i].Root, y = Ranges[j].Root };
                        NodePairIndices[arrayIndex++] = pair;
                    }
                }
            }
        }

        // Builds a list of branch node index pairs (an input to FindOverlappingPairsJob)
        [BurstCompile]
        struct StaticVsDynamicBuildBranchNodePairsJob : IJob
        {
            [ReadOnly] public NativeArray<Builder.Range> StaticRanges;
            [ReadOnly] public NativeArray<Builder.Range> DynamicRanges;
            [ReadOnly] public NativeArray<int> NumStaticBranches;
            [ReadOnly] public NativeArray<int> NumDynamicBranches;
            public NativeArray<int2> NodePairIndices;

            public void Execute()
            {
                int numStaticBranches = NumStaticBranches[0];
                int numDynamicBranches = NumDynamicBranches[0];

                int arrayIndex = 0;
                for (int i = 0; i < numStaticBranches; i++)
                {
                    for (int j = 0; j < numDynamicBranches; j++)
                    {
                        var pair = new int2 { x = StaticRanges[i].Root, y = DynamicRanges[j].Root };
                        NodePairIndices[arrayIndex++] = pair;
                    }
                }
            }
        }

        // An implementation of IOverlapCollector which filters and writes body pairs to a native stream
        internal unsafe struct BodyPairWriter : ITreeOverlapCollector
        {
            const int k_Capacity = 256;
            const int k_Margin = 64;
            const int k_Threshold = k_Capacity - k_Margin;

            fixed int m_PairsLeft[k_Capacity];
            fixed int m_PairsRight[k_Capacity];

            private readonly NativeStream.Writer* m_CollidingPairs;
            private readonly CollisionFilter* m_BodyFiltersLeft;
            private readonly CollisionFilter* m_BodyFiltersRight;
            private readonly bool* m_BodyRespondsToCollisionLeft;
            private readonly bool* m_BodyRespondsToCollisionRight;
            private readonly int m_BodyIndexABase;
            private readonly int m_BodyIndexBBase;
            private int m_Count;

            public BodyPairWriter(NativeStream.Writer* collidingPairs, CollisionFilter* bodyFiltersLeft, CollisionFilter* bodyFiltersRight,
                                  bool* bodyRespondsToCollisionLeft, bool* bodyRespondsToCollisionRight, int bodyIndexABase, int bodyIndexBBase)
            {
                m_CollidingPairs = collidingPairs;
                m_BodyFiltersLeft = bodyFiltersLeft;
                m_BodyFiltersRight = bodyFiltersRight;
                m_BodyRespondsToCollisionLeft = bodyRespondsToCollisionLeft;
                m_BodyRespondsToCollisionRight = bodyRespondsToCollisionRight;
                m_BodyIndexABase = bodyIndexABase;
                m_BodyIndexBBase = bodyIndexBBase;
                m_Count = 0;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void AddPairs(int4 pairsLeft, int4 pairsRight, int count, bool swapped = false)
            {
                if (swapped)
                {
                    fixed(int* l = m_PairsRight)
                    {
                        *((int4*)(l + m_Count)) = pairsLeft;
                    }

                    fixed(int* r = m_PairsLeft)
                    {
                        *((int4*)(r + m_Count)) = pairsRight;
                    }
                }
                else
                {
                    fixed(int* l = m_PairsLeft)
                    {
                        *((int4*)(l + m_Count)) = pairsLeft;
                    }

                    fixed(int* r = m_PairsRight)
                    {
                        *((int4*)(r + m_Count)) = pairsRight;
                    }
                }

                m_Count += count;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void AddPairs(int pairLeft, int4 pairsRight, int countR)
            {
                fixed(int* l = m_PairsLeft)
                {
                    *((int4*)(l + m_Count)) = new int4(pairLeft);
                }

                fixed(int* r = m_PairsRight)
                {
                    *((int4*)(r + m_Count)) = pairsRight;
                }

                m_Count += countR;
            }

            public void Close()
            {
                Flush();
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public void FlushIfNeeded()
            {
                if (m_Count >= k_Threshold)
                {
                    Flush();
                }
            }

            [MethodImpl(MethodImplOptions.NoInlining)]
            private void Flush()
            {
                if (m_Count != 0)
                {
                    fixed(int* l = m_PairsLeft)
                    {
                        fixed(int* r = m_PairsRight)
                        {
                            for (int i = 0; i < m_Count; i++)
                            {
                                int bodyALocalIndex = l[i];
                                int bodyBLocalIndex = r[i];
                                if (m_BodyRespondsToCollisionLeft[bodyALocalIndex] && m_BodyRespondsToCollisionRight[bodyBLocalIndex])
                                {
                                    if (CollisionFilter.IsCollisionEnabled(m_BodyFiltersLeft[bodyALocalIndex], m_BodyFiltersRight[bodyBLocalIndex]))
                                    {
                                        m_CollidingPairs->Write(new BodyIndexPair
                                        {
                                            BodyIndexA = bodyALocalIndex + m_BodyIndexABase,
                                            BodyIndexB = bodyBLocalIndex + m_BodyIndexBBase
                                        });
                                    }
                                }
                            }
                        }
                    }

                    m_Count = 0;
                }
            }
        }

        // Writes pairs of overlapping broadphase AABBs to a stream.
        [BurstCompile]
        internal struct DynamicVsDynamicFindOverlappingPairsJob : IJobParallelForDefer
        {
            [ReadOnly] public Tree DynamicTree;
            [ReadOnly] public NativeArray<int2> NodePairIndices;
            public NativeStream.Writer PairWriter;

            public void Execute(int index)
            {
                PairWriter.BeginForEachIndex(index);

                int2 pair = NodePairIndices[index];
                ExecuteImpl(pair, DynamicTree, ref PairWriter);

                PairWriter.EndForEachIndex();
            }

            internal static unsafe void ExecuteImpl(int2 pair, Tree dynamicTree, ref NativeStream.Writer pairWriter)
            {
                var bodyFilters = (CollisionFilter*)dynamicTree.BodyFilters.GetUnsafeReadOnlyPtr();
                var bodyRespondsToCollision = (bool*)dynamicTree.RespondsToCollision.GetUnsafeReadOnlyPtr();
                var bufferedPairs = new BodyPairWriter((NativeStream.Writer*)UnsafeUtility.AddressOf(ref pairWriter), bodyFilters, bodyFilters, bodyRespondsToCollision, bodyRespondsToCollision, 0, 0);
                new BoundingVolumeHierarchy(dynamicTree.Nodes.AsArray(), dynamicTree.NodeFilters.AsArray()).SelfBvhOverlap(ref bufferedPairs, pair.x, pair.y);
                bufferedPairs.Close();
            }
        }

        // Writes pairs of overlapping broadphase AABBs to a stream.
        [BurstCompile]
        struct StaticVsDynamicFindOverlappingPairsJob : IJobParallelForDefer
        {
            [ReadOnly] public Tree StaticTree;
            [ReadOnly] public Tree DynamicTree;
            [ReadOnly] public NativeArray<int2> NodePairIndices;
            public NativeStream.Writer PairWriter;

            public void Execute(int index)
            {
                PairWriter.BeginForEachIndex(index);

                int2 pair = NodePairIndices[index];
                ExecuteImpl(pair, StaticTree, DynamicTree, ref PairWriter);

                PairWriter.EndForEachIndex();
            }

            internal static unsafe void ExecuteImpl(int2 pair, Tree staticTree, Tree dynamicTree, ref NativeStream.Writer pairWriter)
            {
                var staticBvh = new BoundingVolumeHierarchy(staticTree.Nodes.AsArray(), staticTree.NodeFilters.AsArray());
                var dynamicBvh = new BoundingVolumeHierarchy(dynamicTree.Nodes.AsArray(), dynamicTree.NodeFilters.AsArray());

                var bodyPairWriter = new BodyPairWriter((NativeStream.Writer*)UnsafeUtility.AddressOf(ref pairWriter),
                    (CollisionFilter*)staticTree.BodyFilters.GetUnsafeReadOnlyPtr(), (CollisionFilter*)dynamicTree.BodyFilters.GetUnsafeReadOnlyPtr(),
                    (bool*)staticTree.RespondsToCollision.GetUnsafeReadOnlyPtr(), (bool*)dynamicTree.RespondsToCollision.GetUnsafeReadOnlyPtr(),
                    dynamicTree.NumBodies, 0);

                staticBvh.BvhOverlap(ref bodyPairWriter, dynamicBvh, pair.x, pair.y);

                bodyPairWriter.Close();
            }
        }

        #endregion
    }
}
